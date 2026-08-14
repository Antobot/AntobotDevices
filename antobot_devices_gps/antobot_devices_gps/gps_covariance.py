"""Covariance policy for quality-gated F9P NavSatFix messages."""

from dataclasses import dataclass
import math
from typing import Optional, Sequence


@dataclass(frozen=True)
class SignalHealthSample:
    """The signal-health fields that influence covariance inflation."""

    fresh: bool
    valid: bool
    cno_median_dbhz: float
    cno_p10_dbhz: float
    satellites_in_view: int = 0
    severe_persistent_fault: bool = False
    unavailable_duration_s: Optional[float] = None


@dataclass(frozen=True)
class CovarianceResult:
    """Diagonal covariance selected for one NavSatFix sample."""

    covariance_xx: float
    covariance_yy: float
    covariance_zz: float
    covariance_scale: float
    horizontal_stddev_m: float
    reason: str


class PersistentSignalFaultTracker:
    """Detect a sustained severe C/N0 event accompanied by a satellite drop."""

    def __init__(
        self,
        severe_cno_dbhz: float = 30.0,
        min_consecutive_epochs: int = 3,
        min_satellite_drop: int = 3,
    ) -> None:
        if severe_cno_dbhz < 0.0:
            raise ValueError("severe C/N0 threshold must not be negative")
        if min_consecutive_epochs < 1 or min_satellite_drop < 1:
            raise ValueError("signal fault thresholds must be positive")

        self.severe_cno_dbhz = severe_cno_dbhz
        self.min_consecutive_epochs = min_consecutive_epochs
        self.min_satellite_drop = min_satellite_drop
        self._severe_epochs = 0
        self._pre_fault_satellites: Optional[int] = None
        self._previous_satellites: Optional[int] = None

    def update(self, sample: SignalHealthSample) -> SignalHealthSample:
        """Return a sample marked only after the full fault pattern is present."""
        severe = (
            sample.fresh
            and sample.valid
            and math.isfinite(sample.cno_median_dbhz)
            and sample.cno_median_dbhz < self.severe_cno_dbhz
            and sample.satellites_in_view > 0
        )
        if not severe:
            self._severe_epochs = 0
            self._pre_fault_satellites = None
            self._previous_satellites = sample.satellites_in_view
            return sample

        if self._severe_epochs == 0:
            self._pre_fault_satellites = self._previous_satellites
        self._severe_epochs += 1
        baseline = self._pre_fault_satellites
        satellite_drop = (
            baseline is not None
            and baseline - sample.satellites_in_view >= self.min_satellite_drop
        )
        self._previous_satellites = sample.satellites_in_view
        return SignalHealthSample(
            fresh=sample.fresh,
            valid=sample.valid,
            cno_median_dbhz=sample.cno_median_dbhz,
            cno_p10_dbhz=sample.cno_p10_dbhz,
            satellites_in_view=sample.satellites_in_view,
            severe_persistent_fault=(
                self._severe_epochs >= self.min_consecutive_epochs and satellite_drop
            ),
        )


class GpsCovarianceModel:
    """Convert receiver quality into conservative NavSatFix covariance."""

    def __init__(
        self,
        min_horizontal_stddev_m: float = 0.02,
        vertical_covariance_m2: float = 100.0,
        hdop_good_threshold: float = 2.0,
        hdop_warning_threshold: float = 3.0,
        hdop_max_threshold: float = 5.0,
        hdop_warning_covariance_multiplier: float = 2.0,
        hdop_critical_covariance_multiplier: float = 4.0,
        fixed_covariance_multiplier: float = 1.0,
        fixed_duration_short_s: float = 3.0,
        fixed_duration_warmup_s: float = 5.0,
        fixed_duration_short_covariance_multiplier: float = 9.0,
        fixed_duration_warmup_covariance_multiplier: float = 4.0,
        float_covariance_multiplier: float = 25.0,
        differential_covariance_multiplier: float = 100.0,
        standalone_covariance_multiplier: float = 400.0,
        cno_good_dbhz: float = 40.0,
        cno_warning_dbhz: float = 35.0,
        cno_critical_dbhz: float = 30.0,
        cno_warning_covariance_multiplier: float = 2.0,
        cno_critical_covariance_multiplier: float = 4.0,
        cno_severe_covariance_multiplier: float = 9.0,
        cno_unavailable_grace_s: float = 2.0,
        cno_unavailable_critical_s: float = 10.0,
        cno_unavailable_warning_covariance_multiplier: float = 2.0,
        cno_unavailable_critical_covariance_multiplier: float = 4.0,
        rtcm_good_age_s: float = 2.0,
        rtcm_warning_age_s: float = 5.0,
        rtcm_critical_age_s: float = 10.0,
        rtcm_warning_covariance_multiplier: float = 2.0,
        rtcm_critical_covariance_multiplier: float = 9.0,
        max_covariance_scale: float = 400.0,
        unavailable_covariance_m2: float = 1_000_000.0,
        require_signal_health: bool = False,
        require_rtcm: bool = True,
    ) -> None:
        if min_horizontal_stddev_m <= 0.0:
            raise ValueError("minimum horizontal standard deviation must be positive")
        if vertical_covariance_m2 <= 0.0:
            raise ValueError("vertical covariance must be positive")
        if unavailable_covariance_m2 <= 0.0 or max_covariance_scale < 1.0:
            raise ValueError("covariance parameters must be positive")
        if not 0.0 < hdop_good_threshold < hdop_warning_threshold < hdop_max_threshold:
            raise ValueError("HDOP thresholds must be ascending and positive")
        if (
            hdop_warning_covariance_multiplier < 1.0
            or hdop_critical_covariance_multiplier < hdop_warning_covariance_multiplier
        ):
            raise ValueError("HDOP covariance multipliers must be ascending and at least one")
        if not 0.0 < fixed_duration_short_s < fixed_duration_warmup_s:
            raise ValueError("fixed-duration thresholds must be ascending and positive")
        if (
            fixed_duration_short_covariance_multiplier < 1.0
            or fixed_duration_warmup_covariance_multiplier < 1.0
        ):
            raise ValueError("fixed-duration covariance multipliers must be at least one")
        if not cno_good_dbhz > cno_warning_dbhz > cno_critical_dbhz >= 0.0:
            raise ValueError("C/N0 thresholds must be descending and non-negative")
        if not 0.0 <= cno_unavailable_grace_s < cno_unavailable_critical_s:
            raise ValueError("C/N0 unavailable thresholds must be ascending and non-negative")
        if (
            cno_unavailable_warning_covariance_multiplier < 1.0
            or cno_unavailable_critical_covariance_multiplier
            < cno_unavailable_warning_covariance_multiplier
        ):
            raise ValueError("C/N0 unavailable covariance multipliers must be ascending")
        if not 0.0 <= rtcm_good_age_s < rtcm_warning_age_s < rtcm_critical_age_s:
            raise ValueError("RTCM age thresholds must be ascending and non-negative")

        self.min_horizontal_stddev_m = min_horizontal_stddev_m
        self.vertical_covariance_m2 = vertical_covariance_m2
        self.hdop_good_threshold = hdop_good_threshold
        self.hdop_warning_threshold = hdop_warning_threshold
        self.hdop_max_threshold = hdop_max_threshold
        self.hdop_warning_covariance_multiplier = hdop_warning_covariance_multiplier
        self.hdop_critical_covariance_multiplier = hdop_critical_covariance_multiplier
        self.fixed_covariance_multiplier = fixed_covariance_multiplier
        self.fixed_duration_short_s = fixed_duration_short_s
        self.fixed_duration_warmup_s = fixed_duration_warmup_s
        self.fixed_duration_short_covariance_multiplier = (
            fixed_duration_short_covariance_multiplier
        )
        self.fixed_duration_warmup_covariance_multiplier = (
            fixed_duration_warmup_covariance_multiplier
        )
        self.float_covariance_multiplier = float_covariance_multiplier
        self.differential_covariance_multiplier = differential_covariance_multiplier
        self.standalone_covariance_multiplier = standalone_covariance_multiplier
        self.cno_good_dbhz = cno_good_dbhz
        self.cno_warning_dbhz = cno_warning_dbhz
        self.cno_critical_dbhz = cno_critical_dbhz
        self.cno_warning_covariance_multiplier = cno_warning_covariance_multiplier
        self.cno_critical_covariance_multiplier = cno_critical_covariance_multiplier
        self.cno_severe_covariance_multiplier = cno_severe_covariance_multiplier
        self.cno_unavailable_grace_s = cno_unavailable_grace_s
        self.cno_unavailable_critical_s = cno_unavailable_critical_s
        self.cno_unavailable_warning_covariance_multiplier = (
            cno_unavailable_warning_covariance_multiplier
        )
        self.cno_unavailable_critical_covariance_multiplier = (
            cno_unavailable_critical_covariance_multiplier
        )
        self.rtcm_good_age_s = rtcm_good_age_s
        self.rtcm_warning_age_s = rtcm_warning_age_s
        self.rtcm_critical_age_s = rtcm_critical_age_s
        self.rtcm_warning_covariance_multiplier = rtcm_warning_covariance_multiplier
        self.rtcm_critical_covariance_multiplier = rtcm_critical_covariance_multiplier
        self.max_covariance_scale = max_covariance_scale
        self.unavailable_covariance_m2 = unavailable_covariance_m2
        self.require_signal_health = require_signal_health
        self.require_rtcm = require_rtcm

    def calculate(
        self,
        source_covariance: Sequence[float],
        horizontal_accuracy_m: float,
        hdop: float,
        gps_quality: int,
        fixed_duration_s: Optional[float],
        integrity_ok: bool,
        signal_health: Optional[SignalHealthSample],
        rtcm_age_s: Optional[float],
    ) -> CovarianceResult:
        """Return a covariance based on the latest receiver-quality evidence."""
        if not integrity_ok:
            return self._unavailable("integrity_invalid")
        if self.require_signal_health and (
            signal_health is None or not signal_health.fresh or not signal_health.valid
        ):
            return self._unavailable("signal_health_missing")
        if self.require_rtcm and not self._valid_rtcm_age(rtcm_age_s):
            return self._unavailable("rtcm_missing")

        base_stddev = self._horizontal_stddev(source_covariance, horizontal_accuracy_m)
        if base_stddev is None:
            return self._unavailable("horizontal_accuracy_missing")
        if not math.isfinite(hdop) or hdop < 0.0:
            return self._unavailable("hdop_invalid")
        if hdop > self.hdop_max_threshold:
            return self._unavailable("hdop_excessive")
        if self._rtcm_expired(rtcm_age_s):
            return self._unavailable("rtcm_stale")
        if (
            signal_health is not None
            and signal_health.fresh
            and signal_health.valid
            and signal_health.severe_persistent_fault
        ):
            return self._unavailable("cno_severe_persistent_fault")

        quality_multiplier, quality_reason = self._quality_multiplier(gps_quality)
        duration_multiplier, duration_reason = self._fixed_duration_multiplier(
            gps_quality, fixed_duration_s
        )
        hdop_multiplier, hdop_reason = self._hdop_multiplier(hdop)
        signal_multiplier, signal_reason = self._signal_multiplier(signal_health)
        rtcm_multiplier, rtcm_reason = self._rtcm_multiplier(rtcm_age_s)
        auxiliary_multiplier = max(
            hdop_multiplier,
            signal_multiplier,
            rtcm_multiplier,
        )
        covariance_scale = min(
            quality_multiplier * duration_multiplier * auxiliary_multiplier,
            self.max_covariance_scale,
        )
        horizontal_variance = base_stddev * base_stddev * covariance_scale
        return CovarianceResult(
            covariance_xx=horizontal_variance,
            covariance_yy=horizontal_variance,
            covariance_zz=self.vertical_covariance_m2,
            covariance_scale=covariance_scale,
            horizontal_stddev_m=math.sqrt(horizontal_variance),
            reason="%s,%s,%s,%s,%s"
            % (
                quality_reason,
                duration_reason,
                hdop_reason,
                signal_reason,
                rtcm_reason,
            ),
        )

    def _horizontal_stddev(
        self, source_covariance: Sequence[float], horizontal_accuracy_m: float
    ) -> Optional[float]:
        candidates = [self.min_horizontal_stddev_m]
        if math.isfinite(horizontal_accuracy_m) and horizontal_accuracy_m > 0.0:
            candidates.append(horizontal_accuracy_m)
        for index in (0, 4):
            if len(source_covariance) > index:
                variance = source_covariance[index]
                if math.isfinite(variance) and variance > 0.0:
                    candidates.append(math.sqrt(variance))
        return max(candidates) if candidates else None

    def _quality_multiplier(self, gps_quality: int) -> tuple[float, str]:
        if gps_quality == 4:
            return self.fixed_covariance_multiplier, "rtk_fixed"
        if gps_quality == 5:
            return self.float_covariance_multiplier, "rtk_float"
        if gps_quality == 2:
            return self.differential_covariance_multiplier, "differential"
        if gps_quality == 1:
            return self.standalone_covariance_multiplier, "standalone"
        return self.standalone_covariance_multiplier, "unknown_solution"

    def _fixed_duration_multiplier(
        self, gps_quality: int, fixed_duration_s: Optional[float]
    ) -> tuple[float, str]:
        if gps_quality != 4:
            return 1.0, "fixed_duration_not_applicable"
        if fixed_duration_s is None or not math.isfinite(fixed_duration_s):
            return self.fixed_duration_short_covariance_multiplier, "fixed_duration_missing"
        if fixed_duration_s < self.fixed_duration_short_s:
            return self.fixed_duration_short_covariance_multiplier, "fixed_duration_short"
        if fixed_duration_s < self.fixed_duration_warmup_s:
            return self.fixed_duration_warmup_covariance_multiplier, "fixed_duration_warmup"
        return 1.0, "fixed_duration_ready"

    def _signal_multiplier(
        self, signal_health: Optional[SignalHealthSample]
    ) -> tuple[float, str]:
        if signal_health is None or not signal_health.fresh or not signal_health.valid:
            return self._unavailable_signal_multiplier(signal_health)
        cno = signal_health.cno_median_dbhz
        if not math.isfinite(cno):
            return self._unavailable_signal_multiplier(signal_health)
        if cno >= self.cno_good_dbhz:
            return 1.0, "cno_good"
        if cno >= self.cno_warning_dbhz:
            return self.cno_warning_covariance_multiplier, "cno_warning"
        if cno >= self.cno_critical_dbhz:
            return self.cno_critical_covariance_multiplier, "cno_low"
        return self.cno_severe_covariance_multiplier, "cno_severe"

    def _unavailable_signal_multiplier(
        self, signal_health: Optional[SignalHealthSample]
    ) -> tuple[float, str]:
        """Degrade only after C/N0 has been continuously unavailable."""
        if signal_health is None or signal_health.unavailable_duration_s is None:
            return 1.0, "signal_health_starting"

        duration_s = signal_health.unavailable_duration_s
        if not math.isfinite(duration_s) or duration_s < 0.0:
            return 1.0, "signal_health_unavailable_grace"
        if duration_s <= self.cno_unavailable_grace_s:
            return 1.0, "signal_health_unavailable_grace"
        if duration_s < self.cno_unavailable_critical_s:
            return (
                self.cno_unavailable_warning_covariance_multiplier,
                "signal_health_unavailable_warning",
            )
        return (
            self.cno_unavailable_critical_covariance_multiplier,
            "signal_health_unavailable_critical",
        )

    def _hdop_multiplier(self, hdop: float) -> tuple[float, str]:
        if hdop <= self.hdop_good_threshold:
            return 1.0, "hdop_good"
        if hdop <= self.hdop_warning_threshold:
            return self.hdop_warning_covariance_multiplier, "hdop_warning"
        return self.hdop_critical_covariance_multiplier, "hdop_critical"

    @staticmethod
    def _valid_rtcm_age(rtcm_age_s: Optional[float]) -> bool:
        return rtcm_age_s is not None and math.isfinite(rtcm_age_s) and rtcm_age_s >= 0.0

    def _rtcm_multiplier(self, rtcm_age_s: Optional[float]) -> tuple[float, str]:
        if not self._valid_rtcm_age(rtcm_age_s):
            return 1.0, "rtcm_unavailable"
        if rtcm_age_s <= self.rtcm_good_age_s:
            return 1.0, "rtcm_fresh"
        if rtcm_age_s <= self.rtcm_warning_age_s:
            return self.rtcm_warning_covariance_multiplier, "rtcm_warning"
        if rtcm_age_s <= self.rtcm_critical_age_s:
            return self.rtcm_critical_covariance_multiplier, "rtcm_aging"
        return self.rtcm_critical_covariance_multiplier, "rtcm_stale"

    def _rtcm_expired(self, rtcm_age_s: Optional[float]) -> bool:
        return (
            self._valid_rtcm_age(rtcm_age_s)
            and rtcm_age_s > self.rtcm_critical_age_s
        )

    def _unavailable(self, reason: str) -> CovarianceResult:
        stddev = math.sqrt(self.unavailable_covariance_m2)
        return CovarianceResult(
            covariance_xx=self.unavailable_covariance_m2,
            covariance_yy=self.unavailable_covariance_m2,
            covariance_zz=self.unavailable_covariance_m2,
            covariance_scale=self.unavailable_covariance_m2,
            horizontal_stddev_m=stddev,
            reason=reason,
        )
