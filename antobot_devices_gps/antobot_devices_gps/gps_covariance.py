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


@dataclass(frozen=True)
class CovarianceResult:
    """Diagonal covariance selected for one NavSatFix sample."""

    covariance_xx: float
    covariance_yy: float
    covariance_zz: float
    covariance_scale: float
    horizontal_stddev_m: float
    vertical_stddev_m: float
    reason: str


class GpsCovarianceModel:
    """Convert receiver quality into conservative NavSatFix covariance."""

    def __init__(
        self,
        min_horizontal_stddev_m: float = 0.02,
        vertical_stddev_multiplier: float = 4.0,
        nominal_hdop: float = 1.0,
        fixed_covariance_multiplier: float = 1.0,
        float_covariance_multiplier: float = 25.0,
        differential_covariance_multiplier: float = 100.0,
        standalone_covariance_multiplier: float = 400.0,
        cno_good_dbhz: float = 40.0,
        cno_warning_dbhz: float = 35.0,
        cno_critical_dbhz: float = 25.0,
        cno_warning_covariance_multiplier: float = 4.0,
        cno_critical_covariance_multiplier: float = 25.0,
        rtcm_good_age_s: float = 1.0,
        rtcm_warning_age_s: float = 2.0,
        rtcm_critical_age_s: float = 5.0,
        rtcm_warning_covariance_multiplier: float = 4.0,
        rtcm_critical_covariance_multiplier: float = 25.0,
        unavailable_covariance_m2: float = 1_000_000.0,
        require_signal_health: bool = False,
        require_rtcm: bool = True,
    ) -> None:
        if min_horizontal_stddev_m <= 0.0 or vertical_stddev_multiplier <= 0.0:
            raise ValueError("standard-deviation parameters must be positive")
        if nominal_hdop <= 0.0 or unavailable_covariance_m2 <= 0.0:
            raise ValueError("covariance parameters must be positive")
        if not cno_good_dbhz > cno_warning_dbhz > cno_critical_dbhz >= 0.0:
            raise ValueError("C/N0 thresholds must be descending and non-negative")
        if not 0.0 <= rtcm_good_age_s < rtcm_warning_age_s < rtcm_critical_age_s:
            raise ValueError("RTCM age thresholds must be ascending and non-negative")

        self.min_horizontal_stddev_m = min_horizontal_stddev_m
        self.vertical_stddev_multiplier = vertical_stddev_multiplier
        self.nominal_hdop = nominal_hdop
        self.fixed_covariance_multiplier = fixed_covariance_multiplier
        self.float_covariance_multiplier = float_covariance_multiplier
        self.differential_covariance_multiplier = differential_covariance_multiplier
        self.standalone_covariance_multiplier = standalone_covariance_multiplier
        self.cno_good_dbhz = cno_good_dbhz
        self.cno_warning_dbhz = cno_warning_dbhz
        self.cno_critical_dbhz = cno_critical_dbhz
        self.cno_warning_covariance_multiplier = cno_warning_covariance_multiplier
        self.cno_critical_covariance_multiplier = cno_critical_covariance_multiplier
        self.rtcm_good_age_s = rtcm_good_age_s
        self.rtcm_warning_age_s = rtcm_warning_age_s
        self.rtcm_critical_age_s = rtcm_critical_age_s
        self.rtcm_warning_covariance_multiplier = rtcm_warning_covariance_multiplier
        self.rtcm_critical_covariance_multiplier = rtcm_critical_covariance_multiplier
        self.unavailable_covariance_m2 = unavailable_covariance_m2
        self.require_signal_health = require_signal_health
        self.require_rtcm = require_rtcm

    def calculate(
        self,
        source_covariance: Sequence[float],
        horizontal_accuracy_m: float,
        hdop: float,
        gps_quality: int,
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

        quality_multiplier, quality_reason = self._quality_multiplier(gps_quality)
        hdop_multiplier = max(1.0, hdop / self.nominal_hdop) ** 2
        signal_multiplier, signal_reason = self._signal_multiplier(signal_health)
        rtcm_multiplier, rtcm_reason = self._rtcm_multiplier(rtcm_age_s)
        covariance_scale = (
            quality_multiplier
            * hdop_multiplier
            * signal_multiplier
            * rtcm_multiplier
        )
        horizontal_variance = min(
            base_stddev * base_stddev * covariance_scale,
            self.unavailable_covariance_m2,
        )
        vertical_variance = min(
            horizontal_variance * self.vertical_stddev_multiplier ** 2,
            self.unavailable_covariance_m2,
        )
        return CovarianceResult(
            covariance_xx=horizontal_variance,
            covariance_yy=horizontal_variance,
            covariance_zz=vertical_variance,
            covariance_scale=covariance_scale,
            horizontal_stddev_m=math.sqrt(horizontal_variance),
            vertical_stddev_m=math.sqrt(vertical_variance),
            reason="%s,%s,%s" % (quality_reason, signal_reason, rtcm_reason),
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

    def _signal_multiplier(
        self, signal_health: Optional[SignalHealthSample]
    ) -> tuple[float, str]:
        if signal_health is None or not signal_health.fresh or not signal_health.valid:
            return 1.0, "signal_health_unavailable"
        cno = min(signal_health.cno_median_dbhz, signal_health.cno_p10_dbhz)
        if not math.isfinite(cno):
            return self.cno_critical_covariance_multiplier, "cno_invalid"
        if cno >= self.cno_good_dbhz:
            return 1.0, "cno_good"
        if cno >= self.cno_warning_dbhz:
            return self.cno_warning_covariance_multiplier, "cno_warning"
        if cno >= self.cno_critical_dbhz:
            return self.cno_critical_covariance_multiplier, "cno_low"
        return self.cno_critical_covariance_multiplier ** 2, "cno_critical"

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
        return self.rtcm_critical_covariance_multiplier ** 2, "rtcm_stale"

    def _unavailable(self, reason: str) -> CovarianceResult:
        stddev = math.sqrt(self.unavailable_covariance_m2)
        return CovarianceResult(
            covariance_xx=self.unavailable_covariance_m2,
            covariance_yy=self.unavailable_covariance_m2,
            covariance_zz=self.unavailable_covariance_m2,
            covariance_scale=self.unavailable_covariance_m2,
            horizontal_stddev_m=stddev,
            vertical_stddev_m=stddev,
            reason=reason,
        )
