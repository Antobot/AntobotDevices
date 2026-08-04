"""Pure validation logic for the F9P GPS integrity monitor."""

from dataclasses import dataclass
import math
from typing import Optional, Tuple


@dataclass(frozen=True)
class GpsFixSample:
    """The NavSatFix fields used by the integrity check."""

    received_at: float
    stamp_s: Optional[float]
    latitude: float
    longitude: float
    status: int


@dataclass(frozen=True)
class GpsQualitySample:
    """The GpsQual fields used by the integrity check."""

    received_at: float
    stamp_s: Optional[float]
    gps_quality: int
    horizontal_accuracy_m: float
    satellites: int
    hdop: float


@dataclass(frozen=True)
class RtcmSample:
    """A received RTCM correction message at the rover."""

    received_at: float


@dataclass(frozen=True)
class GpsIntegrityResult:
    """Result published by the ROS node after checking both inputs."""

    gpsfix_received: bool
    quality_received: bool
    rtcm_received: bool
    gpsfix_fresh: bool
    quality_fresh: bool
    rtcm_fresh: bool
    fix_valid: bool
    quality_valid: bool
    timestamps_aligned: bool
    integrity_ok: bool
    gpsfix_age_s: float
    quality_age_s: float
    rtcm_age_s: float
    timestamp_delta_s: float
    navsat_status: int
    gps_quality: int
    reason: str


class GpsIntegrityChecker:
    """Validate F9P fix and quality topic completeness and consistency."""

    def __init__(
        self,
        gpsfix_timeout_s: float = 1.0,
        quality_timeout_s: float = 1.0,
        max_timestamp_delta_s: float = 0.25,
        min_navsat_status: int = 0,
        min_gps_quality: int = 1,
        min_num_sats: int = 4,
        max_h_acc_m: float = 10.0,
        max_hdop: float = 5.0,
        require_timestamp_alignment: bool = True,
        rtcm_timeout_s: float = 5.0,
        require_rtcm: bool = True,
    ) -> None:
        if gpsfix_timeout_s <= 0.0 or quality_timeout_s <= 0.0 or rtcm_timeout_s <= 0.0:
            raise ValueError("input timeouts must be positive")
        if max_timestamp_delta_s < 0.0:
            raise ValueError("max_timestamp_delta_s must not be negative")
        if min_num_sats < 0:
            raise ValueError("min_num_sats must not be negative")
        if max_h_acc_m <= 0.0 or max_hdop <= 0.0:
            raise ValueError("quality thresholds must be positive")

        self.gpsfix_timeout_s = gpsfix_timeout_s
        self.quality_timeout_s = quality_timeout_s
        self.max_timestamp_delta_s = max_timestamp_delta_s
        self.min_navsat_status = min_navsat_status
        self.min_gps_quality = min_gps_quality
        self.min_num_sats = min_num_sats
        self.max_h_acc_m = max_h_acc_m
        self.max_hdop = max_hdop
        self.require_timestamp_alignment = require_timestamp_alignment
        self.rtcm_timeout_s = rtcm_timeout_s
        self.require_rtcm = require_rtcm

    def evaluate(
        self,
        now: float,
        gpsfix: Optional[GpsFixSample],
        quality: Optional[GpsQualitySample],
        rtcm: Optional[RtcmSample],
    ) -> GpsIntegrityResult:
        """Evaluate the latest samples using a monotonic receipt-time clock."""
        gpsfix_age_s = self._age(now, gpsfix.received_at) if gpsfix else -1.0
        quality_age_s = self._age(now, quality.received_at) if quality else -1.0
        rtcm_age_s = self._age(now, rtcm.received_at) if rtcm else -1.0
        gpsfix_received = gpsfix is not None
        quality_received = quality is not None
        rtcm_received = rtcm is not None
        gpsfix_fresh = gpsfix_received and gpsfix_age_s <= self.gpsfix_timeout_s
        quality_fresh = quality_received and quality_age_s <= self.quality_timeout_s
        rtcm_fresh = rtcm_received and rtcm_age_s <= self.rtcm_timeout_s

        fix_valid = gpsfix_received and self._fix_valid(gpsfix)
        quality_valid = quality_received and self._quality_valid(quality)
        timestamps_aligned, timestamp_delta_s = self._timestamps_aligned(gpsfix, quality)

        reasons = []
        if not gpsfix_received:
            reasons.append("gpsfix_missing")
        elif not gpsfix_fresh:
            reasons.append("gpsfix_stale")
        elif not fix_valid:
            reasons.append("gpsfix_invalid")

        if not quality_received:
            reasons.append("quality_missing")
        elif not quality_fresh:
            reasons.append("quality_stale")
        elif not quality_valid:
            reasons.append("quality_invalid")

        if self.require_rtcm:
            if not rtcm_received:
                reasons.append("rtcm_missing")
            elif not rtcm_fresh:
                reasons.append("rtcm_stale")

        if self.require_timestamp_alignment and not timestamps_aligned:
            reasons.append("timestamps_misaligned")

        integrity_ok = not reasons
        return GpsIntegrityResult(
            gpsfix_received=gpsfix_received,
            quality_received=quality_received,
            rtcm_received=rtcm_received,
            gpsfix_fresh=gpsfix_fresh,
            quality_fresh=quality_fresh,
            rtcm_fresh=rtcm_fresh,
            fix_valid=fix_valid,
            quality_valid=quality_valid,
            timestamps_aligned=timestamps_aligned,
            integrity_ok=integrity_ok,
            gpsfix_age_s=gpsfix_age_s,
            quality_age_s=quality_age_s,
            rtcm_age_s=rtcm_age_s,
            timestamp_delta_s=timestamp_delta_s,
            navsat_status=gpsfix.status if gpsfix else -1,
            gps_quality=quality.gps_quality if quality else 0,
            reason="ok" if integrity_ok else ",".join(reasons),
        )

    @staticmethod
    def _age(now: float, received_at: float) -> float:
        return max(0.0, now - received_at)

    def _fix_valid(self, gpsfix: GpsFixSample) -> bool:
        if gpsfix.status < self.min_navsat_status:
            return False
        if not math.isfinite(gpsfix.latitude) or not math.isfinite(gpsfix.longitude):
            return False
        if not -90.0 <= gpsfix.latitude <= 90.0:
            return False
        if not -180.0 <= gpsfix.longitude <= 180.0:
            return False
        return not (abs(gpsfix.latitude) < 1e-9 and abs(gpsfix.longitude) < 1e-9)

    def _quality_valid(self, quality: GpsQualitySample) -> bool:
        if quality.gps_quality < self.min_gps_quality:
            return False
        if quality.satellites < self.min_num_sats:
            return False
        if not math.isfinite(quality.horizontal_accuracy_m):
            return False
        if not math.isfinite(quality.hdop):
            return False
        if quality.horizontal_accuracy_m < 0.0 or quality.horizontal_accuracy_m > self.max_h_acc_m:
            return False
        return 0.0 <= quality.hdop <= self.max_hdop

    def _timestamps_aligned(
        self,
        gpsfix: Optional[GpsFixSample],
        quality: Optional[GpsQualitySample],
    ) -> Tuple[bool, float]:
        if gpsfix is None or quality is None:
            return False, -1.0
        if not self._timestamp_present(gpsfix.stamp_s):
            return False, -1.0
        if not self._timestamp_present(quality.stamp_s):
            return False, -1.0

        delta_s = abs(gpsfix.stamp_s - quality.stamp_s)
        return delta_s <= self.max_timestamp_delta_s, delta_s

    @staticmethod
    def _timestamp_present(stamp_s: Optional[float]) -> bool:
        return stamp_s is not None and math.isfinite(stamp_s) and stamp_s > 0.0
