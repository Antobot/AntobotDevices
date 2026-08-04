from antobot_devices_gps.gps_integrity_checker import (
    GpsFixSample,
    GpsIntegrityChecker,
    GpsQualitySample,
    RtcmSample,
)


def valid_fix(received_at=10.0, stamp_s=1_000.0):
    return GpsFixSample(
        received_at=received_at,
        stamp_s=stamp_s,
        latitude=52.0,
        longitude=-1.0,
        status=0,
    )


def valid_quality(received_at=10.0, stamp_s=1_000.1):
    return GpsQualitySample(
        received_at=received_at,
        stamp_s=stamp_s,
        gps_quality=4,
        horizontal_accuracy_m=0.03,
        satellites=14,
        hdop=0.7,
    )


def valid_rtcm(received_at=10.0):
    return RtcmSample(received_at=received_at)


def test_valid_f9p_samples_pass_integrity_check():
    result = GpsIntegrityChecker().evaluate(10.5, valid_fix(), valid_quality(), valid_rtcm())

    assert result.integrity_ok is True
    assert result.reason == "ok"
    assert result.timestamps_aligned is True


def test_missing_quality_is_reported_even_without_a_new_fix():
    result = GpsIntegrityChecker().evaluate(10.5, valid_fix(), None, valid_rtcm())

    assert result.integrity_ok is False
    assert result.quality_received is False
    assert result.reason == "quality_missing,timestamps_misaligned"


def test_stale_gpsfix_and_quality_are_reported():
    result = GpsIntegrityChecker().evaluate(12.0, valid_fix(), valid_quality(), valid_rtcm())

    assert result.integrity_ok is False
    assert result.gpsfix_fresh is False
    assert result.quality_fresh is False
    assert result.reason == "gpsfix_stale,quality_stale"


def test_unpaired_timestamps_fail_integrity_check():
    result = GpsIntegrityChecker(max_timestamp_delta_s=0.25).evaluate(
        10.5, valid_fix(), valid_quality(stamp_s=1_001.0), valid_rtcm()
    )

    assert result.integrity_ok is False
    assert result.timestamp_delta_s == 1.0
    assert result.reason == "timestamps_misaligned"


def test_invalid_fix_and_quality_payloads_are_reported():
    bad_fix = GpsFixSample(10.0, 1_000.0, 0.0, 0.0, -1)
    bad_quality = GpsQualitySample(10.0, 1_000.0, 0, 20.0, 2, 8.0)

    result = GpsIntegrityChecker().evaluate(10.2, bad_fix, bad_quality, valid_rtcm())

    assert result.integrity_ok is False
    assert result.fix_valid is False
    assert result.quality_valid is False
    assert result.reason == "gpsfix_invalid,quality_invalid"


def test_missing_or_stale_rtcm_fails_integrity_check():
    checker = GpsIntegrityChecker(rtcm_timeout_s=5.0)

    missing = checker.evaluate(10.2, valid_fix(), valid_quality(), None)
    stale = checker.evaluate(16.0, valid_fix(15.8), valid_quality(15.8), valid_rtcm(10.0))

    assert missing.integrity_ok is False
    assert missing.reason == "rtcm_missing"
    assert stale.integrity_ok is False
    assert stale.rtcm_age_s == 6.0
    assert stale.reason == "rtcm_stale"
