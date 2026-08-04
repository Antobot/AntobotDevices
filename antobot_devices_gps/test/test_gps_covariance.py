import math

from antobot_devices_gps.gps_covariance import GpsCovarianceModel, SignalHealthSample


def model():
    return GpsCovarianceModel(vertical_stddev_multiplier=2.0)


def test_fixed_solution_preserves_quality_derived_covariance():
    result = model().calculate(
        source_covariance=[0.0] * 9,
        horizontal_accuracy_m=0.1,
        hdop=0.8,
        gps_quality=4,
        integrity_ok=True,
        signal_health=None,
        rtcm_age_s=0.5,
    )

    assert math.isclose(result.covariance_xx, 0.01)
    assert math.isclose(result.covariance_yy, 0.01)
    assert math.isclose(result.covariance_zz, 0.04)
    assert result.reason == "rtk_fixed,signal_health_unavailable,rtcm_fresh"


def test_float_solution_inflates_covariance():
    result = model().calculate(
        source_covariance=[0.0] * 9,
        horizontal_accuracy_m=0.1,
        hdop=1.0,
        gps_quality=5,
        integrity_ok=True,
        signal_health=None,
        rtcm_age_s=0.5,
    )

    assert math.isclose(result.covariance_xx, 0.25)
    assert result.covariance_scale == 25.0
    assert result.reason == "rtk_float,signal_health_unavailable,rtcm_fresh"


def test_low_cn0_inflates_fixed_solution_covariance():
    low_signal = SignalHealthSample(
        fresh=True,
        valid=True,
        cno_median_dbhz=30.0,
        cno_p10_dbhz=27.0,
    )
    result = model().calculate(
        source_covariance=[0.0] * 9,
        horizontal_accuracy_m=0.1,
        hdop=1.0,
        gps_quality=4,
        integrity_ok=True,
        signal_health=low_signal,
        rtcm_age_s=0.5,
    )

    assert math.isclose(result.covariance_xx, 0.25)
    assert result.covariance_scale == 25.0
    assert result.reason == "rtk_fixed,cno_low,rtcm_fresh"


def test_invalid_integrity_forces_large_covariance():
    result = model().calculate(
        source_covariance=[0.01] * 9,
        horizontal_accuracy_m=0.1,
        hdop=1.0,
        gps_quality=4,
        integrity_ok=False,
        signal_health=None,
        rtcm_age_s=None,
    )

    assert result.covariance_xx == 1_000_000.0
    assert result.covariance_yy == 1_000_000.0
    assert result.covariance_zz == 1_000_000.0
    assert result.reason == "integrity_invalid"


def test_required_signal_health_must_be_fresh_and_valid():
    strict_model = GpsCovarianceModel(require_signal_health=True)
    result = strict_model.calculate(
        source_covariance=[0.01] * 9,
        horizontal_accuracy_m=0.1,
        hdop=1.0,
        gps_quality=4,
        integrity_ok=True,
        signal_health=None,
        rtcm_age_s=0.5,
    )

    assert result.covariance_xx == 1_000_000.0
    assert result.reason == "signal_health_missing"


def test_aging_rtcm_inflates_covariance():
    result = model().calculate(
        source_covariance=[0.0] * 9,
        horizontal_accuracy_m=0.1,
        hdop=1.0,
        gps_quality=4,
        integrity_ok=True,
        signal_health=None,
        rtcm_age_s=1.5,
    )

    assert math.isclose(result.covariance_xx, 0.04)
    assert result.covariance_scale == 4.0
    assert result.reason == "rtk_fixed,signal_health_unavailable,rtcm_warning"


def test_missing_required_rtcm_forces_large_covariance():
    result = model().calculate(
        source_covariance=[0.0] * 9,
        horizontal_accuracy_m=0.1,
        hdop=1.0,
        gps_quality=4,
        integrity_ok=True,
        signal_health=None,
        rtcm_age_s=None,
    )

    assert result.covariance_xx == 1_000_000.0
    assert result.reason == "rtcm_missing"
