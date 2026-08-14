import math

from antobot_devices_gps.gps_covariance import (
    CovariancePolicy,
    GpsCovarianceModel,
    PersistentSignalFaultTracker,
    SignalHealthSample,
)


def model():
    return GpsCovarianceModel(CovariancePolicy(vertical_covariance_m2=100.0))


def healthy_signal(satellites=15):
    return SignalHealthSample(True, True, 42.0, 36.0, satellites)


def calculate(**overrides):
    values = {
        "source_covariance": [0.0] * 9,
        "horizontal_accuracy_m": 0.02,
        "hdop": 1.2,
        "gps_quality": 4,
        "fixed_duration_s": 8.0,
        "integrity_ok": True,
        "signal_health": healthy_signal(),
        "rtcm_age_s": 0.5,
    }
    values.update(overrides)
    return model().calculate(**values)


def test_stable_fixed_uses_receiver_horizontal_accuracy_without_inflation():
    result = calculate()

    assert result.covariance_scale == 1.0
    assert math.isclose(result.covariance_xx, 0.0004)
    assert math.isclose(result.covariance_yy, 0.0004)
    assert result.covariance_zz == 100.0
    assert result.reason == (
        "rtk_fixed,fixed_duration_ready,hdop_good,cno_good,rtcm_fresh"
    )


def test_newly_fixed_solution_uses_short_fixed_multiplier():
    result = calculate(horizontal_accuracy_m=0.03, hdop=1.5, fixed_duration_s=2.0)

    assert result.covariance_scale == 9.0
    assert math.isclose(result.covariance_xx, 0.0081)
    assert math.isclose(result.covariance_yy, 0.0081)


def test_fixed_warmup_uses_worst_auxiliary_weight_once():
    result = calculate(
        horizontal_accuracy_m=0.03,
        hdop=2.5,
        fixed_duration_s=4.0,
        signal_health=SignalHealthSample(True, True, 37.0, 31.0, 15),
        rtcm_age_s=3.0,
    )

    # W_mode=4, W_aux=max(HDOP=2, C/N0=2, RTCM=2), W=8.
    assert result.covariance_scale == 8.0
    assert math.isclose(result.covariance_xx, 0.0072)
    assert math.isclose(result.covariance_yy, 0.0072)


def test_float_with_aging_rtcm_uses_relaxed_critical_multiplier():
    result = calculate(
        horizontal_accuracy_m=0.10,
        hdop=3.2,
        gps_quality=5,
        fixed_duration_s=None,
        signal_health=SignalHealthSample(True, True, 32.0, 27.0, 15),
        rtcm_age_s=6.0,
    )

    # W is 25 * max(HDOP=4, C/N0=4, RTCM=9) = 225.
    assert result.covariance_scale == 225.0
    assert math.isclose(result.covariance_xx, 2.25)
    assert math.isclose(result.covariance_yy, 2.25)


def test_hdop_above_limit_is_hard_rejected():
    result = calculate(hdop=5.1)

    assert result.covariance_xx == 1_000_000.0
    assert result.covariance_yy == 1_000_000.0
    assert result.covariance_zz == 1_000_000.0
    assert result.reason == "hdop_excessive"


def test_rtcm_receive_age_above_limit_is_hard_rejected():
    result = calculate(rtcm_age_s=10.1)

    assert result.covariance_xx == 1_000_000.0
    assert result.covariance_yy == 1_000_000.0
    assert result.covariance_zz == 1_000_000.0
    assert result.reason == "rtcm_stale"


def test_missing_cn0_during_startup_does_not_penalize_or_reject_fixed_solution():
    result = calculate(
        horizontal_accuracy_m=0.03,
        hdop=1.5,
        fixed_duration_s=6.0,
        signal_health=SignalHealthSample(
            fresh=False,
            valid=False,
            cno_median_dbhz=float("nan"),
            cno_p10_dbhz=float("nan"),
            unavailable_duration_s=0.5,
        ),
    )

    assert result.covariance_scale == 1.0
    assert math.isclose(result.covariance_xx, 0.0009)
    assert "signal_health_unavailable_grace" in result.reason


def test_missing_cn0_without_an_elapsed_duration_is_treated_as_startup():
    result = calculate(
        horizontal_accuracy_m=0.03,
        hdop=1.5,
        fixed_duration_s=6.0,
        signal_health=None,
    )

    assert result.covariance_scale == 1.0
    assert math.isclose(result.covariance_xx, 0.0009)
    assert "signal_health_starting" in result.reason


def test_short_cn0_outage_is_within_grace_period():
    result = calculate(
        horizontal_accuracy_m=0.03,
        signal_health=SignalHealthSample(
            fresh=False,
            valid=True,
            cno_median_dbhz=42.0,
            cno_p10_dbhz=36.0,
            unavailable_duration_s=1.9,
        ),
    )

    assert result.covariance_scale == 1.0
    assert math.isclose(result.covariance_xx, 0.0009)
    assert "signal_health_unavailable_grace" in result.reason


def test_cn0_outage_after_grace_period_is_soft_downgrade():
    result = calculate(
        horizontal_accuracy_m=0.03,
        signal_health=SignalHealthSample(
            fresh=False,
            valid=True,
            cno_median_dbhz=42.0,
            cno_p10_dbhz=36.0,
            unavailable_duration_s=3.0,
        ),
    )

    assert result.covariance_scale == 2.0
    assert math.isclose(result.covariance_xx, 0.0018)
    assert "signal_health_unavailable_warning" in result.reason


def test_long_cn0_outage_is_stronger_soft_downgrade():
    result = calculate(
        horizontal_accuracy_m=0.03,
        signal_health=SignalHealthSample(
            fresh=True,
            valid=False,
            cno_median_dbhz=float("nan"),
            cno_p10_dbhz=float("nan"),
            unavailable_duration_s=10.0,
        ),
    )

    assert result.covariance_scale == 4.0
    assert math.isclose(result.covariance_xx, 0.0036)
    assert "signal_health_unavailable_critical" in result.reason


def test_persistent_severe_cn0_with_satellite_drop_is_hard_rejected():
    tracker = PersistentSignalFaultTracker(
        severe_cno_dbhz=30.0,
        min_consecutive_epochs=3,
        min_satellite_drop=3,
    )
    tracker.update(healthy_signal(satellites=15))

    for _ in range(2):
        sample = tracker.update(SignalHealthSample(True, True, 29.0, 24.0, 12))
        assert sample.severe_persistent_fault is False

    sample = tracker.update(SignalHealthSample(True, True, 29.0, 24.0, 12))
    assert sample.severe_persistent_fault is True

    result = calculate(signal_health=sample)
    assert result.covariance_xx == 1_000_000.0
    assert result.reason == "cno_severe_persistent_fault"


def test_low_cn0_without_persistent_fault_is_soft_downgrade_only():
    result = calculate(
        horizontal_accuracy_m=0.03,
        signal_health=SignalHealthSample(True, True, 29.0, 24.0, 12),
    )

    assert result.covariance_scale == 9.0
    assert math.isclose(result.covariance_xx, 0.0081)
    assert result.reason.endswith("cno_severe,rtcm_fresh")


def test_invalid_integrity_remains_hard_rejected():
    result = calculate(integrity_ok=False)

    assert result.covariance_xx == 1_000_000.0
    assert result.reason == "integrity_invalid"


def test_missing_required_rtcm_remains_hard_rejected():
    result = calculate(rtcm_age_s=None)

    assert result.covariance_xx == 1_000_000.0
    assert result.reason == "rtcm_missing"
