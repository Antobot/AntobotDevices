from antobot_devices_gps.gps_signal_health_core import GsvSignalAccumulator


def test_aggregates_signal_strength_across_constellations():
    accumulator = GsvSignalAccumulator(sample_timeout_s=2.0, weak_cno_dbhz=25.0)
    accumulator.add(10.0, "GP", [("01", 42.0), ("02", 35.0)])
    accumulator.add(10.1, "GL", [("01", 20.0), ("02", 30.0)])

    snapshot = accumulator.snapshot(10.2)

    assert snapshot.satellites_in_view == 4
    assert snapshot.constellation_count == 2
    assert snapshot.weak_signal_count == 1
    assert snapshot.cno_mean_dbhz == 31.75
    assert snapshot.cno_median_dbhz == 32.5
    assert snapshot.cno_p10_dbhz == 23.0


def test_expires_old_gsv_samples():
    accumulator = GsvSignalAccumulator(sample_timeout_s=2.0)
    accumulator.add(10.0, "GP", [("01", 42.0)])

    assert accumulator.snapshot(12.1) is None
