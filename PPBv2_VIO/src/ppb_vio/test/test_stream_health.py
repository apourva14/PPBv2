from ppb_vio.lib.stream_health import ERROR, ImuSanity, OK, STALE, StereoSync, StreamHealth, WARN
from ppb_vio.lib.time_utils import NS_PER_S, ns_to_stamp, stamp_to_ns

import pytest

T0 = 1_403_715_273 * NS_PER_S


def feed(health, hz, seconds, start=T0, skip=()):
    period = int(NS_PER_S / hz)
    t = start
    for k in range(int(hz * seconds)):
        if k not in skip:
            health.add(t, t)
        t += period
    return t - period


def test_time_round_trip_is_exact():
    ns = 1_403_715_273_262_142_976
    assert stamp_to_ns(*ns_to_stamp(ns)) == ns
    with pytest.raises(ValueError):
        stamp_to_ns(0, NS_PER_S)


def test_nominal_stream_is_ok():
    h = StreamHealth(20.0)
    last = feed(h, 20.0, 3.0)
    report = h.evaluate(last)
    assert report.level == OK
    assert abs(report.values['rate_hz'] - 20.0) < 0.1


def test_no_messages_is_stale():
    assert StreamHealth(20.0).evaluate(T0).level == STALE


def test_silent_stream_becomes_error():
    h = StreamHealth(20.0, stale_s=1.0)
    last = feed(h, 20.0, 1.0)
    assert h.evaluate(last + 2 * NS_PER_S).level == ERROR


def test_low_rate_warns():
    h = StreamHealth(20.0)
    last = feed(h, 10.0, 3.0)
    assert h.evaluate(last).level == WARN


def test_dropped_frames_produce_gap_warning():
    h = StreamHealth(20.0, max_gap_s=0.12)
    last = feed(h, 20.0, 2.0, skip={20, 21, 22})
    report = h.evaluate(last)
    assert report.level == WARN and 'gap' in report.message


def test_non_increasing_stamps_are_errors_and_age_out():
    h = StreamHealth(200.0, window_s=1.0)
    last = feed(h, 200.0, 1.0)
    h.add(last, last + 1)              # duplicate stamp
    assert h.evaluate(last + 1).level == ERROR
    assert h.total_order_faults == 1
    later = feed(h, 200.0, 2.0, start=last + 5_000_000)
    assert h.evaluate(later).level == OK


def test_stereo_sync_detects_offset():
    sync = StereoSync(max_skew_s=0.001)
    for k in range(50):
        t = T0 + k * 50_000_000
        sync.add(0, t)
        sync.add(1, t)
    assert sync.evaluate().level == OK

    skewed = StereoSync(max_skew_s=0.001)
    for k in range(50):
        t = T0 + k * 50_000_000
        skewed.add(0, t)
        skewed.add(1, t + 5_000_000)
    assert skewed.evaluate().level == WARN


def test_imu_sanity_accepts_si_units():
    imu = ImuSanity()
    for _ in range(100):
        imu.add((0.1, -0.2, 9.79), (0.01, 0.0, -0.02))
    assert imu.evaluate().level == OK


def test_imu_sanity_flags_accel_in_g():
    imu = ImuSanity()
    for _ in range(100):
        imu.add((0.0, 0.0, 1.0), (0.0, 0.0, 0.0))
    assert imu.evaluate().level == ERROR


def test_imu_sanity_flags_gyro_in_deg_per_s():
    imu = ImuSanity()
    for _ in range(100):
        imu.add((0.0, 0.0, 9.81), (0.0, 0.0, 45.0))
    assert imu.evaluate().level == WARN


def test_imu_sanity_flags_nan():
    imu = ImuSanity()
    imu.add((float('nan'), 0.0, 9.81), (0.0, 0.0, 0.0))
    assert imu.evaluate().level == ERROR
