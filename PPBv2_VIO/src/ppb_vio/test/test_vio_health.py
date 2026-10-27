import numpy as np

from ppb_vio.lib.vio_health import ERROR, HealthLimits, OK, VioHealthGate, WARN

RATE_HZ = 140.0
SMALL_COV = np.eye(6) * 0.01


def drive(gate, velocities, start_s=0.0, cov=SMALL_COV, start_pos=(0.0, 0.0, 0.0)):
    """Integrate per-tick world velocities into base_link positions and feed the gate."""
    p = np.array(start_pos, dtype=float)
    levels = []
    for k, v in enumerate(velocities):
        p = p + np.asarray(v, dtype=float) / RATE_HZ
        levels.append(gate.update(start_s + k / RATE_HZ, p, cov))
    return levels


def steady(speed, seconds, vz=0.0):
    return [np.array([speed, 0.0, vz])] * int(seconds * RATE_HZ)


def test_normal_driving_is_ok():
    assert set(drive(VioHealthGate(), steady(1.2, 5.0))[int(RATE_HZ):]) == {OK}


def test_short_overspeed_warns_then_recovers():
    gate = VioHealthGate(HealthLimits(max_accel_mps2=100.0))     # speed check only
    levels = drive(gate, steady(1.2, 2.0) + steady(5.0, 0.6) + steady(1.2, 3.0))
    assert WARN in levels and ERROR not in levels and levels[-1] == OK


def test_sustained_overspeed_latches_error():
    gate = VioHealthGate()
    levels = drive(gate, steady(1.2, 2.0) + steady(5.0, 3.0) + steady(1.2, 2.0))
    assert levels[-1] == ERROR and 'DIVERGED' in gate.state.message
    # window (0.5 s) to see it + persist (1.0 s) to latch, after the overspeed began at 2 s
    assert 2.0 < gate.state.latched_at_s < 2.0 + 0.5 + HealthLimits().persist_s + 0.2


def test_vertical_runaway_is_caught_below_the_speed_limit():
    gate = VioHealthGate()
    levels = drive(gate, steady(1.0, 1.0) + steady(1.0, 3.0, vz=-0.9))
    assert levels[-1] == ERROR and 'vertical' in gate.state.message


def test_mast_vibration_does_not_trip_the_vertical_check():
    """+-1.8 m/s vertical oscillation at 8 Hz (lever-arm effect) averages out over the window."""
    t = np.arange(0, 10, 1 / RATE_HZ)
    vel = [np.array([1.2, 0.0, 1.8 * np.sin(2 * np.pi * 8.0 * ti)]) for ti in t]
    assert ERROR not in drive(VioHealthGate(), vel)


def test_runaway_acceleration_latches_error():
    """A filter runaway: velocity growing steadily at 6 m/s^2 (speed check disabled)."""
    t = np.arange(0, 4.0, 1 / RATE_HZ)
    vel = steady(1.0, 1.0) + [np.array([1.0, 6.0 * ti, 0.0]) for ti in t]
    gate = VioHealthGate(HealthLimits(max_speed_mps=50.0))
    levels = drive(gate, vel)
    assert levels[-1] == ERROR and 'acceleration' in gate.state.message


def test_jumpy_but_bounded_motion_is_not_flagged():
    rng = np.random.default_rng(0)
    vel = [np.array([1.2 + 0.3 * rng.normal(), 0.2 * rng.normal(), 0.3 * rng.normal()])
           for _ in range(int(10 * RATE_HZ))]
    assert ERROR not in drive(VioHealthGate(), vel)


def test_non_finite_state_latches_error():
    gate = VioHealthGate()
    drive(gate, steady(1.0, 1.0))
    for k in range(int(2 * RATE_HZ)):
        gate.update(1.0 + k / RATE_HZ, np.array([np.nan, 0.0, 0.0]), SMALL_COV)
    assert gate.state.level == ERROR


def test_large_position_sigma_only_warns():
    gate = VioHealthGate()
    levels = drive(gate, steady(1.0, 2.0), cov=np.eye(6) * 3000.0)
    assert levels[-1] == WARN and 'sigma' in gate.state.message
