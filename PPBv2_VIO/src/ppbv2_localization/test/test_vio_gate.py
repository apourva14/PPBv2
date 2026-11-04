from ppbv2_localization.lib.vio_gate import ERROR, HEALTH_STATUS, OK, STALE, VioGate, WARN


def test_waits_for_a_health_report():
    gate = VioGate()
    assert not gate.decide(0.0).forward


def test_forwards_while_healthy_and_fresh():
    gate = VioGate(status_timeout_s=3.0)
    gate.on_status(HEALTH_STATUS, OK, 'plausible', 10.0)
    assert gate.decide(11.0).forward
    gate.on_status(HEALTH_STATUS, WARN, 'implausible motion', 12.0)
    assert gate.decide(12.5).forward                      # WARN is transient, still fused
    assert not gate.decide(16.0).forward                  # report stale
    gate.on_status(HEALTH_STATUS, OK, 'plausible', 16.5)
    assert gate.decide(16.6).forward                      # staleness recovers


def test_divergence_drops_vio_until_restart():
    gate = VioGate()
    gate.on_status(HEALTH_STATUS, OK, 'plausible', 1.0)
    gate.on_status(HEALTH_STATUS, ERROR, 'DIVERGED: vertical speed -1.0 m/s', 2.0)
    gate.on_status(HEALTH_STATUS, OK, 'plausible', 3.0)   # never trusted again
    d = gate.decide(3.1)
    assert not d.forward and 'diverged' in d.reason


def test_ignores_other_statuses_and_stale_level():
    gate = VioGate()
    gate.on_status('ppb_vio: odometry', ERROR, 'stale', 1.0)
    assert not gate.decide(1.1).forward and 'no VIO health' in gate.decide(1.1).reason
    gate.on_status(HEALTH_STATUS, STALE, 'waiting', 2.0)
    assert not gate.decide(2.1).forward
