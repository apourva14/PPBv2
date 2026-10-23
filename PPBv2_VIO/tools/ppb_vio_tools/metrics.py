"""Accuracy, availability, and input-health metrics for a VIO run (numpy only)."""

from dataclasses import asdict, dataclass

import numpy as np


@dataclass
class Availability:
    init_latency_s: float       # first /vio/odom stamp - first sensor stamp
    coverage: float             # fraction of the evaluated window covered by output
    longest_gap_s: float        # largest gap between consecutive outputs (window incl. edges)
    output_rate_hz: float
    window_start_s: float
    window_end_s: float

    def as_dict(self):
        return asdict(self)


def availability(odom_stamps_s, sensor_start_s, sensor_end_s, max_gap_s=0.5):
    """Measure how continuously the estimator produced output.

    The window runs from the first output to the end of sensor data, so a
    filter that stops publishing before the bag ends is penalized, and a gap
    longer than `max_gap_s` counts as lost tracking for its whole length.
    """
    t = np.sort(np.asarray(odom_stamps_s, dtype=float))
    if t.size == 0:
        return Availability(float('inf'), 0.0, float('inf'), 0.0, float('nan'), sensor_end_s)
    if sensor_end_s <= t[0]:
        raise ValueError('sensor_end_s must be after the first output')

    window = sensor_end_s - t[0]
    edges = np.append(t, max(sensor_end_s, t[-1]))
    gaps = np.diff(edges)
    lost = float(np.sum(gaps[gaps > max_gap_s]))
    return Availability(
        init_latency_s=float(t[0] - sensor_start_s),
        coverage=float(max(0.0, 1.0 - lost / window)),
        longest_gap_s=float(gaps.max()) if gaps.size else 0.0,
        output_rate_hz=float((t.size - 1) / (t[-1] - t[0])) if t.size > 1 else 0.0,
        window_start_s=float(t[0]),
        window_end_s=float(sensor_end_s),
    )


def check_acceptance(result, limits):
    """Return a list of human-readable failures (empty means pass)."""
    failures = []
    if result['ape_rmse_m'] > limits['max_ape_rmse_m']:
        failures.append(f"APE RMSE {result['ape_rmse_m']:.3f} m > {limits['max_ape_rmse_m']} m")
    if result['availability']['init_latency_s'] > limits['max_init_latency_s']:
        failures.append(f"init latency {result['availability']['init_latency_s']:.1f} s > "
                        f"{limits['max_init_latency_s']} s")
    if result['availability']['coverage'] < limits['min_coverage']:
        failures.append(f"coverage {result['availability']['coverage']:.3f} < "
                        f"{limits['min_coverage']}")
    if result['matched_poses'] < limits['min_matched_poses']:
        failures.append(f"only {result['matched_poses']} poses matched ground truth "
                        f"(< {limits['min_matched_poses']})")
    return failures


def align_position_yaw(ref_xyz, est_xyz):
    """Least-squares yaw rotation + translation mapping `est` onto `ref`.

    VIO observes roll and pitch through gravity; only position and yaw are
    unobservable. Aligning just those 4 DoF (instead of a full SE(3)) keeps
    roll/pitch errors visible in the APE. Both frames must have z along gravity.
    Returns (R, t) with ref ~= R @ est + t.
    """
    ref = np.asarray(ref_xyz, dtype=float)
    est = np.asarray(est_xyz, dtype=float)
    if ref.ndim != 2 or ref.shape[1] != 3 or ref.shape != est.shape or len(ref) < 2:
        raise ValueError('need two equally long (N, 3) position arrays, N >= 2')
    mu_ref, mu_est = ref.mean(axis=0), est.mean(axis=0)
    r, e = ref - mu_ref, est - mu_est
    # maximize sum r_i . Rz(yaw) e_i  ->  yaw = atan2(sum r x e (z), sum r . e (xy))
    yaw = np.arctan2(np.sum(r[:, 1] * e[:, 0] - r[:, 0] * e[:, 1]),
                     np.sum(r[:, 0] * e[:, 0] + r[:, 1] * e[:, 1]))
    c, s = np.cos(yaw), np.sin(yaw)
    rot = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return rot, mu_ref - rot @ mu_est


def position_errors_after_yaw_alignment(ref_xyz, est_xyz):
    rot, t = align_position_yaw(ref_xyz, est_xyz)
    aligned = np.asarray(est_xyz, dtype=float) @ rot.T + t
    return np.linalg.norm(aligned - np.asarray(ref_xyz, dtype=float), axis=1)


# Status names published by vio_stream_monitor for the sensor inputs.
INPUT_STATUS_KEYWORDS = ('stream', 'stereo sync', 'imu sanity')
# These must be reported throughout the window, or the input health is unknown.
REQUIRED_INPUT_STATUSES = ('ppb_vio: imu stream', 'ppb_vio: camera 0 stream')


def input_health(samples, window_start_s, window_end_s, max_degraded_fraction=0.02,
                 status_period_s=1.0, min_report_fraction=0.8):
    """Summarize recorded /vio/status input diagnostics.

    `samples` is an iterable of (stamp_s, name, level). Only samples inside the
    window count (the monitor legitimately reports STALE before the replay
    starts and after it ends). Inputs count as degraded when any input status
    is WARN or worse for more than `max_degraded_fraction` of its reports, or
    when a required status is missing or reported for less than
    `min_report_fraction` of the window (a monitor that stopped proves nothing).
    Returns (per-name degraded fractions, degraded?).
    """
    counts = {}
    for stamp, name, level in samples:
        if not window_start_s <= stamp <= window_end_s:
            continue
        if not any(k in name for k in INPUT_STATUS_KEYWORDS):
            continue
        total, bad = counts.get(name, (0, 0))
        counts[name] = (total + 1, bad + (1 if level >= 1 else 0))
    fractions = {name: bad / total for name, (total, bad) in counts.items()}
    expected_reports = (window_end_s - window_start_s) / status_period_s
    under_reported = any(counts.get(name, (0, 0))[0] < min_report_fraction * expected_reports
                         for name in REQUIRED_INPUT_STATUSES)
    degraded = under_reported or any(f > max_degraded_fraction for f in fractions.values())
    return fractions, degraded
