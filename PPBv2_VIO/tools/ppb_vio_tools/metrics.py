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


def relative_translation_errors(ref_xyz, est_aligned_xyz, delta_m):
    """Translation error of consecutive, non-overlapping segments of `delta_m`
    travelled distance (measured on the reference path).

    Uses positions only, so it works with position-only ground truth (Rosario
    v1). The estimate must already be aligned to the reference: a segment's
    error is then ||(e_j - e_i) - (r_j - r_i)||, i.e. it includes the heading
    error accumulated up to that segment, the quantity a row-following robot
    cares about. Returns an array with one error per segment (may be empty).
    """
    ref = np.asarray(ref_xyz, dtype=float)
    est = np.asarray(est_aligned_xyz, dtype=float)
    if ref.shape != est.shape or ref.ndim != 2 or ref.shape[1] != 3:
        raise ValueError('need two equally long (N, 3) position arrays')
    if delta_m <= 0:
        raise ValueError('delta_m must be positive')
    dist = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(ref, axis=0), axis=1))])
    errors, i = [], 0
    while True:
        j = int(np.searchsorted(dist, dist[i] + delta_m, side='left'))
        if j >= len(ref):
            break
        errors.append(np.linalg.norm((est[j] - est[i]) - (ref[j] - ref[i])))
        i = j
    return np.array(errors)


# 95 % quantile of the chi-square distribution with 3 degrees of freedom.
CHI2_3DOF_95 = 7.814727903251178


def position_nees(errors_xyz, covariances):
    """Normalized estimation error squared of 3-D positions.

    STUB for Task 1: absolute NEES after trajectory alignment is only
    indicative (the alignment absorbs part of the error, and the covariance of
    unobservable position/yaw grows without bound). It is reported, never
    gated, until the covariance convention is verified per frontend (Task 2).

    `errors_xyz` (N, 3) and `covariances` (N, 3, 3) must be in the same frame.
    Samples whose covariance is not positive definite are skipped and counted.
    Returns a dict summary (mean is 3 for a consistent estimator).
    """
    err = np.asarray(errors_xyz, dtype=float).reshape(-1, 3)
    cov = np.asarray(covariances, dtype=float).reshape(-1, 3, 3)
    values, skipped = [], 0
    for e, c in zip(err, cov):
        try:
            chol = np.linalg.cholesky(0.5 * (c + c.T))
        except np.linalg.LinAlgError:
            skipped += 1
            continue
        z = np.linalg.solve(chol, e)
        values.append(float(z @ z))
    values = np.array(values)
    if values.size == 0:
        return {'status': 'unavailable', 'reason': 'no positive-definite covariance',
                'samples': 0, 'skipped': skipped}
    return {
        'status': 'stub (reported, not gated)',
        'dof': 3,
        'samples': int(values.size),
        'skipped': skipped,
        'mean': float(values.mean()),
        'median': float(np.median(values)),
        'fraction_above_chi2_95': float(np.mean(values > CHI2_3DOF_95)),
        # Covariance multiplier that would make the mean NEES 3 (fusion: Requirement 5).
        'suggested_covariance_scale': float(values.mean() / 3.0),
    }


def divergence_onset(stamps_s, ref_xyz, est_xyz, window_s=1.0, tol_mps=1.0, tol_vertical_mps=0.3,
                     persist_s=2.0):
    """First stamp at which the estimate's motion departs from the ground truth's.

    Velocities are displacement over `window_s` of the same tracked point. Only
    quantities that a position+yaw alignment cannot change are compared, so no
    alignment is involved and a runaway is flagged from the data, independently
    of the health gate being tested:
      horizontal speed   |v_xy,est| vs |v_xy,gt|   (diverges if off by > tol_mps)
      vertical velocity  v_z,est vs v_z,gt          (> tol_vertical_mps; both frames
                                                      are gravity-aligned)
    Divergence = either condition holding for >= persist_s. Returns the onset or None.
    """
    t = np.asarray(stamps_s, dtype=float)
    ref, est = np.asarray(ref_xyz, dtype=float), np.asarray(est_xyz, dtype=float)
    if t.size < 3:
        return None
    j = np.searchsorted(t, t + window_s)
    ok = j < t.size
    i = np.flatnonzero(ok)
    j = j[ok]
    dt = (t[j] - t[i])[:, None]
    v_est, v_ref = (est[j] - est[i]) / dt, (ref[j] - ref[i]) / dt
    bad = ((np.abs(np.linalg.norm(v_est[:, :2], axis=1) - np.linalg.norm(v_ref[:, :2], axis=1))
            > tol_mps) | (np.abs(v_est[:, 2] - v_ref[:, 2]) > tol_vertical_mps))
    start = None
    for k, b in zip(i, bad):
        if not b:
            start = None
        elif start is None:
            start = t[k]
        elif t[k] - start >= persist_s:
            return float(start)
    return None
