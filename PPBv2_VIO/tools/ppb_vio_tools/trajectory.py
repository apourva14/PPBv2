"""Estimated trajectories for scoring: association with ground truth, and the
ground-truth reference point (numpy only).

Conventions (same as ppb_vio): quaternions are Hamilton (x, y, z, w); R_WB maps
base_link vectors into the estimator world frame. Pose covariances follow the
/vio/odom contract: [position error in W, orientation error in base_link], i.e.
p_true = p_est + dp and R_true = R_est Exp(dtheta).
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class Trajectory:
    stamps: np.ndarray          # (N,) seconds, strictly increasing
    positions: np.ndarray       # (N, 3)
    quats_xyzw: np.ndarray      # (N, 4)
    covariances: object = None  # (N, 6, 6) or None when the source has none

    def __post_init__(self):
        self.stamps = np.asarray(self.stamps, dtype=float)
        self.positions = np.asarray(self.positions, dtype=float).reshape(-1, 3)
        self.quats_xyzw = np.asarray(self.quats_xyzw, dtype=float).reshape(-1, 4)
        n = self.stamps.size
        if self.positions.shape[0] != n or self.quats_xyzw.shape[0] != n:
            raise ValueError('stamps, positions and quaternions differ in length')
        if n > 1 and np.any(np.diff(self.stamps) <= 0):
            raise ValueError('trajectory stamps are not strictly increasing')
        if self.covariances is not None:
            self.covariances = np.asarray(self.covariances, dtype=float).reshape(n, 6, 6)

    def __len__(self):
        return self.stamps.size


def read_tum(path):
    """Read a TUM file (t x y z qx qy qz qw); comment lines start with '#'."""
    data = np.loadtxt(path, comments='#', ndmin=2)
    if data.shape[1] != 8:
        raise ValueError(f'{path}: expected 8 columns, got {data.shape[1]}')
    return Trajectory(data[:, 0], data[:, 1:4], data[:, 4:8])


def write_tum(path, traj):
    rows = np.column_stack([traj.stamps, traj.positions, traj.quats_xyzw])
    np.savetxt(path, rows, fmt='%.9f')


def quat_to_rot_batch(q_xyzw):
    x, y, z, w = np.moveaxis(np.asarray(q_xyzw, dtype=float), -1, 0)
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], -1),
        np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], -1),
        np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], -1),
    ], -2)


def slerp(q0, q1, f):
    """Row-wise spherical interpolation between unit quaternions (x, y, z, w)."""
    q0, q1 = np.atleast_2d(q0), np.atleast_2d(q1).copy()
    f = np.asarray(f, dtype=float).reshape(-1, 1)
    dot = np.sum(q0 * q1, axis=1, keepdims=True)
    q1[dot[:, 0] < 0] *= -1.0                       # shortest path
    dot = np.abs(dot).clip(max=1.0)
    theta = np.arccos(dot)
    small = theta[:, 0] < 1e-8
    s = np.sin(theta)
    s[small] = 1.0
    out = (np.sin((1 - f) * theta) * q0 + np.sin(f * theta) * q1) / s
    out[small] = ((1 - f) * q0 + f * q1)[small]
    return out / np.linalg.norm(out, axis=1, keepdims=True)


def associate(ref_stamps, est, max_bracket_s):
    """Interpolate `est` at the ground-truth stamps it brackets.

    A reference stamp is used only when estimates exist on both sides of it no
    more than `max_bracket_s` apart, so tracking gaps are never bridged by
    interpolation. Returns (ref indices, interpolated Trajectory). Covariance is
    taken from the nearer of the two estimates (it changes slowly).
    """
    ref_stamps = np.asarray(ref_stamps, dtype=float)
    n = len(est)
    # lo: last estimate at or before the reference stamp; hi: the one after it.
    lo = np.searchsorted(est.stamps, ref_stamps, side='right') - 1
    hi = lo + 1
    valid_lo = lo >= 0
    exact = valid_lo & (est.stamps[lo.clip(0, max(n - 1, 0))] == ref_stamps) if n else valid_lo
    bracketed = valid_lo & (hi < n)
    span = np.zeros_like(ref_stamps)
    span[bracketed] = est.stamps[hi[bracketed]] - est.stamps[lo[bracketed]]
    idx = np.flatnonzero(exact | (bracketed & (span <= max_bracket_s)))
    lo, span = lo[idx], span[idx]
    hi = np.where(exact[idx], lo, hi[idx])
    f = np.where(exact[idx], 0.0, (ref_stamps[idx] - est.stamps[lo]) / np.where(span > 0, span, 1))
    pos = est.positions[lo] + f[:, None] * (est.positions[hi] - est.positions[lo])
    quat = slerp(est.quats_xyzw[lo], est.quats_xyzw[hi], f)
    cov = None
    if est.covariances is not None:
        cov = np.where((f < 0.5)[:, None, None], est.covariances[lo], est.covariances[hi])
    return idx, Trajectory(ref_stamps[idx], pos, quat, cov)


def reference_point(traj, point_in_base):
    """Positions and 3x3 covariances of a point fixed on base_link.

    Ground truth often tracks a point other than the base_link origin (Rosario
    v1: the GPS antenna, 1.8 m forward and 1.6 m up). With p = p_B + R_WB l:
        dp = dp_B - R_WB [l]x dtheta_B
    """
    lever = np.asarray(point_in_base, dtype=float).reshape(3)
    rot = quat_to_rot_batch(traj.quats_xyzw)
    positions = traj.positions + rot @ lever
    if traj.covariances is None:
        return positions, None
    lx = np.array([[0.0, -lever[2], lever[1]], [lever[2], 0.0, -lever[0]],
                   [-lever[1], lever[0], 0.0]])
    jac = np.concatenate([np.broadcast_to(np.eye(3), rot.shape), -rot @ lx], axis=2)
    return positions, jac @ traj.covariances @ np.swapaxes(jac, 1, 2)
