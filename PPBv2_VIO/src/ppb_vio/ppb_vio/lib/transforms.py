"""Rotation and rigid-transform helpers (numpy only, no ROS imports).

Conventions used throughout ppb_vio:
- Quaternions are Hamilton, ordered (x, y, z, w), as in geometry_msgs.
- R_ab rotates vectors expressed in frame b into frame a.
- t_ab is the position of the origin of frame b expressed in frame a.
"""

import numpy as np

_SMALL_ANGLE = 1e-10


def skew(v):
    """Return the 3x3 matrix [v]x such that skew(v) @ u == cross(v, u)."""
    x, y, z = v
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def quat_to_rot(q_xyzw):
    """Convert a unit quaternion (x, y, z, w) to a rotation matrix."""
    x, y, z, w = q_xyzw
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ])


def rot_to_quat(rot):
    """Convert a rotation matrix to a unit quaternion (x, y, z, w) with w >= 0."""
    m = np.asarray(rot, dtype=float)
    trace = np.trace(m)
    if trace > 0.0:
        s = 2.0 * np.sqrt(trace + 1.0)
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w])
    q /= np.linalg.norm(q)
    return q if q[3] >= 0.0 else -q


def so3_exp(phi):
    """Rodrigues formula: rotation vector -> rotation matrix."""
    phi = np.asarray(phi, dtype=float)
    angle = np.linalg.norm(phi)
    k = skew(phi)
    if angle < _SMALL_ANGLE:
        return np.eye(3) + k
    return (np.eye(3)
            + (np.sin(angle) / angle) * k
            + ((1.0 - np.cos(angle)) / angle ** 2) * (k @ k))


def so3_log(rot):
    """Rotation matrix -> rotation vector (valid for angles below pi)."""
    m = np.asarray(rot, dtype=float)
    cos_angle = np.clip((np.trace(m) - 1.0) / 2.0, -1.0, 1.0)
    angle = np.arccos(cos_angle)
    vee = np.array([m[2, 1] - m[1, 2], m[0, 2] - m[2, 0], m[1, 0] - m[0, 1]])
    if angle < 1e-7:
        return 0.5 * vee
    return (angle / (2.0 * np.sin(angle))) * vee


def invert_transform(rot_ab, t_ab):
    """Given T_ab = (R_ab, t_ab), return T_ba = (R_ba, t_ba)."""
    rot_ba = rot_ab.T
    return rot_ba, -rot_ba @ t_ab
