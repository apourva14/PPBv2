"""Convert estimator IMU odometry into base_link odometry (no ROS imports).

Both MINS and OpenVINS publish nav_msgs/Odometry for the IMU frame I in their
own arbitrary world frame W ("global"):

- pose: R_WI (quaternion) and p_WI.
- pose covariance, ordered [position, orientation]: position error is additive
  in W; orientation error is in the IMU frame, R_WI_true = R_WI_est Exp(dtheta).
  (Both use JPL quaternions with q_GtoI_true = dq (x) q_GtoI_est, which is
  exactly this right perturbation once written as a Hamilton R_ItoG.)
- twist.angular: IMU angular rate in I.
- twist.linear: IMU velocity, but the frame differs per estimator:
  OpenVINS -> expressed in I (child); MINS -> expressed in W (parent).
  The caller must state which one via `linear_velocity_frame`.

Given the static mounting T_BI (pose of the IMU in base_link), base_link pose:
    R_WB = R_WI R_IB,     p_WB = p_WI + R_WI t_IB      (t_IB: base origin in I)
First-order covariance mapping for the pose, with the output using the same
convention (position error in W, orientation error in B):
    dp_B     = dp_I - R_WI [t_IB]x dtheta_I
    dtheta_B = R_BI dtheta_I
Twist of base_link expressed in B (rigid body, same angular rate):
    w_B = R_BI w_I
    v_B = R_BI (v_I + w_I x t_IB)
"""

from dataclasses import dataclass

import numpy as np

from ppb_vio.lib.transforms import invert_transform, quat_to_rot, rot_to_quat, skew

LINEAR_VELOCITY_FRAMES = ('child', 'parent')
QUATERNION_NORM_TOLERANCE = 1e-3


class OdometryRejected(ValueError):
    """Raised when an estimator message must not be forwarded."""

    def __init__(self, reason, detail):
        super().__init__(f'{reason}: {detail}')
        self.reason = reason


@dataclass(frozen=True)
class Extrinsic:
    """Static pose of the IMU in base_link (the base_link -> imu_link TF)."""

    rot_base_imu: np.ndarray
    t_base_imu: np.ndarray

    @classmethod
    def from_xyz_quat(cls, xyz, quat_xyzw):
        xyz = np.asarray(xyz, dtype=float)
        quat = np.asarray(quat_xyzw, dtype=float)
        if xyz.shape != (3,) or quat.shape != (4,):
            raise ValueError('extrinsic needs xyz[3] and quaternion xyzw[4]')
        if not (np.all(np.isfinite(xyz)) and np.all(np.isfinite(quat))):
            raise ValueError('extrinsic contains non-finite values')
        norm = np.linalg.norm(quat)
        if abs(norm - 1.0) > QUATERNION_NORM_TOLERANCE:
            raise ValueError(f'extrinsic quaternion is not unit length (|q|={norm:.6f})')
        return cls(quat_to_rot(quat / norm), xyz)

    @property
    def is_identity(self):
        return (np.allclose(self.rot_base_imu, np.eye(3), atol=1e-12)
                and np.allclose(self.t_base_imu, 0.0, atol=1e-12))


@dataclass
class ImuOdometry:
    """Estimator output for the IMU frame (see module docstring)."""

    position: np.ndarray            # p_WI, (3,)
    orientation_xyzw: np.ndarray    # R_WI, (4,)
    linear_velocity: np.ndarray     # (3,), frame given by linear_velocity_frame
    angular_velocity: np.ndarray    # w_I, (3,)
    pose_covariance: np.ndarray     # (6, 6) [p, theta]
    twist_covariance: np.ndarray    # (6, 6) [v, w]


@dataclass
class BaseOdometry:
    position: np.ndarray
    orientation_xyzw: np.ndarray
    linear_velocity: np.ndarray     # expressed in base_link
    angular_velocity: np.ndarray    # expressed in base_link
    pose_covariance: np.ndarray
    twist_covariance: np.ndarray


def _check_finite(name, value):
    if not np.all(np.isfinite(value)):
        raise OdometryRejected('non_finite', f'{name} contains NaN/Inf')


def _symmetrize(cov):
    return 0.5 * (cov + cov.T)


def imu_to_base(odom, extrinsic, linear_velocity_frame):
    """Return BaseOdometry for base_link, or raise OdometryRejected."""
    if linear_velocity_frame not in LINEAR_VELOCITY_FRAMES:
        raise ValueError(f'linear_velocity_frame must be one of {LINEAR_VELOCITY_FRAMES}')

    for name in ('position', 'orientation_xyzw', 'linear_velocity', 'angular_velocity',
                 'pose_covariance', 'twist_covariance'):
        _check_finite(name, getattr(odom, name))

    q = np.asarray(odom.orientation_xyzw, dtype=float)
    norm = np.linalg.norm(q)
    if abs(norm - 1.0) > QUATERNION_NORM_TOLERANCE:
        raise OdometryRejected('bad_quaternion', f'|q| = {norm:.6f}')
    q = q / norm

    rot_wi = quat_to_rot(q)
    rot_ib, t_ib = invert_transform(extrinsic.rot_base_imu, extrinsic.t_base_imu)
    rot_bi = rot_ib.T

    p_wi = np.asarray(odom.position, dtype=float)
    w_i = np.asarray(odom.angular_velocity, dtype=float)
    v = np.asarray(odom.linear_velocity, dtype=float)
    pose_cov = np.asarray(odom.pose_covariance, dtype=float)
    twist_cov = np.asarray(odom.twist_covariance, dtype=float)

    # Bring the input twist into the IMU frame first.
    if linear_velocity_frame == 'parent':
        v_i = rot_wi.T @ v
        to_imu = np.eye(6)
        to_imu[:3, :3] = rot_wi.T
        twist_cov = to_imu @ twist_cov @ to_imu.T
    else:
        v_i = v

    rot_wb = rot_wi @ rot_ib
    p_wb = p_wi + rot_wi @ t_ib
    w_b = rot_bi @ w_i
    v_b = rot_bi @ (v_i + np.cross(w_i, t_ib))

    j_pose = np.zeros((6, 6))
    j_pose[:3, :3] = np.eye(3)
    j_pose[:3, 3:] = -rot_wi @ skew(t_ib)
    j_pose[3:, 3:] = rot_bi

    j_twist = np.zeros((6, 6))
    j_twist[:3, :3] = rot_bi
    j_twist[:3, 3:] = -rot_bi @ skew(t_ib)
    j_twist[3:, 3:] = rot_bi

    return BaseOdometry(
        position=p_wb,
        orientation_xyzw=rot_to_quat(rot_wb),
        linear_velocity=v_b,
        angular_velocity=w_b,
        pose_covariance=_symmetrize(j_pose @ pose_cov @ j_pose.T),
        twist_covariance=_symmetrize(j_twist @ twist_cov @ j_twist.T),
    )
