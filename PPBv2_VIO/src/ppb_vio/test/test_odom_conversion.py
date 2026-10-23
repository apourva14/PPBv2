import numpy as np
import pytest

from ppb_vio.lib.odom_conversion import Extrinsic, ImuOdometry, imu_to_base, OdometryRejected
from ppb_vio.lib.transforms import quat_to_rot, rot_to_quat, so3_exp, so3_log

RNG = np.random.default_rng(7)


def random_rotation():
    return so3_exp(RNG.uniform(-np.pi, np.pi, 3) * 0.9)


def random_cov(scale):
    a = RNG.normal(size=(6, 6)) * scale
    return a @ a.T + np.eye(6) * scale ** 2


def make_odom(rot_wi, p_wi, v, w, pose_cov=None, twist_cov=None):
    return ImuOdometry(position=np.asarray(p_wi, float), orientation_xyzw=rot_to_quat(rot_wi),
                       linear_velocity=np.asarray(v, float), angular_velocity=np.asarray(w, float),
                       pose_covariance=np.eye(6) * 1e-4 if pose_cov is None else pose_cov,
                       twist_covariance=np.eye(6) * 1e-4 if twist_cov is None else twist_cov)


IDENTITY = Extrinsic.from_xyz_quat([0, 0, 0], [0, 0, 0, 1])


# --- transforms ------------------------------------------------------------

def test_quaternion_rotation_round_trip():
    for _ in range(200):
        rot = random_rotation()
        assert np.allclose(quat_to_rot(rot_to_quat(rot)), rot, atol=1e-12)


def test_so3_exp_log_round_trip():
    for _ in range(200):
        axis = RNG.normal(size=3)
        phi = axis / np.linalg.norm(axis) * RNG.uniform(0.0, 3.0)   # log is unique below pi
        assert np.allclose(so3_log(so3_exp(phi)), phi, atol=1e-9)
    assert np.allclose(so3_log(np.eye(3)), 0.0)


def test_known_quaternion_is_90_deg_about_z():
    rot = quat_to_rot([0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5)])
    assert np.allclose(rot @ [1, 0, 0], [0, 1, 0], atol=1e-12)


# --- extrinsic validation --------------------------------------------------

@pytest.mark.parametrize('xyz,quat', [
    ([0, 0], [0, 0, 0, 1]),
    ([0, 0, 0], [0, 0, 0, 2]),
    ([np.nan, 0, 0], [0, 0, 0, 1]),
])
def test_extrinsic_rejects_invalid_input(xyz, quat):
    with pytest.raises(ValueError):
        Extrinsic.from_xyz_quat(xyz, quat)


# --- conversion ------------------------------------------------------------

def test_identity_extrinsic_passes_through_child_frame_odometry():
    rot = random_rotation()
    pose_cov, twist_cov = random_cov(0.01), random_cov(0.01)
    odom = make_odom(rot, [1, 2, 3], [0.5, -0.1, 0.2], [0.01, 0.02, 0.3], pose_cov, twist_cov)
    out = imu_to_base(odom, IDENTITY, 'child')
    assert np.allclose(out.position, [1, 2, 3])
    assert np.allclose(quat_to_rot(out.orientation_xyzw), rot)
    assert np.allclose(out.linear_velocity, odom.linear_velocity)
    assert np.allclose(out.angular_velocity, odom.angular_velocity)
    assert np.allclose(out.pose_covariance, pose_cov)
    assert np.allclose(out.twist_covariance, twist_cov)


def test_parent_frame_velocity_is_rotated_into_body():
    # IMU yawed +90 deg: moving along world +y means moving along body +x.
    rot = so3_exp([0, 0, np.pi / 2])
    out = imu_to_base(make_odom(rot, [0, 0, 0], [0, 1.0, 0], [0, 0, 0]), IDENTITY, 'parent')
    assert np.allclose(out.linear_velocity, [1.0, 0, 0], atol=1e-12)


def test_lever_arm_pose_and_velocity():
    # IMU mounted 1 m ahead of base_link, no rotation. Robot yawed +90 deg in W,
    # spinning at 1 rad/s about z with no IMU translation velocity.
    extrinsic = Extrinsic.from_xyz_quat([1.0, 0, 0], [0, 0, 0, 1])
    rot = so3_exp([0, 0, np.pi / 2])
    out = imu_to_base(make_odom(rot, [5.0, 5.0, 0], [0, 0, 0], [0, 0, 1.0]), extrinsic, 'child')
    # Base is 1 m "behind" the IMU along body x, i.e. along world -y.
    assert np.allclose(out.position, [5.0, 4.0, 0], atol=1e-12)
    # Base origin is at -1 m along body x: v = w x r = z x (-x) = -y (body).
    assert np.allclose(out.linear_velocity, [0, -1.0, 0], atol=1e-12)
    assert np.allclose(out.angular_velocity, [0, 0, 1.0])


def test_rotated_mounting_orientation():
    # IMU rotated +90 deg about z relative to base_link.
    q_bi = rot_to_quat(so3_exp([0, 0, np.pi / 2]))
    extrinsic = Extrinsic.from_xyz_quat([0, 0, 0], q_bi)
    out = imu_to_base(make_odom(np.eye(3), [0, 0, 0], [1, 0, 0], [0, 0, 0.5]), extrinsic, 'child')
    # World == IMU frame here, so base is yawed -90 deg in W.
    assert np.allclose(quat_to_rot(out.orientation_xyzw), so3_exp([0, 0, -np.pi / 2]), atol=1e-12)
    # IMU-x is base-y.
    assert np.allclose(out.linear_velocity, [0, 1, 0], atol=1e-12)
    assert np.allclose(out.angular_velocity, [0, 0, 0.5])


def test_pose_covariance_matches_monte_carlo():
    """The analytic mapping must agree with sampling under the stated convention:
    p_true = p + dp (world), R_true = R Exp(dtheta) (body)."""
    rot_bi = random_rotation()
    t_bi = np.array([0.4, -0.3, 0.8])
    extrinsic = Extrinsic.from_xyz_quat(t_bi, rot_to_quat(rot_bi))
    rot_wi, p_wi = random_rotation(), np.array([3.0, -2.0, 1.0])
    sigma = random_cov(0.01)
    odom = make_odom(rot_wi, p_wi, [0, 0, 0], [0, 0, 0], pose_cov=sigma)
    nominal = imu_to_base(odom, extrinsic, 'child')
    rot_wb = quat_to_rot(nominal.orientation_xyzw)

    samples = RNG.multivariate_normal(np.zeros(6), sigma, size=20000)
    errors = []
    for s in samples:
        perturbed = make_odom(rot_wi @ so3_exp(s[3:]), p_wi + s[:3], [0, 0, 0], [0, 0, 0])
        out = imu_to_base(perturbed, extrinsic, 'child')
        errors.append(np.concatenate([
            out.position - nominal.position,
            so3_log(rot_wb.T @ quat_to_rot(out.orientation_xyzw))]))
    empirical = np.cov(np.array(errors).T)
    assert np.allclose(empirical, nominal.pose_covariance, atol=3e-4, rtol=0.1)


def test_twist_covariance_matches_monte_carlo():
    rot_bi = random_rotation()
    t_bi = np.array([-0.5, 0.2, 0.3])
    extrinsic = Extrinsic.from_xyz_quat(t_bi, rot_to_quat(rot_bi))
    rot_wi = random_rotation()
    v_w, w_i = np.array([1.0, 0.5, 0.0]), np.array([0.1, -0.2, 0.4])
    sigma = random_cov(0.05)
    nominal = imu_to_base(make_odom(rot_wi, [0, 0, 0], v_w, w_i, twist_cov=sigma), extrinsic,
                          'parent')
    samples = RNG.multivariate_normal(np.zeros(6), sigma, size=20000)
    errors = []
    for s in samples:
        out = imu_to_base(make_odom(rot_wi, [0, 0, 0], v_w + s[:3], w_i + s[3:]), extrinsic,
                          'parent')
        errors.append(np.concatenate([out.linear_velocity - nominal.linear_velocity,
                                      out.angular_velocity - nominal.angular_velocity]))
    empirical = np.cov(np.array(errors).T)
    assert np.allclose(empirical, nominal.twist_covariance, atol=2e-3, rtol=0.1)


@pytest.mark.parametrize('field,value,reason', [
    ('position', [np.nan, 0, 0], 'non_finite'),
    ('linear_velocity', [np.inf, 0, 0], 'non_finite'),
    ('pose_covariance', np.full((6, 6), np.nan), 'non_finite'),
    ('orientation_xyzw', [0, 0, 0, 1.1], 'bad_quaternion'),
    ('orientation_xyzw', [0, 0, 0, 0], 'bad_quaternion'),
])
def test_rejects_invalid_estimator_output(field, value, reason):
    odom = make_odom(np.eye(3), [0, 0, 0], [0, 0, 0], [0, 0, 0])
    setattr(odom, field, np.asarray(value, float))
    with pytest.raises(OdometryRejected) as exc:
        imu_to_base(odom, IDENTITY, 'child')
    assert exc.value.reason == reason


def test_unknown_velocity_frame_is_a_programming_error():
    with pytest.raises(ValueError):
        imu_to_base(make_odom(np.eye(3), [0, 0, 0], [0, 0, 0], [0, 0, 0]), IDENTITY, 'world')
