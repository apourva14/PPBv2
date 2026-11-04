"""Tests for the evaluation harness (association, reference point, RTE, NEES)
and the Rosario v1 loader (including an end-to-end conversion of a synthetic
ROS 1 bag)."""

import json
from pathlib import Path
import sys

import numpy as np
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import evaluate_run  # noqa: E402
import prepare_rosario_v1  # noqa: E402
from ppb_vio_tools.euroc import DatasetError  # noqa: E402
from ppb_vio_tools.metrics import position_nees, relative_translation_errors  # noqa: E402
from ppb_vio_tools.rosario_v1 import (  # noqa: E402
    assemble_bag, bgr8_to_mono8, file_md5, read_calibration, read_ground_truth, sequence_files)
from ppb_vio_tools.trajectory import (  # noqa: E402
    associate, quat_to_rot_batch, read_tum, reference_point, slerp, Trajectory, write_tum)


def yaw_quats(yaw):
    yaw = np.atleast_1d(yaw)
    return np.stack([np.zeros_like(yaw), np.zeros_like(yaw), np.sin(yaw / 2), np.cos(yaw / 2)], 1)


# --------------------------------------------------------------------------- association

def test_associate_interpolates_inside_brackets_only():
    est = Trajectory([0.0, 0.1, 0.2, 1.0, 1.1], [[0, 0, 0], [1, 0, 0], [2, 0, 0], [3, 0, 0],
                                                  [4, 0, 0]], yaw_quats(np.zeros(5)))
    idx, at = associate([0.05, 0.1, 0.5, 1.05, 2.0, -1.0], est, max_bracket_s=0.25)
    assert idx.tolist() == [0, 1, 3]          # 0.5 s lies in a 0.8 s gap: not bridged
    assert np.allclose(at.positions[:, 0], [0.5, 1.0, 3.5])


def test_associate_slerps_orientation_and_takes_nearest_covariance():
    cov = np.stack([np.eye(6), 2 * np.eye(6)])
    est = Trajectory([0.0, 1.0], np.zeros((2, 3)), yaw_quats([0.0, 1.0]), cov)
    _, at = associate([0.25, 0.75], est, max_bracket_s=2.0)
    assert np.allclose(at.quats_xyzw, yaw_quats([0.25, 0.75]))
    assert at.covariances[0, 0, 0] == 1.0 and at.covariances[1, 0, 0] == 2.0


def test_slerp_takes_shortest_path():
    q = slerp(yaw_quats(0.2), -yaw_quats(0.4), 0.5)
    assert np.allclose(np.abs(q), np.abs(yaw_quats(0.3)))


def test_tum_round_trip(tmp_path):
    traj = Trajectory([1.0, 2.0], [[1, 2, 3], [4, 5, 6]], yaw_quats([0.0, 0.5]))
    write_tum(tmp_path / 't.tum', traj)
    back = read_tum(tmp_path / 't.tum')
    assert np.allclose(back.positions, traj.positions) and np.allclose(back.quats_xyzw,
                                                                       traj.quats_xyzw)


def test_trajectory_rejects_unsorted_stamps():
    with pytest.raises(ValueError, match='increasing'):
        Trajectory([1.0, 1.0], np.zeros((2, 3)), yaw_quats([0, 0]))


# --------------------------------------------------------------------------- reference point

def test_reference_point_position_and_first_order_covariance():
    rng = np.random.default_rng(0)
    lever = np.array([1.8, -0.03, 1.6])
    q = np.array([[0.1, -0.2, 0.3, 0.9]])
    q /= np.linalg.norm(q)
    a = rng.normal(size=(6, 6)) * 0.01
    cov = a @ a.T
    traj = Trajectory([0.0], [[1.0, 2.0, 3.0]], q, cov[None])
    pos, pcov = reference_point(traj, lever)
    rot = quat_to_rot_batch(q)[0]
    assert np.allclose(pos[0], [1, 2, 3] + rot @ lever)

    # Monte Carlo with the /vio/odom convention p_true = p + dp, R_true = R Exp(dtheta).
    samples = rng.multivariate_normal(np.zeros(6), cov, size=40000)
    pts = []
    for s in samples:
        th = s[3:]
        angle = np.linalg.norm(th)
        k = th / angle
        kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
        exp = np.eye(3) + np.sin(angle) * kx + (1 - np.cos(angle)) * kx @ kx
        pts.append(np.array([1, 2, 3]) + s[:3] + rot @ exp @ lever)
    # Sampling noise of 40k samples is ~1 % of the diagonal; small off-diagonals need atol.
    assert np.allclose(np.cov(np.array(pts).T), pcov[0], rtol=0.05, atol=2e-5)


# --------------------------------------------------------------------------- RTE

def test_rte_zero_for_perfect_estimate_and_counts_segments():
    ref = np.column_stack([np.linspace(0, 100, 1001), np.zeros(1001), np.zeros(1001)])
    errors = relative_translation_errors(ref, ref, 10.0)
    assert errors.size == 10 and np.allclose(errors, 0.0)


def test_rte_measures_scale_drift_per_segment():
    ref = np.column_stack([np.linspace(0, 100, 1001), np.zeros(1001), np.zeros(1001)])
    est = ref * 1.02                                       # 2 % scale error
    assert np.allclose(relative_translation_errors(ref, est, 10.0), 0.2, atol=1e-6)


def test_rte_rejects_bad_input():
    with pytest.raises(ValueError):
        relative_translation_errors(np.zeros((3, 3)), np.zeros((3, 3)), 0.0)


# --------------------------------------------------------------------------- NEES

def test_nees_of_consistent_estimator_is_about_three():
    rng = np.random.default_rng(1)
    cov = np.diag([0.04, 0.09, 0.01])
    err = rng.multivariate_normal(np.zeros(3), cov, size=20000)
    out = position_nees(err, np.broadcast_to(cov, (20000, 3, 3)))
    assert out['mean'] == pytest.approx(3.0, rel=0.03)
    assert out['fraction_above_chi2_95'] == pytest.approx(0.05, abs=0.01)
    assert out['suggested_covariance_scale'] == pytest.approx(1.0, rel=0.03)


def test_nees_flags_overconfidence_and_skips_singular():
    err = np.full((10, 3), 0.3)
    cov = np.broadcast_to(np.eye(3) * 0.01, (10, 3, 3)).copy()
    cov[0] = 0.0
    out = position_nees(err, cov)
    assert out['skipped'] == 1 and out['mean'] > 20
    assert position_nees(err, np.zeros((10, 3, 3)))['status'] == 'unavailable'


# --------------------------------------------------------------------------- full evaluation

def test_evaluate_scores_antenna_point_after_yaw_alignment(tmp_path):
    """An estimate of base_link in its own (rotated, shifted) world frame must
    score ~0 against position-only ground truth of an antenna on base_link."""
    lever = [1.8, -0.03, 1.59]
    t = np.arange(0, 60, 0.2)
    yaw = 0.3 * np.sin(0.1 * t)
    base = np.column_stack([np.cumsum(np.cos(yaw)) * 0.2, np.cumsum(np.sin(yaw)) * 0.2,
                            np.zeros_like(t)])
    truth = Trajectory(t, base, yaw_quats(yaw))
    antenna, _ = reference_point(truth, lever)
    gt = np.column_stack([t, antenna, np.tile([0, 0, 0, 1], (t.size, 1))])
    np.savetxt(tmp_path / 'gt.tum', gt)

    off = 1.2                                       # estimator world is yawed and shifted
    rz = np.array([[np.cos(off), -np.sin(off), 0], [np.sin(off), np.cos(off), 0], [0, 0, 1]])
    est = Trajectory(t + 0.05, base @ rz.T + [5, -3, 0], yaw_quats(yaw + off))
    ds = {'ground_truth': str(tmp_path / 'gt.tum'), 'has_orientation': False,
          'point_in_base': lever}
    r = evaluate_run.evaluate(est, ds, max_bracket_s=0.25)
    assert r['ape_rmse_m'] < 0.02 and r['matched_poses'] > 280
    assert r['alignment_yaw_deg'] == pytest.approx(-np.degrees(off), abs=0.5)
    assert 'rpe_rotation_rmse_deg_per_10m' not in r          # no orientation in ground truth
    assert r['nees_position']['status'] == 'unavailable'

    # Ignoring the lever arm must show up as error (the antenna is 1.8 m forward).
    r0 = evaluate_run.evaluate(est, dict(ds, point_in_base=[0, 0, 0]), max_bracket_s=0.25)
    assert r0['ape_rmse_m'] > 0.1


# --------------------------------------------------------------------------- Rosario v1

CALIBRATION = """cam0:
  T_cam_imu:
  - [0.009471441780975032, 0.9984567420628084, 0.05472134884952806, 0.07585973485189132]
  - [-0.9939081748565041, 0.0033909183804303744, 0.11015916496574298, -0.03342054396674176]
  - [0.10980360533243941, -0.055431362078334365, 0.9924064350628417, -0.027188513339280743]
  - [0.0, 0.0, 0.0, 1.0]
  camera_model: pinhole
  distortion_coeffs: [-0.174497, 0.027127, -0.000359, 0.000457]
  distortion_model: radtan
  intrinsics: [348.522264, 348.449870, 344.483596, 188.808062]
  resolution: [WIDTH, HEIGHT]
  rostopic: /cam0/image_raw
cam1:
  T_cam_imu:
  - [0.010486169744957197, 0.9972728488827708, 0.07305412462908822, -0.04365571656478627]
  - [-0.9940873423040585, 0.002500626397312078, 0.10855460717295605, -0.033255842186752066]
  - [0.10807588128224391, -0.07376050263429253, 0.9914025378907414, -0.02682521301910111]
  - [0.0, 0.0, 0.0, 1.0]
  camera_model: pinhole
  distortion_coeffs: [-0.174606, 0.027855, -0.000108, -0.000141]
  distortion_model: radtan
  intrinsics: [349.569635, 349.390781, 340.836585, 206.105440]
  resolution: [WIDTH, HEIGHT]
  rostopic: /cam1/image_raw
  T_cn_cnm1:
  - [0.9999064, 0.00116937, 0.01362243, -0.118718331]
  - [-0.00112614, 0.99999456, -0.00314845, 0.0000521848390]
  - [-0.01362563, 0.00313285, 0.99990197, -0.000269290935]
  - [0.0, 0.0, 0.0, 1.0]
gps:
  position_gps_baselink: [1.80070337, -0.02982362, 1.59345719]
  orientation_gps_baselink: [0, 0, 0, 1]
imu:
  position_imu_baselink: [1.96, -0.017, 1.02]
  rotation_imu_baselink: [0.81281825, -0.00799763, 0.58239402, 0.00892938]
"""


def write_calibration(path, width=672, height=376):
    path.write_text(CALIBRATION.replace('WIDTH', str(width)).replace('HEIGHT', str(height)))
    return path


def test_sequence_files():
    assert sequence_files(2)[0] == ['sequence02.bag.00', 'sequence02.bag.01']
    assert sequence_files(4)[0] == ['sequence04.bag']
    with pytest.raises(DatasetError):
        sequence_files(7)


def test_assemble_bag_checks_md5_and_leaves_nothing_on_mismatch(tmp_path):
    parts = []
    for k, data in enumerate([b'abc', b'def']):
        parts.append(tmp_path / f'p{k}')
        parts[-1].write_bytes(data)
    out = tmp_path / 'joined.bag'
    with pytest.raises(DatasetError, match='MD5'):
        assemble_bag(parts, out, '0' * 32)
    assert not out.exists() and not (tmp_path / 'joined.bag.partial').exists()
    import hashlib
    assemble_bag(parts, out, hashlib.md5(b'abcdef').hexdigest())
    assert out.read_bytes() == b'abcdef'


def test_calibration_imu_rotation_maps_gravity_to_base_z(tmp_path):
    calib = read_calibration(write_calibration(tmp_path / 'c.yaml'))
    x, y, z, w = calib.imu_in_base_quat_xyzw
    rot = quat_to_rot_batch(np.array([x, y, z, w]))
    # Mean specific force measured at the start of sequence02 (IMU frame).
    assert np.allclose(rot @ [9.149, 0.128, -3.316], [0, 0, 9.73], atol=0.3)
    xyz, q = calib.imu_to_camera_pose(0)
    t = np.asarray(calib.t_cam_imu[0])
    assert np.allclose(quat_to_rot_batch(q), t[:3, :3].T, atol=1e-6)
    assert np.allclose(xyz, -t[:3, :3].T @ t[:3, 3])


def test_bgr8_to_mono8_matches_opencv_weights():
    bgr = np.array([[[255, 0, 0], [0, 255, 0], [0, 0, 255], [255, 255, 255]]], dtype=np.uint8)
    out = bgr8_to_mono8(bgr.reshape(-1), 4, 1, 12)
    assert out.tolist() == [29, 150, 76, 255]                   # cv::cvtColor(BGR2GRAY)
    padded = np.concatenate([bgr.reshape(-1), [7, 7]]).astype(np.uint8)   # step > 3 * width
    assert bgr8_to_mono8(padded, 4, 1, 14).tolist() == [29, 150, 76, 255]
    with pytest.raises(DatasetError):
        bgr8_to_mono8(bgr.reshape(-1), 5, 1, 15)


def test_ground_truth_must_be_position_only(tmp_path):
    p = tmp_path / 'gt.txt'
    p.write_text('1.0 0 0 0 0 0 0 1\n2.0 1 0 0 0 0 0 1\n')
    assert read_ground_truth(p).shape == (2, 8)
    p.write_text('1.0 0 0 0 0 0 0.1 0.99\n2.0 1 0 0 0 0 0 1\n')
    with pytest.raises(DatasetError, match='identity'):
        read_ground_truth(p)


def write_ros1_sequence(folder, accel_z=9.81, n_frames=30, width=8, height=4):
    """A miniature Rosario v1 download: one-part bag (as sequence 4), calibration, ground truth."""
    from rosbags.rosbag1 import Writer
    from rosbags.typesys import get_typestore, Stores

    ts = get_typestore(Stores.ROS1_NOETIC)
    t = ts.types
    folder.mkdir()
    write_calibration(folder / 'calibration04.yaml', width, height)

    def hdr(ns, frame):
        return t['std_msgs/msg/Header'](seq=0, stamp=t['builtin_interfaces/msg/Time'](
            sec=ns // 10**9, nanosec=ns % 10**9), frame_id=frame)

    v3 = lambda x, y, z: t['geometry_msgs/msg/Vector3'](x=x, y=y, z=z)   # noqa: E731
    quat = t['geometry_msgs/msg/Quaternion'](x=0.0, y=0.0, z=0.0, w=1.0)
    start = 1_514_556_846_000_000_000
    events = []
    for k in range(n_frames):                                            # 15 Hz stereo
        ns = start + k * 66_666_667
        for topic, frame in (('/stereo/left/image_raw', 'left'), ('/stereo/right/image_raw',
                                                                   'right')):
            img = np.full(height * width * 3, k % 256, dtype=np.uint8)
            events.append((ns, topic, t['sensor_msgs/msg/Image'](
                header=hdr(ns, frame), height=height, width=width, encoding='bgr8',
                is_bigendian=0, step=width * 3, data=img)))
    for k in range(n_frames * 10):                                       # ~150 Hz IMU
        ns = start + k * 6_666_667
        events.append((ns, '/imu', t['sensor_msgs/msg/Imu'](
            header=hdr(ns, 'imu'), orientation=quat, orientation_covariance=np.zeros(9),
            angular_velocity=v3(0.0, 0.0, 0.01), angular_velocity_covariance=np.zeros(9),
            linear_acceleration=v3(0.0, 0.0, accel_z), linear_acceleration_covariance=np.zeros(9))))
    for k in range(n_frames // 2 + 1):                                   # ~7 Hz wheel odometry
        ns = start + k * 133_333_333
        x = 0.5 * k * 0.133333333
        events.append((ns, '/odom', t['nav_msgs/msg/Odometry'](
            header=hdr(ns, 'odom'), child_frame_id='base_link',
            pose=t['geometry_msgs/msg/PoseWithCovariance'](
                pose=t['geometry_msgs/msg/Pose'](position=t['geometry_msgs/msg/Point'](
                    x=x, y=0.0, z=0.0), orientation=quat), covariance=np.zeros(36)),
            twist=t['geometry_msgs/msg/TwistWithCovariance'](
                twist=t['geometry_msgs/msg/Twist'](linear=v3(0.5, 0.0, 0.0),
                                                   angular=v3(0.0, 0.0, 0.0)),
                covariance=np.zeros(36)))))
    events.sort(key=lambda e: e[0])
    with Writer(folder / 'sequence04.bag') as writer:
        conns = {}
        for ns, topic, msg in events:
            if topic not in conns:
                conns[topic] = writer.add_connection(topic, msg.__msgtype__, typestore=ts)
            writer.write(conns[topic], ns, ts.serialize_ros1(msg, msg.__msgtype__))
    gt = np.array([[(start + k * 200_000_000) / 1e9, 0.1 * k, 0, 0, 0, 0, 0, 1] for k in range(10)])
    np.savetxt(folder / 'sequence04_gt.txt', gt, fmt='%.9f')
    return folder


def test_convert_synthetic_rosario_sequence(tmp_path, monkeypatch):
    src = write_ros1_sequence(tmp_path / 'download')
    monkeypatch.setitem(prepare_rosario_v1.SEQUENCES, 4,
                        {'parts': 1, 'md5': file_md5(src / 'sequence04.bag')})
    report = prepare_rosario_v1.convert(4, src, tmp_path / 'out')
    counts = report['message_counts']
    assert counts['/vio/cam0/image_raw'] == counts['/vio/cam1/image_raw'] == 30
    assert counts['/vio/imu'] == 300 and counts['/wheel/odom'] == 16 and counts['/tf_static'] == 1
    assert report['unpaired_stereo_stamps'] == 0
    assert report['imu_median_accel_norm_mps2'] == pytest.approx(9.81)
    check = report['wheel_odom_twist_frame_check']
    assert check['rms_if_body_frame_mps'] < 0.05

    ds = yaml.safe_load((tmp_path / 'out' / 'dataset.yaml').read_text())
    assert ds['ground_truth_point_in_base'] == [1.80070337, -0.02982362, 1.59345719]
    assert ds['ground_truth_has_orientation'] is False

    from rosbags.highlevel import AnyReader
    with AnyReader([tmp_path / 'out' / 'ros2_bag']) as reader:
        conn = next(c for c in reader.connections if c.topic == '/vio/cam0/image_raw')
        msg = reader.deserialize(next(reader.messages(connections=[conn]))[2], conn.msgtype)
        assert msg.encoding == 'mono8' and msg.step == 8 and msg.header.frame_id == \
            'cam0_optical_frame'
        assert int(msg.data[0]) == 0

    with pytest.raises(DatasetError, match='already exists'):
        prepare_rosario_v1.convert(4, src, tmp_path / 'out')


def test_convert_refuses_imu_not_in_si_units(tmp_path, monkeypatch):
    src = write_ros1_sequence(tmp_path / 'download', accel_z=1.0)        # in g, not m/s^2
    monkeypatch.setitem(prepare_rosario_v1.SEQUENCES, 4,
                        {'parts': 1, 'md5': file_md5(src / 'sequence04.bag')})
    with pytest.raises(DatasetError, match='gravity'):
        prepare_rosario_v1.convert(4, src, tmp_path / 'out')
    assert not (tmp_path / 'out' / 'ros2_bag').exists()
    assert not (tmp_path / 'out' / 'ros2_bag.partial').exists()


def test_convert_refuses_wrong_checksum(tmp_path):
    src = write_ros1_sequence(tmp_path / 'download')
    with pytest.raises(DatasetError, match='MD5'):
        prepare_rosario_v1.convert(4, src, tmp_path / 'out')


def _opencv_yaml(text):
    return yaml.safe_load('\n'.join(l for l in text.splitlines() if not l.startswith('%YAML')))


def test_openvins_calibration_files_invert_kalibr_extrinsics(tmp_path):
    from ppb_vio_tools.rig import openvins_files, platform_yaml
    from ppb_vio_tools.rosario_v1 import IMU_NOISE, to_rig
    calib = read_calibration(write_calibration(tmp_path / 'c.yaml'))
    files = openvins_files(to_rig(calib))
    chain = _opencv_yaml(files['kalibr_imucam_chain.yaml'])
    for i in (0, 1):
        t_imu_cam = np.array(chain[f'cam{i}']['T_imu_cam'])
        assert np.allclose(t_imu_cam @ np.asarray(calib.t_cam_imu[i]), np.eye(4), atol=1e-6)
        assert chain[f'cam{i}']['rostopic'] == f'/vio/cam{i}/image_raw'
        assert chain[f'cam{i}']['resolution'] == [672, 376]
        assert chain[f'cam{i}']['distortion_model'] == 'radtan'
    imu = _opencv_yaml(files['kalibr_imu_chain.yaml'])['imu0']
    assert imu['rostopic'] == '/vio/imu' and imu['gyroscope_noise_density'] == \
        IMU_NOISE['gyroscope_noise_density']
    platform = yaml.safe_load(platform_yaml(to_rig(calib), 'test', {'max_speed_mps': 2.5}))
    params = platform['vio_odom_adapter']['ros__parameters']
    assert params['imu_in_base_xyz'] == [1.96, -0.017, 1.02]
    assert params['health_max_speed_mps'] == 2.5


# --------------------------------------------------------------------------- stress bags

def _contract_bag(tmp_path, monkeypatch):
    src = write_ros1_sequence(tmp_path / 'download')
    monkeypatch.setitem(prepare_rosario_v1.SEQUENCES, 4,
                        {'parts': 1, 'md5': file_md5(src / 'sequence04.bag')})
    prepare_rosario_v1.convert(4, src, tmp_path / 'out')
    return tmp_path / 'out' / 'ros2_bag'


def _camera_stamps(bag, topic):
    from rosbags.highlevel import AnyReader
    with AnyReader([bag]) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        return [reader.deserialize(raw, c.msgtype).header.stamp.sec * 10**9
                + reader.deserialize(raw, c.msgtype).header.stamp.nanosec
                for c, _, raw in reader.messages(connections=conns)]


def test_stress_drops_whole_stereo_pairs_and_keeps_pairing(tmp_path, monkeypatch):
    import stress_bag
    bag = _contract_bag(tmp_path, monkeypatch)
    rep = stress_bag.stress(bag, tmp_path / 'stressed', drop_frames=0.3, jitter_ms=3.0, seed=1)
    left = _camera_stamps(tmp_path / 'stressed', '/vio/cam0/image_raw')
    right = _camera_stamps(tmp_path / 'stressed', '/vio/cam1/image_raw')
    info = _camera_stamps(tmp_path / 'stressed', '/vio/cam0/camera_info')
    assert left == right == info                          # pairs dropped/jittered together
    assert len(left) == 30 - rep['counts']['camera_pairs_dropped'] > 0
    assert all(b > a for a, b in zip(left, left[1:]))      # still increasing
    original = set(_camera_stamps(bag, '/vio/cam0/image_raw'))
    assert not set(left) <= original                       # jitter actually moved stamps
    again = stress_bag.stress(bag, tmp_path / 'again', drop_frames=0.3, jitter_ms=3.0, seed=1)
    assert again['counts'] == rep['counts']                # seeded: repeatable


def test_stress_blackout_and_late_start(tmp_path, monkeypatch):
    import stress_bag
    bag = _contract_bag(tmp_path, monkeypatch)
    rep = stress_bag.stress(bag, tmp_path / 's', blackout=(0.5, 1.0), start_s=0.2, imu_drop=0.5)
    stamps = np.array(_camera_stamps(tmp_path / 's', '/vio/cam0/image_raw'))
    rel = (stamps - 1_514_556_846_000_000_000) / 1e9
    assert rel.min() >= 0.2 and not np.any((rel >= 0.5) & (rel < 1.0))
    assert 0 < rep['counts']['imu_dropped'] < rep['counts']['imu_in']
    with pytest.raises(SystemExit):
        stress_bag.stress(bag, tmp_path / 's')                # never overwrites


def test_cam1_extrinsic_is_chained_from_the_stereo_calibration(tmp_path):
    path = write_calibration(tmp_path / 'c.yaml')
    calib = read_calibration(path)                       # default: stereo from T_cn_cnm1
    raw = read_calibration(path, stereo_source='imu')    # the file's own cam1 T_cam_imu
    t_cn_cnm1 = np.array(yaml.safe_load(CALIBRATION.replace('WIDTH', '1').replace(
        'HEIGHT', '1'))['cam1']['T_cn_cnm1'])
    rel = np.asarray(calib.t_cam_imu[1]) @ np.linalg.inv(np.asarray(calib.t_cam_imu[0]))
    assert np.allclose(rel, t_cn_cnm1, atol=1e-5)          # inputs carry ~7 digits
    # The published file is inconsistent by ~0.29 deg (0.27 about camera y).
    assert 0.25 < calib.stereo_discrepancy_deg < 0.35
    assert np.allclose(calib.t_cam_imu[0], raw.t_cam_imu[0])
    with pytest.raises(ValueError):
        read_calibration(path, stereo_source='nope')


def test_divergence_onset_flags_speed_and_vertical_runaways_only():
    from ppb_vio_tools.metrics import divergence_onset
    t = np.arange(0, 60, 0.2)
    ref = np.column_stack([1.1 * t, np.zeros_like(t), np.zeros_like(t)])
    rz = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    assert divergence_onset(t, ref, ref @ rz.T + 5.0) is None        # alignment-free
    sink = ref.copy()
    sink[t > 30, 2] = -0.8 * (t[t > 30] - 30)                         # sinks at 0.8 m/s from 30 s
    assert abs(divergence_onset(t, ref, sink) - 29.2) < 1.0
    fast = ref.copy()
    fast[t > 40, 0] += 2.5 * (t[t > 40] - 40)                         # +2.5 m/s from 40 s
    assert abs(divergence_onset(t, ref, fast) - 39.2) < 1.0



# --------------------------------------------------------------------------- robot calibration

KALIBR_IMU = """accelerometer_noise_density: 0.002
accelerometer_random_walk: 0.0003
gyroscope_noise_density: 0.00017
gyroscope_random_walk: 0.00002
rostopic: /imu
update_rate: 200.0
"""


def _kalibr_inputs(tmp_path, cam1_timeshift=0.003):
    camchain = CALIBRATION.replace('WIDTH', '672').replace('HEIGHT', '376')
    camchain = camchain.replace('  rostopic: /cam0/image_raw\n',
                                '  rostopic: /cam0/image_raw\n  timeshift_cam_imu: 0.003\n')
    camchain = camchain.replace('  rostopic: /cam1/image_raw\n',
                                f'  rostopic: /cam1/image_raw\n  timeshift_cam_imu: {cam1_timeshift}\n')
    (tmp_path / 'camchain-imucam.yaml').write_text(camchain)
    (tmp_path / 'imu.yaml').write_text(KALIBR_IMU)
    return tmp_path / 'camchain-imucam.yaml', tmp_path / 'imu.yaml'


def _run_kalibr_tool(tmp_path, *extra, cam1_timeshift=0.003):
    import subprocess
    cc, imu = _kalibr_inputs(tmp_path, cam1_timeshift)
    tool = Path(__file__).resolve().parents[1] / 'kalibr_to_openvins.py'
    return subprocess.run([sys.executable, '-I', str(tool), '--camchain', str(cc), '--imu', str(imu),
                           '--imu-in-base-xyz', '0.4', '0.0', '1.1',
                           '--imu-in-base-rpy-deg', '0', '0', '90', '--camera-hz', '30', *extra],
                          capture_output=True, text=True)


def test_kalibr_to_openvins_writes_robot_calibration(tmp_path):
    out = tmp_path / 'ppbv2_cal'
    r = _run_kalibr_tool(tmp_path, '--out', str(out))
    assert r.returncode == 0, r.stderr
    report = json.loads((out / 'calibration_report.json').read_text())
    assert report['timeshift_cam_imu_s'] == 0.003 and report['imu_hz'] == 200.0
    assert 118 < report['stereo_baseline_mm'] < 120
    assert report['warnings'] and 'disagree' in report['warnings'][0]   # fixture: 0.29 deg
    params = yaml.safe_load((out / 'platform.yaml').read_text())['vio_odom_adapter']['ros__parameters']
    assert params['health_max_speed_mps'] == 2.5
    assert np.allclose(params['imu_in_base_quat_xyzw'], [0, 0, np.sin(np.pi / 4), np.cos(np.pi / 4)])
    chain = _opencv_yaml((out / 'openvins' / 'kalibr_imucam_chain.yaml').read_text())
    assert chain['cam1']['timeshift_cam_imu'] == 0.003
    imu = _opencv_yaml((out / 'openvins' / 'kalibr_imu_chain.yaml').read_text())['imu0']
    assert imu['accelerometer_noise_density'] == 0.002 and imu['update_rate'] == 200.0
    again = _run_kalibr_tool(tmp_path, '--out', str(out))
    assert again.returncode == 1 and 'versioned' in again.stderr


def test_kalibr_to_openvins_refuses_unsynchronized_stereo(tmp_path):
    r = _run_kalibr_tool(tmp_path, '--out', str(tmp_path / 'x'), cam1_timeshift=0.010)
    assert r.returncode == 1 and 'timeshift' in r.stderr and not (tmp_path / 'x').exists()


def test_quat_from_rpy_matches_rotation():
    from ppb_vio_tools.rig import quat_from_rpy_deg, rotation_from_quat_xyzw
    r = rotation_from_quat_xyzw(quat_from_rpy_deg(0, 0, 90))
    assert np.allclose(r @ [1, 0, 0], [0, 1, 0], atol=1e-12)
    r = rotation_from_quat_xyzw(quat_from_rpy_deg(10, -20, 30))
    expect_x = np.array([[1, 0, 0], [0, np.cos(np.radians(10)), -np.sin(np.radians(10))],
                         [0, np.sin(np.radians(10)), np.cos(np.radians(10))]])
    p = np.radians(-20)
    expect_y = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    y = np.radians(30)
    expect_z = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    assert np.allclose(r, expect_z @ expect_y @ expect_x, atol=1e-12)


def _robot_recording(path, width=672, height=376, n=30, odom_frame='map', size_override=None):
    """Synthetic PPBv2 recording: /vio/* at 15 Hz / 150 Hz and /robot/odom at 5 Hz."""
    from rosbags.rosbag2 import Writer
    from rosbags.typesys import get_typestore, Stores
    ts = get_typestore(Stores.ROS2_HUMBLE)
    t = ts.types

    def hdr(ns, frame):
        return t['std_msgs/msg/Header'](stamp=t['builtin_interfaces/msg/Time'](
            sec=ns // 10**9, nanosec=ns % 10**9), frame_id=frame)
    v3 = lambda x, y, z: t['geometry_msgs/msg/Vector3'](x=x, y=y, z=z)   # noqa: E731
    quat = t['geometry_msgs/msg/Quaternion'](x=0.0, y=0.0, z=0.0, w=1.0)
    start = 1_800_000_000_000_000_000
    events = []
    for k in range(n):
        ns = start + k * 66_666_667
        w, h = size_override or (width, height)
        for i in (0, 1):
            events.append((ns, f'/vio/cam{i}/image_raw', t['sensor_msgs/msg/Image'](
                header=hdr(ns, f'cam{i}_optical_frame'), height=h, width=w, encoding='mono8',
                is_bigendian=0, step=w, data=np.zeros(w * h, dtype=np.uint8))))
            events.append((ns, f'/vio/cam{i}/camera_info', t['sensor_msgs/msg/CameraInfo'](
                header=hdr(ns, f'cam{i}_optical_frame'), height=h, width=w,
                distortion_model='plumb_bob', d=np.zeros(5), k=np.zeros(9), r=np.zeros(9),
                p=np.zeros(12), binning_x=0, binning_y=0,
                roi=t['sensor_msgs/msg/RegionOfInterest'](x_offset=0, y_offset=0, height=0,
                                                          width=0, do_rectify=False))))
    for k in range(n * 10):
        ns = start + k * 6_666_667
        events.append((ns, '/vio/imu', t['sensor_msgs/msg/Imu'](
            header=hdr(ns, 'imu_link'), orientation=quat, orientation_covariance=np.zeros(9),
            angular_velocity=v3(0.0, 0.0, 0.0), angular_velocity_covariance=np.zeros(9),
            linear_acceleration=v3(0.0, 0.0, 9.81), linear_acceleration_covariance=np.zeros(9))))
    for k in range(n // 3):
        ns = start + k * 200_000_000
        events.append((ns, '/robot/odom', t['nav_msgs/msg/Odometry'](
            header=hdr(ns, odom_frame), child_frame_id='base_link',
            pose=t['geometry_msgs/msg/PoseWithCovariance'](
                pose=t['geometry_msgs/msg/Pose'](position=t['geometry_msgs/msg/Point'](
                    x=0.2 * k, y=0.0, z=0.0), orientation=quat), covariance=np.zeros(36)),
            twist=t['geometry_msgs/msg/TwistWithCovariance'](
                twist=t['geometry_msgs/msg/Twist'](linear=v3(1.0, 0.0, 0.0),
                                                   angular=v3(0.0, 0.0, 0.0)),
                covariance=np.zeros(36)))))
    events.sort(key=lambda e: e[0])
    with Writer(path, version=8) as writer:
        conns = {}
        for ns, topic, msg in events:
            if topic not in conns:
                conns[topic] = writer.add_connection(topic, msg.__msgtype__, typestore=ts)
            writer.write(conns[topic], ns, ts.serialize_cdr(msg, msg.__msgtype__))
    return path


def _robot_calibration(tmp_path):
    cc, imu = _kalibr_inputs(tmp_path)
    imu.write_text(KALIBR_IMU.replace('update_rate: 200.0', 'update_rate: 150.0'))
    out = tmp_path / 'cal'
    import subprocess
    tool = Path(__file__).resolve().parents[1] / 'kalibr_to_openvins.py'
    r = subprocess.run([sys.executable, '-I', str(tool), '--camchain', str(cc), '--imu', str(imu),
                        '--imu-in-base-xyz', '0.4', '0', '1.1', '--imu-in-base-rpy-deg', '0', '0', '0',
                        '--camera-hz', '15', '--out', str(out)], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    return out


def test_prepare_ppbv2_makes_a_bench_dataset_from_a_robot_recording(tmp_path):
    import prepare_ppbv2
    rec = _robot_recording(tmp_path / 'rec')
    report = prepare_ppbv2.prepare(rec, _robot_calibration(tmp_path), tmp_path / 'ppbv2' / 'row1')
    out = tmp_path / 'ppbv2' / 'row1'
    assert report['camera_frames'] == 30 and report['ground_truth_poses'] == 10
    assert (out / 'ros2_bag').resolve() == rec.resolve()
    assert (out / 'openvins' / 'kalibr_imucam_chain.yaml').is_file() and (out / 'platform.yaml').is_file()
    ds = yaml.safe_load((out / 'dataset.yaml').read_text())
    assert ds['dataset'] == 'ppbv2' and ds['ground_truth_has_orientation'] is False
    gt = np.loadtxt(out / 'ground_truth.tum')
    assert gt.shape == (10, 8) and np.isclose(gt[-1, 1], 1.8)
    with pytest.raises(DatasetError, match='already'):
        prepare_ppbv2.prepare(rec, tmp_path / 'cal', out)


def test_prepare_ppbv2_refuses_image_size_other_than_calibration(tmp_path):
    import prepare_ppbv2
    rec = _robot_recording(tmp_path / 'rec', size_override=(640, 480))
    with pytest.raises(DatasetError, match='image size'):
        prepare_ppbv2.prepare(rec, _robot_calibration(tmp_path), tmp_path / 'out')


def test_continuity_flags_only_physical_jumps():
    from ppb_vio_tools.metrics import continuity
    t = np.arange(0, 10, 0.02)
    p = np.column_stack([1.0 * t, np.zeros_like(t), np.zeros_like(t)])
    assert continuity(t, p)['jumps'] == 0
    p[300:, 1] += 0.8                                    # 0.8 m sideways in one 20 ms step
    c = continuity(t, p)
    assert c['jumps'] == 1 and abs(c['first_jump_s'] - t[300]) < 1e-9 and c['max_step_m'] > 0.8
    tiny = np.column_stack([t, np.zeros_like(t), np.zeros_like(t)])
    tiny[100, 0] += 0.05                                 # sub-threshold noise is no jump
    assert continuity(t, tiny)['jumps'] == 0
