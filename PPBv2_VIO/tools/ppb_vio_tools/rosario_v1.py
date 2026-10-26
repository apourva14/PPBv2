"""The Rosario Dataset (v1, Pire et al., IJRR 2019): soybean field, ZED stereo
672x376 @ 15 Hz, IMU ~142 Hz, wheel odometry 10 Hz, RTK GPS 5 Hz. Pure Python.

Distribution (http://www.cifasis-conicet.gov.ar/robot/), per sequence NN:
    sequenceNN.bag[.00, .01, ...]  ROS 1 bag, bz2 chunks, split into parts
    calibrationNN.yaml             Kalibr camchain (cam0/cam1) + base_link extrinsics
    sequenceNN_gt.txt              ground truth: GPS-RTK antenna positions, TUM
                                   format, identity orientation (none measured)

Bag contents, checked on sequence02: /stereo/{left,right}/image_raw (bgr8),
/stereo/{left,right}/camera_info, /imu (SI units, frame "imu", no orientation),
/odom (nav_msgs/Odometry odom -> base_link), /gps/fix, /gps/vel, tf_static.
Header stamps equal bag times.
"""

from dataclasses import dataclass
import hashlib
from pathlib import Path

import numpy as np

from ppb_vio_tools.euroc import _read_sensor_yaml, DatasetError, read_kalibr_camchain

BASE_URL = 'http://fs01.cifasis-conicet.gov.ar:90/~robot_desmalezador/robot'

# MD5 of the complete ROS 1 bag (parts concatenated), as published by CIFASIS.
SEQUENCES = {
    1: {'parts': 3, 'md5': '392f65c2188bbf84a980590c393ba731'},
    2: {'parts': 2, 'md5': '0777f5440b2cf25e8cc5dda3110e2888'},
    3: {'parts': 1, 'md5': '0edaa1a2b1bfef7aa7e04e7e7bf42ab3'},
    4: {'parts': 1, 'md5': '86e162fe26c19813ac347d49de072a9e'},
    5: {'parts': 2, 'md5': 'c954c68c490732a3bfe5bd8439a63e9e'},
    6: {'parts': 2, 'md5': '53df51601dcb1476e1cf910350796e58'},
}

LEFT, RIGHT = '/stereo/left/image_raw', '/stereo/right/image_raw'
IMU, ODOM = '/imu', '/odom'
REQUIRED_TOPICS = {
    LEFT: 'sensor_msgs/msg/Image',
    RIGHT: 'sensor_msgs/msg/Image',
    IMU: 'sensor_msgs/msg/Imu',
    ODOM: 'nav_msgs/msg/Odometry',
}


def sequence_files(number):
    """File names for sequence `number`: (bag parts, calibration, ground truth)."""
    if number not in SEQUENCES:
        raise DatasetError(f'Rosario v1 has sequences 1-6, not {number}')
    nn = f'{number:02d}'
    count = SEQUENCES[number]['parts']
    parts = ([f'sequence{nn}.bag'] if count == 1
             else [f'sequence{nn}.bag.{k:02d}' for k in range(count)])
    return parts, f'calibration{nn}.yaml', f'sequence{nn}_gt.txt'


def file_md5(path, chunk_bytes=16 << 20):
    md5 = hashlib.md5()
    with open(path, 'rb') as f:
        while block := f.read(chunk_bytes):
            md5.update(block)
    return md5.hexdigest()


def assemble_bag(part_paths, out_path, expected_md5, chunk_bytes=16 << 20):
    """Concatenate bag parts into `out_path`, verifying the published MD5.

    Writes a sibling file and renames it only when the checksum matches, so a
    corrupt or interrupted download never leaves a bag at `out_path`.
    """
    out_path = Path(out_path)
    partial = out_path.with_name(out_path.name + '.partial')
    md5 = hashlib.md5()
    try:
        with open(partial, 'wb') as out:
            for part in part_paths:
                with open(part, 'rb') as f:
                    while block := f.read(chunk_bytes):
                        md5.update(block)
                        out.write(block)
        if md5.hexdigest() != expected_md5:
            raise DatasetError(f'{out_path.name}: MD5 {md5.hexdigest()} != published {expected_md5} '
                               '(incomplete or corrupt download)')
    except BaseException:
        partial.unlink(missing_ok=True)
        raise
    partial.rename(out_path)
    return md5.hexdigest()


def _rotation_from_quat_xyzw(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


@dataclass(frozen=True)
class RosarioCalibration:
    cameras: tuple              # (CameraCalibration cam0, cam1)
    t_cam_imu: tuple            # 4x4 Kalibr T_cam_imu per camera (imu -> camera)
    imu_in_base_xyz: tuple      # pose of the IMU in base_link
    imu_in_base_quat_xyzw: tuple
    gps_in_base_xyz: tuple      # GPS antenna (the ground-truth point) in base_link

    def imu_to_camera_pose(self, i):
        """Pose of camera i in the IMU frame: (xyz, quat xyzw), i.e. inv(T_cam_imu)."""
        t = np.asarray(self.t_cam_imu[i], dtype=float)
        rot, trans = t[:3, :3].T, -t[:3, :3].T @ t[:3, 3]
        return trans, _quat_from_rotation(rot)


def _quat_from_rotation(m):
    w = np.sqrt(max(0.0, 1 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    x = np.copysign(np.sqrt(max(0.0, 1 + m[0, 0] - m[1, 1] - m[2, 2])) / 2, m[2, 1] - m[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1 - m[0, 0] + m[1, 1] - m[2, 2])) / 2, m[0, 2] - m[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1 - m[0, 0] - m[1, 1] + m[2, 2])) / 2, m[1, 0] - m[0, 1])
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def read_calibration(path):
    """Read calibrationNN.yaml. `rotation_imu_baselink` is R_base_imu (checked
    against the bag's tf_static base_link -> imu and against gravity)."""
    d = _read_sensor_yaml(path)
    try:
        cams = tuple(read_kalibr_camchain(path, f'cam{i}') for i in (0, 1))
        t_cam_imu = tuple(np.array(d[f'cam{i}']['T_cam_imu'], dtype=float) for i in (0, 1))
        imu_xyz = tuple(float(v) for v in d['imu']['position_imu_baselink'])
        imu_q = tuple(float(v) for v in d['imu']['rotation_imu_baselink'])
        gps_xyz = tuple(float(v) for v in d['gps']['position_gps_baselink'])
    except KeyError as exc:
        raise DatasetError(f'{path}: missing calibration entry {exc}') from exc
    if any(t.shape != (4, 4) for t in t_cam_imu):
        raise DatasetError(f'{path}: T_cam_imu must be 4x4')
    if len(imu_xyz) != 3 or len(imu_q) != 4 or len(gps_xyz) != 3:
        raise DatasetError(f'{path}: malformed imu/gps extrinsics')
    if abs(np.linalg.norm(imu_q) - 1.0) > 1e-3:
        raise DatasetError(f'{path}: rotation_imu_baselink is not a unit quaternion')
    return RosarioCalibration(cams, t_cam_imu, imu_xyz, imu_q, gps_xyz)


def bgr8_to_mono8(data, width, height, step):
    """Grayscale exactly as OpenCV's cvtColor(BGR2GRAY) (what cv_bridge does for
    an estimator asking for mono8): fixed-point weights, round half up."""
    rows = (np.asarray(data, dtype=np.uint8).reshape(-1) if isinstance(data, np.ndarray)
            else np.frombuffer(bytes(data), dtype=np.uint8))
    if rows.size != step * height or step < 3 * width:
        raise DatasetError(f'bgr8 image size {rows.size} does not match {width}x{height} '
                           f'step {step}')
    bgr = rows.reshape(height, step)[:, :3 * width].reshape(height, width, 3).astype(np.uint32)
    gray = (bgr[..., 0] * 1868 + bgr[..., 1] * 9617 + bgr[..., 2] * 4899 + (1 << 13)) >> 14
    return gray.astype(np.uint8).reshape(-1)


def read_ground_truth(path):
    """Return (N, 8) TUM rows; validates increasing stamps and identity orientation."""
    rows = np.loadtxt(path, ndmin=2)
    if rows.shape[1] != 8:
        raise DatasetError(f'{path}: expected 8 columns (TUM), got {rows.shape[1]}')
    if np.any(np.diff(rows[:, 0]) <= 0):
        raise DatasetError(f'{path}: ground-truth stamps not strictly increasing')
    if not np.allclose(rows[:, 4:8], [0, 0, 0, 1]):
        raise DatasetError(f'{path}: expected identity orientation (position-only ground truth)')
    return rows


# IMU noise (continuous time). The dataset paper's Allan values (sigma_g 8.27 ...)
# are not in SI units; these are the values CIFASIS used for VINS-Fusion on
# Rosario in the VIO evaluation (CIFASIS/VINS-Fusion@ba42bf1,
# config/rosario/rosario_stereo_imu_config.yaml: acc_n, gyr_n, acc_w, gyr_w).
IMU_NOISE = {
    'accelerometer_noise_density': 0.1,       # m/s^2/sqrt(Hz)
    'gyroscope_noise_density': 0.01,          # rad/s/sqrt(Hz)
    'accelerometer_random_walk': 0.001,       # m/s^3/sqrt(Hz)
    'gyroscope_random_walk': 0.0001,          # rad/s^2/sqrt(Hz)
}
CAMERA_HZ, IMU_HZ = 15.0, 142.0   # nominal; measured 15.385 and 141.9 on sequence02


def _row(values):
    return '[' + ', '.join(repr(float(v)) for v in values) + ']'


def _matrix(m, indent):
    return ''.join(f'\n{indent}- {_row(r)}' for r in np.asarray(m))


def openvins_calibration_files(calib, imu_noise=IMU_NOISE, imu_hz=IMU_HZ):
    """{file name: text} for OpenVINS' kalibr_imucam_chain.yaml / kalibr_imu_chain.yaml,
    on the /vio/* topics. Written from the dataset calibration (not upstream files)."""
    cams = ['%YAML:1.0\n']
    for i, cam in enumerate(calib.cameras):
        xyz, q = calib.imu_to_camera_pose(i)
        t_imu_cam = np.eye(4)
        t_imu_cam[:3, :3] = _rotation_from_quat_xyzw(q)
        t_imu_cam[:3, 3] = xyz
        cams.append(
            f'cam{i}:\n'
            f'  T_imu_cam: # camera pose in the IMU frame (inverse of Kalibr T_cam_imu)'
            f'{_matrix(t_imu_cam, "    ")}\n'
            f'  cam_overlaps: [{1 - i}]\n'
            f'  camera_model: pinhole\n'
            f'  distortion_coeffs: {_row(cam.distortion)}\n'
            f'  distortion_model: radtan\n'
            f'  intrinsics: {_row([cam.fx, cam.fy, cam.cx, cam.cy])}\n'
            f'  resolution: [{cam.width}, {cam.height}]\n'
            f'  rostopic: /vio/cam{i}/image_raw\n')
    eye, zero = np.eye(3), np.zeros((3, 3))
    imu = (
        '%YAML:1.0\n\nimu0:\n'
        f'  T_i_b:{_matrix(np.eye(4), "    ")}\n'
        + ''.join(f'  {k}: {v!r}\n' for k, v in imu_noise.items())
        + '  rostopic: /vio/imu\n'
        '  time_offset: 0.0\n'
        f'  update_rate: {float(imu_hz)!r}\n'
        '  model: "kalibr"\n'
        f'  Tw:{_matrix(eye, "    ")}\n'
        f'  R_IMUtoGYRO:{_matrix(eye, "    ")}\n'
        f'  Ta:{_matrix(eye, "    ")}\n'
        f'  R_IMUtoACC:{_matrix(eye, "    ")}\n'
        f'  Tg:{_matrix(zero, "    ")}\n')
    return {'kalibr_imucam_chain.yaml': '\n'.join(cams), 'kalibr_imu_chain.yaml': imu}


def platform_yaml(calib, camera_hz=CAMERA_HZ, imu_hz=IMU_HZ):
    """ppb_vio platform parameters (adapter + stream monitor) for this sequence."""
    return (
        '# Rosario v1 platform, generated by prepare_rosario_v1.py from calibrationNN.yaml.\n'
        'vio_odom_adapter:\n  ros__parameters:\n'
        '    world_frame: vio_world\n    base_frame: base_link\n    imu_frame: imu_link\n'
        f'    imu_in_base_xyz: {_row(calib.imu_in_base_xyz)}\n'
        f'    imu_in_base_quat_xyzw: {_row(calib.imu_in_base_quat_xyzw)}\n'
        '    stale_timeout_s: 0.5\n\n'
        'vio_stream_monitor:\n  ros__parameters:\n'
        '    camera_topics: [/vio/cam0/image_raw, /vio/cam1/image_raw]\n'
        '    camera_info_topics: [/vio/cam0/camera_info, /vio/cam1/camera_info]\n'
        '    imu_topic: /vio/imu\n'
        f'    camera_hz: {float(camera_hz)!r}\n'
        f'    imu_hz: {float(imu_hz)!r}\n'
        '    rate_tolerance: 0.2\n'
        '    stereo_max_skew_s: 0.001\n'
        '    expected_imu_frame: imu_link\n')
