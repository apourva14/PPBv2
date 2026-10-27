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
from ppb_vio_tools.rig import chain_stereo, quat_from_rotation, RigCalibration, STEREO_SOURCES

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


@dataclass(frozen=True)
class RosarioCalibration:
    cameras: tuple              # (CameraCalibration cam0, cam1)
    t_cam_imu: tuple            # 4x4 T_cam_imu per camera (imu -> camera), as used
    imu_in_base_xyz: tuple      # pose of the IMU in base_link
    imu_in_base_quat_xyzw: tuple
    gps_in_base_xyz: tuple      # GPS antenna (the ground-truth point) in base_link
    stereo_source: str = 'camchain'
    stereo_discrepancy_deg: float = 0.0   # file's two T_cam_imu vs its T_cn_cnm1

    def imu_to_camera_pose(self, i):
        """Pose of camera i in the IMU frame: (xyz, quat xyzw), i.e. inv(T_cam_imu)."""
        t = np.asarray(self.t_cam_imu[i], dtype=float)
        rot, trans = t[:3, :3].T, -t[:3, :3].T @ t[:3, 3]
        return trans, quat_from_rotation(rot)


def read_calibration(path, stereo_source='camchain'):
    """Read calibrationNN.yaml. `rotation_imu_baselink` is R_base_imu (checked
    against the bag's tf_static base_link -> imu and against gravity).

    The file's two camera-IMU extrinsics disagree with its own stereo extrinsic
    T_cn_cnm1 by 0.27 deg about the camera y axis (all six sequences share one
    calibration): ~1.6 px of horizontal disparity bias, which made stereo VIO
    11 % short on sequence02. T_cn_cnm1 is the stereo calibration proper (the
    bag's rectification R/P agree with it to < 0.001 deg), so by default cam1's
    extrinsic is cam0's chained with it: T_cam1_imu = T_cn_cnm1 T_cam0_imu.
    stereo_source='imu' keeps the file's cam1 T_cam_imu (the original behavior).
    """
    if stereo_source not in STEREO_SOURCES:
        raise ValueError(f'stereo_source must be one of {STEREO_SOURCES}')
    d = _read_sensor_yaml(path)
    try:
        cams = tuple(read_kalibr_camchain(path, f'cam{i}') for i in (0, 1))
        t_cam_imu = tuple(np.array(d[f'cam{i}']['T_cam_imu'], dtype=float) for i in (0, 1))
        imu_xyz = tuple(float(v) for v in d['imu']['position_imu_baselink'])
        imu_q = tuple(float(v) for v in d['imu']['rotation_imu_baselink'])
        gps_xyz = tuple(float(v) for v in d['gps']['position_gps_baselink'])
        t_cn_cnm1 = np.array(d['cam1']['T_cn_cnm1'], dtype=float)
    except KeyError as exc:
        raise DatasetError(f'{path}: missing calibration entry {exc}') from exc
    if any(t.shape != (4, 4) for t in t_cam_imu):
        raise DatasetError(f'{path}: T_cam_imu must be 4x4')
    if len(imu_xyz) != 3 or len(imu_q) != 4 or len(gps_xyz) != 3:
        raise DatasetError(f'{path}: malformed imu/gps extrinsics')
    if abs(np.linalg.norm(imu_q) - 1.0) > 1e-3:
        raise DatasetError(f'{path}: rotation_imu_baselink is not a unit quaternion')
    chained, disc_deg = chain_stereo(t_cam_imu[0], t_cn_cnm1, t_cam_imu[1])
    if stereo_source == 'camchain':
        t_cam_imu = (t_cam_imu[0], chained)
    return RosarioCalibration(cams, t_cam_imu, imu_xyz, imu_q, gps_xyz, stereo_source, disc_deg)


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


def to_rig(calib, imu_noise=IMU_NOISE, imu_hz=IMU_HZ, camera_hz=CAMERA_HZ):
    """The sequence's camera-IMU rig, for ppb_vio_tools.rig (OpenVINS + platform files)."""
    return RigCalibration(calib.cameras, calib.t_cam_imu, dict(imu_noise), float(imu_hz),
                          float(camera_hz), calib.imu_in_base_xyz, calib.imu_in_base_quat_xyzw,
                          0.0, calib.stereo_source, calib.stereo_discrepancy_deg)
