"""Stereo camera + IMU rig calibration -> OpenVINS config and ppb_vio platform file.

One code path for every rig the VIO runs on: the PPBv2 navigation camera after
its Kalibr calibration (tools/kalibr_to_openvins.py), and the datasets used to
prove the pipeline before the hardware exists (Rosario v1). Pure Python.

Conventions: Kalibr T_cam_imu maps IMU-frame points into the camera frame;
OpenVINS T_imu_cam is its inverse (camera pose in the IMU frame). Kalibr
timeshift_cam_imu: t_imu = t_cam + shift.
"""

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from ppb_vio_tools.euroc import _read_sensor_yaml, DatasetError, read_kalibr_camchain

STEREO_SOURCES = ('camchain', 'imu')
# Above this the camchain's own stereo extrinsic and its two camera-IMU extrinsics
# disagree enough to bias stereo depth (Rosario v1: 0.29 deg -> VIO 11 % short).
STEREO_DISCREPANCY_WARN_DEG = 0.05
IMU_NOISE_KEYS = ('accelerometer_noise_density', 'gyroscope_noise_density',
                  'accelerometer_random_walk', 'gyroscope_random_walk')
_OPENVINS_DISTORTION = {'radial-tangential': 'radtan', 'equidistant': 'equidistant'}


def rotation_from_quat_xyzw(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quat_from_rotation(m):
    w = np.sqrt(max(0.0, 1 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    x = np.copysign(np.sqrt(max(0.0, 1 + m[0, 0] - m[1, 1] - m[2, 2])) / 2, m[2, 1] - m[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1 - m[0, 0] + m[1, 1] - m[2, 2])) / 2, m[0, 2] - m[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1 - m[0, 0] - m[1, 1] + m[2, 2])) / 2, m[1, 0] - m[0, 1])
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def quat_from_rpy_deg(roll, pitch, yaw):
    r, p, y = np.radians([roll, pitch, yaw])
    cr, sr, cp, sp, cy, sy = np.cos(r / 2), np.sin(r / 2), np.cos(p / 2), np.sin(p / 2), \
        np.cos(y / 2), np.sin(y / 2)
    return np.array([sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
                     cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy])


def chain_stereo(t_cam0_imu, t_cn_cnm1, t_cam1_imu_given):
    """cam1's extrinsic from the stereo calibration: T_cam1_imu = T_cn_cnm1 T_cam0_imu.

    Returns (chained 4x4, rotation discrepancy in degrees vs the given cam1 T_cam_imu).
    The rotation is re-projected onto SO(3) (inputs are printed to ~7 digits).
    """
    chained = np.asarray(t_cn_cnm1, dtype=float) @ np.asarray(t_cam0_imu, dtype=float)
    u, _, vt = np.linalg.svd(chained[:3, :3])
    chained[:3, :3] = u @ vt
    d = chained[:3, :3] @ np.asarray(t_cam1_imu_given, dtype=float)[:3, :3].T
    return chained, float(np.degrees(np.arccos(np.clip((np.trace(d) - 1) / 2, -1.0, 1.0))))


@dataclass(frozen=True)
class RigCalibration:
    cameras: tuple                 # CameraCalibration per camera (ppb_vio_tools.euroc)
    t_cam_imu: tuple               # 4x4 per camera, as used (imu -> camera)
    imu_noise: dict                # IMU_NOISE_KEYS, continuous time
    imu_hz: float
    camera_hz: float
    imu_in_base_xyz: tuple         # measured mounting: pose of the IMU in base_link
    imu_in_base_quat_xyzw: tuple
    timeshift_cam_imu: float = 0.0
    stereo_source: str = 'camchain'
    stereo_discrepancy_deg: float = 0.0

    def imu_to_camera_pose(self, i):
        """Pose of camera i in the IMU frame: (xyz, quat xyzw), i.e. inv(T_cam_imu)."""
        t = np.asarray(self.t_cam_imu[i], dtype=float)
        rot, trans = t[:3, :3].T, -t[:3, :3].T @ t[:3, 3]
        return trans, quat_from_rotation(rot)


def read_kalibr_imu(path):
    """(noise dict, update_rate or None) from a Kalibr / allan_variance_ros IMU YAML
    (keys at top level or under imu0)."""
    d = _read_sensor_yaml(path)
    d = d.get('imu0', d)
    missing = [k for k in IMU_NOISE_KEYS if k not in d]
    if missing:
        raise DatasetError(f'{path}: missing {missing}')
    noise = {k: float(d[k]) for k in IMU_NOISE_KEYS}
    if any(v <= 0 for v in noise.values()):
        raise DatasetError(f'{path}: IMU noise values must be positive')
    return noise, (float(d['update_rate']) if 'update_rate' in d else None)


def read_kalibr_rig(camchain_imucam, imu_noise, imu_hz, camera_hz, imu_in_base_xyz,
                    imu_in_base_quat_xyzw, stereo_source='camchain'):
    """Rig from a Kalibr camchain-imucam YAML (two pinhole cameras, cam1 with T_cn_cnm1)."""
    if stereo_source not in STEREO_SOURCES:
        raise ValueError(f'stereo_source must be one of {STEREO_SOURCES}')
    d = _read_sensor_yaml(camchain_imucam)
    try:
        cams = tuple(read_kalibr_camchain(camchain_imucam, f'cam{i}') for i in (0, 1))
        t_cam_imu = [np.array(d[f'cam{i}']['T_cam_imu'], dtype=float) for i in (0, 1)]
        t_cn_cnm1 = np.array(d['cam1']['T_cn_cnm1'], dtype=float)
    except KeyError as exc:
        raise DatasetError(f'{camchain_imucam}: missing {exc} (need a stereo camchain-imucam '
                           'from kalibr_calibrate_imu_camera)') from exc
    if any(t.shape != (4, 4) for t in (*t_cam_imu, t_cn_cnm1)):
        raise DatasetError(f'{camchain_imucam}: transforms must be 4x4')
    shifts = {float(d[f'cam{i}'].get('timeshift_cam_imu', 0.0)) for i in (0, 1)}
    if len(shifts) != 1:
        raise DatasetError(f'{camchain_imucam}: cameras have different timeshift_cam_imu {shifts} '
                           '(stereo frames must be hardware-synchronized)')
    q = np.asarray(imu_in_base_quat_xyzw, dtype=float)
    if q.shape != (4,) or abs(np.linalg.norm(q) - 1.0) > 1e-3:
        raise DatasetError('imu_in_base_quat_xyzw must be a unit quaternion (x, y, z, w)')
    chained, disc = chain_stereo(t_cam_imu[0], t_cn_cnm1, t_cam_imu[1])
    if stereo_source == 'camchain':
        t_cam_imu[1] = chained
    return RigCalibration(cams, tuple(t_cam_imu), dict(imu_noise), float(imu_hz), float(camera_hz),
                          tuple(float(v) for v in imu_in_base_xyz), tuple(float(v) for v in q),
                          shifts.pop(), stereo_source, disc)


def _row(values):
    return '[' + ', '.join(repr(float(v)) for v in values) + ']'


def _matrix(m, indent):
    return ''.join(f'\n{indent}- {_row(r)}' for r in np.asarray(m))


def openvins_files(rig):
    """{file name: text}: OpenVINS kalibr_imucam_chain.yaml and kalibr_imu_chain.yaml on
    the /vio/* topics, written from the rig calibration (not from upstream files)."""
    cams = ['%YAML:1.0\n']
    for i, cam in enumerate(rig.cameras):
        xyz, q = rig.imu_to_camera_pose(i)
        t_imu_cam = np.eye(4)
        t_imu_cam[:3, :3] = rotation_from_quat_xyzw(q)
        t_imu_cam[:3, 3] = xyz
        cams.append(
            f'cam{i}:\n'
            f'  T_imu_cam: # camera pose in the IMU frame (inverse of Kalibr T_cam_imu)'
            f'{_matrix(t_imu_cam, "    ")}\n'
            f'  cam_overlaps: [{1 - i}]\n'
            f'  camera_model: pinhole\n'
            f'  distortion_coeffs: {_row(cam.distortion)}\n'
            f'  distortion_model: {_OPENVINS_DISTORTION[cam.distortion_model]}\n'
            f'  intrinsics: {_row([cam.fx, cam.fy, cam.cx, cam.cy])}\n'
            f'  resolution: [{cam.width}, {cam.height}]\n'
            f'  rostopic: /vio/cam{i}/image_raw\n'
            f'  timeshift_cam_imu: {float(rig.timeshift_cam_imu)!r}\n')
    eye, zero = np.eye(3), np.zeros((3, 3))
    imu = (
        '%YAML:1.0\n\nimu0:\n'
        f'  T_i_b:{_matrix(np.eye(4), "    ")}\n'
        + ''.join(f'  {k}: {float(rig.imu_noise[k])!r}\n' for k in IMU_NOISE_KEYS)
        + '  rostopic: /vio/imu\n'
        '  time_offset: 0.0\n'
        f'  update_rate: {float(rig.imu_hz)!r}\n'
        '  model: "kalibr"\n'
        f'  Tw:{_matrix(eye, "    ")}\n'
        f'  R_IMUtoGYRO:{_matrix(eye, "    ")}\n'
        f'  Ta:{_matrix(eye, "    ")}\n'
        f'  R_IMUtoACC:{_matrix(eye, "    ")}\n'
        f'  Tg:{_matrix(zero, "    ")}\n')
    return {'kalibr_imucam_chain.yaml': '\n'.join(cams), 'kalibr_imu_chain.yaml': imu}


def platform_yaml(rig, header, health_limits=None):
    """ppb_vio platform parameters (vio_odom_adapter + vio_stream_monitor).

    `health_limits`: optional {name: value} for the divergence gate
    (ppb_vio.lib.vio_health.HealthLimits fields), written as health_<name>.
    """
    health = ''.join(f'    health_{k}: {float(v)!r}\n' for k, v in (health_limits or {}).items())
    return (
        ''.join(f'# {line}\n' for line in header.splitlines())
        + 'vio_odom_adapter:\n  ros__parameters:\n'
        '    world_frame: vio_world\n    base_frame: base_link\n    imu_frame: imu_link\n'
        f'    imu_in_base_xyz: {_row(rig.imu_in_base_xyz)}\n'
        f'    imu_in_base_quat_xyzw: {_row(rig.imu_in_base_quat_xyzw)}\n'
        '    stale_timeout_s: 0.5\n'
        '    # Calibrate from recordings: evaluate_run.py reports suggested_covariance_scale.\n'
        '    covariance_scale: 1.0\n'
        + health + '\n'
        'vio_stream_monitor:\n  ros__parameters:\n'
        '    camera_topics: [/vio/cam0/image_raw, /vio/cam1/image_raw]\n'
        '    camera_info_topics: [/vio/cam0/camera_info, /vio/cam1/camera_info]\n'
        '    imu_topic: /vio/imu\n'
        f'    camera_hz: {float(rig.camera_hz)!r}\n'
        f'    imu_hz: {float(rig.imu_hz)!r}\n'
        '    rate_tolerance: 0.2\n'
        '    stereo_max_skew_s: 0.001\n'
        '    expected_imu_frame: imu_link\n')


def write_rig_outputs(rig, out_dir, header, health_limits=None):
    """Write <out>/openvins/*.yaml and <out>/platform.yaml; return the written paths."""
    out_dir = Path(out_dir)
    ov_dir = out_dir / 'openvins'
    ov_dir.mkdir(parents=True, exist_ok=True)
    paths = []
    for name, text in openvins_files(rig).items():
        (ov_dir / name).write_text(text)
        paths.append(ov_dir / name)
    (out_dir / 'platform.yaml').write_text(platform_yaml(rig, header, health_limits))
    paths.append(out_dir / 'platform.yaml')
    return [str(p) for p in paths]
