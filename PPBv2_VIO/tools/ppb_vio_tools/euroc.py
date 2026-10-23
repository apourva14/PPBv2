"""Readers for the ASL dataset layout (mav0/...) used by EuRoC MAV and the
TUM-VI "exported/euroc" release. Pure Python."""

from dataclasses import dataclass
import csv
from pathlib import Path

import yaml


class DatasetError(RuntimeError):
    pass


# ASL distortion model -> (sensor_msgs/CameraInfo model, coefficients to append).
# radial-tangential (k1 k2 p1 p2) maps to plumb_bob (k1 k2 p1 p2 k3) with k3 = 0;
# equidistant (k1 k2 k3 k4) is the ROS "equidistant" fisheye model unchanged.
DISTORTION_MODELS = {
    'radial-tangential': ('plumb_bob', [0.0]),
    'equidistant': ('equidistant', []),
}


@dataclass(frozen=True)
class CameraCalibration:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    distortion_model: str      # ASL name, a key of DISTORTION_MODELS
    distortion: tuple          # 4 coefficients as listed in sensor.yaml
    rate_hz: object            # float, or None when the source does not state it

    def camera_info_fields(self):
        """sensor_msgs/CameraInfo fields for the unrectified camera."""
        ros_model, extra = DISTORTION_MODELS[self.distortion_model]
        k = [self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0]
        p = [self.fx, 0.0, self.cx, 0.0, 0.0, self.fy, self.cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return {
            'width': self.width,
            'height': self.height,
            'distortion_model': ros_model,
            'd': list(self.distortion) + extra,
            'k': k,
            'r': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            'p': p,
        }


def _read_sensor_yaml(path):
    text = Path(path).read_text()
    # EuRoC files may start with an OpenCV-style "%YAML:1.0" directive.
    lines = [line for line in text.splitlines() if not line.startswith('%YAML')]
    return yaml.safe_load('\n'.join(lines))


def read_camera_calibration(sensor_yaml):
    d = _read_sensor_yaml(sensor_yaml)
    if d.get('camera_model') != 'pinhole':
        raise DatasetError(f'{sensor_yaml}: unsupported camera_model {d.get("camera_model")!r}')
    model = d.get('distortion_model')
    if model not in DISTORTION_MODELS:
        raise DatasetError(f'{sensor_yaml}: unsupported distortion_model {model!r}')
    fx, fy, cx, cy = (float(v) for v in d['intrinsics'])
    width, height = (int(v) for v in d['resolution'])
    dist = tuple(float(v) for v in d['distortion_coefficients'])
    if len(dist) != 4:
        raise DatasetError(f'{sensor_yaml}: expected 4 distortion coefficients, got {len(dist)}')
    return CameraCalibration(width, height, fx, fy, cx, cy, model, dist, float(d['rate_hz']))


# Kalibr spells the radial-tangential model "radtan".
_KALIBR_DISTORTION_NAMES = {'radtan': 'radial-tangential', 'equidistant': 'equidistant'}


def read_kalibr_camchain(camchain_yaml, cam_name):
    """Calibration of `cam_name` from a Kalibr camchain (e.g. kalibr_imucam_chain.yaml)."""
    chain = _read_sensor_yaml(camchain_yaml)
    if cam_name not in chain:
        raise DatasetError(f'{camchain_yaml}: no {cam_name} entry')
    d = chain[cam_name]
    if d.get('camera_model') != 'pinhole':
        raise DatasetError(f'{camchain_yaml}: {cam_name} unsupported camera_model '
                           f'{d.get("camera_model")!r}')
    model = _KALIBR_DISTORTION_NAMES.get(d.get('distortion_model'))
    if model is None:
        raise DatasetError(f'{camchain_yaml}: {cam_name} unsupported distortion_model '
                           f'{d.get("distortion_model")!r}')
    fx, fy, cx, cy = (float(v) for v in d['intrinsics'])
    width, height = (int(v) for v in d['resolution'])
    dist = tuple(float(v) for v in d['distortion_coeffs'])
    if len(dist) != 4:
        raise DatasetError(f'{camchain_yaml}: {cam_name} expected 4 distortion coefficients')
    return CameraCalibration(width, height, fx, fy, cx, cy, model, dist, None)


def _rows(csv_path):
    with open(csv_path, newline='') as f:
        for lineno, row in enumerate(csv.reader(f), start=1):
            if not row or row[0].lstrip().startswith('#'):
                continue
            yield lineno, [c.strip() for c in row]


def _check_increasing(name, stamps):
    for a, b in zip(stamps, stamps[1:]):
        if b <= a:
            raise DatasetError(f'{name}: timestamps not strictly increasing ({a} then {b})')


def read_camera_index(cam_dir):
    """Return [(stamp_ns, image_path)] for mav0/camN, validated."""
    cam_dir = Path(cam_dir)
    entries = []
    for lineno, row in _rows(cam_dir / 'data.csv'):
        if len(row) != 2:
            raise DatasetError(f'{cam_dir}/data.csv:{lineno}: expected 2 columns')
        entries.append((int(row[0]), cam_dir / 'data' / row[1]))
    if not entries:
        raise DatasetError(f'{cam_dir}: no images listed')
    _check_increasing(f'{cam_dir.name}', [s for s, _ in entries])
    missing = [str(p) for _, p in entries if not p.is_file()]
    if missing:
        raise DatasetError(f'{cam_dir}: {len(missing)} listed images missing, e.g. {missing[0]}')
    return entries


def read_imu(imu_dir):
    """Return [(stamp_ns, (wx, wy, wz), (ax, ay, az))] in rad/s and m/s^2."""
    samples = []
    for lineno, row in _rows(Path(imu_dir) / 'data.csv'):
        if len(row) != 7:
            raise DatasetError(f'{imu_dir}/data.csv:{lineno}: expected 7 columns')
        v = [float(x) for x in row[1:]]
        samples.append((int(row[0]), tuple(v[0:3]), tuple(v[3:6])))
    if not samples:
        raise DatasetError(f'{imu_dir}: no IMU samples')
    _check_increasing('imu0', [s for s, _, _ in samples])
    return samples


def find_mav0(root):
    """Accept either the sequence folder or its mav0/ subfolder."""
    root = Path(root)
    for candidate in (root / 'mav0', root):
        if (candidate / 'cam0' / 'data.csv').is_file() and (candidate / 'imu0' / 'data.csv').is_file():
            return candidate
    raise DatasetError(f'{root}: not an EuRoC ASL folder (expected mav0/cam0, mav0/imu0)')
