#!/opt/vio-tools/bin/python -I
"""Make a PPBv2 robot recording a scored VIO dataset (spec, Task 7: record robot datasets,
RTK-fixed = ground truth, rerun the harness).

The recording (ROS 2 bag, recorded on the robot) must hold the navigation camera and IMU
on the /vio/* contract and the RTK pose from um982_driver:

    /vio/cam{0,1}/image_raw, /vio/cam{0,1}/camera_info, /vio/imu
    /robot/odom      nav_msgs/Odometry map -> base_link (published only while RTK is fixed)

    prepare_ppbv2.py RECORDING CALIBRATION OUT_DIR [--gt-topic /robot/odom]
                     [--gt-point-in-base X Y Z]

OUT_DIR (e.g. /data/ppbv2/<name>) becomes a bench dataset: ros2_bag (link to the
recording), ground_truth.tum, dataset.yaml, platform.yaml and openvins/ (from the
calibration folder written by kalibr_to_openvins.py), prepare_report.json. Then:

    scripts/run_bench.sh openvins ppbv2 <name>

Ground truth: /robot/odom positions (orientation is RTK yaw only, so scoring is
position-only). um982_driver reports the dual-antenna midpoint as base_link; if the
calibration's base_link (the IMU mounting reference) is a different point, give that
point's offset with --gt-point-in-base. Camera/IMU stamps (Orin) and /robot/odom stamps
(Pi 5) must come from synchronized clocks (chrony/PTP, spec Task 8); the report flags a
recording whose streams do not overlap.

Refuses recordings with missing topics, stamps going backwards, unpaired stereo
(> 1 %), image sizes other than the calibration's, IMU not measuring gravity in
m/s^2, or rates far from the calibration's.
"""

import argparse
import json
from pathlib import Path
import shutil
import sys

import numpy as np
from rosbags.highlevel import AnyReader
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ppb_vio_tools.euroc import DatasetError  # noqa: E402

CAMERAS = ('/vio/cam0/image_raw', '/vio/cam1/image_raw')
INFOS = ('/vio/cam0/camera_info', '/vio/cam1/camera_info')
IMU = '/vio/imu'
GRAVITY_RANGE = (9.3, 10.3)
RATE_TOLERANCE = 0.2
MAX_UNPAIRED_FRACTION = 0.01


def stamp_s(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def read_recording(bag, gt_topic):
    with AnyReader([Path(bag)]) as reader:
        have = {c.topic: c.msgtype for c in reader.connections}
        need = {**{t: 'sensor_msgs/msg/Image' for t in CAMERAS},
                **{t: 'sensor_msgs/msg/CameraInfo' for t in INFOS},
                IMU: 'sensor_msgs/msg/Imu', gt_topic: 'nav_msgs/msg/Odometry'}
        wrong = {t: have.get(t) for t, ty in need.items() if have.get(t) != ty}
        if wrong:
            raise DatasetError(f'{bag}: missing or mistyped topics {wrong}')
        cams = {t: [] for t in CAMERAS}
        imu, gt, encodings, frames = [], [], set(), set()
        sizes = {t: set() for t in CAMERAS}
        conns = [c for c in reader.connections if c.topic in (*CAMERAS, IMU, gt_topic)]
        for conn, _, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)
            t = stamp_s(msg.header)
            if conn.topic in cams:
                cams[conn.topic].append(t)
                encodings.add(msg.encoding)
                sizes[conn.topic].add((msg.width, msg.height))
            elif conn.topic == IMU:
                a = msg.linear_acceleration
                imu.append((t, a.x, a.y, a.z))
                frames.add(('imu', msg.header.frame_id))
            else:
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                gt.append((t, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
                frames.add(('gt', msg.header.frame_id, msg.child_frame_id))
    return cams, np.array(imu), np.array(gt), encodings, frames, sizes


def median_rate(stamps):
    d = np.diff(np.asarray(stamps, dtype=float))
    return float(1.0 / np.median(d)) if d.size else 0.0


def calibration_resolutions(calibration):
    text = (Path(calibration) / 'openvins' / 'kalibr_imucam_chain.yaml').read_text()
    chain = yaml.safe_load('\n'.join(l for l in text.splitlines() if not l.startswith('%YAML')))
    return [tuple(chain[f'cam{i}']['resolution']) for i in (0, 1)]


def validate(cams, imu, gt, encodings, frames, calib_report, sizes=None, resolutions=None):
    errors, warnings = [], []
    for topic, expected in zip(CAMERAS, resolutions or []):
        if sizes and sizes[topic] != {expected}:
            errors.append(f'{topic}: image size {sorted(sizes[topic])} != calibration {expected}')
    left, right = (cams[t] for t in CAMERAS)
    for name, stamps in (('cam0', left), ('cam1', right), ('imu', imu[:, 0] if len(imu) else []),
                         ('ground truth', gt[:, 0] if len(gt) else [])):
        if len(stamps) < 2:
            errors.append(f'{name}: fewer than 2 messages')
        elif np.any(np.diff(stamps) <= 0):
            errors.append(f'{name}: stamps not strictly increasing')
    unpaired = len(set(left) ^ set(right))
    if unpaired > MAX_UNPAIRED_FRACTION * max(len(left), 1):
        errors.append(f'{unpaired} camera stamps without an identical stereo partner '
                      '(the stereo pair must be hardware-triggered)')
    if encodings - {'mono8'}:
        errors.append(f'camera encodings {sorted(encodings)}; the /vio/* contract is mono8')
    accel = float(np.median(np.linalg.norm(imu[:, 1:4], axis=1))) if len(imu) else 0.0
    if not GRAVITY_RANGE[0] <= accel <= GRAVITY_RANGE[1]:
        errors.append(f'median |accel| {accel:.2f}: IMU not in m/s^2 or not measuring gravity')
    rates = {'camera_hz': median_rate(left), 'imu_hz': median_rate(imu[:, 0]) if len(imu) else 0.0}
    for key, measured in rates.items():
        expected = calib_report[key]
        if abs(measured - expected) > RATE_TOLERANCE * expected:
            errors.append(f'{key} {measured:.1f} vs calibration {expected:g} (> 20 % off)')
    gt_frames = {f[1:] for f in frames if f[0] == 'gt'}
    if gt_frames and gt_frames != {('map', 'base_link')}:
        warnings.append(f'ground-truth frames {sorted(gt_frames)}, expected map -> base_link')
    if len(gt) >= 2 and len(left) >= 2:
        overlap = min(gt[-1, 0], left[-1]) - max(gt[0, 0], left[0])
        if overlap <= 0:
            errors.append('ground truth and camera stamps do not overlap: '
                          'are the Orin and Pi 5 clocks synchronized?')
    return errors, warnings, unpaired, accel, rates


def prepare(recording, calibration, out_dir, gt_topic='/robot/odom', gt_point_in_base=(0, 0, 0)):
    recording, calibration, out_dir = Path(recording), Path(calibration), Path(out_dir)
    if (out_dir / 'dataset.yaml').exists():
        raise DatasetError(f'{out_dir} is already a dataset')
    report_path = calibration / 'calibration_report.json'
    if not report_path.is_file() or not (calibration / 'openvins').is_dir():
        raise DatasetError(f'{calibration}: not a calibration folder (tools/kalibr_to_openvins.py)')
    calib_report = json.loads(report_path.read_text())
    cams, imu, gt, encodings, frames, sizes = read_recording(recording, gt_topic)
    errors, warnings, unpaired, accel, rates = validate(
        cams, imu, gt, encodings, frames, calib_report, sizes, calibration_resolutions(calibration))
    if errors:
        raise DatasetError('; '.join(errors))

    out_dir.mkdir(parents=True, exist_ok=True)
    link = out_dir / 'ros2_bag'
    if not link.exists():
        link.symlink_to(recording.resolve())
    np.savetxt(out_dir / 'ground_truth.tum', gt, fmt='%.9f')
    shutil.copytree(calibration / 'openvins', out_dir / 'openvins', dirs_exist_ok=True)
    shutil.copy2(calibration / 'platform.yaml', out_dir / 'platform.yaml')
    (out_dir / 'dataset.yaml').write_text(yaml.safe_dump({
        'sequence': out_dir.name,
        'dataset': 'ppbv2',
        'sensor_bag': 'ros2_bag',
        'ground_truth': 'ground_truth.tum',
        'ground_truth_source': f'{gt_topic} (RTK fixed, um982_driver)',
        # RTK gives yaw only (roll/pitch are not measured): score positions.
        'ground_truth_has_orientation': False,
        'ground_truth_point_in_base': [float(v) for v in gt_point_in_base],
        'calibration': str(calibration),
    }, sort_keys=False))
    path_len = float(np.sum(np.linalg.norm(np.diff(gt[:, 1:3], axis=0), axis=1)))
    report = {
        'recording': str(recording), 'calibration': str(calibration),
        'camera_frames': len(cams[CAMERAS[0]]), 'imu_samples': int(len(imu)),
        'ground_truth_poses': int(len(gt)), 'ground_truth_path_m': path_len,
        'unpaired_stereo_stamps': unpaired, 'imu_median_accel_norm_mps2': accel,
        'rates_measured': rates, 'warnings': warnings,
    }
    (out_dir / 'prepare_report.json').write_text(json.dumps(report, indent=2))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('recording', help='robot ROS 2 bag')
    parser.add_argument('calibration', help='calibration folder from kalibr_to_openvins.py')
    parser.add_argument('out_dir', help='dataset folder to create, e.g. /data/ppbv2/<name>')
    parser.add_argument('--gt-topic', default='/robot/odom')
    parser.add_argument('--gt-point-in-base', type=float, nargs=3, default=(0.0, 0.0, 0.0),
                        metavar=('X', 'Y', 'Z'))
    args = parser.parse_args()
    try:
        report = prepare(args.recording, args.calibration, args.out_dir, args.gt_topic,
                         args.gt_point_in_base)
    except DatasetError as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        return 1
    print(json.dumps(report, indent=2))
    for w in report['warnings']:
        print(f'WARNING: {w}', file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())
