#!/opt/vio-tools/bin/python -I
"""Convert an ASL-format sequence (EuRoC MAV, TUM-VI exported/euroc) into a ROS 2
bag on the PPBv2 VIO topic contract.

Output topics (identical to what the robot's navigation camera will publish):
    /vio/cam0/image_raw, /vio/cam1/image_raw   sensor_msgs/Image (mono8)
    /vio/cam0/camera_info, /vio/cam1/camera_info  sensor_msgs/CameraInfo (from sensor.yaml)
    /vio/imu                                   sensor_msgs/Imu (rad/s, m/s^2, no orientation)

With --ground-truth (a TUM file tracking the body/IMU frame with orientation,
e.g. OpenVINS' ov_data), it is copied next to the bag as ground_truth.tum and a
dataset.yaml is written for evaluate_run.py. --descriptor-only does just that for
an already converted bag.

Every message's bag time equals its header stamp (the dataset's hardware time).

Calibration comes from each camera's sensor.yaml (EuRoC) or, when the dataset
has none (TUM-VI), from a Kalibr camchain passed with --camchain.

16-bit images (TUM-VI) hold MSB-aligned sensor data and are reduced to mono8 by
keeping the high byte; frames whose maximum is below 256 are counted in the
report because their bit alignment cannot be confirmed.

Usage:
    prepare_asl.py /data/euroc/V1_01_easy/V1_01_easy /data/euroc/V1_01_easy/ros2_bag
    prepare_asl.py /data/tumvi/dataset-room1_512_16/dataset-room1_512_16 \
        /data/tumvi/dataset-room1_512_16/ros2_bag \
        --camchain /opt/vio/openvins_ws/src/open_vins/config/tum_vi/kalibr_imucam_chain.yaml
    prepare_asl.py /data/euroc/V1_01_easy/V1_01_easy /data/euroc/V1_01_easy/ros2_bag \
        --descriptor-only --sequence V1_01_easy \
        --ground-truth /opt/vio/openvins_ws/src/open_vins/ov_data/euroc_mav/V1_01_easy.txt
"""

import argparse
import json
from pathlib import Path
import shutil
import sys

import numpy as np
from rosbags.rosbag2 import StoragePlugin, Writer
from rosbags.typesys import get_typestore, Stores
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ppb_vio_tools.euroc import (  # noqa: E402
    DatasetError, find_mav0, read_camera_calibration, read_camera_index, read_imu,
    read_kalibr_camchain)
from ppb_vio_tools.images import load_image_mono8  # noqa: E402

# rosbags writes metadata version 8 or 9; Humble's rosbag2 rejects 9 ("bad
# conversion" in metadata.yaml) and reads 8 (checked with `ros2 bag info/play`).
HUMBLE_BAG_VERSION = 8
CAM_FRAMES = ('cam0_optical_frame', 'cam1_optical_frame')
IMU_FRAME = 'imu_link'


def build_messages(typestore):
    t = typestore.types
    return (t['builtin_interfaces/msg/Time'], t['std_msgs/msg/Header'],
            t['sensor_msgs/msg/Image'], t['sensor_msgs/msg/CameraInfo'],
            t['sensor_msgs/msg/RegionOfInterest'], t['sensor_msgs/msg/Imu'],
            t['geometry_msgs/msg/Quaternion'], t['geometry_msgs/msg/Vector3'])


def median_rate_hz(stamps_ns):
    return 1e9 / float(np.median(np.diff(np.asarray(stamps_ns, dtype=np.int64))))


def _write_bag(path, events, imu, cams, typestore):
    """Write all events to a new MCAP bag at `path`; returns (counts, ambiguous_16bit)."""
    Time, Header, Image, CameraInfo, Roi, Imu, Quaternion, Vector3 = build_messages(typestore)

    def header(stamp_ns, frame):
        return Header(stamp=Time(sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000),
                      frame_id=frame)

    counts = {}
    ambiguous_16bit = 0
    unknown_cov = np.zeros(9)
    orientation_unknown = np.zeros(9)
    orientation_unknown[0] = -1.0

    with Writer(path, version=HUMBLE_BAG_VERSION, storage_plugin=StoragePlugin.MCAP) as writer:
        conns = {}

        def add(topic, msgtype):
            conns[topic] = (writer.add_connection(topic, msgtype, typestore=typestore), msgtype)

        for i in (0, 1):
            add(f'/vio/cam{i}/image_raw', Image.__msgtype__)
            add(f'/vio/cam{i}/camera_info', CameraInfo.__msgtype__)
        add('/vio/imu', Imu.__msgtype__)

        def write(topic, stamp_ns, msg):
            conn, msgtype = conns[topic]
            writer.write(conn, stamp_ns, typestore.serialize_cdr(msg, msgtype))
            counts[topic] = counts.get(topic, 0) + 1

        for stamp, kind, k in events:
            if kind == 'imu':
                _, w, a = imu[k]
                write('/vio/imu', stamp, Imu(
                    header=header(stamp, IMU_FRAME),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                    orientation_covariance=orientation_unknown,
                    angular_velocity=Vector3(x=w[0], y=w[1], z=w[2]),
                    angular_velocity_covariance=unknown_cov,
                    linear_acceleration=Vector3(x=a[0], y=a[1], z=a[2]),
                    linear_acceleration_covariance=unknown_cov))
                continue

            i = int(kind[-1])
            calib, index = cams[i]
            pixels, ambiguous = load_image_mono8(index[k][1], calib)
            ambiguous_16bit += ambiguous
            hdr = header(stamp, CAM_FRAMES[i])
            write(f'/vio/cam{i}/image_raw', stamp, Image(
                header=hdr, height=calib.height, width=calib.width, encoding='mono8',
                is_bigendian=0, step=calib.width, data=pixels))
            f = calib.camera_info_fields()
            write(f'/vio/cam{i}/camera_info', stamp, CameraInfo(
                header=hdr, height=f['height'], width=f['width'],
                distortion_model=f['distortion_model'], d=np.array(f['d']),
                k=np.array(f['k']), r=np.array(f['r']), p=np.array(f['p']),
                binning_x=0, binning_y=0,
                roi=Roi(x_offset=0, y_offset=0, height=0, width=0, do_rectify=False)))
    return counts, ambiguous_16bit


def convert(asl_root, out_dir, camchain=None):
    out_dir = Path(out_dir)
    if out_dir.exists():
        raise DatasetError(f'output already exists: {out_dir}')

    mav0 = find_mav0(asl_root)
    cams = []
    for i in (0, 1):
        cam_dir = mav0 / f'cam{i}'
        if camchain is not None:
            calib = read_kalibr_camchain(camchain, f'cam{i}')
        else:
            calib = read_camera_calibration(cam_dir / 'sensor.yaml')
        cams.append((calib, read_camera_index(cam_dir)))
    imu = read_imu(mav0 / 'imu0')

    stamps0 = [s for s, _ in cams[0][1]]
    stamps1 = [s for s, _ in cams[1][1]]
    unpaired = len(set(stamps0) ^ set(stamps1))

    events = [(s, 'imu', k) for k, (s, _, _) in enumerate(imu)]
    for i, (_, index) in enumerate(cams):
        events += [(s, f'cam{i}', k) for k, (s, _) in enumerate(index)]
    events.sort(key=lambda e: (e[0], e[1]))

    # Write to a sibling and rename only after success, so an interrupted or
    # failed conversion never leaves a half-written bag at the final path.
    partial = out_dir.with_name(out_dir.name + '.partial')
    if partial.exists():
        shutil.rmtree(partial)   # leftover of an earlier failed conversion
    try:
        counts, ambiguous_16bit = _write_bag(
            partial, events, imu, cams, get_typestore(Stores.ROS2_HUMBLE))
    except BaseException:
        shutil.rmtree(partial, ignore_errors=True)
        raise
    partial.rename(out_dir)

    report = {
        'source': str(mav0.resolve()),
        'bag': str(out_dir.resolve()),
        'storage': 'mcap',
        'bag_metadata_version': HUMBLE_BAG_VERSION,
        'message_counts': counts,
        'unpaired_stereo_stamps': unpaired,
        'start_ns': events[0][0],
        'end_ns': events[-1][0],
        'duration_s': (events[-1][0] - events[0][0]) / 1e9,
        'calibration_source': str(camchain) if camchain else 'sensor.yaml',
        'camera_rate_hz_stated': [c.rate_hz for c, _ in cams],
        'camera_rate_hz_measured': [round(median_rate_hz([s for s, _ in idx]), 3)
                                    for _, idx in cams],
        'imu_rate_hz_measured': round(median_rate_hz([s for s, _, _ in imu]), 3),
        'ambiguous_16bit_frames': ambiguous_16bit,
    }
    (out_dir.parent / 'conversion_report.json').write_text(json.dumps(report, indent=2))
    return report


def write_descriptor(out_dir, sequence, ground_truth):
    """Copy the ground truth next to the bag and write dataset.yaml (EuRoC
    conventions: ground truth tracks base_link = the body/IMU frame, with
    orientation)."""
    out_dir = Path(out_dir)
    if not out_dir.is_dir():
        raise DatasetError(f'no converted bag at {out_dir}')
    rows = np.loadtxt(ground_truth, comments='#', ndmin=2)
    if rows.shape[1] != 8 or np.any(np.diff(rows[:, 0]) <= 0):
        raise DatasetError(f'{ground_truth}: not a TUM file with increasing stamps')
    np.savetxt(out_dir.parent / 'ground_truth.tum', rows, fmt='%.9f')
    (out_dir.parent / 'dataset.yaml').write_text(yaml.safe_dump({
        'sequence': sequence,
        'dataset': 'asl',
        'sensor_bag': out_dir.name,
        'ground_truth': 'ground_truth.tum',
        'ground_truth_source': str(ground_truth),
        'ground_truth_has_orientation': True,
        'ground_truth_point_in_base': [0.0, 0.0, 0.0],
    }, sort_keys=False))


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('asl_root', help='sequence folder (containing mav0/)')
    parser.add_argument('out_dir', help='ROS 2 bag directory to create (must not exist)')
    parser.add_argument('--camchain', help='Kalibr camchain YAML (when there is no sensor.yaml)')
    parser.add_argument('--ground-truth', help='TUM ground truth to write dataset.yaml with')
    parser.add_argument('--sequence', help='sequence name for dataset.yaml (default: folder name)')
    parser.add_argument('--descriptor-only', action='store_true',
                        help='only write dataset.yaml for an existing bag (needs --ground-truth)')
    args = parser.parse_args()
    sequence = args.sequence or Path(args.asl_root).resolve().name
    try:
        if args.descriptor_only:
            if not args.ground_truth:
                parser.error('--descriptor-only needs --ground-truth')
            write_descriptor(args.out_dir, sequence, args.ground_truth)
            print(f'wrote {Path(args.out_dir).parent / "dataset.yaml"}')
            return 0
        report = convert(args.asl_root, args.out_dir, args.camchain)
        if args.ground_truth:
            write_descriptor(args.out_dir, sequence, args.ground_truth)
    except DatasetError as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        return 1
    print(json.dumps(report, indent=2))
    if report['unpaired_stereo_stamps']:
        print(f"WARNING: {report['unpaired_stereo_stamps']} camera stamps have no stereo partner",
              file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())
