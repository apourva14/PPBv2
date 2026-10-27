#!/opt/vio-tools/bin/python -I
"""Convert one Rosario v1 sequence (ROS 1 bag) into a ROS 2 bag on the PPBv2
topic contract, plus the ground truth and a dataset.yaml for evaluate_run.py.

Input: a folder with the downloaded files of sequence NN (scripts/fetch_rosario_v1.sh):
    sequenceNN.bag[.00 ...]   calibrationNN.yaml   sequenceNN_gt.txt

Output folder:
    ros2_bag/                 MCAP, metadata v8 (Humble)
        /vio/cam0/image_raw, /vio/cam1/image_raw    sensor_msgs/Image mono8 (from bgr8,
                                                    converted exactly as cv_bridge would)
        /vio/cam0/camera_info, /vio/cam1/camera_info from calibrationNN.yaml (Kalibr)
        /vio/imu              sensor_msgs/Imu, frame imu_link, no orientation
        /wheel/odom           nav_msgs/Odometry odom -> base_link, as recorded
        /tf_static            base_link -> imu_link -> cam{0,1}_optical_frame, base_link -> gps
    ground_truth.tum          GPS antenna position, identity orientation
    dataset.yaml              ground-truth conventions for evaluate_run.py
    platform.yaml             ppb_vio adapter + stream monitor parameters (base_link -> imu)
    openvins/                 OpenVINS calibration files for this sequence (launch with
                              calibration_dir:=<out>/openvins)
    conversion_report.json    counts, rates, and the validation checks below

Refuses to produce output if a required topic is missing, IMU or camera stamps
go backwards, stereo frames are unpaired (> 1 %), or the IMU does not measure
gravity in m/s^2. Every message's bag time equals its header stamp.

The split parts are verified against the published MD5 when they are joined;
--delete-parts removes them afterwards. The joined ROS 1 bag is kept (it is the
source; re-downloading takes long) unless --delete-source is given.

Usage:
    prepare_rosario_v1.py 2 /data/rosario_v1/sequence02/download /data/rosario_v1/sequence02
    # regenerate only platform.yaml and the estimator calibration files:
    prepare_rosario_v1.py 2 /data/rosario_v1/sequence02/download /data/rosario_v1/sequence02 \
        --configs-only
"""

import argparse
import json
from pathlib import Path
import shutil
import sys

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.interfaces import (
    Qos, QosDurability, QosHistory, QosLiveliness, QosReliability, QosTime)
from rosbags.rosbag2 import StoragePlugin, Writer
from rosbags.typesys import get_typestore, Stores
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ppb_vio_tools.euroc import DatasetError  # noqa: E402
from ppb_vio_tools.rosario_v1 import (  # noqa: E402
    assemble_bag, bgr8_to_mono8, file_md5, IMU, LEFT, ODOM, read_calibration, read_ground_truth,
    REQUIRED_TOPICS, RIGHT, SEQUENCES, sequence_files, to_rig)
from ppb_vio_tools.rig import write_rig_outputs  # noqa: E402

HUMBLE_BAG_VERSION = 8      # see prepare_asl.py
CAM_TOPICS = {LEFT: 0, RIGHT: 1}
CAM_FRAMES = ('cam0_optical_frame', 'cam1_optical_frame')
IMU_FRAME, GPS_FRAME = 'imu_link', 'gps'
MAX_UNPAIRED_FRACTION = 0.01
GRAVITY_RANGE = (9.3, 10.3)            # median |accel| over the sequence, m/s^2
# Latched, like a live static_transform_publisher.
_FOREVER = QosTime(sec=2147483647, nsec=4294967295)
TF_STATIC_QOS = (Qos(history=QosHistory.KEEP_LAST, depth=1, reliability=QosReliability.RELIABLE,
                     durability=QosDurability.TRANSIENT_LOCAL, deadline=_FOREVER, lifespan=_FOREVER,
                     liveliness=QosLiveliness.AUTOMATIC, liveliness_lease_duration=_FOREVER,
                     avoid_ros_namespace_conventions=False),)


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


class BagWriter:
    """Thin wrapper: typed message construction + per-topic counts."""

    def __init__(self, writer, typestore):
        self.writer, self.ts, self.t = writer, typestore, typestore.types
        self.conns, self.counts = {}, {}

    def add(self, topic, msgtype, qos=()):
        self.conns[topic] = (self.writer.add_connection(
            topic, msgtype, typestore=self.ts, offered_qos_profiles=qos), msgtype)

    def write(self, topic, stamp, msg):
        conn, msgtype = self.conns[topic]
        self.writer.write(conn, stamp, self.ts.serialize_cdr(msg, msgtype))
        self.counts[topic] = self.counts.get(topic, 0) + 1

    def header(self, stamp, frame):
        return self.t['std_msgs/msg/Header'](
            stamp=self.t['builtin_interfaces/msg/Time'](sec=stamp // 1_000_000_000,
                                                        nanosec=stamp % 1_000_000_000),
            frame_id=frame)

    def transform(self, stamp, parent, child, xyz, q):
        t = self.t
        return t['geometry_msgs/msg/TransformStamped'](
            header=self.header(stamp, parent), child_frame_id=child,
            transform=t['geometry_msgs/msg/Transform'](
                translation=t['geometry_msgs/msg/Vector3'](x=xyz[0], y=xyz[1], z=xyz[2]),
                rotation=t['geometry_msgs/msg/Quaternion'](x=q[0], y=q[1], z=q[2], w=q[3])))


def camera_info(w, stamp, i, calib):
    f = calib.camera_info_fields()
    t = w.t
    return t['sensor_msgs/msg/CameraInfo'](
        header=w.header(stamp, CAM_FRAMES[i]), height=f['height'], width=f['width'],
        distortion_model=f['distortion_model'], d=np.array(f['d']), k=np.array(f['k']),
        r=np.array(f['r']), p=np.array(f['p']), binning_x=0, binning_y=0,
        roi=t['sensor_msgs/msg/RegionOfInterest'](x_offset=0, y_offset=0, height=0, width=0,
                                                  do_rectify=False))


def convert_messages(reader, w, calib, stats):
    """Stream every required message from the ROS 1 reader into the ROS 2 writer."""
    t = w.t
    conns = [c for c in reader.connections if c.topic in REQUIRED_TOPICS]
    orientation_unknown = np.zeros(9)
    orientation_unknown[0] = -1.0
    last = {}
    tf_written = False
    for conn, _, raw in reader.messages(connections=conns):
        msg = reader.deserialize(raw, conn.msgtype)
        stamp = stamp_ns(msg.header)
        topic = conn.topic
        if stamp <= last.get(topic, -1):
            stats['non_increasing'][topic] = stats['non_increasing'].get(topic, 0) + 1
            continue
        last[topic] = stamp
        if not tf_written:
            write_tf_static(w, stamp, calib)
            tf_written = True

        if topic in CAM_TOPICS:
            i = CAM_TOPICS[topic]
            cal = calib.cameras[i]
            if (msg.width, msg.height) != (cal.width, cal.height):
                raise DatasetError(f'{topic}: {msg.width}x{msg.height} != calibration '
                                   f'{cal.width}x{cal.height}')
            stats['encodings'].add(msg.encoding)
            if msg.encoding == 'bgr8':
                pixels = bgr8_to_mono8(msg.data, msg.width, msg.height, msg.step)
            elif msg.encoding == 'mono8' and msg.step == msg.width:
                pixels = np.asarray(msg.data, dtype=np.uint8)
            else:
                raise DatasetError(f'{topic}: unsupported encoding {msg.encoding!r}')
            hdr = w.header(stamp, CAM_FRAMES[i])
            w.write(f'/vio/cam{i}/image_raw', stamp, t['sensor_msgs/msg/Image'](
                header=hdr, height=msg.height, width=msg.width, encoding='mono8',
                is_bigendian=0, step=msg.width, data=pixels))
            w.write(f'/vio/cam{i}/camera_info', stamp, camera_info(w, stamp, i, cal))
            stats['cam_stamps'][i].append(stamp)
        elif topic == IMU:
            a, g = msg.linear_acceleration, msg.angular_velocity
            w.write('/vio/imu', stamp, t['sensor_msgs/msg/Imu'](
                header=w.header(stamp, IMU_FRAME),
                orientation=t['geometry_msgs/msg/Quaternion'](x=0.0, y=0.0, z=0.0, w=1.0),
                orientation_covariance=orientation_unknown,
                angular_velocity=t['geometry_msgs/msg/Vector3'](x=g.x, y=g.y, z=g.z),
                angular_velocity_covariance=np.zeros(9),
                linear_acceleration=t['geometry_msgs/msg/Vector3'](x=a.x, y=a.y, z=a.z),
                linear_acceleration_covariance=np.zeros(9)))
            stats['imu'].append((stamp, a.x, a.y, a.z, g.x, g.y, g.z))
        elif topic == ODOM:
            p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
            w.write('/wheel/odom', stamp, t['nav_msgs/msg/Odometry'](
                header=w.header(stamp, 'odom'), child_frame_id='base_link',
                pose=t['geometry_msgs/msg/PoseWithCovariance'](
                    pose=t['geometry_msgs/msg/Pose'](
                        position=t['geometry_msgs/msg/Point'](x=p.x, y=p.y, z=p.z),
                        orientation=t['geometry_msgs/msg/Quaternion'](x=q.x, y=q.y, z=q.z, w=q.w)),
                    covariance=np.asarray(msg.pose.covariance, dtype=float)),
                twist=t['geometry_msgs/msg/TwistWithCovariance'](
                    twist=t['geometry_msgs/msg/Twist'](
                        linear=t['geometry_msgs/msg/Vector3'](
                            x=v.linear.x, y=v.linear.y, z=v.linear.z),
                        angular=t['geometry_msgs/msg/Vector3'](
                            x=v.angular.x, y=v.angular.y, z=v.angular.z)),
                    covariance=np.asarray(msg.twist.covariance, dtype=float))))
            stats['odom'].append((stamp, p.x, p.y, q.x, q.y, q.z, q.w, v.linear.x, v.linear.y))


def write_tf_static(w, stamp, calib):
    tfs = [w.transform(stamp, 'base_link', IMU_FRAME, calib.imu_in_base_xyz,
                       calib.imu_in_base_quat_xyzw),
           w.transform(stamp, 'base_link', GPS_FRAME, calib.gps_in_base_xyz, (0, 0, 0, 1))]
    for i in (0, 1):
        xyz, q = calib.imu_to_camera_pose(i)
        tfs.append(w.transform(stamp, IMU_FRAME, CAM_FRAMES[i], xyz, q))
    w.write('/tf_static', stamp, w.t['tf2_msgs/msg/TFMessage'](transforms=tfs))


def recorded_tf_static(reader):
    """The bag's own base_link -> imu transform, for comparison with the calibration file."""
    conns = [c for c in reader.connections if c.topic.lstrip('/') == 'tf_static']
    if not conns:               # messages(connections=[]) would yield every message
        return None
    for conn, _, raw in reader.messages(connections=conns):
        for tf in reader.deserialize(raw, conn.msgtype).transforms:
            if (tf.header.frame_id, tf.child_frame_id) == ('base_link', 'imu'):
                tr, r = tf.transform.translation, tf.transform.rotation
                return [tr.x, tr.y, tr.z], [r.x, r.y, r.z, r.w]
    return None


def odom_twist_frame_check(odom):
    """RMS mismatch between the recorded twist and the velocity obtained by
    differentiating the recorded pose, with the twist read as body-frame
    (nav_msgs convention) and as odom-frame. Tells which one the data uses."""
    o = np.array(odom)
    if len(o) < 3:
        return None
    t = o[:, 0] / 1e9
    v_world = np.gradient(o[:, 1:3], t, axis=0)
    yaw = np.arctan2(2 * (o[:, 6] * o[:, 5] + o[:, 3] * o[:, 4]),
                     1 - 2 * (o[:, 4] ** 2 + o[:, 5] ** 2))
    c, s = np.cos(yaw), np.sin(yaw)
    v_body = np.stack([c * v_world[:, 0] + s * v_world[:, 1],
                       -s * v_world[:, 0] + c * v_world[:, 1]], axis=1)
    rec = o[:, 7:9]
    rms = lambda d: float(np.sqrt(np.mean(np.sum(d ** 2, axis=1))))  # noqa: E731
    return {'rms_if_body_frame_mps': rms(rec - v_body), 'rms_if_odom_frame_mps': rms(rec - v_world),
            'mean_speed_mps': float(np.mean(np.linalg.norm(v_world, axis=1)))}


def validate(stats, n_gt_inside):
    left, right = (set(s) for s in stats['cam_stamps'])
    unpaired = len(left ^ right)
    imu = np.array(stats['imu'])
    accel_norm = float(np.median(np.linalg.norm(imu[:, 1:4], axis=1))) if len(imu) else 0.0
    errors = []
    for topic, n in stats['non_increasing'].items():
        if topic != ODOM:
            errors.append(f'{topic}: {n} stamps not after the previous one')
    if unpaired > MAX_UNPAIRED_FRACTION * max(len(left), 1):
        errors.append(f'{unpaired} camera stamps without a stereo partner')
    if not GRAVITY_RANGE[0] <= accel_norm <= GRAVITY_RANGE[1]:
        errors.append(f'median |accel| {accel_norm:.2f} not gravity in m/s^2 {GRAVITY_RANGE}')
    if n_gt_inside < 2:
        errors.append('ground truth does not overlap the bag')
    return errors, unpaired, accel_norm, imu


def rate_hz(stamps_ns):
    return round(1e9 / float(np.median(np.diff(np.asarray(stamps_ns, dtype=np.int64)))), 3)


def write_estimator_inputs(out_dir, calib):
    """platform.yaml and OpenVINS calibration files, from the sequence calibration."""
    return write_rig_outputs(to_rig(calib), out_dir,
                             'Rosario v1 platform, generated by prepare_rosario_v1.py from '
                             'calibrationNN.yaml.')


def convert(number, download_dir, out_dir, delete_parts=False, delete_source=False,
            stereo_source='camchain'):
    download_dir, out_dir = Path(download_dir), Path(out_dir)
    parts, calib_name, gt_name = sequence_files(number)
    bag_dir = out_dir / 'ros2_bag'
    if bag_dir.exists():
        raise DatasetError(f'output already exists: {bag_dir}')
    calib = read_calibration(download_dir / calib_name, stereo_source)
    gt = read_ground_truth(download_dir / gt_name)

    # Single-part sequences are downloaded as sequenceNN.bag directly.
    source = download_dir / f'sequence{number:02d}.bag'
    part_paths = [download_dir / p for p in parts]
    expected = SEQUENCES[number]['md5']
    if len(parts) > 1 and not source.exists():
        missing = [str(p) for p in part_paths if not p.exists()]
        if missing:
            raise DatasetError(f'missing downloads: {missing}')
        md5 = assemble_bag(part_paths, source, expected)
    elif source.exists():
        md5 = file_md5(source)
        if md5 != expected:
            raise DatasetError(f'{source.name}: MD5 {md5} != published {expected}')
    else:
        raise DatasetError(f'missing download: {source}')
    if delete_parts and len(parts) > 1:
        for p in part_paths:
            p.unlink(missing_ok=True)

    out_dir.mkdir(parents=True, exist_ok=True)
    partial = bag_dir.with_name(bag_dir.name + '.partial')
    if partial.exists():
        shutil.rmtree(partial)
    stats = {'cam_stamps': ([], []), 'imu': [], 'odom': [], 'non_increasing': {},
             'encodings': set()}
    try:
        with AnyReader([source]) as reader:
            have = {c.topic: c.msgtype for c in reader.connections}
            wrong = {t: have.get(t) for t, ty in REQUIRED_TOPICS.items() if have.get(t) != ty}
            if wrong:
                raise DatasetError(f'{source.name}: required topics missing or mistyped: {wrong}')
            bag_span = (reader.start_time, reader.end_time)
            tf_recorded = recorded_tf_static(reader)
            ts = get_typestore(Stores.ROS2_HUMBLE)
            with Writer(partial, version=HUMBLE_BAG_VERSION,
                        storage_plugin=StoragePlugin.MCAP) as writer:
                w = BagWriter(writer, ts)
                for i in (0, 1):
                    w.add(f'/vio/cam{i}/image_raw', 'sensor_msgs/msg/Image')
                    w.add(f'/vio/cam{i}/camera_info', 'sensor_msgs/msg/CameraInfo')
                w.add('/vio/imu', 'sensor_msgs/msg/Imu')
                w.add('/wheel/odom', 'nav_msgs/msg/Odometry')
                w.add('/tf_static', 'tf2_msgs/msg/TFMessage', TF_STATIC_QOS)
                convert_messages(reader, w, calib, stats)
        gt_ns = (gt[:, 0] * 1e9).astype(np.int64)
        n_gt_inside = int(np.sum((gt_ns >= bag_span[0]) & (gt_ns <= bag_span[1])))
        errors, unpaired, accel_norm, imu = validate(stats, n_gt_inside)
        if errors:
            raise DatasetError('; '.join(errors))
    except BaseException:
        shutil.rmtree(partial, ignore_errors=True)
        raise
    partial.rename(bag_dir)

    np.savetxt(out_dir / 'ground_truth.tum', gt, fmt='%.9f')
    shutil.copy2(download_dir / calib_name, out_dir / 'calibration.yaml')
    sequence = f'rosario_v1_sequence{number:02d}'
    (out_dir / 'dataset.yaml').write_text(yaml.safe_dump({
        'sequence': sequence,
        'dataset': 'rosario_v1',
        'sensor_bag': 'ros2_bag',
        'ground_truth': 'ground_truth.tum',
        'ground_truth_has_orientation': False,
        # The ground truth is the GPS antenna, not base_link (calibrationNN.yaml gps entry).
        'ground_truth_point_in_base': list(calib.gps_in_base_xyz),
        'imu_in_base_xyz': list(calib.imu_in_base_xyz),
        'imu_in_base_quat_xyzw': list(calib.imu_in_base_quat_xyzw),
    }, sort_keys=False))

    write_estimator_inputs(out_dir, calib)
    if delete_source:
        source.unlink()
    imu_t = imu[:, 0]
    tf_diff = None
    if tf_recorded:
        q_cal, q_rec = np.array(calib.imu_in_base_quat_xyzw), np.array(tf_recorded[1])
        tf_diff = {
            'translation_m': float(np.linalg.norm(np.subtract(tf_recorded[0],
                                                              calib.imu_in_base_xyz))),
            'rotation_deg': float(np.degrees(2 * np.arccos(min(1.0, abs(q_cal @ q_rec))))),
        }
    report = {
        'sequence': sequence,
        'source': str(source),
        'source_md5': md5,
        'source_deleted': delete_source,
        'bag': str(bag_dir),
        'storage': 'mcap',
        'bag_metadata_version': HUMBLE_BAG_VERSION,
        'message_counts': w.counts,
        'image_encodings_in_source': sorted(stats['encodings']),
        'non_increasing_stamps_dropped': stats['non_increasing'],
        'unpaired_stereo_stamps': unpaired,
        'start_ns': int(bag_span[0]),
        'end_ns': int(bag_span[1]),
        'duration_s': (bag_span[1] - bag_span[0]) / 1e9,
        'camera_rate_hz_measured': [rate_hz(s) for s in stats['cam_stamps']],
        'imu_rate_hz_measured': rate_hz(imu_t),
        'odom_rate_hz_measured': rate_hz([o[0] for o in stats['odom']]),
        'imu_median_accel_norm_mps2': accel_norm,
        'imu_max_abs_accel_mps2': np.abs(imu[:, 1:4]).max(axis=0).round(3).tolist(),
        'imu_max_abs_gyro_radps': np.abs(imu[:, 4:7]).max(axis=0).round(3).tolist(),
        'ground_truth_poses': int(len(gt)),
        'ground_truth_poses_inside_bag': n_gt_inside,
        'tf_static_vs_calibration_imu': tf_diff,
        'stereo_extrinsic_source': calib.stereo_source,
        'stereo_calibration_discrepancy_deg': calib.stereo_discrepancy_deg,
        'wheel_odom_twist_frame_check': odom_twist_frame_check(stats['odom']),
    }
    (out_dir / 'conversion_report.json').write_text(json.dumps(report, indent=2))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('sequence', type=int, help='sequence number, 1-6')
    parser.add_argument('download_dir', help='folder with the downloaded sequence files')
    parser.add_argument('out_dir', help='output folder (its ros2_bag/ must not exist)')
    parser.add_argument('--delete-parts', action='store_true',
                        help='delete the split download parts once joined and verified')
    parser.add_argument('--delete-source', action='store_true',
                        help='also delete the joined ROS 1 bag after a successful conversion')
    parser.add_argument('--stereo-source', choices=('camchain', 'imu'), default='camchain',
                        help="cam1 extrinsic: chained from the stereo calibration (default) or the "
                             "file's own cam1 T_cam_imu (see rosario_v1.read_calibration)")
    parser.add_argument('--configs-only', action='store_true',
                        help='only (re)write platform.yaml and the estimator calibration files')
    args = parser.parse_args()
    try:
        if args.configs_only:
            calib = read_calibration(Path(args.download_dir) / sequence_files(args.sequence)[1],
                                     args.stereo_source)
            for path in write_estimator_inputs(args.out_dir, calib):
                print(f'wrote {path}')
            return 0
        report = convert(args.sequence, args.download_dir, args.out_dir,
                         args.delete_parts, args.delete_source, args.stereo_source)
    except DatasetError as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        return 1
    print(json.dumps(report, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
