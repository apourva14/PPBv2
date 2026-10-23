#!/opt/vio-tools/bin/python -I
"""Package a finished replay run into one MCAP file for viewing in Foxglove.

No estimator runs during the demo: the file holds what a validated run already
produced, so playback cannot be disturbed by machine load. Every message's log
time is its header stamp (dataset time), so all topics line up in Foxglove.

Topics written:
    /vio/cam0/image_raw     sensor_msgs/Image      (from the replayed sensor bag)
    /vio/odom               nav_msgs/Odometry      (recorded estimator output)
    /tf                     vio_world -> base_link (from /vio/odom)
    /vio/path               nav_msgs/Path          (estimated trail, grows at 2 Hz)
    /ground_truth/path      nav_msgs/Path          (whole ground truth, once, aligned
                                                    into vio_world by position + yaw)

Usage:
    make_demo_mcap.py --run-dir /data/runs/<run> --sensor-bag /data/euroc/V1_01_easy/ros2_bag \\
        --gt /opt/vio/openvins_ws/src/open_vins/ov_data/euroc_mav/V1_01_easy.txt \\
        --out /data/demo/V1_01_easy_mins.mcap
"""

import argparse
from pathlib import Path
import shutil
import sys

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.rosbag2 import StoragePlugin, Writer
from rosbags.typesys import get_typestore, Stores

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ppb_vio_tools.metrics import align_position_yaw  # noqa: E402

IMAGE_TOPIC = '/vio/cam0/image_raw'
ODOM_TOPIC = '/vio/odom'
PATH_PERIOD_NS = 500_000_000
PATH_DECIMATE = 20          # ~10 Hz trail from ~200 Hz odometry


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--run-dir', required=True)
    parser.add_argument('--sensor-bag', required=True)
    parser.add_argument('--gt', required=True, help='ground truth, TUM format')
    parser.add_argument('--out', required=True, help='output .mcap path (must not exist)')
    args = parser.parse_args()

    out = Path(args.out)
    if out.exists():
        sys.exit(f'output already exists: {out}')
    ts = get_typestore(Stores.ROS2_HUMBLE)
    T = ts.types
    Odometry, Path_, PoseStamped = (T['nav_msgs/msg/Odometry'], T['nav_msgs/msg/Path'],
                                    T['geometry_msgs/msg/PoseStamped'])
    Header, Time, Pose = T['std_msgs/msg/Header'], T['builtin_interfaces/msg/Time'], T['geometry_msgs/msg/Pose']
    Point, Quat = T['geometry_msgs/msg/Point'], T['geometry_msgs/msg/Quaternion']
    TFMsg, TStamped = T['tf2_msgs/msg/TFMessage'], T['geometry_msgs/msg/TransformStamped']
    Transform, Vector3 = T['geometry_msgs/msg/Transform'], T['geometry_msgs/msg/Vector3']

    def header(ns, frame):
        return Header(stamp=Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000),
                      frame_id=frame)

    def pose_stamped(ns, frame, p, q):
        return PoseStamped(header=header(ns, frame), pose=Pose(
            position=Point(x=float(p[0]), y=float(p[1]), z=float(p[2])),
            orientation=Quat(x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3]))))

    # Recorded estimator output.
    odoms = []
    with AnyReader([Path(args.run_dir) / 'record'], default_typestore=ts) as reader:
        conns = [c for c in reader.connections if c.topic == ODOM_TOPIC]
        if not conns:
            sys.exit(f'no {ODOM_TOPIC} in {args.run_dir}/record')
        for conn, _, raw in reader.messages(connections=conns):
            m = reader.deserialize(raw, conn.msgtype)
            odoms.append((stamp_ns(m.header), raw, m))
    world = odoms[0][2].header.frame_id
    base = odoms[0][2].child_frame_id
    est_t = np.array([o[0] for o in odoms]) / 1e9
    est_p = np.array([[o[2].pose.pose.position.x, o[2].pose.pose.position.y,
                       o[2].pose.pose.position.z] for o in odoms])

    # Ground truth, aligned into the estimator's frame (inverse of est -> gt alignment).
    gt = np.loadtxt(args.gt, comments='#')
    idx = np.clip(np.searchsorted(gt[:, 0], est_t), 1, len(gt) - 1)
    nearest = np.where(np.abs(gt[idx - 1, 0] - est_t) < np.abs(gt[idx, 0] - est_t), idx - 1, idx)
    ok = np.abs(gt[nearest, 0] - est_t) < 0.02
    rot, t = align_position_yaw(est_p[ok], gt[nearest[ok], 1:4])      # est ~= rot @ gt + t
    gt_in_world = gt[:, 1:4] @ rot.T + t
    in_span = (gt[:, 0] >= est_t[0]) & (gt[:, 0] <= est_t[-1])

    tmp = out.with_name(out.stem + '.partial')
    if tmp.exists():
        shutil.rmtree(tmp)
    with Writer(tmp, version=8, storage_plugin=StoragePlugin.MCAP) as writer:
        c_img = writer.add_connection(IMAGE_TOPIC, T['sensor_msgs/msg/Image'].__msgtype__, typestore=ts)
        c_odom = writer.add_connection(ODOM_TOPIC, Odometry.__msgtype__, typestore=ts)
        c_tf = writer.add_connection('/tf', TFMsg.__msgtype__, typestore=ts)
        c_path = writer.add_connection('/vio/path', Path_.__msgtype__, typestore=ts)
        c_gt = writer.add_connection('/ground_truth/path', Path_.__msgtype__, typestore=ts)

        first_ns = odoms[0][0]
        # Positions only: the path is drawn as a line, so orientation is not needed.
        gt_poses = [pose_stamped(int(round(s * 1e9)), world, p, (0.0, 0.0, 0.0, 1.0))
                    for s, p, keep in zip(gt[:, 0], gt_in_world, in_span) if keep]
        writer.write(c_gt, first_ns, ts.serialize_cdr(
            Path_(header=header(first_ns, world), poses=gt_poses), Path_.__msgtype__))

        with AnyReader([Path(args.sensor_bag)], default_typestore=ts) as reader:
            for conn, _, raw in reader.messages(
                    connections=[c for c in reader.connections if c.topic == IMAGE_TOPIC]):
                ns = stamp_ns(reader.deserialize(raw, conn.msgtype).header)
                if odoms[0][0] - 2_000_000_000 <= ns <= odoms[-1][0]:
                    writer.write(c_img, ns, raw)

        trail, next_path_ns = [], first_ns
        for k, (ns, raw, m) in enumerate(odoms):
            writer.write(c_odom, ns, raw)
            p, q = m.pose.pose.position, m.pose.pose.orientation
            tf = TStamped(header=header(ns, world), child_frame_id=base, transform=Transform(
                translation=Vector3(x=p.x, y=p.y, z=p.z), rotation=q))
            writer.write(c_tf, ns, ts.serialize_cdr(TFMsg(transforms=[tf]), TFMsg.__msgtype__))
            if k % PATH_DECIMATE == 0:
                trail.append(pose_stamped(ns, world, (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)))
            if ns >= next_path_ns:
                writer.write(c_path, ns, ts.serialize_cdr(
                    Path_(header=header(ns, world), poses=list(trail)), Path_.__msgtype__))
                next_path_ns = ns + PATH_PERIOD_NS

    mcap = next(tmp.glob('*.mcap'))
    mcap.rename(out)
    shutil.rmtree(tmp)
    print(f'wrote {out} ({out.stat().st_size / 1e9:.2f} GB), fixed frame: {world}')


if __name__ == '__main__':
    main()
