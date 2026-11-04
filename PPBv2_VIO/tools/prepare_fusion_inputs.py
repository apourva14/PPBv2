#!/opt/vio-tools/bin/python -I
"""Build the input bag for the fusion layer (spec: "Runtime graph", local EKF) from a
recording and a VIO run, on the same topics the robot publishes live:

    /imu/data      sensor_msgs/Imu        raw IMU (from /vio/imu), imu_link
    /wheel/odom    nav_msgs/Odometry      odom -> base_link, twist in base_link (vx, vyaw)
    /vio/odom      nav_msgs/Odometry      from the VIO run's record (/vio/odom)
    /vio/status    diagnostic_msgs/...    from the VIO run (health gate)
    /tf_static                            from the recording (base_link -> imu_link, ...)

    prepare_fusion_inputs.py --recording DATASET_DIR/ros2_bag --vio-run RUN_DIR --out OUT_BAG

Wheel odometry: published twist must be in base_link (nav_msgs convention). Recordings
whose twist is in the odom frame (Rosario v1, see its conversion report) are rotated by
the recorded yaw (--wheel-twist-frame odom). --vio-blackout T0:T1 removes VIO output and
its health reports in a window (Task 3: the EKF must coast through VIO dropout). Wheel
covariance is set from
--wheel-sigma-v / --wheel-sigma-yawrate (encoders report none); calibrate on recordings.
"""

import argparse
import json
from pathlib import Path
import shutil
import sys

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.interfaces import Qos, QosDurability, QosHistory, QosLiveliness, QosReliability, QosTime
from rosbags.rosbag2 import StoragePlugin, Writer
from rosbags.typesys import get_typestore, Stores

HUMBLE_BAG_VERSION = 8
COPY_FROM_VIO_RUN = ('/vio/odom', '/vio/status')
# /tf_static must be replayed latched (transient local), or TF listeners (the EKF's IMU
# transform lookup) reject it as incompatible QoS and never see base_link -> imu_link.
_FOREVER = QosTime(sec=2147483647, nsec=4294967295)
TF_STATIC_QOS = (Qos(history=QosHistory.KEEP_LAST, depth=1, reliability=QosReliability.RELIABLE,
                     durability=QosDurability.TRANSIENT_LOCAL, deadline=_FOREVER, lifespan=_FOREVER,
                     liveliness=QosLiveliness.AUTOMATIC, liveliness_lease_duration=_FOREVER,
                     avoid_ros_namespace_conventions=False),)


def yaw_of(q):
    return float(np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


def prepare(recording, vio_run, out_bag, wheel_twist_frame='base_link', sigma_v=0.05,
            sigma_yawrate=0.05, vio_blackout=None):
    out_bag = Path(out_bag)
    if out_bag.exists():
        raise SystemExit(f'output exists: {out_bag}')
    ts = get_typestore(Stores.ROS2_HUMBLE)
    t = ts.types
    counts = {}
    twist_cov = np.zeros(36)
    twist_cov[0], twist_cov[7], twist_cov[14] = sigma_v ** 2, sigma_v ** 2, sigma_v ** 2
    twist_cov[21], twist_cov[28], twist_cov[35] = sigma_yawrate ** 2, sigma_yawrate ** 2, \
        sigma_yawrate ** 2
    partial = out_bag.with_name(out_bag.name + '.partial')
    shutil.rmtree(partial, ignore_errors=True)
    try:
        with Writer(partial, version=HUMBLE_BAG_VERSION, storage_plugin=StoragePlugin.MCAP) as w:
            conns = {}

            def write(topic, msgtype, stamp_ns, msg=None, raw=None):
                if topic not in conns:
                    qos = TF_STATIC_QOS if topic == '/tf_static' else ()
                    conns[topic] = w.add_connection(topic, msgtype, typestore=ts,
                                                    offered_qos_profiles=qos)
                w.write(conns[topic], stamp_ns, raw if raw is not None else ts.serialize_cdr(msg, msgtype))
                counts[topic] = counts.get(topic, 0) + 1

            with AnyReader([Path(recording)]) as r:
                cs = [c for c in r.connections if c.topic in ('/vio/imu', '/wheel/odom', '/tf_static')]
                for conn, stamp, raw in r.messages(connections=cs):
                    if conn.topic == '/vio/imu':
                        write('/imu/data', conn.msgtype, stamp, raw=raw)
                    elif conn.topic == '/tf_static':
                        write('/tf_static', conn.msgtype, stamp, raw=raw)
                    else:
                        msg = r.deserialize(raw, conn.msgtype)
                        v = msg.twist.twist.linear
                        vx, vy = v.x, v.y
                        if wheel_twist_frame == 'odom':
                            yaw = yaw_of(msg.pose.pose.orientation)
                            c, s = np.cos(yaw), np.sin(yaw)
                            vx, vy = c * v.x + s * v.y, -s * v.x + c * v.y
                        msg.twist.twist.linear = t['geometry_msgs/msg/Vector3'](x=vx, y=vy, z=0.0)
                        msg.twist.covariance = twist_cov
                        msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
                        write('/wheel/odom', conn.msgtype, stamp, msg)
            with AnyReader([Path(recording)]) as r:
                t0_ns = r.start_time            # blackout window is relative to the recording
            with AnyReader([Path(vio_run) / 'record']) as r:
                cs = [c for c in r.connections if c.topic in COPY_FROM_VIO_RUN]
                for conn, stamp, raw in r.messages(connections=cs):
                    msg = r.deserialize(raw, conn.msgtype)
                    # Bag time = header stamp (the offline bench writes data-time stamps).
                    ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
                    if vio_blackout and t0_ns is not None and \
                            vio_blackout[0] <= (ns - t0_ns) / 1e9 < vio_blackout[1]:
                        counts['vio_blacked_out'] = counts.get('vio_blacked_out', 0) + 1
                        continue
                    write(conn.topic, conn.msgtype, ns, raw=raw)
    except BaseException:
        shutil.rmtree(partial, ignore_errors=True)
        raise
    partial.rename(out_bag)
    report = {'recording': str(recording), 'vio_run': str(vio_run), 'counts': counts,
              'wheel_twist_frame_in': wheel_twist_frame,
              'wheel_sigma_v': sigma_v, 'wheel_sigma_yawrate': sigma_yawrate,
              'vio_blackout_s': list(vio_blackout) if vio_blackout else None}
    Path(str(out_bag) + '.json').write_text(json.dumps(report, indent=2))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--recording', required=True, help='contract bag (/vio/imu, /wheel/odom)')
    parser.add_argument('--vio-run', required=True, help='run folder with record/ (/vio/odom, /vio/status)')
    parser.add_argument('--out', required=True, help='bag to create')
    parser.add_argument('--wheel-twist-frame', choices=('base_link', 'odom'), default='base_link',
                        help='frame of the recorded wheel twist (Rosario v1: odom)')
    parser.add_argument('--wheel-sigma-v', type=float, default=0.05, help='m/s')
    parser.add_argument('--wheel-sigma-yawrate', type=float, default=0.05, help='rad/s')
    parser.add_argument('--vio-blackout', help='T0:T1 s after the recording start: no VIO output '
                        '(and no VIO health report) in that window, i.e. the VIO process is lost')
    args = parser.parse_args()
    blackout = tuple(float(v) for v in args.vio_blackout.split(':')) if args.vio_blackout else None
    print(json.dumps(prepare(args.recording, args.vio_run, args.out, args.wheel_twist_frame,
                             args.wheel_sigma_v, args.wheel_sigma_yawrate, blackout), indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
