#!/opt/vio-tools/bin/python -I
"""Offline counterpart of vio_odom_adapter for the bench.

Converts recorded estimator odometry (IMU frame, e.g. /ov_msckf/odomimu from
openvins_serial) into /vio/odom (vio_world -> base_link) and /vio/status, using
the same pure library code as the live node (ppb_vio.lib: odom_conversion and
the vio_health divergence gate), so offline and live results are comparable.

    offline_adapter.py --input <odom bag> --profile <run>/profile.yaml \
        --platform /data/rosario_v1/sequence02/platform.yaml --output <run>/record

/vio/status is written once per second of data time with the same status names
as the live adapter ('ppb_vio: odometry', 'ppb_vio: vio health').
"""

import argparse
import json
from pathlib import Path
import sys

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.rosbag2 import StoragePlugin, Writer
from rosbags.typesys import get_typestore, Stores
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'ppb_vio'))
from ppb_vio.lib.odom_conversion import (  # noqa: E402
    Extrinsic, ImuOdometry, imu_to_base, OdometryRejected)
from ppb_vio.lib.vio_health import HealthLimits, VioHealthGate  # noqa: E402

HUMBLE_BAG_VERSION = 8
STATUS_PERIOD_S = 1.0


def vec(v):
    return np.array([v.x, v.y, v.z], dtype=float)


def platform_params(path):
    params = yaml.safe_load(Path(path).read_text())['vio_odom_adapter']['ros__parameters']
    limits = HealthLimits(**{k: float(params[f'health_{k}'])
                             for k in HealthLimits().__dict__ if f'health_{k}' in params})
    return params, limits


def convert(input_bag, profile_path, platform_path, output_bag):
    profile = yaml.safe_load(Path(profile_path).read_text())
    topic = profile['output']['odom_topic']
    velocity_frame = profile['output']['linear_velocity_frame']
    params, limits = platform_params(platform_path)
    extrinsic = Extrinsic.from_xyz_quat(params['imu_in_base_xyz'], params['imu_in_base_quat_xyzw'])
    world, base = params.get('world_frame', 'vio_world'), params.get('base_frame', 'base_link')
    cov_scale = float(params.get('covariance_scale', 1.0))      # same parameter as the live adapter
    gate = VioHealthGate(limits)

    ts = get_typestore(Stores.ROS2_HUMBLE)
    t = ts.types
    counts = {'received': 0, 'published': 0, 'rejected': 0}
    rejected = {}

    def header(stamp_ns, frame):
        return t['std_msgs/msg/Header'](
            stamp=t['builtin_interfaces/msg/Time'](sec=stamp_ns // 10**9, nanosec=stamp_ns % 10**9),
            frame_id=frame)

    def status_msg(stamp_ns):
        h = gate.state
        kv = t['diagnostic_msgs/msg/KeyValue']
        odom_level = 0 if counts['published'] else 3
        odometry = t['diagnostic_msgs/msg/DiagnosticStatus'](
            level=odom_level, name='ppb_vio: odometry', message='offline bench', hardware_id=topic,
            values=[kv(key=k, value=str(v)) for k, v in counts.items()])
        values = [kv(key=f'violations_{k}', value=str(v)) for k, v in h.reasons.items()]
        if h.latched_at_s is not None:
            values.append(kv(key='diverged_at_s', value=f'{h.latched_at_s:.3f}'))
        health = t['diagnostic_msgs/msg/DiagnosticStatus'](
            level=h.level if counts['published'] else 3, name='ppb_vio: vio health',
            message=h.message, hardware_id=topic, values=values)
        return t['diagnostic_msgs/msg/DiagnosticArray'](header=header(stamp_ns, ''),
                                                       status=[odometry, health])

    last_ns, next_status_ns = None, None
    with AnyReader([Path(input_bag)]) as reader, \
            Writer(Path(output_bag), version=HUMBLE_BAG_VERSION,
                   storage_plugin=StoragePlugin.MCAP) as writer:
        conns = [c for c in reader.connections if c.topic == topic]
        if not conns:
            raise SystemExit(f'{input_bag}: no {topic}')
        odom_conn = writer.add_connection('/vio/odom', 'nav_msgs/msg/Odometry', typestore=ts)
        status_conn = writer.add_connection('/vio/status', 'diagnostic_msgs/msg/DiagnosticArray',
                                            typestore=ts)
        for conn, _, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)
            counts['received'] += 1
            stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            if last_ns is not None and stamp_ns <= last_ns:
                rejected['non_increasing_stamp'] = rejected.get('non_increasing_stamp', 0) + 1
                counts['rejected'] += 1
                continue
            pose, twist = msg.pose.pose, msg.twist.twist
            q = pose.orientation
            try:
                b = imu_to_base(ImuOdometry(
                    position=vec(pose.position),
                    orientation_xyzw=np.array([q.x, q.y, q.z, q.w], dtype=float),
                    linear_velocity=vec(twist.linear), angular_velocity=vec(twist.angular),
                    pose_covariance=np.asarray(msg.pose.covariance, dtype=float).reshape(6, 6),
                    twist_covariance=np.asarray(msg.twist.covariance, dtype=float).reshape(6, 6)),
                    extrinsic, velocity_frame)
            except OdometryRejected as exc:
                rejected[exc.reason] = rejected.get(exc.reason, 0) + 1
                counts['rejected'] += 1
                continue
            last_ns = stamp_ns
            gate.update(stamp_ns / 1e9, b.position, b.pose_covariance)
            o = t['nav_msgs/msg/Odometry'](
                header=header(stamp_ns, world), child_frame_id=base,
                pose=t['geometry_msgs/msg/PoseWithCovariance'](
                    pose=t['geometry_msgs/msg/Pose'](
                        position=t['geometry_msgs/msg/Point'](*b.position.tolist()),
                        orientation=t['geometry_msgs/msg/Quaternion'](*b.orientation_xyzw.tolist())),
                    covariance=(cov_scale * b.pose_covariance).reshape(-1)),
                twist=t['geometry_msgs/msg/TwistWithCovariance'](
                    twist=t['geometry_msgs/msg/Twist'](
                        linear=t['geometry_msgs/msg/Vector3'](*b.linear_velocity.tolist()),
                        angular=t['geometry_msgs/msg/Vector3'](*b.angular_velocity.tolist())),
                    covariance=(cov_scale * b.twist_covariance).reshape(-1)))
            writer.write(odom_conn, stamp_ns, ts.serialize_cdr(o, 'nav_msgs/msg/Odometry'))
            counts['published'] += 1
            if next_status_ns is None:
                next_status_ns = stamp_ns
            while stamp_ns >= next_status_ns:
                writer.write(status_conn, next_status_ns, ts.serialize_cdr(
                    status_msg(next_status_ns), 'diagnostic_msgs/msg/DiagnosticArray'))
                next_status_ns += int(STATUS_PERIOD_S * 1e9)
    return {'counts': counts, 'rejected': rejected,
            'health_level': gate.state.level, 'health_message': gate.state.message,
            'diverged_at_s': gate.state.latched_at_s}


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--input', required=True, help='bag with the estimator odometry')
    parser.add_argument('--profile', required=True, help='estimator profile YAML (odom topic, velocity frame)')
    parser.add_argument('--platform', required=True, help='platform YAML (base_link -> imu, health limits)')
    parser.add_argument('--output', required=True, help='bag directory to create')
    args = parser.parse_args()
    summary = convert(args.input, args.profile, args.platform, args.output)
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
