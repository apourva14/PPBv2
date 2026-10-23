"""Republish estimator IMU odometry as /vio/odom (vio_world -> base_link).

This node is the only place where estimator output enters the PPBv2 VIO topic
contract. It never publishes /robot/odom or any velocity command.
"""

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from ppb_vio.lib.odom_conversion import (
    Extrinsic,
    ImuOdometry,
    imu_to_base,
    LINEAR_VELOCITY_FRAMES,
    OdometryRejected,
)
from ppb_vio.lib.time_utils import NS_PER_S, stamp_to_ns
from ppb_vio.lib.transforms import rot_to_quat

FORBIDDEN_OUTPUT_TOPICS = ('/robot/odom', '/cmd_vel', '/cmd_vel_nav', '/cmd_vel_out',
                           '/cmd_vel_stop', '/cmd_vel_key')


def _vec3(v):
    return np.array([v.x, v.y, v.z], dtype=float)


class VioOdomAdapter(Node):

    def __init__(self):
        super().__init__('vio_odom_adapter')

        # No defaults for estimator-specific semantics: a wrong guess here
        # silently corrupts the output, so the launch file must set them.
        self.declare_parameter('input_topic', '')
        self.declare_parameter('input_linear_velocity_frame', '')
        self.declare_parameter('output_topic', '/vio/odom')
        self.declare_parameter('status_topic', '/vio/status')
        self.declare_parameter('world_frame', 'vio_world')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('imu_in_base_xyz', [0.0, 0.0, 0.0])
        self.declare_parameter('imu_in_base_quat_xyzw', [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('publish_extrinsic_tf', True)
        self.declare_parameter('stale_timeout_s', 0.5)
        self.declare_parameter('status_period_s', 1.0)

        p = self.get_parameter
        self.input_topic = p('input_topic').value
        self.velocity_frame = p('input_linear_velocity_frame').value
        self.output_topic = p('output_topic').value
        self.world_frame = p('world_frame').value
        self.base_frame = p('base_frame').value
        self.imu_frame = p('imu_frame').value
        self.publish_tf = bool(p('publish_tf').value)
        self.stale_ns = int(float(p('stale_timeout_s').value) * NS_PER_S)

        if not self.input_topic:
            raise ValueError('parameter input_topic must be set')
        if self.velocity_frame not in LINEAR_VELOCITY_FRAMES:
            raise ValueError(
                f'parameter input_linear_velocity_frame must be one of {LINEAR_VELOCITY_FRAMES}')
        if self.output_topic in FORBIDDEN_OUTPUT_TOPICS:
            raise ValueError(f'refusing to publish VIO output on {self.output_topic}')
        if self.input_topic == self.output_topic:
            raise ValueError('input_topic and output_topic must differ')

        self.extrinsic = Extrinsic.from_xyz_quat(
            list(p('imu_in_base_xyz').value), list(p('imu_in_base_quat_xyzw').value))

        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        self.odom_pub = self.create_publisher(Odometry, self.output_topic, qos)
        self.status_pub = self.create_publisher(DiagnosticArray, p('status_topic').value, 10)
        self.create_subscription(Odometry, self.input_topic, self._on_odom, qos)

        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        if bool(p('publish_extrinsic_tf').value):
            self._publish_extrinsic()

        self.counts = {'received': 0, 'published': 0, 'bad_stamp': 0, 'non_finite': 0,
                       'bad_quaternion': 0, 'non_increasing_stamp': 0}
        self.last_stamp_ns = None
        self.last_rx_ns = None
        self._counts_at_last_status = dict(self.counts)
        self.create_timer(float(p('status_period_s').value), self._publish_status)

        self.get_logger().info(
            f'{self.input_topic} (twist.linear in {self.velocity_frame} frame) -> '
            f'{self.output_topic} [{self.world_frame} -> {self.base_frame}], '
            f'extrinsic identity={self.extrinsic.is_identity}, publish_tf={self.publish_tf}')

    def _publish_extrinsic(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.imu_frame
        x, y, z = self.extrinsic.t_base_imu
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = x, y, z
        q = rot_to_quat(self.extrinsic.rot_base_imu)
        (t.transform.rotation.x, t.transform.rotation.y,
         t.transform.rotation.z, t.transform.rotation.w) = q
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.static_broadcaster.sendTransform(t)

    def _reject(self, reason, detail):
        self.counts[reason] += 1
        self.get_logger().warn(f'dropping estimator odometry ({reason}): {detail}',
                               throttle_duration_sec=5.0)

    def _on_odom(self, msg):
        self.counts['received'] += 1
        self.last_rx_ns = self.get_clock().now().nanoseconds

        try:
            stamp_ns = stamp_to_ns(msg.header.stamp.sec, msg.header.stamp.nanosec)
        except ValueError as exc:   # malformed input must never crash the adapter
            self._reject('bad_stamp', str(exc))
            return
        if self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns:
            self._reject('non_increasing_stamp',
                         f'{stamp_ns} <= previous {self.last_stamp_ns}')
            return

        pose, twist = msg.pose.pose, msg.twist.twist
        q = pose.orientation
        imu_odom = ImuOdometry(
            position=_vec3(pose.position),
            orientation_xyzw=np.array([q.x, q.y, q.z, q.w], dtype=float),
            linear_velocity=_vec3(twist.linear),
            angular_velocity=_vec3(twist.angular),
            pose_covariance=np.array(msg.pose.covariance, dtype=float).reshape(6, 6),
            twist_covariance=np.array(msg.twist.covariance, dtype=float).reshape(6, 6),
        )
        try:
            base = imu_to_base(imu_odom, self.extrinsic, self.velocity_frame)
        except OdometryRejected as exc:
            self._reject(exc.reason, str(exc))
            return

        self.last_stamp_ns = stamp_ns
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.world_frame
        out.child_frame_id = self.base_frame
        (out.pose.pose.position.x, out.pose.pose.position.y,
         out.pose.pose.position.z) = base.position.tolist()
        (out.pose.pose.orientation.x, out.pose.pose.orientation.y,
         out.pose.pose.orientation.z, out.pose.pose.orientation.w) = base.orientation_xyzw.tolist()
        (out.twist.twist.linear.x, out.twist.twist.linear.y,
         out.twist.twist.linear.z) = base.linear_velocity.tolist()
        (out.twist.twist.angular.x, out.twist.twist.angular.y,
         out.twist.twist.angular.z) = base.angular_velocity.tolist()
        out.pose.covariance = base.pose_covariance.reshape(-1).tolist()
        out.twist.covariance = base.twist_covariance.reshape(-1).tolist()
        self.odom_pub.publish(out)
        self.counts['published'] += 1

        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header = out.header
            t.child_frame_id = self.base_frame
            t.transform.translation.x = out.pose.pose.position.x
            t.transform.translation.y = out.pose.pose.position.y
            t.transform.translation.z = out.pose.pose.position.z
            t.transform.rotation = out.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)

    def _publish_status(self):
        now_ns = self.get_clock().now().nanoseconds
        status = DiagnosticStatus()
        status.name = 'ppb_vio: odometry'
        status.hardware_id = self.input_topic
        if self.last_rx_ns is None:
            status.level = DiagnosticStatus.STALE
            status.message = 'waiting for estimator output (not initialized yet?)'
        elif now_ns - self.last_rx_ns > self.stale_ns:
            status.level = DiagnosticStatus.ERROR
            status.message = (f'estimator output stale for '
                              f'{(now_ns - self.last_rx_ns) / NS_PER_S:.2f} s')
        elif self.counts['published'] == 0:
            status.level = DiagnosticStatus.ERROR
            status.message = 'receiving estimator output but every message is rejected'
        else:
            prev = self._counts_at_last_status
            received = self.counts['received'] - prev['received']
            dropped = received - (self.counts['published'] - prev['published'])
            status.level = DiagnosticStatus.WARN if dropped else DiagnosticStatus.OK
            status.message = (f'tracking ({dropped} of {received} messages dropped since last status)'
                              if dropped else 'tracking')
        self._counts_at_last_status = dict(self.counts)
        status.values = [KeyValue(key=k, value=str(v)) for k, v in self.counts.items()]
        if self.last_stamp_ns is not None:
            status.values.append(KeyValue(key='last_stamp_ns', value=str(self.last_stamp_ns)))

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.status_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VioOdomAdapter()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass   # Ctrl-C / launch shutdown is a normal exit
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
