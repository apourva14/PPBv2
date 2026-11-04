"""Forward /vio/odom to the fusion filter only while VIO is healthy (spec Requirement 4).

    /vio/odom, /vio/status  ->  vio_gate  ->  /vio/odom_gated  (odom1 of ekf_local / ekf_global)

Publishes its own state on /localization/vio_gate (diagnostic_msgs/DiagnosticArray).
"""

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from ppbv2_localization.lib.vio_gate import VioGate


def _level(raw):
    return raw if isinstance(raw, int) else int.from_bytes(raw, 'little')


class VioGateNode(Node):

    def __init__(self):
        super().__init__('vio_gate')
        self.declare_parameter('input_topic', '/vio/odom')
        self.declare_parameter('status_topic', '/vio/status')
        self.declare_parameter('output_topic', '/vio/odom_gated')
        self.declare_parameter('status_timeout_s', 3.0)
        p = self.get_parameter
        self.gate = VioGate(float(p('status_timeout_s').value))
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(Odometry, p('output_topic').value, qos)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/localization/vio_gate', 10)
        self.create_subscription(Odometry, p('input_topic').value, self._on_odom, qos)
        self.create_subscription(DiagnosticArray, p('status_topic').value, self._on_status, 10)
        self.counts = {'forwarded': 0, 'dropped': 0}
        self.last_reason = None
        self.create_timer(1.0, self._publish_diag)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_status(self, msg):
        for st in msg.status:
            self.gate.on_status(st.name, _level(st.level), st.message, self._now())

    def _on_odom(self, msg):
        decision = self.gate.decide(self._now())
        if decision.reason != self.last_reason:
            # Separate call sites: rclpy fixes a call site's severity on first use.
            if decision.forward:
                self.get_logger().info(f'VIO forwarded: {decision.reason}')
            else:
                self.get_logger().warn(f'VIO dropped: {decision.reason}')
            self.last_reason = decision.reason
        if decision.forward:
            self.pub.publish(msg)
            self.counts['forwarded'] += 1
        else:
            self.counts['dropped'] += 1

    def _publish_diag(self):
        decision = self.gate.decide(self._now())
        st = DiagnosticStatus()
        st.name = 'ppbv2_localization: vio gate'
        st.level = DiagnosticStatus.OK if decision.forward else DiagnosticStatus.WARN
        st.message = decision.reason
        st.values = [KeyValue(key=k, value=str(v)) for k, v in self.counts.items()]
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [st]
        self.diag_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VioGateNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
