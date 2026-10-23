"""Monitor VIO input streams (stereo images + raw IMU) and report on /vio/status.

Read-only: subscribes to sensor topics and publishes diagnostics, nothing else.

The monitor must see exactly what the estimator sees and must not drop messages
itself, or it would report its own losses as input faults:
- reliability matches the estimators' camera subscriptions (MINS and OpenVINS
  both use RELIABLE message_filters subscribers); a best-effort subscriber loses
  whole fragmented images that the estimators still receive;
- images are taken as serialized bytes and only their header is decoded;
- every subscription has a deep (configurable) queue.
"""

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, Imu

from ppb_vio.lib.cdr_image_header import CdrError, parse_image_header
from ppb_vio.lib.stream_health import (
    ERROR, HealthReport, ImuSanity, OK, StereoSync, StreamHealth, WARN)
from ppb_vio.lib.time_utils import stamp_to_ns


class VioStreamMonitor(Node):

    def __init__(self):
        super().__init__('vio_stream_monitor')
        self.declare_parameter('camera_topics', ['/vio/cam0/image_raw', '/vio/cam1/image_raw'])
        self.declare_parameter('camera_info_topics',
                               ['/vio/cam0/camera_info', '/vio/cam1/camera_info'])
        self.declare_parameter('imu_topic', '/vio/imu')
        self.declare_parameter('camera_hz', 20.0)
        self.declare_parameter('imu_hz', 200.0)
        self.declare_parameter('rate_tolerance', 0.2)
        self.declare_parameter('stereo_max_skew_s', 0.001)
        self.declare_parameter('expected_imu_frame', 'imu_link')
        self.declare_parameter('status_topic', '/vio/status')
        self.declare_parameter('status_period_s', 1.0)
        self.declare_parameter('qos_depth', 100)
        self.declare_parameter('qos_reliability', 'reliable')

        p = self.get_parameter
        cam_topics = list(p('camera_topics').value)
        info_topics = list(p('camera_info_topics').value)
        if len(cam_topics) != len(info_topics) or not 1 <= len(cam_topics) <= 2:
            raise ValueError('need 1 or 2 camera topics with matching camera_info topics')
        tol = float(p('rate_tolerance').value)
        cam_hz = float(p('camera_hz').value)
        self.expected_imu_frame = p('expected_imu_frame').value

        self.cam_topics = cam_topics
        self.cam_health = [StreamHealth(cam_hz, tol) for _ in cam_topics]
        self.cam_frames = [None] * len(cam_topics)
        self.cam_info = [None] * len(cam_topics)
        self.stereo = StereoSync(float(p('stereo_max_skew_s').value)) if len(cam_topics) == 2 else None
        self.imu_topic = p('imu_topic').value
        self.imu_health = StreamHealth(float(p('imu_hz').value), tol)
        self.imu_sanity = ImuSanity()
        self.imu_frame_faults = 0
        self.imu_bad_stamps = 0
        self.image_parse_errors = [0] * len(cam_topics)

        reliability = {'reliable': ReliabilityPolicy.RELIABLE,
                       'best_effort': ReliabilityPolicy.BEST_EFFORT}.get(p('qos_reliability').value)
        if reliability is None:
            raise ValueError('qos_reliability must be reliable or best_effort')
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=int(p('qos_depth').value),
                         reliability=reliability)
        for i, (topic, info_topic) in enumerate(zip(cam_topics, info_topics)):
            self.create_subscription(
                Image, topic, lambda m, i=i: self._on_image(i, m), qos, raw=True)
            self.create_subscription(
                CameraInfo, info_topic, lambda m, i=i: self._on_info(i, m), qos)
        self.create_subscription(Imu, self.imu_topic, self._on_imu, qos)

        self.status_pub = self.create_publisher(DiagnosticArray, p('status_topic').value, 10)
        self.create_timer(float(p('status_period_s').value), self._publish_status)

    def _now_ns(self):
        return self.get_clock().now().nanoseconds

    def _on_image(self, i, serialized):
        try:
            h = parse_image_header(serialized)
            stamp = stamp_to_ns(h.sec, h.nanosec)
        except (CdrError, ValueError):   # malformed input is reported, never fatal
            self.image_parse_errors[i] += 1
            return
        self.cam_health[i].add(stamp, self._now_ns())
        self.cam_frames[i] = (h.frame_id, h.encoding, h.width, h.height)
        if self.stereo is not None:
            self.stereo.add(i, stamp)

    def _on_info(self, i, msg):
        self.cam_info[i] = msg

    def _on_imu(self, msg):
        try:
            stamp = stamp_to_ns(msg.header.stamp.sec, msg.header.stamp.nanosec)
        except ValueError:
            self.imu_bad_stamps += 1
            return
        self.imu_health.add(stamp, self._now_ns())
        if msg.header.frame_id != self.expected_imu_frame:
            self.imu_frame_faults += 1
        a, w = msg.linear_acceleration, msg.angular_velocity
        self.imu_sanity.add((a.x, a.y, a.z), (w.x, w.y, w.z))

    def _camera_info_report(self, i):
        info, frame = self.cam_info[i], self.cam_frames[i]
        if info is None:
            return HealthReport(WARN, 'no CameraInfo received')
        if frame is not None and (info.width, info.height) != (frame[2], frame[3]):
            return HealthReport(WARN, f'CameraInfo {info.width}x{info.height} does not match '
                                f'image {frame[2]}x{frame[3]}')
        if not any(info.k):
            return HealthReport(WARN, 'CameraInfo K is all zeros (uncalibrated)')
        return HealthReport(OK, 'ok', {'distortion_model': info.distortion_model})

    def _publish_status(self):
        now = self._now_ns()
        statuses = []

        def add(name, hardware_id, report, extra=None):
            s = DiagnosticStatus(level=bytes([report.level]), name=f'ppb_vio: {name}',
                                 message=report.message, hardware_id=hardware_id)
            values = dict(report.values)
            values.update(extra or {})
            s.values = [KeyValue(key=k, value=str(v)) for k, v in values.items()]
            statuses.append(s)

        for i, topic in enumerate(self.cam_topics):
            frame = self.cam_frames[i]
            extra = {} if frame is None else {
                'frame_id': frame[0], 'encoding': frame[1], 'resolution': f'{frame[2]}x{frame[3]}'}
            report = self.cam_health[i].evaluate(now)
            if self.image_parse_errors[i]:
                report = HealthReport(ERROR, f'{self.image_parse_errors[i]} images could not '
                                      'be parsed (malformed message or stamp)', report.values)
            add(f'camera {i} stream', topic, report, extra)
            add(f'camera {i} info', topic, self._camera_info_report(i))
        if self.stereo is not None:
            add('stereo sync', ','.join(self.cam_topics), self.stereo.evaluate())

        imu_stream = self.imu_health.evaluate(now)
        if self.imu_bad_stamps:
            imu_stream = HealthReport(ERROR, f'{self.imu_bad_stamps} IMU messages with an '
                                      'invalid stamp', imu_stream.values)
        add('imu stream', self.imu_topic, imu_stream)
        imu_report = self.imu_sanity.evaluate()
        if self.imu_frame_faults and imu_report.level < WARN:
            imu_report = HealthReport(
                WARN, f'{self.imu_frame_faults} IMU messages not in frame '
                f'{self.expected_imu_frame!r}', imu_report.values)
        add('imu sanity', self.imu_topic, imu_report)

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = statuses
        self.status_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VioStreamMonitor()
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
