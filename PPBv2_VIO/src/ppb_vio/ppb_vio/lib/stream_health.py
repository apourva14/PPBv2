"""Sensor stream health checks for VIO inputs (no ROS imports).

All times are integer nanoseconds. `stamp` is the message header stamp,
`recv` is the local clock when the message arrived (ROS time, so it follows
/clock during bag replay).
"""

from collections import deque
from dataclasses import dataclass, field
import math

from ppb_vio.lib.time_utils import NS_PER_S

OK, WARN, ERROR, STALE = 0, 1, 2, 3  # diagnostic_msgs/DiagnosticStatus levels


@dataclass
class HealthReport:
    level: int
    message: str
    values: dict = field(default_factory=dict)


class StreamHealth:
    """Rate, gap, and timestamp-ordering checks for one topic."""

    def __init__(self, expected_hz, rate_tolerance=0.2, max_gap_s=None,
                 window_s=2.0, stale_s=1.0):
        if expected_hz <= 0.0:
            raise ValueError('expected_hz must be positive')
        self.expected_hz = float(expected_hz)
        self.rate_tolerance = float(rate_tolerance)
        self.max_gap_ns = int((max_gap_s if max_gap_s is not None else 3.0 / expected_hz) * NS_PER_S)
        self.window_ns = int(window_s * NS_PER_S)
        self.stale_ns = int(stale_s * NS_PER_S)
        self._samples = deque()          # (stamp, recv) of accepted-order samples
        self._order_faults = deque()     # recv times of non-increasing stamps
        self._last_stamp = None
        self._last_recv = None
        self.total = 0
        self.total_order_faults = 0

    def add(self, stamp_ns, recv_ns):
        self.total += 1
        self._last_recv = recv_ns
        if self._last_stamp is not None and stamp_ns <= self._last_stamp:
            self.total_order_faults += 1
            self._order_faults.append(recv_ns)
            return
        self._last_stamp = stamp_ns
        self._samples.append((stamp_ns, recv_ns))

    def _evict(self, now_ns):
        horizon = now_ns - self.window_ns
        while self._samples and self._samples[0][1] < horizon:
            self._samples.popleft()
        while self._order_faults and self._order_faults[0] < horizon:
            self._order_faults.popleft()

    def evaluate(self, now_ns):
        if self._last_recv is None:
            return HealthReport(STALE, 'no messages received')
        self._evict(now_ns)

        age_s = (now_ns - self._last_recv) / NS_PER_S
        values = {
            'total_msgs': self.total,
            'total_order_faults': self.total_order_faults,
            'last_msg_age_s': round(age_s, 3),
            'expected_hz': self.expected_hz,
        }
        if now_ns - self._last_recv > self.stale_ns:
            return HealthReport(ERROR, f'stale: no message for {age_s:.2f} s', values)

        stamps = [s for s, _ in self._samples]
        rate = math.nan
        max_gap_s = math.nan
        if len(stamps) >= 2:
            span = stamps[-1] - stamps[0]
            rate = (len(stamps) - 1) * NS_PER_S / span
            max_gap_s = max(b - a for a, b in zip(stamps, stamps[1:])) / NS_PER_S
        values.update({'rate_hz': round(rate, 2), 'max_stamp_gap_s': round(max_gap_s, 4),
                       'order_faults_in_window': len(self._order_faults)})

        if self._order_faults:
            return HealthReport(
                ERROR, f'{len(self._order_faults)} non-increasing header stamps in window', values)
        if math.isnan(rate):
            return HealthReport(WARN, 'not enough messages in window to estimate rate', values)
        low = self.expected_hz * (1.0 - self.rate_tolerance)
        high = self.expected_hz * (1.0 + self.rate_tolerance)
        if not low <= rate <= high:
            return HealthReport(
                WARN, f'rate {rate:.1f} Hz outside [{low:.1f}, {high:.1f}] Hz', values)
        if max_gap_s * NS_PER_S > self.max_gap_ns:
            return HealthReport(WARN, f'stamp gap {max_gap_s * 1e3:.0f} ms exceeds limit', values)
        return HealthReport(OK, f'{rate:.1f} Hz', values)


class StereoSync:
    """Checks that left/right images carry matching header stamps."""

    def __init__(self, max_skew_s=0.001, history=100, min_match_fraction=0.95):
        self.max_skew_ns = int(max_skew_s * NS_PER_S)
        self.min_match_fraction = min_match_fraction
        self._left = deque(maxlen=history)
        self._right = deque(maxlen=history)

    def add(self, side, stamp_ns):
        (self._left if side == 0 else self._right).append(stamp_ns)

    def evaluate(self):
        if not self._left or not self._right:
            return HealthReport(STALE, 'waiting for both cameras')
        right = sorted(self._right)
        # Only judge left frames the right camera has had a chance to deliver.
        candidates = [s for s in self._left if s <= right[-1]]
        if not candidates:
            return HealthReport(WARN, 'no overlapping stereo stamps yet')
        skews = [min(abs(s - r) for r in right) for s in candidates]
        matched = sum(1 for k in skews if k <= self.max_skew_ns)
        fraction = matched / len(candidates)
        values = {'match_fraction': round(fraction, 3),
                  'max_skew_ms': round(max(skews) / 1e6, 3)}
        if fraction < self.min_match_fraction:
            return HealthReport(
                WARN, f'only {fraction:.0%} of left frames have a right frame within '
                f'{self.max_skew_ns / 1e6:.1f} ms', values)
        return HealthReport(OK, f'{fraction:.0%} stereo pairs matched', values)


class ImuSanity:
    """Unit and plausibility checks on raw IMU samples (SI units expected)."""

    def __init__(self, accel_norm_range=(3.0, 30.0), max_gyro_rad_s=10.0, history=400):
        self.accel_norm_range = accel_norm_range
        self.max_gyro_rad_s = max_gyro_rad_s
        self._accel = deque(maxlen=history)
        self._gyro = deque(maxlen=history)
        self.non_finite = 0

    def add(self, accel_xyz, gyro_xyz):
        values = tuple(accel_xyz) + tuple(gyro_xyz)
        if not all(math.isfinite(v) for v in values):
            self.non_finite += 1
            return
        self._accel.append(math.sqrt(sum(a * a for a in accel_xyz)))
        self._gyro.append(math.sqrt(sum(g * g for g in gyro_xyz)))

    def evaluate(self):
        if self.non_finite:
            return HealthReport(ERROR, f'{self.non_finite} IMU samples with NaN/Inf',
                                {'non_finite': self.non_finite})
        if not self._accel:
            return HealthReport(STALE, 'no IMU samples')
        mean_accel = sum(self._accel) / len(self._accel)
        max_gyro = max(self._gyro)
        values = {'mean_accel_norm': round(mean_accel, 3), 'max_gyro_norm': round(max_gyro, 3)}
        low, high = self.accel_norm_range
        if not low <= mean_accel <= high:
            return HealthReport(
                ERROR, f'mean |accel| {mean_accel:.2f} not in [{low}, {high}] m/s^2 '
                '(expected ~9.81; units in g?)', values)
        if max_gyro > self.max_gyro_rad_s:
            return HealthReport(
                WARN, f'|gyro| {max_gyro:.1f} rad/s is implausible (units in deg/s?)', values)
        return HealthReport(OK, f'|accel| {mean_accel:.2f} m/s^2', values)
