"""Decide whether VIO odometry may enter the fusion filter (no ROS imports).

Spec, Requirement 4 (graceful VIO dropout): the /vio/status gate drops VIO and the EKF
coasts on wheel odometry + IMU. VIO is forwarded only while the adapter's divergence
gate ('ppb_vio: vio health') reports OK or WARN and that report is fresh. Once it
reports ERROR the VIO stays dropped until the estimator is restarted (the health gate
latches too: a diverged filter does not recover, and a restarted one starts in a new
vio_world frame, which differential fusion must not straddle).

Levels follow diagnostic_msgs/DiagnosticStatus: 0 OK, 1 WARN, 2 ERROR, 3 STALE.
"""

from dataclasses import dataclass

HEALTH_STATUS = 'ppb_vio: vio health'
OK, WARN, ERROR, STALE = 0, 1, 2, 3


@dataclass
class GateDecision:
    forward: bool
    reason: str


class VioGate:

    def __init__(self, status_timeout_s=3.0):
        self.status_timeout_s = status_timeout_s
        self.last_level = None
        self.last_status_time = None
        self.dropped_permanently = False
        self.reason = 'no VIO health report yet'

    def on_status(self, name, level, message, now_s):
        """Feed one DiagnosticStatus (name, level, message) received at `now_s`."""
        if name != HEALTH_STATUS:
            return
        self.last_level, self.last_status_time = level, now_s
        if level == ERROR and not self.dropped_permanently:
            self.dropped_permanently = True
            self.reason = f'VIO diverged ({message}); dropped until the estimator restarts'

    def decide(self, now_s):
        if self.dropped_permanently:
            return GateDecision(False, self.reason)
        if self.last_status_time is None:
            return GateDecision(False, 'no VIO health report yet')
        if now_s - self.last_status_time > self.status_timeout_s:
            return GateDecision(False, f'VIO health report stale (> {self.status_timeout_s:g} s)')
        if self.last_level in (OK, WARN):
            return GateDecision(True, 'VIO healthy')
        return GateDecision(False, f'VIO health level {self.last_level}')
