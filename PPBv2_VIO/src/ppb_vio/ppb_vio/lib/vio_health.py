"""Plausibility gate on /vio/odom for a ground robot (no ROS imports).

A diverging filter usually stays overconfident (its covariance does not grow),
so covariance alone cannot flag it. What it cannot hide is physics: a field
robot does not exceed its top speed, does not climb or sink at metres per
second, and does not accelerate like a drone. The gate latches ERROR once such
a violation persists: a diverged filter does not recover without
re-initialization, and the fusion layer must stop using it (spec: Requirement
4, graceful VIO dropout).

Motion is measured from base_link *displacement* over `window_s`, never from
the instantaneous twist: base_link is metres away from the IMU on a vibrating
mast (Rosario: 2.2 m), so mast pitch oscillation alone puts ~1.8 m/s of
apparent vertical velocity on base_link, and filter corrections make the
velocity estimate jump. Both average out over the window; a runaway does not.

Levels follow diagnostic_msgs/DiagnosticStatus: 0 OK, 1 WARN, 2 ERROR.
"""

from collections import deque
from dataclasses import dataclass, field

import numpy as np

OK, WARN, ERROR = 0, 1, 2


@dataclass(frozen=True)
class HealthLimits:
    max_speed_mps: float = 3.0           # horizontal; Amiga / Rosario robots drive <= ~1.5 m/s
    max_vertical_speed_mps: float = 0.5  # ground robot on a field
    max_accel_mps2: float = 4.0          # between consecutive window velocities
    max_position_sigma_m: float = 50.0   # WARN only: VIO position uncertainty grows by design
    window_s: float = 0.5                # displacement window for velocities
    persist_s: float = 1.0               # a violation must last this long to latch ERROR


@dataclass
class HealthState:
    level: int = OK
    message: str = 'waiting for first output'
    latched_at_s: object = None          # stamp (s) when ERROR latched, else None
    violation_since_s: object = None
    last_window_velocity: object = None  # (stamp_s, velocity) of the last full window
    accel_violation: object = None       # result of the last acceleration check, kept until the next
    reasons: dict = field(default_factory=dict)


class VioHealthGate:
    """Feed base_link odometry samples in stamp order; read `state` afterwards."""

    def __init__(self, limits=HealthLimits()):
        self.limits = limits
        self.state = HealthState()
        self._history = deque()          # (stamp_s, position_world)

    def update(self, stamp_s, position_world, pose_covariance):
        """One /vio/odom sample: base_link position in the VIO world frame, 6x6 pose cov."""
        s, lim = self.state, self.limits
        if s.latched_at_s is not None:
            return s.level
        p = np.asarray(position_world, dtype=float)
        cov = np.asarray(pose_covariance, dtype=float).reshape(6, 6)

        problems = []
        if not (np.all(np.isfinite(p)) and np.all(np.isfinite(cov))):
            problems.append('non-finite state')
        else:
            self._history.append((stamp_s, p))
            while len(self._history) > 1 and stamp_s - self._history[1][0] >= lim.window_s:
                self._history.popleft()
            t0, p0 = self._history[0]
            if stamp_s - t0 >= lim.window_s:
                v = (p - p0) / (stamp_s - t0)
                speed = float(np.hypot(v[0], v[1]))
                if speed > lim.max_speed_mps:
                    problems.append(f'speed {speed:.1f} m/s > {lim.max_speed_mps}')
                if abs(v[2]) > lim.max_vertical_speed_mps:
                    problems.append(f'vertical speed {v[2]:+.2f} m/s')
                last = s.last_window_velocity
                if last is not None and stamp_s - last[0] >= lim.window_s:
                    accel = float(np.linalg.norm(v - last[1]) / (stamp_s - last[0]))
                    s.accel_violation = (f'acceleration {accel:.1f} m/s^2'
                                         if accel > lim.max_accel_mps2 else None)
                    s.last_window_velocity = (stamp_s, v)
                elif last is None:
                    s.last_window_velocity = (stamp_s, v)
            if s.accel_violation:
                problems.append(s.accel_violation)

        if problems:
            for prob in problems:
                key = prob.split(' ')[0]
                s.reasons[key] = s.reasons.get(key, 0) + 1
            if s.violation_since_s is None:
                s.violation_since_s = stamp_s
            if stamp_s - s.violation_since_s >= lim.persist_s:
                s.level = ERROR
                s.latched_at_s = stamp_s
                s.message = 'DIVERGED (latched until estimator restart): ' + '; '.join(problems)
            else:
                s.level = WARN
                s.message = 'implausible motion: ' + '; '.join(problems)
            return s.level

        s.violation_since_s = None
        sigma = float(np.sqrt(max(np.trace(cov[:3, :3]), 0.0)))
        if sigma > lim.max_position_sigma_m:
            s.level, s.message = WARN, f'position sigma {sigma:.1f} m'
        else:
            s.level, s.message = OK, 'plausible'
        return s.level
