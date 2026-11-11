"""Differential-drive odometry from the WAVE ROVER's per-wheel distances.

Kept free of ROS imports so it can be unit-tested anywhere.

Why this exists (Oct 18):
- The firmware's ``dist_centre`` never goes down, so reversing looked like
  driving forward and a spin in place looked like forward travel.
- The firmware's ``yaw_odom`` barely moves during a real spin (about 0.8 rad
  over two 90 degree turns in the yaw_diagnostic run), so it cannot be used
  as the robot's heading.

Both are rebuilt here from the signed ``dist_L`` / ``dist_R`` totals:
  forward distance = (dR + dL) / 2
  heading change   = (dR - dL) / wheel_track
"""

import math


class WheelOdometry:
    def __init__(self, wheel_track=0.288, dist_scale=1.0, max_step=0.20,
                 velocity_window=0.05):
        # wheel_track: effective left-right wheel separation in metres. On a
        # skid-steer base this is larger than the measured 0.288 m because
        # the wheels slip sideways in a turn; calibrate it with a 360 spin.
        self.wheel_track = float(wheel_track)
        self.dist_scale = float(dist_scale)
        # A jump bigger than this between two packets is treated as the
        # firmware resetting its counters, not as motion.
        self.max_step = float(max_step)
        # Velocities are measured over at least this many seconds so a
        # 100 Hz packet stream with quantised distances does not jitter.
        self.velocity_window = float(velocity_window)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.resets = 0

        self._prev_l = None
        self._prev_r = None
        self._win_t = None
        self._win_dist = 0.0
        self._win_yaw = 0.0

    def update(self, dist_l, dist_r, t):
        """Integrate one feedback packet. ``t`` is in seconds."""
        dist_l = float(dist_l) * self.dist_scale
        dist_r = float(dist_r) * self.dist_scale

        if self._prev_l is None:
            self._prev_l, self._prev_r, self._win_t = dist_l, dist_r, t
            return

        d_l = dist_l - self._prev_l
        d_r = dist_r - self._prev_r
        self._prev_l, self._prev_r = dist_l, dist_r

        if abs(d_l) > self.max_step or abs(d_r) > self.max_step:
            self.resets += 1
            return

        d_centre = 0.5 * (d_l + d_r)
        d_yaw = (d_r - d_l) / self.wheel_track

        # Midpoint heading keeps arcs accurate at low packet rates.
        mid_yaw = self.yaw + 0.5 * d_yaw
        self.x += d_centre * math.cos(mid_yaw)
        self.y += d_centre * math.sin(mid_yaw)
        self.yaw = math.atan2(math.sin(self.yaw + d_yaw),
                              math.cos(self.yaw + d_yaw))

        self._win_dist += d_centre
        self._win_yaw += d_yaw
        elapsed = t - self._win_t
        if elapsed >= self.velocity_window:
            self.linear_velocity = self._win_dist / elapsed
            self.angular_velocity = self._win_yaw / elapsed
            self._win_t = t
            self._win_dist = 0.0
            self._win_yaw = 0.0

    def decay_if_stale(self, t, timeout=0.2):
        """Zero the velocities when no packet has arrived for ``timeout`` s."""
        if self._win_t is not None and t - self._win_t > timeout:
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0
