"""Run with: python3 -m pytest test/test_wheel_odometry.py (no ROS needed)."""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "waverover_base"))

from wheel_odometry import WheelOdometry  # noqa: E402

TRACK = 0.288


def drive(v_left, v_right, seconds, hz=100):
    odom = WheelOdometry(wheel_track=TRACK)
    left = right = t = 0.0
    odom.update(left, right, t)
    for _ in range(int(seconds * hz)):
        t += 1.0 / hz
        left += v_left / hz
        right += v_right / hz
        # Firmware reports distances to 0.1 mm.
        odom.update(round(left, 4), round(right, 4), t)
    return odom


def test_forward():
    odom = drive(0.2, 0.2, 5)
    assert abs(odom.x - 1.0) < 0.01
    assert abs(odom.yaw) < 1e-3
    assert abs(odom.linear_velocity - 0.2) < 0.01


def test_reverse_is_negative():
    odom = drive(-0.2, -0.2, 5)
    assert abs(odom.x + 1.0) < 0.01
    assert odom.linear_velocity < 0.0


def test_spin_in_place_rotates_without_moving():
    v = 1.0 * TRACK / 2
    odom = drive(-v, v, math.pi / 2)
    assert abs(odom.yaw - math.pi / 2) < 0.01
    assert math.hypot(odom.x, odom.y) < 0.01
    assert abs(odom.angular_velocity - 1.0) < 0.02


def test_clockwise_spin_is_negative_yaw():
    v = 1.0 * TRACK / 2
    odom = drive(v, -v, math.pi / 2)
    assert abs(odom.yaw + math.pi / 2) < 0.01


def test_quarter_arc():
    radius, w = 1.0, 0.2
    odom = drive(w * (radius - TRACK / 2), w * (radius + TRACK / 2), math.pi / 2 / w)
    assert abs(odom.x - 1.0) < 0.01
    assert abs(odom.y - 1.0) < 0.01
    assert abs(odom.yaw - math.pi / 2) < 0.01


def test_counter_reset_is_ignored():
    odom = WheelOdometry()
    odom.update(5.0, 5.0, 0.00)
    odom.update(5.01, 5.01, 0.01)
    odom.update(0.0, 0.0, 0.02)
    odom.update(0.01, 0.01, 0.03)
    assert abs(odom.x - 0.02) < 1e-6
    assert odom.resets == 1


def test_stale_velocity_decays():
    odom = drive(0.2, 0.2, 1)
    odom.decay_if_stale(10.0)
    assert odom.linear_velocity == 0.0
    assert odom.angular_velocity == 0.0
