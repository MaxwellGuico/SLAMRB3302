#!/usr/bin/python3
"""Exercise the RB3302 simulation's ROS-facing hardware contract."""

from __future__ import annotations

import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState, LaserScan


class SimulationSmokeTest(Node):
    def __init__(self) -> None:
        super().__init__("rb3302_sim_smoke_test")
        self.clock: Clock | None = None
        self.odom: Odometry | None = None
        self.imu: Imu | None = None
        self.scan: LaserScan | None = None
        self.joints: JointState | None = None
        self.initial_odom: tuple[float, float, float] | None = None
        self.initial_rear_range: float | None = None

        self.create_subscription(Clock, "/clock", self.on_clock, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom/raw", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Imu, "/imu_data_ros", self.on_imu, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(JointState, "/joint_states", self.on_joints, qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def on_clock(self, message: Clock) -> None:
        self.clock = message

    def on_odom(self, message: Odometry) -> None:
        self.odom = message
        if self.initial_odom is None:
            pose = message.pose.pose
            self.initial_odom = (pose.position.x, pose.position.y, self._yaw(message))

    def on_imu(self, message: Imu) -> None:
        self.imu = message

    def on_scan(self, message: LaserScan) -> None:
        self.scan = message
        if self.initial_rear_range is None and message.ranges:
            zero_index = round((0.0 - message.angle_min) / message.angle_increment)
            zero_index = min(max(zero_index, 0), len(message.ranges) - 1)
            value = message.ranges[zero_index]
            if math.isfinite(value):
                self.initial_rear_range = value

    def on_joints(self, message: JointState) -> None:
        self.joints = message

    @staticmethod
    def _yaw(message: Odometry) -> float:
        q = message.pose.pose.orientation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def ready(self) -> bool:
        return all((self.clock, self.odom, self.imu, self.scan, self.joints))

    def spin_until(self, predicate, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def command_for(self, linear: float, angular: float, duration: float) -> None:
        command = Twist()
        command.linear.x = linear
        command.angular.z = angular
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            self.cmd_pub.publish(command)
            rclpy.spin_once(self, timeout_sec=0.05)
        stop = Twist()
        for _ in range(5):
            self.cmd_pub.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.04)

    def validate(self) -> list[str]:
        failures: list[str] = []
        assert self.odom and self.imu and self.scan and self.joints and self.initial_odom
        if self.odom.header.frame_id != "odom":
            failures.append(f"odometry frame is {self.odom.header.frame_id!r}, expected 'odom'")
        if self.odom.child_frame_id != "base_link":
            failures.append(f"odometry child frame is {self.odom.child_frame_id!r}, expected 'base_link'")
        if self.imu.header.frame_id != "gyro_link":
            failures.append(f"IMU frame is {self.imu.header.frame_id!r}, expected 'gyro_link'")
        if self.scan.header.frame_id != "laser":
            failures.append(f"scan frame is {self.scan.header.frame_id!r}, expected 'laser'")
        if len(self.scan.ranges) != 720:
            failures.append(f"scan has {len(self.scan.ranges)} samples, expected 720")
        expected_joints = {"left_wheel_joint", "right_wheel_joint"}
        if not expected_joints.issubset(set(self.joints.name)):
            failures.append(f"joint states missing {sorted(expected_joints - set(self.joints.name))}")

        x0, y0, yaw0 = self.initial_odom
        pose = self.odom.pose.pose.position
        distance = math.hypot(pose.x - x0, pose.y - y0)
        yaw_delta = abs(math.atan2(math.sin(self._yaw(self.odom) - yaw0), math.cos(self._yaw(self.odom) - yaw0)))
        if distance < 0.15:
            failures.append(f"straight-line odometry displacement was only {distance:.3f} m")
        if yaw_delta < 0.35:
            failures.append(f"rotation odometry change was only {yaw_delta:.3f} rad")
        return failures


def main() -> int:
    rclpy.init()
    node = SimulationSmokeTest()
    try:
        if not node.spin_until(node.ready, timeout=20.0):
            missing = [
                name for name, value in (
                    ("clock", node.clock),
                    ("odom/raw", node.odom),
                    ("imu_data_ros", node.imu),
                    ("scan", node.scan),
                    ("joint_states", node.joints),
                ) if value is None
            ]
            print(f"FAIL: timed out waiting for: {', '.join(missing)}", file=sys.stderr)
            return 1

        node.command_for(linear=0.18, angular=0.0, duration=1.5)
        node.command_for(linear=0.0, angular=0.55, duration=1.2)
        node.spin_until(lambda: False, timeout=0.5)
        failures = node.validate()
        if failures:
            for failure in failures:
                print(f"FAIL: {failure}", file=sys.stderr)
            return 1

        assert node.odom and node.scan
        x0, y0, _ = node.initial_odom or (0.0, 0.0, 0.0)
        pose = node.odom.pose.pose.position
        distance = math.hypot(pose.x - x0, pose.y - y0)
        print(
            "PASS: clock, odometry, IMU, 720-ray LiDAR, joint states, and drive control; "
            f"travel={distance:.3f} m yaw={SimulationSmokeTest._yaw(node.odom):.3f} rad"
        )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
