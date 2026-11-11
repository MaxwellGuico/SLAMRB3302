#!/usr/bin/env python3
"""Publish hardware-like IMU and wheel odometry from ideal Gazebo sensors.

The WAVE ROVER driver on the IRL branch integrates centre-wheel distance using
firmware yaw, while the H30 is fused primarily as a yaw-rate sensor.  Gazebo's
native DiffDrive and IMU outputs are much cleaner than either source.  This
node preserves the real topics but models the errors that matter to that EKF:
wheel scale/mismatch, skid-turn error, quantization, IMU bias/random walk,
white noise, covariance, transport latency, jitter, and packet loss.
"""

from __future__ import annotations

import copy
import heapq
import math
import random
from dataclasses import dataclass
from typing import Callable

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def clamp_probability(value: float) -> float:
    return min(1.0, max(0.0, value))


def angle_difference(current: float, previous: float) -> float:
    return math.atan2(math.sin(current - previous), math.cos(current - previous))


def quantize(value: float, resolution: float) -> float:
    if resolution <= 0.0:
        return value
    return round(value / resolution) * resolution


def quaternion_to_rpy(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def stamp_seconds(message: Odometry | Imu) -> float:
    stamp = message.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def diagonal_covariance(x: float, y: float, z: float) -> list[float]:
    covariance = [0.0] * 9
    covariance[0] = x * x
    covariance[4] = y * y
    covariance[8] = z * z
    return covariance


@dataclass
class RealismParameters:
    enabled: bool
    odom_linear_scale: float
    odom_angular_scale: float
    odom_wheel_mismatch_rad_per_m: float
    odom_distance_noise_m_per_sqrt_m: float
    odom_turn_slip_stddev_fraction: float
    odom_distance_quantization_m: float
    odom_yaw_quantization_rad: float
    odom_linear_velocity_noise_stddev: float
    odom_angular_velocity_noise_stddev: float
    odom_position_stddev: float
    odom_position_drift_stddev_per_m: float
    odom_yaw_stddev: float
    odom_yaw_drift_stddev_per_rad: float
    odom_linear_velocity_stddev: float
    odom_angular_velocity_stddev: float
    imu_orientation_noise_stddev: float
    imu_orientation_stddev: float
    imu_invert_yaw: bool
    imu_orientation_bias_stddev: float
    imu_orientation_bias_random_walk: float
    imu_gyro_noise_stddev: float
    imu_gyro_bias_stddev: float
    imu_gyro_bias_random_walk: float
    imu_accel_noise_stddev: float
    imu_accel_bias_stddev: float
    imu_accel_bias_random_walk: float
    imu_accel_stddev: float


class SensorErrorModel:
    """Stateful stochastic model, kept separate from ROS transport/latency."""

    def __init__(self, parameters: RealismParameters, rng: random.Random):
        self.p = parameters
        self.rng = rng
        self.reset()

    def reset(self) -> None:
        self.previous_odom_time: float | None = None
        self.previous_ideal_x = 0.0
        self.previous_ideal_y = 0.0
        self.previous_ideal_yaw = 0.0
        self.measured_x = 0.0
        self.measured_y = 0.0
        self.measured_yaw = 0.0
        self.travelled_distance = 0.0
        self.accumulated_turn = 0.0

        self.previous_imu_time: float | None = None
        # The H30 publishes an internally fused absolute orientation.  Its
        # gyro bias still appears in angular_velocity, but must not be
        # integrated open-loop into that fused attitude.  Model the latter
        # with its own much slower bias/random walk instead.
        self.imu_yaw_error = self.rng.gauss(
            0.0, self.p.imu_orientation_bias_stddev
        )
        self.gyro_bias = [self.rng.gauss(0.0, self.p.imu_gyro_bias_stddev) for _ in range(3)]
        self.accel_bias = [self.rng.gauss(0.0, self.p.imu_accel_bias_stddev) for _ in range(3)]

    def distort_odom(self, source: Odometry) -> Odometry:
        message = copy.deepcopy(source)
        if not self.p.enabled:
            return message

        now = stamp_seconds(source)
        pose = source.pose.pose
        _, _, ideal_yaw = quaternion_to_rpy(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )

        if self.previous_odom_time is None or now <= self.previous_odom_time:
            self.previous_odom_time = now
            self.previous_ideal_x = pose.position.x
            self.previous_ideal_y = pose.position.y
            self.previous_ideal_yaw = ideal_yaw
            self.measured_x = pose.position.x
            self.measured_y = pose.position.y
            self.measured_yaw = ideal_yaw
            self._set_odom_covariance(message)
            return message

        dt = now - self.previous_odom_time
        dx = pose.position.x - self.previous_ideal_x
        dy = pose.position.y - self.previous_ideal_y
        ideal_delta_yaw = angle_difference(ideal_yaw, self.previous_ideal_yaw)
        midpoint_yaw = self.previous_ideal_yaw + 0.5 * ideal_delta_yaw
        signed_distance = dx * math.cos(midpoint_yaw) + dy * math.sin(midpoint_yaw)

        distance_noise_stddev = self.p.odom_distance_noise_m_per_sqrt_m * math.sqrt(
            max(abs(signed_distance), 1.0e-6)
        )
        measured_distance = (
            signed_distance * self.p.odom_linear_scale
            + self.rng.gauss(0.0, distance_noise_stddev)
        )
        measured_delta_yaw = (
            ideal_delta_yaw * self.p.odom_angular_scale
            + signed_distance * self.p.odom_wheel_mismatch_rad_per_m
            + self.rng.gauss(
                0.0,
                self.p.odom_turn_slip_stddev_fraction * abs(ideal_delta_yaw),
            )
        )

        self.measured_yaw = quantize(
            self.measured_yaw + measured_delta_yaw,
            self.p.odom_yaw_quantization_rad,
        )
        measured_distance = quantize(
            measured_distance,
            self.p.odom_distance_quantization_m,
        )
        measured_midpoint_yaw = self.measured_yaw - 0.5 * measured_delta_yaw
        self.measured_x += measured_distance * math.cos(measured_midpoint_yaw)
        self.measured_y += measured_distance * math.sin(measured_midpoint_yaw)
        self.travelled_distance += abs(signed_distance)
        self.accumulated_turn += abs(ideal_delta_yaw)

        message.pose.pose.position.x = self.measured_x
        message.pose.pose.position.y = self.measured_y
        qx, qy, qz, qw = rpy_to_quaternion(0.0, 0.0, self.measured_yaw)
        message.pose.pose.orientation.x = qx
        message.pose.pose.orientation.y = qy
        message.pose.pose.orientation.z = qz
        message.pose.pose.orientation.w = qw
        message.twist.twist.linear.x = (
            measured_distance / dt
            + self.rng.gauss(0.0, self.p.odom_linear_velocity_noise_stddev)
        )
        message.twist.twist.angular.z = (
            measured_delta_yaw / dt
            + self.rng.gauss(0.0, self.p.odom_angular_velocity_noise_stddev)
        )
        self._set_odom_covariance(message)

        self.previous_odom_time = now
        self.previous_ideal_x = pose.position.x
        self.previous_ideal_y = pose.position.y
        self.previous_ideal_yaw = ideal_yaw
        return message

    def _set_odom_covariance(self, message: Odometry) -> None:
        position_stddev = self.p.odom_position_stddev + (
            self.p.odom_position_drift_stddev_per_m * self.travelled_distance
        )
        yaw_stddev = self.p.odom_yaw_stddev + (
            self.p.odom_yaw_drift_stddev_per_rad * self.accumulated_turn
        )
        message.pose.covariance = [0.0] * 36
        message.pose.covariance[0] = position_stddev * position_stddev
        message.pose.covariance[7] = position_stddev * position_stddev
        message.pose.covariance[14] = 1.0e3
        message.pose.covariance[21] = 1.0e3
        message.pose.covariance[28] = 1.0e3
        message.pose.covariance[35] = yaw_stddev * yaw_stddev

        message.twist.covariance = [0.0] * 36
        message.twist.covariance[0] = self.p.odom_linear_velocity_stddev**2
        message.twist.covariance[7] = 1.0e3
        message.twist.covariance[14] = 1.0e3
        message.twist.covariance[21] = 1.0e3
        message.twist.covariance[28] = 1.0e3
        message.twist.covariance[35] = self.p.odom_angular_velocity_stddev**2

    def distort_imu(self, source: Imu) -> Imu:
        message = copy.deepcopy(source)
        if not self.p.enabled:
            return message

        now = stamp_seconds(source)
        if self.previous_imu_time is None or now <= self.previous_imu_time:
            dt = 0.0
        else:
            dt = min(now - self.previous_imu_time, 0.1)
        self.previous_imu_time = now

        if dt > 0.0:
            sqrt_dt = math.sqrt(dt)
            for axis in range(3):
                self.gyro_bias[axis] += self.rng.gauss(
                    0.0, self.p.imu_gyro_bias_random_walk * sqrt_dt
                )
                self.accel_bias[axis] += self.rng.gauss(
                    0.0, self.p.imu_accel_bias_random_walk * sqrt_dt
                )
            self.imu_yaw_error += self.rng.gauss(
                0.0,
                self.p.imu_orientation_bias_random_walk * sqrt_dt,
            )

        angular_values = [
            source.angular_velocity.x,
            source.angular_velocity.y,
            source.angular_velocity.z,
        ]
        acceleration_values = [
            source.linear_acceleration.x,
            source.linear_acceleration.y,
            source.linear_acceleration.z,
        ]
        noisy_angular = [
            value + self.gyro_bias[axis] + self.rng.gauss(0.0, self.p.imu_gyro_noise_stddev)
            for axis, value in enumerate(angular_values)
        ]
        noisy_acceleration = [
            value + self.accel_bias[axis] + self.rng.gauss(0.0, self.p.imu_accel_noise_stddev)
            for axis, value in enumerate(acceleration_values)
        ]
        message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z = noisy_angular
        (
            message.linear_acceleration.x,
            message.linear_acceleration.y,
            message.linear_acceleration.z,
        ) = noisy_acceleration

        orientation = source.orientation
        roll, pitch, yaw = quaternion_to_rpy(
            orientation.x, orientation.y, orientation.z, orientation.w
        )
        # Gazebo reports the orientation of the roll-pi FRD sensor link, so
        # its yaw changes opposite to base_link.  The IRL navigation stack
        # consumes the H30 attitude as the robot heading; convert that sign
        # before adding its realistic fused-attitude errors.
        if self.p.imu_invert_yaw:
            yaw = -yaw
        orientation_noise = self.p.imu_orientation_noise_stddev
        qx, qy, qz, qw = rpy_to_quaternion(
            roll + self.rng.gauss(0.0, orientation_noise),
            pitch + self.rng.gauss(0.0, orientation_noise),
            yaw + self.imu_yaw_error + self.rng.gauss(0.0, orientation_noise),
        )
        message.orientation.x = qx
        message.orientation.y = qy
        message.orientation.z = qz
        message.orientation.w = qw

        message.orientation_covariance = diagonal_covariance(
            self.p.imu_orientation_stddev,
            self.p.imu_orientation_stddev,
            self.p.imu_orientation_stddev,
        )
        message.angular_velocity_covariance = diagonal_covariance(
            self.p.imu_gyro_noise_stddev,
            self.p.imu_gyro_noise_stddev,
            self.p.imu_gyro_noise_stddev,
        )
        message.linear_acceleration_covariance = diagonal_covariance(
            self.p.imu_accel_stddev,
            self.p.imu_accel_stddev,
            self.p.imu_accel_stddev,
        )
        return message


class SensorRealismNode(Node):
    def __init__(self) -> None:
        super().__init__("sensor_realism")
        defaults: dict[str, object] = {
            "enabled": True,
            "random_seed": 3302,
            "ideal_odom_topic": "/sim/odom/ideal",
            "output_odom_topic": "/odom/raw",
            "ideal_imu_topic": "/sim/imu/ideal",
            "output_imu_topic": "/imu_data_ros",
            "odom_latency_ms": 18.0,
            "odom_latency_jitter_ms": 5.0,
            "odom_dropout_probability": 0.002,
            "odom_linear_scale": 1.012,
            "odom_angular_scale": 0.94,
            "odom_wheel_mismatch_rad_per_m": 0.012,
            "odom_distance_noise_m_per_sqrt_m": 0.004,
            "odom_turn_slip_stddev_fraction": 0.045,
            "odom_distance_quantization_m": 0.0005,
            "odom_yaw_quantization_rad": 0.0015,
            "odom_linear_velocity_noise_stddev": 0.015,
            "odom_angular_velocity_noise_stddev": 0.035,
            "odom_position_stddev": 0.025,
            "odom_position_drift_stddev_per_m": 0.018,
            "odom_yaw_stddev": 0.08,
            "odom_yaw_drift_stddev_per_rad": 0.06,
            "odom_linear_velocity_stddev": 0.03,
            "odom_angular_velocity_stddev": 0.10,
            "imu_latency_ms": 7.0,
            "imu_latency_jitter_ms": 2.0,
            "imu_dropout_probability": 0.001,
            "imu_orientation_noise_stddev": 0.012,
            "imu_orientation_stddev": 0.10,
            "imu_invert_yaw": True,
            "imu_orientation_bias_stddev": 0.025,
            "imu_orientation_bias_random_walk": 0.0003,
            "imu_gyro_noise_stddev": 0.03,
            "imu_gyro_bias_stddev": 0.010,
            "imu_gyro_bias_random_walk": 0.0008,
            "imu_accel_noise_stddev": 0.12,
            "imu_accel_bias_stddev": 0.08,
            "imu_accel_bias_random_walk": 0.006,
            "imu_accel_stddev": 0.35,
            "release_rate_hz": 500.0,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

        value: Callable[[str], object] = lambda name: self.get_parameter(name).value
        model_parameters = RealismParameters(
            **{
                field: value(field)
                for field in RealismParameters.__dataclass_fields__
            }
        )
        self.rng = random.Random(int(value("random_seed")))
        self.model = SensorErrorModel(model_parameters, self.rng)
        self.enabled = bool(value("enabled"))
        self.odom_latency = float(value("odom_latency_ms")) * 1.0e-3
        self.odom_jitter = float(value("odom_latency_jitter_ms")) * 1.0e-3
        self.odom_dropout = clamp_probability(float(value("odom_dropout_probability")))
        self.imu_latency = float(value("imu_latency_ms")) * 1.0e-3
        self.imu_jitter = float(value("imu_latency_jitter_ms")) * 1.0e-3
        self.imu_dropout = clamp_probability(float(value("imu_dropout_probability")))
        self.pending: list[tuple[float, int, object, object]] = []
        self.last_due_time = {"odom": -math.inf, "imu": -math.inf}
        self.sequence = 0

        self.odom_publisher = self.create_publisher(
            Odometry, str(value("output_odom_topic")), qos_profile_sensor_data
        )
        self.imu_publisher = self.create_publisher(
            Imu, str(value("output_imu_topic")), qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry,
            str(value("ideal_odom_topic")),
            self._odom_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            str(value("ideal_imu_topic")),
            self._imu_callback,
            qos_profile_sensor_data,
        )
        release_rate = max(50.0, float(value("release_rate_hz")))
        self.create_timer(1.0 / release_rate, self._release_messages)
        state = "enabled" if self.enabled else "pass-through"
        self.get_logger().info(
            f"Sensor realism {state}; seed={int(value('random_seed'))}, "
            f"odom latency={self.odom_latency * 1e3:.1f} ms, "
            f"IMU latency={self.imu_latency * 1e3:.1f} ms"
        )

    def _odom_callback(self, message: Odometry) -> None:
        if self.enabled and self.rng.random() < self.odom_dropout:
            return
        self._enqueue(
            self.model.distort_odom(message),
            self.odom_publisher,
            self.odom_latency,
            self.odom_jitter,
            "odom",
        )

    def _imu_callback(self, message: Imu) -> None:
        if self.enabled and self.rng.random() < self.imu_dropout:
            return
        self._enqueue(
            self.model.distort_imu(message),
            self.imu_publisher,
            self.imu_latency,
            self.imu_jitter,
            "imu",
        )

    def _enqueue(
        self,
        message: object,
        publisher: object,
        latency: float,
        jitter: float,
        stream: str,
    ) -> None:
        if not self.enabled:
            publisher.publish(message)
            return
        sampled_delay = max(0.0, latency + self.rng.gauss(0.0, jitter))
        now = self.get_clock().now().nanoseconds * 1.0e-9
        if now < self.last_due_time[stream] - 1.0:
            # A simulation clock reset starts a new delivery timeline.
            self.last_due_time[stream] = -math.inf
        due = max(now + sampled_delay, self.last_due_time[stream] + 1.0e-6)
        self.last_due_time[stream] = due
        self.sequence += 1
        heapq.heappush(self.pending, (due, self.sequence, message, publisher))

    def _release_messages(self) -> None:
        now = self.get_clock().now().nanoseconds * 1.0e-9
        while self.pending and self.pending[0][0] <= now:
            _, _, message, publisher = heapq.heappop(self.pending)
            publisher.publish(message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SensorRealismNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # ROS launch may already have shut the shared context down on SIGINT.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
