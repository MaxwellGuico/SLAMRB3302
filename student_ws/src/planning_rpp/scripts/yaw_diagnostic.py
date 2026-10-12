#!/usr/bin/env python3

import csv
import json
import math
import os
import statistics
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener


def quaternion_to_yaw(q) -> float:
    """Return wrapped yaw in radians."""
    sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
    cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(sin_yaw, cos_yaw)


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def degrees_or_blank(value):
    if value is None:
        return ""
    return math.degrees(value)


class WrappedAngleTracker:
    """Convert a sequence of wrapped angles into a continuous angle."""

    def __init__(self):
        self.previous_wrapped = None
        self.unwrapped = None

    def update(self, wrapped_angle: float) -> float:
        if self.previous_wrapped is None:
            self.previous_wrapped = wrapped_angle
            self.unwrapped = wrapped_angle
            return self.unwrapped

        change = wrap_angle(wrapped_angle - self.previous_wrapped)
        self.unwrapped += change
        self.previous_wrapped = wrapped_angle
        return self.unwrapped


class YawDiagnostic(Node):
    def __init__(self):
        super().__init__("yaw_diagnostic")

        # ---------------- Parameters ----------------

        self.declare_parameter("active_test", False)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("raw_odom_topic", "/odom/raw")
        self.declare_parameter("filtered_odom_topic", "/odom")
        self.declare_parameter("imu_topic", "/imu_data_ros")
        self.declare_parameter(
            "waverover_feedback_topic",
            "/waverover/json_feedback",
        )

        self.declare_parameter("sample_rate", 20.0)
        self.declare_parameter("start_delay", 5.0)
        self.declare_parameter("stationary_duration", 10.0)
        self.declare_parameter("turn_speed", 0.25)
        self.declare_parameter("turn_duration", 6.28)
        self.declare_parameter("settle_duration", 5.0)
        self.declare_parameter("output_file", "")

        # The current URDF has base_link -> gyro_link with roll = pi.
        # This normally changes sensor-frame +Z into base-frame -Z.
        # Set this to +1.0 if your IMU driver already performs that conversion.
        self.declare_parameter("imu_gyro_z_sign", -1.0)

        self.active_test = bool(
            self.get_parameter("active_test").value
        )
        self.cmd_topic = str(
            self.get_parameter("cmd_vel_topic").value
        )
        self.sample_rate = float(
            self.get_parameter("sample_rate").value
        )
        self.start_delay = float(
            self.get_parameter("start_delay").value
        )
        self.imu_gyro_z_sign = float(
            self.get_parameter("imu_gyro_z_sign").value
        )

        raw_odom_topic = str(
            self.get_parameter("raw_odom_topic").value
        )
        filtered_odom_topic = str(
            self.get_parameter("filtered_odom_topic").value
        )
        imu_topic = str(
            self.get_parameter("imu_topic").value
        )
        feedback_topic = str(
            self.get_parameter("waverover_feedback_topic").value
        )

        # ---------------- Latest measurements ----------------

        self.latest = {
            "cmd_z": 0.0,
            "cmd_integrated_yaw": 0.0,

            # WaveRover JSON values converted to radians.
            "waverover_yaw_odom": None,
            "waverover_yaw_imu": None,

            # External sensor_msgs/Imu.
            "imu_orientation_raw": None,
            "imu_gyro_z_raw": None,
            "imu_gyro_z_base": None,
            "imu_gyro_integrated": 0.0,

            # Odometry messages.
            "raw_odom_yaw": None,
            "raw_odom_wz": None,
            "filtered_odom_yaw": None,
            "filtered_odom_wz": None,

            # TF measurements.
            "tf_odom_base_yaw": None,
            "tf_map_odom_yaw": None,
            "tf_map_base_yaw": None,
        }

        self.trackers = {
            "waverover_yaw_imu": WrappedAngleTracker(),
            "imu_orientation_raw": WrappedAngleTracker(),
            "raw_odom_yaw": WrappedAngleTracker(),
            "filtered_odom_yaw": WrappedAngleTracker(),
            "tf_odom_base_yaw": WrappedAngleTracker(),
            "tf_map_odom_yaw": WrappedAngleTracker(),
            "tf_map_base_yaw": WrappedAngleTracker(),
        }

        self.last_sample_time = None
        self.last_imu_time = None

        # ---------------- Subscribers ----------------

        self.create_subscription(
            Twist,
            self.cmd_topic,
            self.cmd_callback,
            10,
        )

        self.create_subscription(
            Odometry,
            raw_odom_topic,
            self.raw_odom_callback,
            20,
        )

        self.create_subscription(
            Odometry,
            filtered_odom_topic,
            self.filtered_odom_callback,
            20,
        )

        self.create_subscription(
            Imu,
            imu_topic,
            self.imu_callback,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            String,
            feedback_topic,
            self.feedback_callback,
            20,
        )

        # ---------------- TF ----------------

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
            spin_thread=False,
        )

        # ---------------- CSV ----------------

        output_file = str(
            self.get_parameter("output_file").value
        )

        if not output_file:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_file = os.path.abspath(
                f"yaw_diagnostic_{timestamp}.csv"
            )

        self.output_file = output_file
        self.csv_handle = open(
            self.output_file,
            "w",
            newline="",
            encoding="utf-8",
        )

        self.csv_fields = [
            "ros_time",
            "phase",
            "cmd_z",
            "cmd_integrated_yaw",
            "waverover_yaw_odom",
            "waverover_yaw_imu",
            "imu_orientation_raw",
            "imu_gyro_z_raw",
            "imu_gyro_z_base",
            "imu_gyro_integrated",
            "raw_odom_yaw",
            "raw_odom_wz",
            "filtered_odom_yaw",
            "filtered_odom_wz",
            "tf_odom_base_yaw",
            "tf_map_odom_yaw",
            "tf_map_base_yaw",
        ]

        self.csv_writer = csv.DictWriter(
            self.csv_handle,
            fieldnames=self.csv_fields,
        )
        self.csv_writer.writeheader()

        # ---------------- Test phases ----------------

        self.current_phase = "READY"
        self.phase_start_time = None
        self.phase_start_snapshot = None
        self.phase_samples = []
        self.phase_summaries = []
        self.phase_index = -1
        self.test_finished = False
        self.active_started = False
        self.active_aborted = False

        self.manual_phases = [
            (
                "STATIONARY",
                "Keep the robot completely stationary for about 15 seconds.",
            ),
            (
                "CCW_MANUAL",
                "Manually rotate the robot +90 degrees counter-clockwise.",
            ),
            (
                "SETTLE_CCW",
                "Keep the robot stationary for about 5 seconds.",
            ),
            (
                "CW_MANUAL",
                "Manually rotate the robot -90 degrees clockwise.",
            ),
            (
                "SETTLE_CW",
                "Keep the robot stationary for about 5 seconds.",
            ),
        ]

        stationary_duration = float(
            self.get_parameter("stationary_duration").value
        )
        turn_speed = float(
            self.get_parameter("turn_speed").value
        )
        turn_duration = float(
            self.get_parameter("turn_duration").value
        )
        settle_duration = float(
            self.get_parameter("settle_duration").value
        )

        self.active_phases = [
            ("STATIONARY", stationary_duration, 0.0),
            ("CCW_POWERED", turn_duration, abs(turn_speed)),
            ("SETTLE_CCW", settle_duration, 0.0),
            ("CW_POWERED", turn_duration, -abs(turn_speed)),
            ("SETTLE_CW", settle_duration, 0.0),
        ]

        self.cmd_publisher = None

        if self.active_test:
            self.cmd_publisher = self.create_publisher(
                Twist,
                self.cmd_topic,
                10,
            )
            self.active_launch_time = self.now_seconds()
            self.get_logger().warning(
                "ACTIVE TEST ENABLED. The robot will move after "
                f"{self.start_delay:.1f} seconds."
            )
        else:
            self.next_phase_service = self.create_service(
                Trigger,
                "~/next_phase",
                self.next_phase_callback,
            )

            self.get_logger().info(
                "Passive test ready. Start each phase with:\n"
                "ros2 service call /yaw_diagnostic/next_phase "
                "std_srvs/srv/Trigger {}"
            )

        period = 1.0 / max(self.sample_rate, 1.0)
        self.sample_timer = self.create_timer(
            period,
            self.sample_callback,
        )

        if self.active_test:
            self.control_timer = self.create_timer(
                0.05,
                self.active_control_callback,
            )

        self.get_logger().info(
            f"Writing diagnostic data to: {self.output_file}"
        )

    # ================================================================
    # ROS callbacks
    # ================================================================

    def now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def cmd_callback(self, msg: Twist):
        self.latest["cmd_z"] = float(msg.angular.z)

    def raw_odom_callback(self, msg: Odometry):
        wrapped_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.latest["raw_odom_yaw"] = self.trackers[
            "raw_odom_yaw"
        ].update(wrapped_yaw)

        self.latest["raw_odom_wz"] = float(
            msg.twist.twist.angular.z
        )

    def filtered_odom_callback(self, msg: Odometry):
        wrapped_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.latest["filtered_odom_yaw"] = self.trackers[
            "filtered_odom_yaw"
        ].update(wrapped_yaw)

        self.latest["filtered_odom_wz"] = float(
            msg.twist.twist.angular.z
        )

    def imu_callback(self, msg: Imu):
        # This is the raw sensor-frame quaternion. Its yaw may require
        # frame and ENU/NED correction before being treated as ground truth.
        orientation_norm = math.sqrt(
            msg.orientation.x ** 2
            + msg.orientation.y ** 2
            + msg.orientation.z ** 2
            + msg.orientation.w ** 2
        )

        orientation_available = (
            msg.orientation_covariance[0] >= 0.0
            and orientation_norm > 0.5
        )

        if orientation_available:
            wrapped_yaw = quaternion_to_yaw(msg.orientation)
            self.latest["imu_orientation_raw"] = self.trackers[
                "imu_orientation_raw"
            ].update(wrapped_yaw)

        raw_wz = float(msg.angular_velocity.z)
        base_wz = self.imu_gyro_z_sign * raw_wz

        self.latest["imu_gyro_z_raw"] = raw_wz
        self.latest["imu_gyro_z_base"] = base_wz

        stamp = (
            float(msg.header.stamp.sec)
            + float(msg.header.stamp.nanosec) * 1e-9
        )

        if stamp <= 0.0:
            stamp = self.now_seconds()

        if self.last_imu_time is not None:
            dt = stamp - self.last_imu_time

            # Reject duplicate, reversed, or very delayed samples.
            if 0.0 < dt < 0.5:
                self.latest["imu_gyro_integrated"] += base_wz * dt

        self.last_imu_time = stamp

    def feedback_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return

        t_code = data.get("T")

        if t_code == 1001 and "yaw_odom" in data:
            # Confirmed assumption: cumulative radians.
            self.latest["waverover_yaw_odom"] = float(
                data["yaw_odom"]
            )

        if "yaw_imu" in data:
            # Confirmed assumption: degrees.
            yaw_imu_rad_wrapped = math.radians(
                float(data["yaw_imu"])
            )

            self.latest["waverover_yaw_imu"] = self.trackers[
                "waverover_yaw_imu"
            ].update(yaw_imu_rad_wrapped)

    # ================================================================
    # TF and logging
    # ================================================================

    def update_tf(self, target, source, key):
        try:
            transform = self.tf_buffer.lookup_transform(
                target,
                source,
                Time(),
            )

            wrapped_yaw = quaternion_to_yaw(
                transform.transform.rotation
            )

            self.latest[key] = self.trackers[key].update(
                wrapped_yaw
            )

        except TransformException:
            pass

    def sample_callback(self):
        now = self.now_seconds()

        if self.last_sample_time is not None:
            dt = now - self.last_sample_time

            if 0.0 < dt < 0.5:
                self.latest["cmd_integrated_yaw"] += (
                    self.latest["cmd_z"] * dt
                )

        self.last_sample_time = now

        self.update_tf(
            "odom",
            "base_link",
            "tf_odom_base_yaw",
        )
        self.update_tf(
            "map",
            "odom",
            "tf_map_odom_yaw",
        )
        self.update_tf(
            "map",
            "base_link",
            "tf_map_base_yaw",
        )

        row = {
            "ros_time": now,
            "phase": self.current_phase,
        }

        for key in self.csv_fields:
            if key in ("ros_time", "phase"):
                continue
            row[key] = self.latest.get(key)

        self.csv_writer.writerow(row)

        if int(now * 10) % 10 == 0:
            self.csv_handle.flush()

        if (
            self.current_phase not in ("READY", "DONE", "ABORTED")
            and not self.test_finished
        ):
            self.phase_samples.append(dict(self.latest))

    # ================================================================
    # Phase handling
    # ================================================================

    def snapshot(self):
        return dict(self.latest)

    def begin_phase(self, name: str, instruction: str = ""):
        self.current_phase = name
        self.phase_start_time = self.now_seconds()
        self.phase_start_snapshot = self.snapshot()
        self.phase_samples = []

        message = f"Starting phase: {name}"
        if instruction:
            message += f"\n{instruction}"

        self.get_logger().info(message)

    def measurement_delta(self, start, end, key):
        start_value = start.get(key)
        end_value = end.get(key)

        if start_value is None or end_value is None:
            return None

        return end_value - start_value

    def finish_phase(self):
        if self.phase_start_snapshot is None:
            return

        end_snapshot = self.snapshot()
        duration = self.now_seconds() - self.phase_start_time

        angle_keys = [
            "cmd_integrated_yaw",
            "waverover_yaw_odom",
            "waverover_yaw_imu",
            "imu_orientation_raw",
            "imu_gyro_integrated",
            "raw_odom_yaw",
            "filtered_odom_yaw",
            "tf_odom_base_yaw",
            "tf_map_odom_yaw",
            "tf_map_base_yaw",
        ]

        summary = {
            "phase": self.current_phase,
            "duration": duration,
        }

        for key in angle_keys:
            summary[key] = self.measurement_delta(
                self.phase_start_snapshot,
                end_snapshot,
                key,
            )

        rate_keys = [
            "cmd_z",
            "imu_gyro_z_base",
            "raw_odom_wz",
            "filtered_odom_wz",
        ]

        for key in rate_keys:
            values = [
                sample[key]
                for sample in self.phase_samples
                if sample.get(key) is not None
            ]

            if values:
                summary[f"{key}_mean"] = statistics.fmean(values)
                summary[f"{key}_stddev"] = (
                    statistics.pstdev(values)
                    if len(values) > 1
                    else 0.0
                )
            else:
                summary[f"{key}_mean"] = None
                summary[f"{key}_stddev"] = None

        self.phase_summaries.append(summary)
        self.print_phase_summary(summary)

        self.phase_start_snapshot = None
        self.phase_samples = []

    def print_phase_summary(self, summary):
        def fmt_angle(key):
            value = summary.get(key)
            if value is None:
                return "no data"
            return f"{math.degrees(value):+.2f} deg"

        def fmt_rate(key):
            value = summary.get(key)
            if value is None:
                return "no data"
            return f"{value:+.4f} rad/s"

        self.get_logger().info(
            "\n"
            f"===== {summary['phase']} =====\n"
            f"Duration:              {summary['duration']:.2f} s\n"
            f"Command integral:      {fmt_angle('cmd_integrated_yaw')}\n"
            f"WaveRover yaw_odom:    {fmt_angle('waverover_yaw_odom')}\n"
            f"WaveRover yaw_imu:     {fmt_angle('waverover_yaw_imu')}\n"
            f"External IMU orient.:  {fmt_angle('imu_orientation_raw')}\n"
            f"External gyro integral:{fmt_angle('imu_gyro_integrated')}\n"
            f"Raw odometry:          {fmt_angle('raw_odom_yaw')}\n"
            f"Filtered odometry:     {fmt_angle('filtered_odom_yaw')}\n"
            f"TF odom -> base:       {fmt_angle('tf_odom_base_yaw')}\n"
            f"TF map -> odom:        {fmt_angle('tf_map_odom_yaw')}\n"
            f"TF map -> base:        {fmt_angle('tf_map_base_yaw')}\n"
            f"Mean raw odom wz:      {fmt_rate('raw_odom_wz_mean')}\n"
            f"Mean IMU gyro wz:      {fmt_rate('imu_gyro_z_base_mean')}\n"
        )

    def print_final_summary(self):
        self.get_logger().info(
            "\n"
            "========================================\n"
            "YAW DIAGNOSTIC COMPLETE\n"
            f"CSV file: {self.output_file}\n"
            "========================================"
        )

        for summary in self.phase_summaries:
            self.print_phase_summary(summary)

    # ================================================================
    # Passive test
    # ================================================================

    def next_phase_callback(self, request, response):
        del request

        if self.test_finished:
            response.success = False
            response.message = "The test has already finished."
            return response

        if self.phase_index >= 0:
            self.finish_phase()

        self.phase_index += 1

        if self.phase_index >= len(self.manual_phases):
            self.current_phase = "DONE"
            self.test_finished = True
            self.print_final_summary()

            response.success = True
            response.message = "Test complete."
            return response

        name, instruction = self.manual_phases[self.phase_index]
        self.begin_phase(name, instruction)

        response.success = True
        response.message = (
            f"Started {name}. Call next_phase when finished."
        )
        return response

    # ================================================================
    # Active test
    # ================================================================

    def publisher_fqn(self, info):
        namespace = info.node_namespace.rstrip("/")
        if not namespace:
            namespace = ""

        return f"{namespace}/{info.node_name}".replace("//", "/")

    def foreign_cmd_publishers(self):
        if self.cmd_publisher is None:
            return []

        own_name = self.get_fully_qualified_name()
        publishers = self.get_publishers_info_by_topic(self.cmd_topic)

        return [
            self.publisher_fqn(info)
            for info in publishers
            if self.publisher_fqn(info) != own_name
        ]

    def publish_velocity(self, angular_z: float):
        if self.cmd_publisher is None:
            return

        msg = Twist()
        msg.angular.z = float(angular_z)
        self.cmd_publisher.publish(msg)

    def abort_active_test(self, reason: str):
        self.publish_velocity(0.0)
        self.current_phase = "ABORTED"
        self.active_aborted = True
        self.test_finished = True

        self.get_logger().error(
            f"ACTIVE TEST ABORTED: {reason}"
        )

    def active_control_callback(self):
        if self.active_aborted:
            self.publish_velocity(0.0)
            return

        if self.test_finished:
            self.publish_velocity(0.0)
            return

        now = self.now_seconds()

        if not self.active_started:
            remaining = (
                self.start_delay
                - (now - self.active_launch_time)
            )

            if remaining > 0.0:
                self.publish_velocity(0.0)
                return

            foreign_publishers = self.foreign_cmd_publishers()

            if foreign_publishers:
                self.abort_active_test(
                    "Other /cmd_vel publishers were detected: "
                    + ", ".join(foreign_publishers)
                )
                return

            self.active_started = True
            self.phase_index = 0

            name, duration, command = self.active_phases[0]
            self.begin_phase(
                name,
                f"Duration {duration:.2f} s; "
                f"angular command {command:+.3f} rad/s",
            )
            return

        foreign_publishers = self.foreign_cmd_publishers()

        if foreign_publishers:
            self.abort_active_test(
                "A new /cmd_vel publisher appeared: "
                + ", ".join(foreign_publishers)
            )
            return

        name, duration, command = self.active_phases[
            self.phase_index
        ]

        phase_elapsed = now - self.phase_start_time

        if phase_elapsed >= duration:
            self.publish_velocity(0.0)
            self.finish_phase()

            self.phase_index += 1

            if self.phase_index >= len(self.active_phases):
                self.current_phase = "DONE"
                self.test_finished = True
                self.print_final_summary()
                return

            name, duration, command = self.active_phases[
                self.phase_index
            ]

            self.begin_phase(
                name,
                f"Duration {duration:.2f} s; "
                f"angular command {command:+.3f} rad/s",
            )
            return

        self.publish_velocity(command)

    # ================================================================
    # Shutdown
    # ================================================================

    def close(self):
        if self.cmd_publisher is not None:
            for _ in range(5):
                self.publish_velocity(0.0)
                time.sleep(0.02)

        if not self.csv_handle.closed:
            self.csv_handle.flush()
            self.csv_handle.close()


def main(args=None):
    rclpy.init(args=args)
    node = YawDiagnostic()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()