#!/usr/bin/env python3

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.srv import ManageLifecycleNodes
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener
from rclpy.executors import ExternalShutdownException

class InitialPoseNavStarter(Node):

    def __init__(self):
        super().__init__('initial_pose_nav_starter')

        self.initial_pose_received = False
        self.start_request_in_progress = False
        self.navigation_started = False

        # AMCL only updates when odometry reports motion, so a pose set from
        # RViz never snaps onto the walls while the robot sits still. After
        # each initial pose, force a few scan-matching updates in place.
        self.declare_parameter('nomotion_updates', 8)
        self.nomotion_updates = int(self.get_parameter('nomotion_updates').value)
        self.nomotion_remaining = 0
        self.nomotion_client = self.create_client(
            Empty, '/request_nomotion_update')
        self.nomotion_timer = self.create_timer(0.5, self.nomotion_tick)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.initial_pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initial_pose_callback,
            10
        )

        self.lifecycle_client = self.create_client(
            ManageLifecycleNodes,
            '/lifecycle_manager_navigation/manage_nodes'
        )

        self.timer = self.create_timer(
            0.5,
            self.try_start_navigation
        )

        self.get_logger().info(
            'Waiting for an initial pose from RViz...'
        )

    def initial_pose_callback(self, msg):
        # Re-arm the in-place AMCL updates on every new initial pose.
        self.nomotion_remaining = self.nomotion_updates

        if self.navigation_started:
            return

        self.initial_pose_received = True
        self.get_logger().info(
            'Initial pose received. Waiting for map -> base_link...'
        )

    def nomotion_tick(self):
        if self.nomotion_remaining <= 0:
            return
        if not self.nomotion_client.service_is_ready():
            return
        self.nomotion_client.call_async(Empty.Request())
        self.nomotion_remaining -= 1
        if self.nomotion_remaining == 0:
            self.get_logger().info(
                f'Sent {self.nomotion_updates} AMCL in-place updates '
                'for the initial pose.')

    def try_start_navigation(self):
        if not self.initial_pose_received:
            return

        if self.navigation_started:
            return

        if self.start_request_in_progress:
            return

        # Wait until AMCL has processed the initial pose and published
        # the complete map -> odom -> base_link transform chain.
        transform_available = self.tf_buffer.can_transform(
            'map',
            'base_link',
            Time(),
            timeout=Duration(seconds=0.1)
        )

        if not transform_available:
            return

        if not self.lifecycle_client.service_is_ready():
            self.get_logger().info(
                'Waiting for navigation lifecycle manager service...'
            )
            return

        self.get_logger().info(
            'map -> base_link is available. Starting navigation...'
        )

        request = ManageLifecycleNodes.Request()
        request.command = 0  # STARTUP

        self.start_request_in_progress = True

        future = self.lifecycle_client.call_async(request)
        future.add_done_callback(self.startup_finished_callback)

    def startup_finished_callback(self, future):
        self.start_request_in_progress = False

        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(
                f'Lifecycle startup service failed: {error}'
            )
            return

        if response is not None and response.success:
            self.navigation_started = True
            self.timer.cancel()

            self.get_logger().info(     
                'Navigation lifecycle started successfully.'
            )
        else:
            self.get_logger().error(
                'Navigation lifecycle startup returned false. Retrying...'
            )


def main(args=None):
    rclpy.init(args=args)

    node = InitialPoseNavStarter()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
