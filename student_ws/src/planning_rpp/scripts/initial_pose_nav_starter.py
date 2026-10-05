#!/usr/bin/env python3

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.srv import ManageLifecycleNodes
from tf2_ros import Buffer, TransformListener
from rclpy.executors import ExternalShutdownException

class InitialPoseNavStarter(Node):

    def __init__(self):
        super().__init__('initial_pose_nav_starter')

        self.initial_pose_received = False
        self.start_request_in_progress = False
        self.navigation_started = False

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
        if self.navigation_started:
            return

        self.initial_pose_received = True
        self.get_logger().info(
            'Initial pose received. Waiting for map -> base_link...'
        )

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
