#!/usr/bin/env python3
"""Run a four-letter RB3302 waypoint mission through Nav2."""

import argparse
import math
import sys
import time
from typing import Iterable, List, Sequence, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args


GRID_COLUMNS = 6
GRID_ROWS = 4
FIRST_LABEL = ord('A')
LAST_LABEL = ord('X')

# The IRL map is a cropped SLAM map, so its outer wall centres do not sit at
# (0, 0) and its horizontal scale is slightly compressed.  These values are
# measured from rb3302_map.pgm together with its YAML resolution/origin.
MAP_GEOMETRIES = {
    'irl': {
        'left_wall_x': -0.262,
        'bottom_wall_y': -2.071,
        'cell_width': 3.50 / GRID_COLUMNS,
        'cell_height': 2.40 / GRID_ROWS,
    },
    'ideal': {
        'left_wall_x': 0.0,
        'bottom_wall_y': 0.0,
        'cell_width': 0.60,
        'cell_height': 0.60,
    },
}


def parse_labels(values: Iterable[str]) -> List[str]:
    """Normalize space- or comma-separated labels and require four A-X entries."""
    labels = [
        label.strip().upper()
        for value in values
        for label in value.split(',')
        if label.strip()
    ]
    if len(labels) != 4:
        raise ValueError(f'exactly four waypoint letters are required; received {len(labels)}')

    invalid = [label for label in labels if len(label) != 1 or not 'A' <= label <= 'X']
    if invalid:
        raise ValueError(
            'waypoint letters must be in the range A-X; invalid value(s): '
            + ', '.join(invalid)
        )
    return labels


def landmark_coordinates(label: str, map_profile: str = 'irl') -> Tuple[float, float]:
    """Return the centre of a course cell in the map frame."""
    index = ord(label) - FIRST_LABEL
    if index < 0 or ord(label) > LAST_LABEL:
        raise ValueError(f'waypoint letter must be in the range A-X: {label}')

    geometry = MAP_GEOMETRIES[map_profile]
    row, column = divmod(index, GRID_COLUMNS)
    x = geometry['left_wall_x'] + (column + 0.5) * geometry['cell_width']
    y = geometry['bottom_wall_y'] + (
        GRID_ROWS - row - 0.5
    ) * geometry['cell_height']
    return x, y


class LetterWaypointMission(Node):
    """Submit and monitor an ordered FollowWaypoints action."""

    def __init__(
        self, labels: Sequence[str], server_timeout: float, map_profile: str
    ) -> None:
        super().__init__('letter_waypoint_mission')
        self._labels = list(labels)
        self._server_timeout = server_timeout
        self._map_profile = map_profile
        self._current_waypoint = -1
        self._client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

    def _make_pose(self, index: int) -> PoseStamped:
        label = self._labels[index]
        x, y = landmark_coordinates(label, self._map_profile)

        # Face along the outgoing leg.  For the last point, retain the heading
        # of the incoming leg.  This avoids an artificial zero-yaw turn at
        # every waypoint while still providing valid goal orientations.
        neighbour_index = index + 1 if index + 1 < len(self._labels) else index - 1
        neighbour_x, neighbour_y = landmark_coordinates(
            self._labels[neighbour_index], self._map_profile
        )
        if index + 1 < len(self._labels):
            yaw = math.atan2(neighbour_y - y, neighbour_x - x)
        else:
            yaw = math.atan2(y - neighbour_y, x - neighbour_x)

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _feedback(self, feedback_message) -> None:
        index = int(feedback_message.feedback.current_waypoint)
        if index == self._current_waypoint or index >= len(self._labels):
            return

        self._current_waypoint = index
        label = self._labels[index]
        x, y = landmark_coordinates(label, self._map_profile)
        self.get_logger().info(
            f'Navigating to waypoint {index + 1}/4: {label} at ({x:.2f}, {y:.2f})'
        )

    def _wait_for_action_server(self) -> bool:
        deadline = time.monotonic() + self._server_timeout
        self.get_logger().info('Waiting for the Nav2 follow_waypoints action server...')
        while rclpy.ok() and time.monotonic() < deadline:
            if self._client.wait_for_server(timeout_sec=1.0):
                return True
        return False

    def _send_when_active(self, goal):
        """Retry while the lifecycle action server exists but is not active yet."""
        deadline = time.monotonic() + self._server_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            send_future = self._client.send_goal_async(
                goal, feedback_callback=self._feedback
            )
            rclpy.spin_until_future_complete(self, send_future)
            goal_handle = send_future.result()
            if goal_handle is not None and goal_handle.accepted:
                return goal_handle

            self.get_logger().warning(
                'Nav2 is still activating; retrying the waypoint mission in 1 second'
            )
            time.sleep(1.0)
        return None

    def run(self) -> bool:
        if not self._wait_for_action_server():
            self.get_logger().error(
                f'follow_waypoints was not available after {self._server_timeout:.0f} seconds'
            )
            return False

        goal = FollowWaypoints.Goal()
        goal.number_of_loops = 0
        goal.goal_index = 0
        goal.poses = [self._make_pose(index) for index in range(len(self._labels))]

        route = ' -> '.join(self._labels)
        self.get_logger().info(f'Starting four-waypoint mission: {route}')
        self.get_logger().info('The robot will wait 3 seconds at each reached waypoint.')
        started_at = time.monotonic()

        goal_handle = self._send_when_active(goal)
        if goal_handle is None:
            self.get_logger().error(
                f'Nav2 did not accept the mission within {self._server_timeout:.0f} seconds'
            )
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped_result = result_future.result()
        if wrapped_result is None:
            self.get_logger().error('The waypoint mission ended without a result')
            return False

        elapsed = time.monotonic() - started_at
        result = wrapped_result.result
        missed = [int(item.index) for item in result.missed_waypoints]
        if wrapped_result.status == GoalStatus.STATUS_SUCCEEDED and not missed:
            self.get_logger().info(
                f'Mission complete: visited {route} in {elapsed:.1f} seconds'
            )
            return True

        missed_labels = [self._labels[index] for index in missed if index < len(self._labels)]
        error_text = result.error_msg or f'action status {wrapped_result.status}'
        if missed_labels:
            error_text += '; missed: ' + ', '.join(missed_labels)
        self.get_logger().error(f'Mission failed after {elapsed:.1f} seconds: {error_text}')
        return False


def argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            'Visit exactly four RB3302 grid letters in order. '
            'Letters may be separated by spaces or commas.'
        )
    )
    parser.add_argument('letters', nargs='+', help='four letters in the range A-X')
    parser.add_argument(
        '--server-timeout',
        type=float,
        default=60.0,
        help='seconds to wait for the Nav2 action server (default: 60)',
    )
    parser.add_argument(
        '--map-profile',
        choices=sorted(MAP_GEOMETRIES),
        default='irl',
        help='map geometry used for letter coordinates (default: irl)',
    )
    return parser


def main(args=None) -> int:
    ros_args = sys.argv if args is None else args
    non_ros_args = remove_ros_args(args=ros_args)[1:]
    try:
        parsed = argument_parser().parse_args(non_ros_args)
        labels = parse_labels(parsed.letters)
        if not math.isfinite(parsed.server_timeout) or parsed.server_timeout <= 0.0:
            raise ValueError('--server-timeout must be a positive finite number')
    except ValueError as error:
        argument_parser().error(str(error))

    rclpy.init(args=ros_args)
    node = LetterWaypointMission(labels, parsed.server_timeout, parsed.map_profile)
    try:
        return 0 if node.run() else 1
    except KeyboardInterrupt:
        node.get_logger().warning('Waypoint mission interrupted')
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
