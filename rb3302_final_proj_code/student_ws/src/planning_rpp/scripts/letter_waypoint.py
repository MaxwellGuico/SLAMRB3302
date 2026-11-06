#!/usr/bin/env python3
"""Run four A-X waypoints using the supplied rb3302_map.yaml/PGM geometry.

Reference layout (viewed from above, matching the PGM):
    A B C D E F       +y (north) is up
    G H I J K L       +x (east) is right
    M N O P Q R
    S T U V W X

Example:
    python3 letter_waypoint.py A H Q X

The IRL coordinates below are calibrated to this specific map. If the map is
replaced, rotated, resized, or given a different YAML origin, recalibrate them.
Waypoint pauses are configured in Nav2's waypoint task executor, not here.

Ctrl+C or SIGTERM cancels this entire four-waypoint mission, then waits up to
--cancel-timeout seconds (default 5) before exiting. Other clients' goals are
not cancelled. If confirmation times out, use the robot's stop control and
inspect Nav2. SIGKILL cannot be handled. Nav2's stack is left running.
"""

import argparse
import math
import signal
import sys
import time
from typing import Iterable, List, Sequence, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args


GRID_COLUMNS = 6
GRID_ROWS = 4
FIRST_LABEL = ord('A')
LAST_LABEL = ord('X')

# Supplied rb3302_map.yaml:
#   resolution: 0.050 m/pixel
#   origin: [-0.787, -2.596, 0.0]
# The PGM is 90 columns x 69 rows. PGM row 0 is the TOP of the image.
# For a pixel centre at (u, v), zero-indexed:
#   x = -0.787 + (u + 0.5) * 0.050
#   y = -2.596 + (69 - v - 0.5) * 0.050
#
# Wall centre lines measured in the PGM, including internal grid boundaries:
#   u (west -> east): 10.5, 22.5, 33.5, 45.5, 57.5, 68.5, 80.5
#   v (north -> south): 9.5, 21.5, 33.5, 45.5, 57.5
# Cell centres are halfway between these boundary lines. Column widths vary
# slightly in this raster, so use the measured centres rather than a single
# averaged width. All 24 centres were checked against the PGM's free cells.
MAP_GEOMETRIES = {
    'irl': {
        'column_centres_x': (0.063, 0.638, 1.213, 1.813, 2.388, 2.963),
        'row_centres_y': (0.054, -0.546, -1.146, -1.746),
    },
    'ideal': {
        'column_centres_x': (0.30, 0.90, 1.50, 2.10, 2.70, 3.30),
        'row_centres_y': (2.10, 1.50, 0.90, 0.30),
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
    """Return a labelled cell's measured centre in the ROS map frame."""
    if len(label) != 1 or not 'A' <= label <= 'X':
        raise ValueError(f'waypoint letter must be in the range A-X: {label}')

    geometry = MAP_GEOMETRIES[map_profile]
    row, column = divmod(ord(label) - FIRST_LABEL, GRID_COLUMNS)
    return geometry['column_centres_x'][column], geometry['row_centres_y'][row]


class MissionInterrupted(Exception):
    """Request a graceful exit while leaving ROS alive for cancellation."""


class LetterWaypointMission(Node):
    """Submit and monitor an ordered FollowWaypoints action."""

    def __init__(
        self, labels: Sequence[str], server_timeout: float, map_profile: str,
        cancel_timeout: float = 5.0,
    ) -> None:
        super().__init__('letter_waypoint_mission')
        self._labels = list(labels)
        self._server_timeout = server_timeout
        self._map_profile = map_profile
        self._current_waypoint = -1
        self._cancel_timeout = cancel_timeout
        self._stop_signal = None
        self._send_future = None
        self._goal_handle = None
        self._result_future = None
        self._client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

    def request_stop(self, signum, _frame=None) -> None:
        # Signal handlers only set a flag. ROS calls happen in normal execution.
        if self._stop_signal is None:
            self._stop_signal = signum

    def _check_stop(self) -> None:
        if self._stop_signal is not None:
            raise MissionInterrupted()

    def _wait_future(self, future, deadline=None, interruptible=True) -> bool:
        while rclpy.ok():
            if interruptible:
                self._check_stop()
            if future.done():
                return True
            remaining = None if deadline is None else deadline - time.monotonic()
            if remaining is not None and remaining <= 0.0:
                return False
            rclpy.spin_once(
                self, timeout_sec=0.1 if remaining is None else min(0.1, remaining)
            )
        if interruptible:
            raise RuntimeError('ROS context shut down before the mission completed')
        return False

    def _pause(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self._check_stop()
            time.sleep(min(0.1, max(0.0, deadline - time.monotonic())))

    def cancel_active_mission(self) -> bool:
        """Cancel only our FollowWaypoints goal, with a bounded cleanup wait.

        The accepted goal contains all four waypoints. Nav2's waypoint follower
        handles cancellation of its active navigation task. Never cancel all
        clients' goals, stop lifecycle nodes, or publish competing cmd_vel here.
        """
        if self._send_future is None:
            return True
        deadline = time.monotonic() + self._cancel_timeout
        if not rclpy.ok():
            self.get_logger().error(
                'ROS is already shut down; mission cancellation cannot be confirmed.'
            )
            return False
        try:
            # Ctrl+C can arrive before Nav2's acceptance reply. Resolve that
            # request before exiting so an accepted mission is not abandoned.
            if self._goal_handle is None:
                if not self._wait_future(self._send_future, deadline, False):
                    self.get_logger().error(
                        'Goal acceptance timed out during exit; Nav2 may still be executing '
                        'the mission. Use the robot stop control and check Nav2.'
                    )
                    return False
                self._goal_handle = self._send_future.result()
            if self._goal_handle is None or not self._goal_handle.accepted:
                return True
            if self._result_future is None:
                self._result_future = self._goal_handle.get_result_async()
            if self._result_future.done():
                return self._confirm_terminal_result()

            self.get_logger().warning(
                'Cancelling this four-waypoint mission (current and remaining waypoints)...'
            )
            cancel_future = self._goal_handle.cancel_goal_async()
            if not self._wait_future(cancel_future, deadline, False):
                self.get_logger().error(
                    'Cancellation acknowledgement timed out; robot stop is NOT confirmed.'
                )
                return False
            reply = cancel_future.result()
            if reply is None or not reply.goals_canceling:
                self.get_logger().warning(
                    'Cancellation was not accepted, or the mission already finished; '
                    'checking its final result.'
                )
            if not self._wait_future(self._result_future, deadline, False):
                self.get_logger().error(
                    'No terminal mission result before the exit timeout; robot stop '
                    'is NOT confirmed. Use the robot stop control and check Nav2.'
                )
                return False
            return self._confirm_terminal_result()
        except Exception as error:
            self.get_logger().error(
                f'Could not confirm mission cancellation: {error}. '
                'Use the robot stop control and check Nav2.'
            )
            return False

    def _confirm_terminal_result(self) -> bool:
        result = self._result_future.result()
        terminal = {
            GoalStatus.STATUS_SUCCEEDED: 'succeeded',
            GoalStatus.STATUS_CANCELED: 'cancelled',
            GoalStatus.STATUS_ABORTED: 'aborted',
        }
        if result is not None and result.status in terminal:
            self.get_logger().info(f'Mission is terminal: {terminal[result.status]}.')
            return True
        self.get_logger().error('Mission termination could not be confirmed.')
        return False

    def _make_pose(self, index: int) -> PoseStamped:
        label = self._labels[index]
        x, y = landmark_coordinates(label, self._map_profile)

        # Face along the outgoing leg. For the last point, retain the heading
        # of the incoming leg. Nav2 plans the route around walls between goals.
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
        if index == self._current_waypoint or not 0 <= index < len(self._labels):
            return

        self._current_waypoint = index
        label = self._labels[index]
        x, y = landmark_coordinates(label, self._map_profile)
        self.get_logger().info(
            f'Navigating to waypoint {index + 1}/4: {label} at ({x:.3f}, {y:.3f})'
        )

    def _wait_for_action_server(self) -> bool:
        deadline = time.monotonic() + self._server_timeout
        self.get_logger().info('Waiting for the Nav2 follow_waypoints action server...')
        while rclpy.ok() and time.monotonic() < deadline:
            self._check_stop()
            if self._client.wait_for_server(timeout_sec=0.1):
                return True
        return False

    def _send_when_active(self, goal):
        """Retry while the lifecycle action server exists but is not active yet."""
        deadline = time.monotonic() + self._server_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            self._check_stop()
            self._send_future = self._client.send_goal_async(
                goal, feedback_callback=self._feedback
            )
            if not self._wait_future(self._send_future, deadline):
                # Do not resend an unresolved request: it may already be accepted.
                return None
            self._goal_handle = self._send_future.result()
            if self._goal_handle is not None and self._goal_handle.accepted:
                self._result_future = self._goal_handle.get_result_async()
                return self._goal_handle

            self.get_logger().warning(
                'Nav2 rejected the mission; retrying in 1 second '
                '(the waypoint server may still be activating)'
            )
            self._pause(1.0)
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
        self.get_logger().info(f'Using map geometry: {self._map_profile}')
        self.get_logger().info(
            'Waypoint pauses follow the Nav2 waypoint task executor configuration.'
        )
        started_at = time.monotonic()

        goal_handle = self._send_when_active(goal)
        if goal_handle is None:
            self.get_logger().error(
                f'Nav2 did not accept the mission within {self._server_timeout:.0f} seconds'
            )
            return False

        self._wait_future(self._result_future)
        wrapped_result = self._result_future.result()
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

        missed_labels = [
            self._labels[index] for index in missed if 0 <= index < len(self._labels)
        ]
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
        '--cancel-timeout',
        type=float,
        default=5.0,
        help='maximum seconds to wait for mission cancellation on exit (default: 5)',
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
        if not math.isfinite(parsed.cancel_timeout) or parsed.cancel_timeout <= 0.0:
            raise ValueError('--cancel-timeout must be a positive finite number')
    except ValueError as error:
        argument_parser().error(str(error))

    # rclpy's default SIGINT/SIGTERM handlers shut down the context immediately.
    # Own these signals so the action cancellation can finish BEFORE shutdown.
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    previous_handlers = {}
    exit_code = 1
    try:
        node = LetterWaypointMission(
            labels, parsed.server_timeout, parsed.map_profile, parsed.cancel_timeout
        )
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[signum] = signal.signal(signum, node.request_stop)
        exit_code = 0 if node.run() else 1
    except MissionInterrupted:
        node.get_logger().warning('Mission interrupted; requesting graceful cancellation.')
        exit_code = 128 + node._stop_signal
    except KeyboardInterrupt:
        exit_code = 130
    except Exception as error:
        if node is not None:
            node.get_logger().error(f'Mission error: {error}')
        else:
            print(f'Could not create mission node: {error}', file=sys.stderr)
    finally:
        if node is not None:
            confirmed = node.cancel_active_mission()
            if not confirmed and exit_code == 0:
                exit_code = 1
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for signum, previous in previous_handlers.items():
            signal.signal(signum, previous)
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
