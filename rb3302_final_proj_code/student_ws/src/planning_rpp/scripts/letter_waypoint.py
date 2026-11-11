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
NavigateToPose goals are sent one at a time. This script handles the three-second
arrival pauses itself and announces failed goals before sending the next one.
The configured Nav2 NavigateToPose behaviour tree still handles recovery.
Do not run another mission client concurrently.

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
from nav2_msgs.action import NavigateToPose
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


class LetterWaypointMission(Node):
    """Run four NavigateToPose goals and report each result before advancing."""

    def __init__(self, labels, server_timeout, map_profile, pause_duration=3.0,
                 skip_delay=3.0, cancel_timeout=5.0):
        super().__init__('letter_waypoint_mission')
        self._labels = list(labels)
        self._server_timeout = server_timeout
        self._map_profile = map_profile
        self._pause_duration = pause_duration
        self._skip_delay = skip_delay
        self._cancel_timeout = cancel_timeout
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._send_future = None
        self._goal_handle = None
        self._result_future = None
        self.stop_requested = False
        self.stop_signal = None
        self.missed = []  # Store indices so repeated labels stay distinct.

    def request_stop(self, signum=None, frame=None):
        # Leave the ROS context alive until this node's goal is cancelled.
        self.stop_requested = True
        self.stop_signal = signum

    def _wait(self, future, timeout=None, interruptible=True):
        deadline = None if timeout is None else time.monotonic() + timeout
        while rclpy.ok():
            if interruptible and self.stop_requested:
                return False
            if future.done():
                return True
            if deadline is not None and time.monotonic() >= deadline:
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def _delay(self, seconds, label='Waiting'):
        deadline = time.monotonic() + seconds
        last_remaining = None
        while rclpy.ok() and not self.stop_requested:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                self.get_logger().info(f'{label}: complete.')
                return True
            countdown = math.ceil(remaining)
            if countdown != last_remaining:
                self.get_logger().info(f'{label}: {countdown}s remaining...')
                last_remaining = countdown
            rclpy.spin_once(self, timeout_sec=min(0.1, remaining))
        self.get_logger().warning(f'{label}: interrupted; mission will not advance.')
        return False

    def _make_pose(self, index):
        x, y = landmark_coordinates(self._labels[index], self._map_profile)
        # Preserve the original headings. Goal yaw is largely unconstrained by
        # the current Nav2 goal checker; Nav2 still plans around the walls.
        neighbour = index + 1 if index + 1 < len(self._labels) else index - 1
        nx, ny = landmark_coordinates(self._labels[neighbour], self._map_profile)
        yaw = (math.atan2(ny - y, nx - x) if neighbour > index
               else math.atan2(y - ny, x - nx))
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = x, y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _wait_for_server(self):
        deadline = time.monotonic() + self._server_timeout
        self.get_logger().info('Waiting for navigate_to_pose...')
        while rclpy.ok() and not self.stop_requested:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                self.get_logger().error('navigate_to_pose was not available in time')
                return False
            if self._client.wait_for_server(timeout_sec=min(0.2, remaining)):
                return True
        return False

    def _start_goal(self, index):
        # Retry explicit rejections during lifecycle startup. Never resend a
        # request whose acceptance is unknown, which could create another goal.
        deadline = time.monotonic() + self._server_timeout
        while rclpy.ok() and not self.stop_requested:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                self.get_logger().error('Nav2 did not accept the goal in time')
                return False
            goal = NavigateToPose.Goal()
            goal.pose = self._make_pose(index)
            goal.behavior_tree = ''  # Use configured default_nav_to_pose_bt_xml.
            self._send_future = self._client.send_goal_async(goal)
            if not self._wait(self._send_future, timeout=remaining):
                self.get_logger().warning(
                    'Goal acceptance interrupted or timed out; stopping mission')
                return False
            handle = self._send_future.result()
            self._send_future = None
            if handle is None:
                self.get_logger().error('Nav2 returned no goal handle')
                return False
            if handle.accepted:
                self._goal_handle = handle
                self._result_future = handle.get_result_async()
                return True
            self.get_logger().warning('Goal rejected; retrying while Nav2 activates')
            if not self._delay(min(1.0, max(0.0, deadline-time.monotonic())),
                               'Nav2 startup retry delay'):
                return False
        return False

    def cancel_active_goal(self):
        """Cancel only this node's goal and await its terminal result."""
        deadline = time.monotonic() + self._cancel_timeout
        def remaining():
            return max(0.0, deadline - time.monotonic())
        try:
            if self._send_future is not None:
                if not self._wait(self._send_future, remaining(), False):
                    self.get_logger().error(
                        'Could not resolve pending goal acceptance; cancellation '
                        'is unconfirmed. Check the robot before leaving it.')
                    return False
                handle = self._send_future.result()
                self._send_future = None
                if handle is not None and handle.accepted:
                    self._goal_handle = handle
                    self._result_future = handle.get_result_async()
            if self._goal_handle is None:
                return True
            if self._result_future is None:
                self._result_future = self._goal_handle.get_result_async()
            if not self._result_future.done():
                self.get_logger().warning('Cancelling this mission\'s active goal...')
                cancel_future = self._goal_handle.cancel_goal_async()
                if not self._wait(cancel_future, remaining(), False):
                    self.get_logger().error('Cancellation acknowledgement timed out')
                    return False
                response = cancel_future.result()
                if response is None or not response.goals_canceling:
                    self.get_logger().warning(
                        'Cancel was not accepted; checking whether goal already ended')
            if not self._wait(self._result_future, remaining(), False):
                self.get_logger().error(
                    'Goal termination is unconfirmed. Check the robot before leaving it.')
                return False
            result = self._result_future.result()
            if result is None or result.status not in (
                    GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED,
                    GoalStatus.STATUS_CANCELED):
                self.get_logger().error('No confirmed terminal goal result')
                return False
            self.get_logger().info('This mission\'s goal has ended')
            self._goal_handle = self._result_future = None
            return True
        except Exception as error:
            self.get_logger().error(f'Could not confirm cancellation: {error}')
            return False

    def run(self):
        if not self._wait_for_server():
            return False
        started = time.monotonic()
        self.get_logger().info(
            f'Starting four-waypoint mission: {" -> ".join(self._labels)} '
            f'(map profile: {self._map_profile})')
        for index, label in enumerate(self._labels):
            if self.stop_requested or not rclpy.ok():
                return False
            x, y = landmark_coordinates(label, self._map_profile)
            self.get_logger().info(
                f'Navigating to waypoint {index+1}/4: {label} at ({x:.3f}, {y:.3f})')
            if not self._start_goal(index):
                return False
            if not self._wait(self._result_future):
                return False
            wrapped = self._result_future.result()
            if wrapped is None:
                self.get_logger().error('Missing navigation result; stopping mission')
                return False
            # A received result is terminal: cleanup must not cancel another goal.
            self._goal_handle = self._result_future = None
            if self.stop_requested:
                return False
            code = int(getattr(wrapped.result, 'error_code', 0))
            reason = getattr(wrapped.result, 'error_msg', '')
            if wrapped.status == GoalStatus.STATUS_SUCCEEDED and code == 0:
                self.get_logger().info(
                    f'ARRIVED: {label} (waypoint {index+1}/4). '
                    f'Waiting {self._pause_duration:.1f} seconds.')
                # NavigateToPose does not invoke WaitAtWaypoint. The mission
                # owns this pause, including after the final destination.
                if not self._delay(self._pause_duration, f'Arrival pause at {label}'):
                    return False
            elif wrapped.status == GoalStatus.STATUS_ABORTED or (
                    wrapped.status == GoalStatus.STATUS_SUCCEEDED and code != 0):
                self.missed.append(index)
                detail = reason or f'action status {wrapped.status}, error code {code}'
                if index+1 < len(self._labels):
                    next_label = self._labels[index+1]
                    self.get_logger().error(
                        f'SKIPPED: {label} failed (waypoint {index+1}/4): {detail}. '
                        f'Skipping to {next_label} in {self._skip_delay:.1f} seconds. '
                        'Declare this skip verbally for the graded run.')
                    if not self._delay(self._skip_delay, f'Skip delay after {label}'):
                        return False
                else:
                    self.get_logger().error(
                        f'FAILED: final waypoint {label}: {detail}. No next waypoint.')
            else:
                # External cancellation/preemption is not permission to resume.
                self.get_logger().warning(
                    f'Waypoint {label} ended with status {wrapped.status}; '
                    'mission stopped without advancing.')
                return False
        elapsed = time.monotonic() - started
        if self.missed:
            missed = ', '.join(f'{self._labels[i]} (#{i+1})' for i in self.missed)
            self.get_logger().error(
                f'Mission finished after {elapsed:.1f} seconds; missed: {missed}')
            return False
        self.get_logger().info(
            f'Mission complete: visited {" -> ".join(self._labels)} '
            f'in {elapsed:.1f} seconds')
        return True


def argument_parser():
    parser = argparse.ArgumentParser(
        description='Visit exactly four A-X goals, announcing failed waypoints.')
    parser.add_argument('letters', nargs='+', help='four A-X letters; repeats allowed')
    parser.add_argument('--server-timeout', type=float, default=60.0,
                        help='seconds to wait for server/goal acceptance (default: 60)')
    parser.add_argument('--map-profile', choices=sorted(MAP_GEOMETRIES), default='irl')
    parser.add_argument('--pause-duration', type=float, default=3.0,
                        help='seconds at each successful goal; minimum 3 (default: 3)')
    parser.add_argument('--skip-delay', type=float, default=3.0,
                        help='announcement delay before next goal after failure (default: 3)')
    parser.add_argument('--cancel-timeout', type=float, default=5.0,
                        help='seconds to confirm goal termination on exit (default: 5)')
    return parser


def main(args=None):
    ros_args = sys.argv if args is None else args
    parser = argument_parser()
    parsed = parser.parse_args(remove_ros_args(args=ros_args)[1:])
    try:
        labels = parse_labels(parsed.letters)
        for name in ('server_timeout', 'cancel_timeout'):
            value = getattr(parsed, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'--{name.replace("_", "-")} must be positive and finite')
        if not math.isfinite(parsed.pause_duration) or parsed.pause_duration < 3:
            raise ValueError('--pause-duration must be finite and at least 3 seconds')
        if not math.isfinite(parsed.skip_delay) or parsed.skip_delay < 0:
            raise ValueError('--skip-delay must be finite and non-negative')
    except ValueError as error:
        parser.error(str(error))

    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    previous_handlers = {}
    exit_code = 1
    try:
        node = LetterWaypointMission(labels, parsed.server_timeout, parsed.map_profile,
                                     parsed.pause_duration, parsed.skip_delay,
                                     parsed.cancel_timeout)
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[signum] = signal.signal(signum, node.request_stop)
        exit_code = 0 if node.run() else 1
    except KeyboardInterrupt:
        exit_code = 130
    except Exception as error:
        if node is not None:
            node.get_logger().error(f'Mission stopped: {error}')
        else:
            print(f'Could not create mission node: {error}', file=sys.stderr)
    finally:
        if node is not None:
            confirmed = node.cancel_active_goal()
            if not confirmed:
                exit_code = 1
            if node.stop_requested:
                exit_code = 128 + (node.stop_signal or signal.SIGINT)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
    return exit_code


if __name__ == '__main__':
    sys.exit(main())