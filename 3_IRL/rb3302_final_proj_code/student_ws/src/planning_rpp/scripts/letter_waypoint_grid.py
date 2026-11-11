#!/usr/bin/env python3
"""Experimental grid mission. Default: print route only; --execute enables motion.

Run beside your CURRENT letter_waypoint_precision.py so its coordinates and
precision fixes are retained. --start is the cell the robot currently occupies;
all positional letters are requested stops. Uses Spin/DriveOnHeading, not RPP.
Static routes are checked against the live /map before execution. Nav2 behaviors
check dynamic collisions. A blocked action stops the mission (no auto-recovery).
AMCL accuracy is estimated, not a guarantee of physical two-centimetre accuracy.
"""
import argparse
import ast
import heapq
import math
from pathlib import Path
import signal
import sys
import time

# Open centre-to-centre passages checked against the committed IRL map.
# Runtime /map validation may REMOVE edges; it never invents new ones.
OPEN_EDGES = 'AB AG BC CD CI DE DJ EF FL GM HI IO KQ LR MN MS OP OU PQ QR RX ST TU UV VW WX'.split()


def coordinates_from_current_script():
    source = Path(__file__).with_name('letter_waypoint_precision.py')
    for item in ast.parse(source.read_text()).body:
        if isinstance(item, ast.Assign) and any(
                isinstance(t, ast.Name) and t.id == 'MAP_GEOMETRIES' for t in item.targets):
            geometry = ast.literal_eval(item.value)['irl']
            return {chr(65+i): (geometry['column_centres_x'][i % 6],
                               geometry['row_centres_y'][i // 6]) for i in range(24)}
    raise ValueError('MAP_GEOMETRIES missing from neighbouring precision script')


def direction(a, b):
    ra, ca = divmod(ord(a)-65, 6)
    rb, cb = divmod(ord(b)-65, 6)
    if abs(ra-rb)+abs(ca-cb) != 1:
        raise ValueError('Route contains non-neighbour cells')
    return (cb-ca, ra-rb)  # map x,y signs


def graph_from_edges(edges):
    graph = {chr(65+i): [] for i in range(24)}
    for edge in edges:
        a, b = edge
        direction(a, b)
        graph[a].append(b)
        graph[b].append(a)
    return graph


def route_between(start, goal, graph, coordinates):
    # Minimise metric distance with a small penalty for each direction change.
    queue = [(0.0, (start,), (0, 0))]
    best = {}
    while queue:
        cost, path, incoming = heapq.heappop(queue)
        cell = path[-1]
        key = (cell, incoming)
        if key in best and best[key] <= cost:
            continue
        best[key] = cost
        if cell == goal:
            return list(path)
        for neighbour in sorted(graph[cell]):
            outgoing = direction(cell, neighbour)
            step = math.dist(coordinates[cell], coordinates[neighbour])
            turn = 0.20 if incoming != (0, 0) and outgoing != incoming else 0.0
            heapq.heappush(queue, (cost+step+turn, path+(neighbour,), outgoing))
    raise ValueError(f'No open grid route from {start} to {goal}')


def sections(route):
    """Each section includes its endpoints and straight-through cells."""
    if len(route) < 2:
        return []
    result, first = [], 0
    for i in range(1, len(route)-1):
        if direction(route[i-1], route[i]) != direction(route[i], route[i+1]):
            result.append(route[first:i+1])
            first = i
    result.append(route[first:])
    return result


def line_metrics(x, y, start, end):
    dx, dy = end[0]-start[0], end[1]-start[1]
    length = math.hypot(dx, dy)
    ux, uy = dx/length, dy/length
    along = (x-start[0])*ux + (y-start[1])*uy
    cross = -(x-start[0])*uy + (y-start[1])*ux
    return length-along, cross


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def edge_clear_on_map(message, a, b, radius):
    """Conservative capsule check against occupied/unknown map pixels.

    Includes half a pixel diagonal so any intersecting cell is considered.
    Handles a rotated map origin. Also rejects corridors outside map bounds.
    """
    info = message.info
    q = info.origin.orientation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    c, s = math.cos(yaw), math.sin(yaw)
    def local(point):
        x, y = point[0]-info.origin.position.x, point[1]-info.origin.position.y
        return c*x+s*y, -s*x+c*y
    ax, ay = local(a)
    bx, by = local(b)
    res, width, height = info.resolution, info.width, info.height
    if res <= 0 or len(message.data) != width*height:
        return False
    pad = radius + res/math.sqrt(2)
    if (min(ax, bx)-radius < 0 or min(ay, by)-radius < 0 or
            max(ax, bx)+radius > width*res or max(ay, by)+radius > height*res):
        return False
    dx, dy = bx-ax, by-ay
    length2 = dx*dx+dy*dy
    for row in range(max(0, int((min(ay, by)-pad)/res)),
                     min(height, math.ceil((max(ay, by)+pad)/res))):
        for col in range(max(0, int((min(ax, bx)-pad)/res)),
                         min(width, math.ceil((max(ax, bx)+pad)/res))):
            value = message.data[row*width+col]
            if 0 <= value < 50:
                continue
            x, y = (col+.5)*res, (row+.5)*res
            t = max(0.0, min(1.0, ((x-ax)*dx+(y-ay)*dy)/length2))
            if math.hypot(x-ax-t*dx, y-ay-t*dy) <= pad:
                return False
    return True


def preview(start, destinations, graph, coordinates):
    for destination in destinations:
        route = route_between(start, destination, graph, coordinates)
        print('Route: ' + ' -> '.join(route))
        for part in sections(route):
            print(f'  Straight: {" -> ".join(part)}; stop at {part[-1]}')
        print(f'  Requested stop: {destination} (3 s minimum)')
        start = destination


def execute(args, coordinates):
    # Lazy imports keep the default preview runnable without ROS installed.
    import rclpy
    from action_msgs.msg import GoalStatus, GoalStatusArray
    from nav_msgs.msg import OccupancyGrid
    from nav2_msgs.action import Spin, DriveOnHeading
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from letter_waypoint_precision import PrecisionWaypointMission, PrecisionFailure

    class GridMission(PrecisionWaypointMission):
        def __init__(self):
            super().__init__(args.letters, 30.0, 'irl', args.pause, 3.0, 5.0)
            self.heading_tolerance = 0.20  # endpoint correction; retain patched method
            self.approach_timeout = 45.0
            self.max_corrections = 20
            self._grid_map = None
            self._watch_section = None
            self._navigation_busy = False
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(OccupancyGrid, '/map', self.on_map, qos)
            self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status',
                                     self.on_navigation_status, qos)

        def on_navigation_status(self, message):
            self._navigation_busy = any(item.status in (1, 2, 3)
                                        for item in message.status_list)

        def on_map(self, message):
            self._grid_map = message

        def pose_state(self):
            # Existing geometry checks pose timestamps and scan freshness.
            self._geometry(coordinates[args.start])
            pose = self._pose.pose.pose
            q = pose.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            return pose.position.x, pose.position.y, yaw

        def _behavior(self, client, goal):
            # Same owned-goal cancellation slots as the working precision script.
            if self._navigation_busy:
                raise PrecisionFailure('Another navigation goal is active')
            if not client.wait_for_server(timeout_sec=2.0):
                raise PrecisionFailure('Required Nav2 behavior unavailable')
            self._send_future = client.send_goal_async(goal)
            if not self._wait(self._send_future, timeout=3.0):
                raise PrecisionFailure('Behavior acceptance unconfirmed')
            handle = self._send_future.result()
            self._send_future = None
            if handle is None or not handle.accepted:
                raise PrecisionFailure('Nav2 rejected movement')
            self._goal_handle = handle
            self._result_future = handle.get_result_async()
            deadline = time.monotonic() + goal.time_allowance.sec + 3.0
            while not self._result_future.done():
                if self.stop_requested or not rclpy.ok():
                    raise PrecisionFailure('Mission interrupted')
                if self._navigation_busy:
                    raise PrecisionFailure('Another navigation goal became active')
                if time.monotonic() > deadline:
                    raise PrecisionFailure('Movement timed out')
                now = time.monotonic()
                if now-self._scan_received > 1.0 or now-self._motion_received > .5:
                    raise PrecisionFailure('Scan or odometry stale during movement')
                if self._watch_section is not None:
                    start, end, heading = self._watch_section
                    x, y, yaw = self.pose_state()
                    remaining, cross = line_metrics(x, y, start, end)
                    if abs(cross) > args.max_cross_track:
                        raise PrecisionFailure('Centre-line drift exceeded limit')
                    if abs(wrap(heading-yaw)) > .25:
                        raise PrecisionFailure('Heading drift exceeded limit')
                    if remaining < -.02:
                        raise PrecisionFailure('Passed section endpoint')
                    if remaining <= .08:
                        if not self.cancel_active_goal():
                            raise PrecisionFailure('Section cancellation unconfirmed')
                        return  # close enough to start precision correction
                rclpy.spin_once(self, timeout_sec=.05)
            result = self._result_future.result()
            self._goal_handle = self._result_future = None
            if (result is None or result.status != GoalStatus.STATUS_SUCCEEDED or
                    int(getattr(result.result, 'error_code', 0)) != 0):
                reason = getattr(getattr(result, 'result', None), 'error_msg', '')
                raise PrecisionFailure('Movement aborted: '+reason)

        def align(self, heading, target):
            previous = None
            reversals = 0
            for _ in range(10):
                self._fresh_pose(target)
                _, _, yaw = self.pose_state()
                error = wrap(heading-yaw)
                if abs(error) <= .08:
                    return
                if previous is not None and error*previous < 0:
                    reversals += 1
                if reversals >= 3:
                    raise PrecisionFailure('Alignment oscillating; stopping')
                previous = error
                goal = Spin.Goal()
                # Large corrections reduce latency; small corrections reduce overshoot.
                goal.target_yaw = error if abs(error) > .5 else .5*error
                goal.time_allowance.sec = 30
                self._behavior(self._spin, goal)
                self._settle(.3)
            raise PrecisionFailure('Alignment correction limit reached')

        def drive_section(self, part):
            start, end = coordinates[part[0]], coordinates[part[-1]]
            heading = math.atan2(end[1]-start[1], end[0]-start[0])
            self.align(heading, end)
            best = float('inf')
            for _ in range(8):
                self._fresh_pose(end)
                x, y, yaw = self.pose_state()
                remaining, cross = line_metrics(x, y, start, end)
                if abs(cross) > args.max_cross_track:
                    raise PrecisionFailure('Robot outside section centre corridor')
                if remaining < -.02:
                    raise PrecisionFailure('Robot already beyond endpoint')
                if math.hypot(x-end[0], y-end[1]) <= .10:
                    break
                if remaining > best-.01:
                    raise PrecisionFailure('No map-position progress along section')
                best = remaining
                if abs(wrap(heading-yaw)) > .12:
                    raise PrecisionFailure('Heading changed before straight movement')
                goal = DriveOnHeading.Goal()
                goal.target.x = min(2.0, max(.02, remaining-.08))
                goal.speed = args.speed
                goal.time_allowance.sec = math.ceil(goal.target.x/args.speed*3+5)
                self._watch_section = (start, end, heading)
                try:
                    self._behavior(self._drive, goal)
                finally:
                    self._watch_section = None
                self._settle(.3)
            else:
                raise PrecisionFailure('Section progress limit reached')
            # Reuse YOUR working endpoint fixes; no coarse NavigateToPose handoff.
            self._correct_position(end)

        def run_grid(self):
            deadline = time.monotonic()+15
            while self._grid_map is None:
                if self.stop_requested or time.monotonic() > deadline:
                    raise PrecisionFailure('No live /map received')
                rclpy.spin_once(self, timeout_sec=.1)
            self._settle(.3)
            if self._navigation_busy:
                raise PrecisionFailure('Cancel the existing navigation mission first')
            self._fresh_pose(coordinates[args.start])
            if self._geometry(coordinates[args.start])[0] > .10:
                raise PrecisionFailure('--start does not match robot position within 10 cm')
            self._correct_position(coordinates[args.start])
            current = args.start
            for destination in args.letters:
                # Recheck static corridors at each requested stop.
                edges = [edge for edge in OPEN_EDGES if edge_clear_on_map(
                    self._grid_map, coordinates[edge[0]], coordinates[edge[1]],
                    args.clearance_radius)]
                graph = graph_from_edges(edges)
                route = route_between(current, destination, graph, coordinates)
                self.get_logger().info('GRID ROUTE: '+' -> '.join(route))
                for part in sections(route):
                    self.get_logger().info('GRID STRAIGHT: '+' -> '.join(part))
                    self.drive_section(part)
                # Include repeated destinations: always recheck before scoring pause.
                self._fresh_pose(coordinates[destination])
                if self._geometry(coordinates[destination])[0] > self.position_tolerance:
                    raise PrecisionFailure('Destination position changed after correction')
                self.get_logger().info(f'ARRIVED: {destination}; estimated map tolerance 2 cm')
                if not self._delay(args.pause, f'Arrival pause at {destination}'):
                    return False
                current = destination
            return True

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    handlers = {}
    success = False
    try:
        node = GridMission()
        for signum in (signal.SIGINT, signal.SIGTERM):
            handlers[signum] = signal.signal(signum, node.request_stop)
        success = node.run_grid()
    except Exception as error:
        if node is not None:
            node.get_logger().error(str(error)+'; stopping grid mission')
        else:
            print(str(error), file=sys.stderr)
    finally:
        if node is not None:
            if not node.cancel_active_goal():
                success = False
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for signum, handler in handlers.items():
            signal.signal(signum, handler)
    return 0 if success else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('letters', nargs='+', help='requested destination letters A-X')
    parser.add_argument('--start', required=True, help='cell currently occupied')
    parser.add_argument('--execute', action='store_true', help='enable robot movement')
    parser.add_argument('--speed', type=float, default=.15, help='straight speed m/s')
    parser.add_argument('--pause', type=float, default=3.0)
    parser.add_argument('--clearance-radius', type=float, default=.20,
                        help='circumscribed robot radius plus clearance, metres')
    parser.add_argument('--max-cross-track', type=float, default=.06,
                        help='abort if map-estimated lateral drift exceeds this many metres')
    args = parser.parse_args()
    args.start = args.start.upper()
    args.letters = [label.upper() for label in args.letters]
    for label in [args.start]+args.letters:
        if len(label) != 1 or not 'A' <= label <= 'X':
            parser.error('All letters must be A-X')
    for name in ('speed', 'pause', 'clearance_radius', 'max_cross_track'):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            parser.error(f'{name} must be positive and finite')
    if not .08 <= args.speed <= .25 or args.pause < 3:
        parser.error('Use speed 0.08–0.25 m/s and pause >= 3 seconds')
    try:
        coordinates = coordinates_from_current_script()
        preview(args.start, args.letters, graph_from_edges(OPEN_EDGES), coordinates)
        if not args.execute:
            print('PREVIEW ONLY: no ROS connection or movement. Add --execute to move.')
            return 0
        return execute(args, coordinates)
    except (ValueError, OSError) as error:
        parser.error(str(error))


if __name__ == '__main__':
    sys.exit(main())
