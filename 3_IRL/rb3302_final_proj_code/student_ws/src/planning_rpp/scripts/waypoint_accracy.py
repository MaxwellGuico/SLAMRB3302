#!/usr/bin/env python3
"""
Measure how accurately the robot stops at each waypoint of a letter mission.

Record one run (start this BEFORE letter_waypoint.py, with the same letters):

    ros2 run planning_rpp waypoint_accuracy.py record A F K X --run run1
    ros2 run planning_rpp letter_waypoint.py A F K X

letter_waypoint.py sends one NavigateToPose goal per letter, in order, so the
recorder numbers the goals it sees on navigate_to_pose's status topic. One
second after a goal succeeds (inside the 3 s arrival pause) it stores the
robot's pose from the map -> base_link transform; an aborted goal is recorded
as SKIPPED. After the mission it asks for the tape-measured offset of the robot
from each reached square centre, so the total error can be split into its two
causes:

    stop error          |AMCL pose - target|     controller / goal tolerance
    localisation error  |AMCL pose - true pose|  AMCL estimate was wrong
    true error          |true pose - target|     what an observer measures

A mission only counts as complete when no goal was aborted (skipped). The
recorder also counts the recovery behaviours (spin / back up / wait) Nav2 ran.

Results go to <out-dir>/<run>.csv. Summarise all runs in a folder with:

    ros2 run planning_rpp waypoint_accuracy.py summary ~/rb3302_results --plot
"""

import argparse
import collections
import csv
import math
import os
import statistics
import sys
import time
from typing import Dict, List, Optional, Sequence, Tuple

from letter_waypoint import landmark_coordinates, MAP_GEOMETRIES, parse_labels

CSV_FIELDS = [
    'run', 'mission_status', 'mission_complete', 'mission_time_s',
    'collisions', 'interventions', 'recoveries',
    'index', 'label', 'reached', 'target_x', 'target_y',
    'amcl_x', 'amcl_y', 'amcl_yaw_deg', 'stop_error_cm',
    'true_dx_cm', 'true_dy_cm', 'true_error_cm', 'localisation_error_cm',
]

# action_msgs/GoalStatus codes for a finished goal.
STATUS_NAMES = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}

# Take the pose this long after a goal succeeds: the velocity smoother has
# finished braking, and letter_waypoint.py's 3 s pause has not ended yet.
SETTLE_DELAY_S = 1.0
POSE_SAMPLE_HZ = 20.0
# Nav2 behaviour-server actions counted as recoveries.
RECOVERY_ACTIONS = ('spin', 'backup', 'wait')
# End the recording if no new goal arrives this long after the last one
# finished (letter_waypoint.py waits 3 s between goals, or was stopped).
IDLE_TIMEOUT_S = 180.0


Pose2D = Tuple[float, float, float]  # x [m], y [m], yaw [rad] in the map frame


def distance_cm(ax: float, ay: float, bx: float, by: float) -> float:
    return 100.0 * math.hypot(ax - bx, ay - by)


# --------------------------------------------------------------------------
# record
# --------------------------------------------------------------------------

def make_recorder(labels: Sequence[str], map_profile: str, action_name: str,
                  map_frame: str, base_frame: str):
    """Build the ROS node. ROS imports live here so `summary` runs without ROS."""
    from action_msgs.msg import GoalStatusArray
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformException, TransformListener

    class WaypointAccuracyRecorder(Node):
        """Store the settled map pose at every waypoint of one mission."""

        def __init__(self) -> None:
            super().__init__('waypoint_accuracy_recorder')
            self.labels = list(labels)
            self.targets = [landmark_coordinates(label, map_profile) for label in labels]
            self.stops: Dict[int, Pose2D] = {}
            self.status: Optional[str] = None
            self.started_at: Optional[float] = None
            self.finished_at: Optional[float] = None
            # Indices of aborted (skipped) goals; None until the mission ends.
            self.missed: Optional[List[int]] = None
            self.recovery_goals = set()
            self._old_recovery_goals = set()

            # Waypoint index of each goal this mission sent, keyed by goal UUID.
            self._goal_index: Dict[bytes, int] = {}
            self._finished_goals = set()
            self._aborted: List[int] = []
            # (index, time to take the pose) for goals that just succeeded.
            self._pending: List[Tuple[int, float]] = []
            self._last_done: Optional[float] = None

            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
            self._tf_warned = False

            self.create_subscription(
                GoalStatusArray, f'{action_name}/_action/status', self._on_status, 10)
            # Action status topics are transient-local in rclcpp.
            status_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            for name in RECOVERY_ACTIONS:
                self.create_subscription(
                    GoalStatusArray, f'{name}/_action/status',
                    lambda msg, name=name: self._on_recovery_status(name, msg), status_qos)
            self.create_timer(1.0 / POSE_SAMPLE_HZ, self._sample_pose)
            self.create_timer(1.0, self._check_idle)
            self.get_logger().info(
                f'Recording {" -> ".join(self.labels)}; start letter_waypoint.py now.')

        @property
        def started(self) -> bool:
            return self.started_at is not None

        @property
        def finished(self) -> bool:
            return self.status is not None and not self._pending

        @property
        def complete(self) -> Optional[bool]:
            """Return True only if every goal was reached; None if unknown."""
            if self.status != 'SUCCEEDED':
                return False
            if self.missed is None:
                return None
            return not self.missed

        def _on_recovery_status(self, name: str, message) -> None:
            # Status arrays keep listing older goals, so remember everything seen
            # before this mission and count only goals that appear during it.
            keys = [(name, bytes(s.goal_info.goal_id.uuid)) for s in message.status_list]
            if not self.started:
                self._old_recovery_goals.update(keys)
                return
            if self.status is not None:
                return
            for key in keys:
                if key not in self.recovery_goals and key not in self._old_recovery_goals:
                    self.recovery_goals.add(key)
                    self.get_logger().info(f'Recovery behaviour: {name}')

        def _sample_pose(self) -> None:
            if not self._pending:
                return
            try:
                tf = self._tf_buffer.lookup_transform(map_frame, base_frame, Time())
            except TransformException as error:
                if not self._tf_warned:
                    self.get_logger().warning(
                        f'No {map_frame} -> {base_frame} transform yet ({error}); '
                        'is AMCL running with an initial pose?')
                    self._tf_warned = True
                return
            q = tf.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            t = tf.transform.translation
            # Record succeeded goals once the robot has had time to settle.
            now = time.monotonic()
            for index, due in [p for p in self._pending if p[1] <= now]:
                self._record_stop(index, (t.x, t.y, yaw))
                self._pending.remove((index, due))

        def _record_stop(self, index: int, pose: Pose2D) -> None:
            self.stops[index] = pose
            tx, ty = self.targets[index]
            self.get_logger().info(
                f'Waypoint {index + 1}/{len(self.labels)} {self.labels[index]}: '
                f'stopped at ({pose[0]:.3f}, {pose[1]:.3f}), '
                f'{distance_cm(pose[0], pose[1], tx, ty):.1f} cm from target (AMCL)')

        def _on_status(self, message) -> None:
            if self.status is not None:
                return
            for entry in message.status_list:
                uuid = bytes(entry.goal_info.goal_id.uuid)
                if uuid not in self._goal_index:
                    # A new goal is first listed as accepted/executing. Goals
                    # first seen already finished belong to an earlier mission.
                    if entry.status in STATUS_NAMES or len(self._goal_index) >= len(self.labels):
                        continue
                    self._goal_index[uuid] = len(self._goal_index)
                    if not self.started:
                        self.started_at = time.monotonic()
                        self.get_logger().info('Mission started.')
                if entry.status in STATUS_NAMES and uuid not in self._finished_goals:
                    self._finished_goals.add(uuid)
                    self._on_goal_done(self._goal_index[uuid], STATUS_NAMES[entry.status])

        def _check_idle(self) -> None:
            if (self.status is None and self._last_done is not None
                    and time.monotonic() - self._last_done > IDLE_TIMEOUT_S):
                self.get_logger().warning(
                    f'No new goal for {IDLE_TIMEOUT_S:.0f} s; letter_waypoint.py stopped early.')
                self._end_mission('STOPPED')

        def _on_goal_done(self, index: int, result: str) -> None:
            self._last_done = time.monotonic()
            if result == 'SUCCEEDED':
                self._pending.append((index, time.monotonic() + SETTLE_DELAY_S))
            elif result == 'ABORTED':
                self._aborted.append(index)
                self.get_logger().warning(
                    f'Waypoint {index + 1} {self.labels[index]} skipped (goal aborted).')
            else:
                # letter_waypoint.py stops the whole mission after a cancel.
                self._end_mission('CANCELED')
                return
            if len(self._finished_goals) == len(self.labels):
                self._end_mission('SUCCEEDED')

        def _end_mission(self, status: str) -> None:
            self.status = status
            self.finished_at = time.monotonic()
            self.missed = sorted(self._aborted)
            names = ', '.join(self.labels[i] for i in self.missed)
            self.get_logger().info(
                f'Mission {status}' + (f'; skipped: {names}' if names else '; no goal skipped.'))

    return WaypointAccuracyRecorder()


def ask_offset(label: str) -> Optional[Tuple[float, float]]:
    """Ask for the tape-measured robot-centre offset from a square's centre."""
    while True:
        reply = input(f'  {label}: offset east,north in cm (blank = not measured): ').strip()
        if not reply:
            return None
        try:
            east, north = (float(part) for part in reply.replace(' ', ',').split(',') if part)
            return east, north
        except ValueError:
            print('    Enter two numbers, e.g. "3,-2" (3 cm east, 2 cm south).')


def ask_int(prompt: str, default: int) -> int:
    reply = input(f'{prompt} [{default}]: ').strip()
    try:
        return int(reply) if reply else default
    except ValueError:
        return default


def completion_text(complete: Optional[bool]) -> str:
    return {True: 'yes', False: 'no', None: 'unverified'}[complete]


def build_rows(run: str, recorder, offsets: Dict[int, Tuple[float, float]],
               collisions: int, interventions: int) -> List[dict]:
    mission_time = ''
    if recorder.started_at is not None and recorder.finished_at is not None:
        mission_time = f'{recorder.finished_at - recorder.started_at:.1f}'
    rows = []
    for index, label in enumerate(recorder.labels):
        tx, ty = recorder.targets[index]
        row = {field: '' for field in CSV_FIELDS}
        if recorder.missed is not None and index in recorder.missed:
            reached = 'missed'
        elif index in recorder.stops:
            reached = 'yes'
        else:
            reached = 'no'
        row.update(run=run, mission_status=recorder.status or 'INCOMPLETE',
                   mission_complete=completion_text(recorder.complete),
                   mission_time_s=mission_time, collisions=str(collisions),
                   interventions=str(interventions),
                   recoveries=str(len(recorder.recovery_goals)),
                   index=str(index + 1), label=label, reached=reached,
                   target_x=f'{tx:.3f}', target_y=f'{ty:.3f}')
        stop = recorder.stops.get(index)
        if stop is not None:
            ax, ay, ayaw = stop
            row.update(amcl_x=f'{ax:.3f}', amcl_y=f'{ay:.3f}',
                       amcl_yaw_deg=f'{math.degrees(ayaw):.1f}',
                       stop_error_cm=f'{distance_cm(ax, ay, tx, ty):.1f}')
        if index in offsets and reached == 'yes':
            east, north = offsets[index]
            true_x, true_y = tx + east / 100.0, ty + north / 100.0
            row.update(true_dx_cm=f'{east:.1f}', true_dy_cm=f'{north:.1f}',
                       true_error_cm=f'{math.hypot(east, north):.1f}')
            if stop is not None:
                error = distance_cm(stop[0], stop[1], true_x, true_y)
                row['localisation_error_cm'] = f'{error:.1f}'
        rows.append(row)
    return rows


def write_csv(path: str, rows: List[dict]) -> None:
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    with open(path, 'w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def record(parsed, ros_args: List[str]) -> int:
    import rclpy

    labels = parse_labels(parsed.letters)
    run = parsed.run or time.strftime('run_%Y%m%d_%H%M%S')
    path = os.path.join(os.path.expanduser(parsed.out_dir), f'{run}.csv')
    if os.path.exists(path) and not parsed.overwrite:
        print(f'{path} already exists; choose another --run or pass --overwrite.')
        return 2

    rclpy.init(args=ros_args)
    recorder = make_recorder(labels, parsed.map_profile, parsed.action,
                             parsed.map_frame, parsed.base_frame)
    try:
        while rclpy.ok() and not recorder.finished:
            rclpy.spin_once(recorder, timeout_sec=0.1)
    except KeyboardInterrupt:
        print('\nStopped before the mission finished; saving what was recorded.')
    finally:
        recorder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    offsets: Dict[int, Tuple[float, float]] = {}
    if not parsed.no_measure:
        print('\nMeasure the robot centre against each square centre '
              '(+east = +x, +north = +y in RViz).')
        for index, label in enumerate(labels):
            if index not in recorder.stops:
                print(f'  {label}: not reached, skipped')
                continue
            offset = ask_offset(label)
            if offset is not None:
                offsets[index] = offset
        collisions = ask_int('Collisions or wall/obstacle contacts during the run', 0)
        interventions = ask_int('Manual interventions (touched / repositioned the robot)', 0)
    else:
        collisions = interventions = 0

    rows = build_rows(run, recorder, offsets, collisions, interventions)
    write_csv(path, rows)
    print(f'\nSaved {path}')
    print_run_table(rows)
    print(f'Mission complete (all goals reached in order): {rows[0]["mission_complete"]}; '
          f'recoveries: {rows[0]["recoveries"]}')
    return 0 if recorder.complete else 1


# --------------------------------------------------------------------------
# summary
# --------------------------------------------------------------------------

def floats(rows: List[dict], field: str) -> List[float]:
    return [float(row[field]) for row in rows if row.get(field) not in (None, '')]


def describe(values: List[float]) -> str:
    if not values:
        return 'not measured'
    return f'{statistics.mean(values):.1f} cm mean, {max(values):.1f} cm max (n={len(values)})'


def print_run_table(rows: List[dict]) -> None:
    print('\n| # | Square | Reached | Stop error (AMCL) | True error | Localisation error |')
    print('|---|---|---|---|---|---|')
    for row in rows:
        fields = ('stop_error_cm', 'true_error_cm', 'localisation_error_cm')
        cells = [row[f] or '-' for f in fields]
        print(f"| {row['index']} | {row['label']} | {row.get('reached') or '-'} | "
              + ' | '.join(cells) + ' |')


def load_rows(folder: str) -> List[dict]:
    rows = []
    for name in sorted(os.listdir(folder)):
        if name.endswith('.csv'):
            with open(os.path.join(folder, name), newline='') as handle:
                rows.extend(csv.DictReader(handle))
    return rows


def run_complete(row: dict) -> str:
    """
    Return 'yes', 'no' or 'unverified' for one run.

    CSVs from before the missed-waypoint check only have mission_status,
    which cannot prove that no goal was skipped.
    """
    if row.get('mission_complete'):
        return row['mission_complete']
    return 'unverified' if row['mission_status'] == 'SUCCEEDED' else 'no'


def summary(parsed) -> int:
    folder = os.path.expanduser(parsed.folder)
    rows = load_rows(folder)
    if not rows:
        print(f'No run CSVs found in {folder}')
        return 1

    runs: Dict[str, List[dict]] = collections.OrderedDict()
    for row in rows:
        runs.setdefault(row['run'], []).append(row)
    first_rows = [run_rows[0] for run_rows in runs.values()]
    completed = [r for r in first_rows if run_complete(r) == 'yes']
    unverified = sum(1 for r in first_rows if run_complete(r) == 'unverified')
    missed = sum(1 for r in rows if r.get('reached') == 'missed')
    times = floats(completed, 'mission_time_s')
    collisions = sum(int(row['collisions'] or 0) for row in first_rows)
    interventions = sum(int(row.get('interventions') or 0) for row in first_rows)
    recoveries = sum(int(row.get('recoveries') or 0) for row in first_rows)
    true_errors = floats(rows, 'true_error_cm')
    worst = max((r for r in rows if r['true_error_cm']),
                key=lambda r: float(r['true_error_cm']), default=None)

    print(f'Runs: {len(runs)} ({", ".join(runs)})\n')
    print('| Metric | Result |')
    print('|---|---|')
    note = f' ({unverified} unverified)' if unverified else ''
    print(f'| All 4 goals reached in order | {len(completed)} / {len(runs)} runs{note} |')
    print(f'| Goals skipped (aborted) | {missed} |')
    print(f'| Mean waypoint error (tape-measured) | '
          f'{statistics.mean(true_errors):.1f} cm |' if true_errors else
          '| Mean waypoint error | not measured |')
    if worst is not None:
        print(f"| Worst waypoint error | {float(worst['true_error_cm']):.1f} cm "
              f"(square {worst['label']}, {worst['run']}) |")
    if times:
        spread = f', sd {statistics.stdev(times):.1f} s' if len(times) > 1 else ''
        print('| Completion time (complete runs, incl. pauses) | '
              f'{statistics.mean(times):.1f} s mean '
              f'({min(times):.1f}-{max(times):.1f} s{spread}) |')
    print(f'| Collisions / interventions / recoveries | '
          f'{collisions} / {interventions} / {recoveries} |')

    print('\nError breakdown (for the analysis):')
    print(f'  stop error         (AMCL vs target):    {describe(floats(rows, "stop_error_cm"))}')
    localisation = describe(floats(rows, 'localisation_error_cm'))
    print(f'  localisation error (AMCL vs measured):  {localisation}')
    print(f'  true error         (measured vs target): {describe(true_errors)}')

    print('\nPer square (true error):')
    by_label: Dict[str, List[dict]] = collections.defaultdict(list)
    for row in rows:
        by_label[row['label']].append(row)
    for label in sorted(by_label):
        print(f'  {label}: {describe(floats(by_label[label], "true_error_cm"))}')

    if parsed.plot:
        plot_offsets(rows, os.path.join(folder, 'waypoint_accuracy.png'), parsed.tolerance_cm)
    return 0


def plot_offsets(rows: List[dict], path: str, tolerance_cm: float) -> None:
    """Scatter of measured stop offsets around the target, one colour per square."""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle
    except ImportError:
        print('matplotlib is not installed; skipping the plot (pip install matplotlib).')
        return

    measured = [r for r in rows if r['true_dx_cm'] and r['true_dy_cm']]
    if not measured:
        print('No measured offsets to plot.')
        return
    fig, ax = plt.subplots(figsize=(5, 5))
    for label in sorted({r['label'] for r in measured}):
        points = [r for r in measured if r['label'] == label]
        ax.scatter([float(r['true_dx_cm']) for r in points],
                   [float(r['true_dy_cm']) for r in points], label=f'square {label}', s=40)
    ax.add_patch(Circle((0, 0), tolerance_cm, fill=False, linestyle='--', color='grey',
                        label=f'goal tolerance ({tolerance_cm:g} cm)'))
    ax.axhline(0, color='lightgrey', linewidth=0.8)
    ax.axvline(0, color='lightgrey', linewidth=0.8)
    limit = max(tolerance_cm * 1.5, max(abs(float(r[f])) for r in measured
                                        for f in ('true_dx_cm', 'true_dy_cm')) * 1.2)
    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_aspect('equal')
    ax.set_xlabel('east offset from square centre (cm)')
    ax.set_ylabel('north offset from square centre (cm)')
    ax.set_title('Where the robot stopped (tape-measured)')
    ax.legend(fontsize=8, loc='upper right')
    fig.tight_layout()
    fig.savefig(path, dpi=200)
    print(f'\nSaved plot {path}')


# --------------------------------------------------------------------------
# entry point
# --------------------------------------------------------------------------

def argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Record and summarise waypoint stop accuracy for letter missions.')
    sub = parser.add_subparsers(dest='command', required=True)

    rec = sub.add_parser('record', help='record one mission (start before letter_waypoint.py)')
    rec.add_argument('letters', nargs='+',
                     help='the same four letters given to letter_waypoint.py')
    rec.add_argument('--run', help='run name, used as the CSV file name (default: timestamp)')
    rec.add_argument('--out-dir', default='~/rb3302_results', help='folder for run CSVs')
    rec.add_argument('--map-profile', choices=sorted(MAP_GEOMETRIES), default='irl')
    rec.add_argument('--action', default='navigate_to_pose',
                     help='NavigateToPose action used by letter_waypoint.py')
    rec.add_argument('--map-frame', default='map')
    rec.add_argument('--base-frame', default='base_link')
    rec.add_argument('--no-measure', action='store_true',
                     help='skip the tape-measure prompts (AMCL stop error only)')
    rec.add_argument('--overwrite', action='store_true', help='replace an existing run CSV')

    summ = sub.add_parser('summary', help='combine all run CSVs in a folder')
    summ.add_argument('folder', nargs='?', default='~/rb3302_results')
    summ.add_argument('--plot', action='store_true', help='save waypoint_accuracy.png')
    summ.add_argument('--tolerance-cm', type=float, default=5.0,
                      help='goal tolerance circle on the plot (xy_goal_tolerance x 100)')
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    ros_args = sys.argv if argv is None else argv
    try:
        from rclpy.utilities import remove_ros_args
        cli_args = remove_ros_args(args=ros_args)[1:]
    except ImportError:  # summary on a machine without ROS
        cli_args = ros_args[1:]
    parsed = argument_parser().parse_args(cli_args)
    if parsed.command == 'summary':
        return summary(parsed)
    try:
        return record(parsed, ros_args)
    except ValueError as error:
        argument_parser().error(str(error))
    return 2


if __name__ == '__main__':
    sys.exit(main())
