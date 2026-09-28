# Real-Robot Heading and Planning Guide

This guide explains which source files to update on the physical RB3302 robot
after correcting the WAVE ROVER wheel-odometry yaw rate. It deliberately keeps
the hardware driver, localization, and controller changes separate so each
stage can be tested safely.

## The simple picture

The data travels through three stages:

```text
WAVE ROVER firmware
        |
        v
/odom/raw from waverover_driver.py
        |
        v
EKF in planning.launch.py
        |
        +--> /odom
        +--> odom -> base_link TF
                    |
                    v
             Nav2 / planning_rpp
```

Fixing `odom_msg.twist.twist.angular.z` makes the raw odometry report measured
turning speed instead of commanded turning speed. The next important change is
to make the EKF use the wheel-derived yaw pose from `/odom/raw`. RPP then gets
the robot's filtered heading through TF.

`use_rotate_to_heading` controls how RPP behaves; it does not select the
heading sensor.

## Confirm the active robot workspace

The expected real-robot repository is:

```text
~/rb3302_final_proj_code
```

Do not assume that a file in another clone is the one ROS is running. In a
fresh terminal on the robot, run:

```bash
source /opt/ros/jazzy/setup.bash
source ~/rb3302_final_proj_code/robot_ws/install/setup.bash
source ~/rb3302_final_proj_code/student_ws/install/setup.bash

ros2 pkg prefix waverover_base
ros2 pkg prefix planning_rpp
```

The reported prefixes should be inside `~/rb3302_final_proj_code`. Edit files
under `src`, never files under `build` or `install`.

## File 1: physical WAVE ROVER driver

Edit:

```text
~/rb3302_final_proj_code/robot_ws/src/waverover_base/waverover_base/waverover_driver.py
```

This is a real-hardware file. Confirm that it contains all of the following:

1. `T:151` is not sent during startup. Testing showed that this command makes
   this robot move without a `/cmd_vel` publisher.
2. The previous wheel yaw, previous feedback time, and measured angular
   velocity are initialized safely.
3. Angular velocity is calculated only when a fresh `T:1001` message containing
   `yaw_odom` arrives:

   ```text
   measured angular velocity = change in yaw_odom / elapsed seconds
   ```

4. The first yaw sample establishes a baseline and produces zero angular
   velocity.
5. The time interval is checked before division.
6. `yaw_odom` is treated directly as radians. Do not guess units based on
   whether its magnitude is greater than `2*pi`.
7. `_publish_odom()` copies the measured angular velocity while holding
   `feedback_lock`, then publishes that local copy as `twist.angular.z`.
8. The driver keeps `publish_odom_tf` set to `False`; the EKF, not the raw
   driver, should publish `odom -> base_link` on the real robot.

For later tuning, give the odometry pose and twist realistic nonzero
covariances. The current driver leaves them at zero, which does not communicate
the real uncertainty from wheel slip. First prove the signs and data flow, then
tune covariance without changing several localization inputs at once.

Build this workspace after editing:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/rb3302_final_proj_code/robot_ws
colcon build --symlink-install --packages-select waverover_base
source install/setup.bash
```

### Driver-only safety test

Raise the wheels before launching. Start only the robot driver, with no
navigation or teleoperation node publishing `/cmd_vel`.

Confirm:

- the robot remains stationary during startup;
- stopping the driver does not cause movement;
- `/waverover/json_feedback` continues arriving;
- stationary `twist.angular.z` returns to approximately zero;
- clockwise and counter-clockwise turns produce opposite signs;
- manual rotation with `/cmd_vel` equal to zero still changes the measured
  angular velocity.

Useful commands:

```bash
ros2 topic info /cmd_vel --verbose
ros2 topic hz /waverover/json_feedback
ros2 topic echo /odom/raw --field twist.twist.angular.z
```

Do not proceed to autonomous navigation until startup and shutdown are safe.

## File 2: EKF heading selection

Edit:

```text
~/rb3302_final_proj_code/student_ws/src/planning_rpp/launch/planning.launch.py
```

Find `odom0_config`. Its 15 Boolean entries mean:

```text
[x, y, z,
 roll, pitch, yaw,
 vx, vy, vz,
 vroll, vpitch, vyaw,
 ax, ay, az]
```

The existing configuration enables only `vx`, so the EKF currently ignores
wheel yaw. Enable the sixth entry, `yaw`, while keeping `vx` enabled:

```python
'odom0_config': [
    False, False, False,
    False, False, True,   # use yaw pose from /odom/raw
    True, False, False,   # use forward velocity from /odom/raw
    False, False, False,
    False, False, False,
],
```

Do not also enable `vyaw` from the same `/odom/raw` source at this stage. The
yaw pose and calculated yaw rate both originate from the wheel encoders, so
using both as independent measurements would double-count the same information.

The `imu0` entry currently uses `angular_velocity.z` from `/imu_data_ros`. That
topic is the external IMU and is separate from the lower controller's JSON
field named `yaw_imu`. Keep the external IMU rate enabled for the first test if
it has already behaved reliably. Change one sensor choice at a time.

Keep these settings:

```python
'world_frame': odom_frame,
'odom0_differential': False,
'odom0_relative': False,
'publish_tf': True,
```

Build the planning workspace:

```bash
source /opt/ros/jazzy/setup.bash
source ~/rb3302_final_proj_code/robot_ws/install/setup.bash
cd ~/rb3302_final_proj_code/student_ws
colcon build --symlink-install --packages-select planning_rpp
source install/setup.bash
```

### EKF heading test

Launch the robot driver and `planning_rpp`, but do not send a navigation goal
yet. Rotate the robot slowly and run:

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

Also inspect:

```bash
ros2 topic echo /odom --field pose.pose.orientation
ros2 topic hz /odom
```

The filtered heading must:

- change in the correct direction;
- stop changing when the robot stops;
- avoid the old incorrect degree conversion when `yaw_odom` exceeds `2*pi`.

The raw `yaw_odom` should continue accumulating. A displayed TF roll-pitch-yaw
angle may legitimately wrap from `+pi` to `-pi`; the quaternion should remain
valid and the physical orientation should not jump.

If this test fails, do not tune RPP yet. Fix the EKF input, sign, units, or TF
first.

## File 3: RPP behavior

Edit only when the EKF heading test passes:

```text
~/rb3302_final_proj_code/student_ws/src/planning_rpp/config/nav2_params.yaml
```

The relevant parameter is:

```yaml
use_rotate_to_heading: false
```

In simple terms:

- `false`: RPP starts following the path with forward and curved motion;
- `true`: RPP may stop and rotate in place until it faces roughly along the
  path, then begin following it.

This parameter does not turn heading estimation on or off. It only changes how
the controller reacts to a heading error.

Keep it `false` for the first real-robot navigation test. Once `odom ->
base_link` clearly follows the robot, optionally change it to `true` and test
whether the initial alignment now completes instead of rotating indefinitely.

## Files not to edit for this real-robot fix

Do not change these for the heading problem:

- anything under `robot_ws/build`, `robot_ws/install`, `student_ws/build`, or
  `student_ws/install`;
- `robot_ws/src/rb3302_sim/*`, which is for Gazebo;
- the Gazebo differential-drive plugin in
  `robot_ws/src/robot_model/urdf/diffbot.urdf.xacro`;
- simulation bridge or smoke-test files.

The robot model and TF package are still used on the physical robot, but the
Gazebo `wheel_separation` value does not control the firmware's wheel odometry.

## Recommended real-robot test order

1. Put the rover on a stand.
2. Build and launch the hardware driver only.
3. Confirm no uncommanded movement and validate `/odom/raw`.
4. Build and launch `planning_rpp` with no goal.
5. Validate `odom -> base_link` while manually turning the rover.
6. Keep `use_rotate_to_heading: false` and send a short, unobstructed goal.
7. Confirm the rover moves and turns in the expected direction.
8. Optionally test `use_rotate_to_heading: true` only after heading is proven.
9. Record `/odom/raw`, `/odom`, `/cmd_vel`, `/tf`, `/scan`, and
   `/imu_data_ros` in a rosbag during autonomous tests.

## If the computer and robot repositories differ

Do not copy the generated `install` directories. Either transfer the relevant
source changes or reproduce them manually in the three source files listed
above, then rebuild both workspaces on the robot.

Before launching, review exactly what changed:

```bash
cd ~/rb3302_final_proj_code
git diff -- robot_ws/src/waverover_base/waverover_base/waverover_driver.py
git diff -- student_ws/src/planning_rpp/launch/planning.launch.py
git diff -- student_ws/src/planning_rpp/config/nav2_params.yaml
```
