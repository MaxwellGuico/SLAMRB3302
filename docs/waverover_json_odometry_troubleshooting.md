# Wave Rover JSON Feedback and Odometry Troubleshooting

This guide explains how the Wave Rover ROS driver obtains feedback, how to
inspect the original serial JSON, how to decide whether its odometry is a real
measurement, and when it is safe to provide that odometry to Cartographer.

## Important result for the current Cartographer configuration

The current configuration in
`student_ws/src/mapping_cartographer/config/cartographer.lua` contains:

```lua
provide_odom_frame = true
use_odometry = false
TRAJECTORY_BUILDER_2D.use_imu_data = false
```

Therefore, the current mapping stack:

- uses LiDAR scan matching to estimate motion;
- does not subscribe to odometry as a SLAM input;
- does not use the IMU as a SLAM input;
- publishes its own continuous `odom -> base_link` result because
  `provide_odom_frame` is enabled.

The `odom` remapping in `mapping.launch.py` does not mean odometry is being
used. It only says which topic Cartographer should use if `use_odometry` is
later changed to `true`.

Bad `/odom/raw` data will therefore not directly deform the current map.
However, a second node publishing a conflicting `odom -> base_link` transform
can still break the TF tree. Do not run the navigation EKF at the same time as
the current Cartographer mapping launch unless the TF ownership has been
redesigned deliberately.

## Data path inside `waverover_driver.py`

The driver expects the lower controller to send newline-terminated JSON over
the serial device `/dev/waverover`:

```text
Wave Rover lower controller / ESP32
              |
              | serial JSON
              v
SerialTransport.read_json_line()
              |
              v
WaveRoverDriver._process_feedback()
              |
              +--> /waverover/json_feedback  original JSON as a String
              +--> /waverover/raw_feedback   selected values as an array
              +--> /encoders                 encoder fields as an array
              +--> /odom/raw                 derived ROS odometry
```

The Python driver does not calculate `self.yaw_odom` from the wheels. It
initializes the variable to zero and then looks for a field named `yaw_odom` in
a `T: 1001` JSON message:

```python
self.yaw_odom = float(data.get("yaw_odom", self.yaw_odom))
```

If `yaw_odom` is absent, `data.get()` silently returns the previous value. The
value may therefore remain zero forever without producing an error.

The driver similarly expects the following `T: 1001` fields:

```text
dist_L
dist_R
dist_centre
yaw_odom
yaw_imu
encA
encB
voltage
```

For a `T: 1002` message it expects:

```text
yaw_imu
r
p
```

These names must match the firmware installed on the lower controller. Some
official Waveshare firmware versions use different names such as `L`, `R`,
`y`, `odl`, and `odr`. The actual JSON emitted by the robot is the source of
truth.

## 1. Start only the hardware driver

Do not start Nav2 or Cartographer for the first feedback test. Place the robot
in a clear area, keep an emergency stop available, and raise the drive wheels
off the floor if manually testing encoders.

From the repository root:

```bash
source /opt/ros/jazzy/setup.bash
source robot_ws/install/setup.bash
ros2 launch waverover_base driver.launch.py
```

The driver should report that it opened `/dev/waverover` at `115200` baud.

In a second terminal, source the same workspace:

```bash
source /opt/ros/jazzy/setup.bash
source robot_ws/install/setup.bash
```

Confirm that the driver exists:

```bash
ros2 node list | grep waverover
```

Expected result:

```text
/waverover_driver
```

Confirm the configured serial device:

```bash
ros2 param get /waverover_driver serial_port
```

Confirm that Linux can see it:

```bash
ls -l /dev/waverover
```

## 2. Read the original JSON feedback

Show every received JSON message:

```bash
ros2 topic echo /waverover/json_feedback
```

Show one message and exit:

```bash
ros2 topic echo /waverover/json_feedback --once
```

Show only the String payload:

```bash
ros2 topic echo /waverover/json_feedback --field data
```

The ROS message wraps the JSON in a `std_msgs/msg/String`, so normal output
looks similar to:

```yaml
data: '{"T":1001,"dist_L":0.12,"dist_R":0.13,"yaw_odom":0.04}'
---
```

Check its publication rate:

```bash
ros2 topic hz /waverover/json_feedback
```

The driver requests continuous feedback from the lower controller and sets a
10 ms interval, so a high-rate stream is expected. The precise observed rate
may be lower because of serial traffic and scheduling.

## 3. Read the decoded topics

The selected numeric fields are also published as:

```bash
ros2 topic echo /waverover/raw_feedback
```

The array order is:

```text
[dist_L,
 dist_R,
 dist_centre,
 yaw_odom,
 yaw_imu,
 imu_roll,
 imu_pitch,
 voltage,
 battery_percentage]
```

The two fields interpreted as encoders are published separately:

```bash
ros2 topic echo /encoders
```

Their order is:

```text
[encA, encB]
```

Inspect the resulting raw odometry:

```bash
ros2 topic echo /odom/raw
```

Useful individual fields are:

```bash
ros2 topic echo /odom/raw --field twist.twist.linear.x
ros2 topic echo /odom/raw --field twist.twist.angular.z
ros2 topic echo /odom/raw --field pose.pose.orientation
```

## 4. Check whether the expected JSON fields exist

Capture several `T: 1001` messages and answer these questions:

1. Is there a `yaw_odom` field?
2. Is there a `yaw_imu` field?
3. Are the wheel fields named `dist_L` and `dist_R`, or are they named
   something else such as `odl` and `odr`?
4. Are `encA` and `encB` present?
5. Do the values change when the relevant physical sensor changes?

If `yaw_odom` is missing, the current Python variable is not valid odometry.
Do not merely rename another field to `yaw_odom` until its definition, units,
axis direction, and source are known.

## 5. Determine whether the wheel fields are measured

With motor power disabled and the robot safely raised, turn one wheel manually
and watch:

```bash
ros2 topic echo /encoders
```

and:

```bash
ros2 topic echo /waverover/raw_feedback
```

Interpretation:

| Observation | Likely meaning |
| --- | --- |
| Encoder count changes with manual wheel rotation | A physical encoder signal is probably available |
| Wheel distance changes with encoder count | The firmware probably integrates encoder motion |
| Nothing changes during manual rotation | The chassis may not have wheel encoders, or the field names/configuration are wrong |
| Distance changes only when commands are sent | The distance may be open-loop command integration rather than measured wheel motion |
| `yaw_imu` changes when the chassis is physically rotated | It is probably sourced from the lower-controller IMU |
| `yaw_odom` always stays at zero | The JSON field may be absent, stale, or unsupported |

Do not intentionally stall powered motors for this test. A prolonged stall can
damage the motors or driver electronics.

## 6. Compare command, raw odometry, and the external IMU

Use separate terminals to observe:

```bash
ros2 topic echo /cmd_vel
```

```bash
ros2 topic echo /odom/raw
```

```bash
ros2 topic echo /imu_data_ros --field angular_velocity.z
```

The current Python hardware driver has one known problem:

```python
odom_msg.twist.twist.angular.z = float(self.current_angular)
```

`self.current_angular` is copied from `/cmd_vel`, so `/odom/raw` angular
velocity is commanded angular velocity rather than measured angular velocity.
Do not fuse that field in `robot_localization` until it has been replaced with
a measured value.

This incorrect twist field does **not directly affect Cartographer**.
Cartographer ROS converts an incoming `nav_msgs/msg/Odometry` message using
its timestamp, `child_frame_id`, and `pose.pose`; it does not copy the message's
`twist` into Cartographer's odometry data. The line above is still wrong for
Nav2, `robot_localization`, velocity feedback, and any other consumer that
uses odometry twist.

The fields that matter directly to Cartographer are the odometry pose produced
later in `_publish_odom()`:

```python
odom_msg.pose.pose.position.x = self.odom_x
odom_msg.pose.pose.position.y = self.odom_y
odom_msg.pose.pose.orientation.z = orientation_z
odom_msg.pose.pose.orientation.w = orientation_w
```

Those values depend on `dist_centre` and `yaw_odom`. If either one is missing,
command-derived, incorrectly scaled, or has the wrong sign, enabling odometry
can still harm mapping even though Cartographer ignores `twist.angular.z`.

The current planning EKF avoids this specific error by fusing only forward
velocity from `/odom/raw` and yaw rate from `/imu_data_ros`.

## 7. Basic calibration tests

Perform these tests slowly in an open space and record the physical result.

### Stationary test

Leave the robot motionless for at least 30 seconds.

Expected:

- wheel distances and encoder counts do not steadily increase;
- external IMU yaw rate averages close to zero;
- raw odometry position does not move significantly;
- no field follows old commands after the command timeout.

### Straight-line test

Mark a physical distance of 1 m and drive the robot straight to that mark.

Compare:

- actual tape-measured distance;
- change in `dist_L`;
- change in `dist_R`;
- change in `dist_centre`;
- change in `/odom/raw` position.

The left and right distances should have the same sign and similar magnitude.

### Rotation test

Rotate the robot through a physically measured 90 or 360 degrees.

Compare:

- `yaw_odom` from the lower controller, if present;
- `yaw_imu` from the lower controller;
- external `/imu_data_ros` yaw rate integrated over the turn;
- `/odom/raw` orientation.

ROS convention is positive counter-clockwise yaw about the upward Z axis. Do
not trust a yaw source until its sign and units have been verified.

The helper `_yaw_radians()` currently guesses degrees versus radians from the
value's magnitude. That is unsafe for small degree values and for unbounded
radian angles. Once the firmware units are known, the driver should use one
explicit conversion instead of guessing.

## 8. Record a diagnostic rosbag

From the repository root, choose a new output name for every run:

```bash
mkdir -p bags
ros2 bag record -o bags/waverover_feedback_test_01 \
  /waverover/json_feedback \
  /waverover/raw_feedback \
  /encoders \
  /odom/raw \
  /imu_data_ros \
  /cmd_vel \
  /tf \
  /tf_static
```

Stop recording with `Ctrl+C` after performing a stationary period, a short
straight drive, and a slow rotation.

Inspect the recording:

```bash
ros2 bag info bags/waverover_feedback_test_01
```

## 9. Cartographer decision guide

### Current recommended setting

Keep:

```lua
use_odometry = false
TRAJECTORY_BUILDER_2D.use_imu_data = false
```

until the JSON and odometry tests above pass. Drive slowly while mapping so
consecutive LiDAR scans overlap sufficiently for scan matching.

### What happens if odometry is enabled

When this is changed to:

```lua
use_odometry = true
```

Cartographer subscribes to `nav_msgs/msg/Odometry` on its `odom` input. In the
current launch file that input is remapped to the `odom_topic` launch argument,
whose default is `/odom`.

The mapping launch does not start the EKF, so `/odom` must actually exist if
odometry is enabled. `/odom/raw` and `/odom` are different topics.

With the default launch argument, Cartographer would consume the filtered
`/odom` pose, not the hardware driver's `/odom/raw` message. If no EKF or other
node publishes `/odom`, Cartographer will wait for an input that never arrives.

The command-derived line:

```python
odom_msg.twist.twist.angular.z = float(self.current_angular)
```

does not by itself corrupt Cartographer because Cartographer ignores odometry
twist. It would affect an EKF if that EKF were configured to fuse the raw
odometry yaw-rate field. The current planning EKF deliberately does not fuse
that field.

Bad odometry can then cause:

- doubled or smeared walls;
- curved walls after a nominally straight drive;
- poor scan matching during turns;
- jumps between submaps;
- incorrect loop closures;
- failure to recognize a previously visited area.

Cartographer can correct moderate wheel-pose drift with scan matching and loop
closure, but command-derived or otherwise incorrect odometry **pose** can
provide a strongly misleading motion prior.

Enable odometry only after confirming all of the following:

- the source measures actual motion rather than repeating `/cmd_vel`;
- timestamps are current and monotonic;
- linear distance scale is calibrated;
- yaw sign and units are correct;
- frame IDs are correct;
- pose and twist covariances are nonzero and realistic;
- only one node owns each TF transform.

## 10. TF ownership during mapping

The hardware driver defaults to:

```text
publish_odom_tf = false
```

Verify it while the driver is running:

```bash
ros2 param get /waverover_driver publish_odom_tf
```

For the current mapping configuration, Cartographer should own the continuous
`odom -> base_link` transform. Avoid simultaneously running another EKF that
publishes the same transform.

Inspect the TF tree with:

```bash
ros2 run tf2_tools view_frames
```

Then inspect `frames.pdf` and check for one connected chain:

```text
map -> odom -> base_link -> laser
```

## Troubleshooting table

| Symptom | Check | Likely action |
| --- | --- | --- |
| `/waverover_driver` is absent | `ros2 node list` | Source `robot_ws/install/setup.bash` and launch `driver.launch.py` |
| Driver cannot open the port | `ls -l /dev/waverover` | Fix the device path, udev rule, permissions, or serial connection |
| Topic exists but has no messages | `ros2 topic hz /waverover/json_feedback` | Check serial feedback flow and whether another process owns the port |
| JSON remains `{}` | Driver log and serial connection | No valid JSON line has been accepted |
| `T:1001` exists but expected keys are absent | Read `/waverover/json_feedback` | Update the Python parser to the actual firmware schema |
| `yaw_odom` is always zero | Check whether the key exists | Do not use it until the firmware source is identified |
| `/odom/raw.angular.z` exactly follows `/cmd_vel.angular.z` | Compare both topics | This is the current driver behavior; do not fuse that odometry field |
| Map has doubled walls | Confirm `use_odometry`, TF publishers and scan timing | Disable bad odometry, eliminate duplicate TF, then inspect LiDAR timing |
| Map bends during turns | Compare yaw sources and scan alignment | Correct yaw sign/scale or continue with LiDAR-only mapping |

## Recommended next action

Run the hardware driver and save one real `T:1001` message from:

```bash
ros2 topic echo /waverover/json_feedback --once
```

Compare its exact keys with those expected by `waverover_driver.py`. Do not
enable Cartographer odometry until that schema and the physical source of each
motion field are confirmed.
