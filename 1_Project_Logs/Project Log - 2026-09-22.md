# What did I do
- Was able to make the robot move from one point to another
	- That was the biggest debug
		`use_rotate_to_heading: false # putting this to true makes the robot only yaw`
	- Under `nav2_params.yaml`
- Changed the size of the inflation layer

# Future direction
### Debugging rotate to heading
1. One thing found was 
> [!Warning] Watch out
> odom_msg.twist.twist.angular.z = float(self.current_angular)
> This publishes command velocity as odom. Which doesnt make sense. This affects odom readings.

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
	1. Yes there is 
2. Is there a `yaw_imu` field?
	1. yes there is
3. Are the wheel fields named `dist_L` and `dist_R`, or are they named
   something else such as `odl` and `odr`?
	1. there is dist_centre which is publishing 0, dist_L: 0, dist_R:0
4. Are `encA` and `encB` present?
	1. yes, its the last pne
5. Do the values change when the relevant physical sensor changes?
6. 

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

Bad odometry can then cause:

- doubled or smeared walls;
- curved walls after a nominally straight drive;
- poor scan matching during turns;
- jumps between submaps;
- incorrect loop closures;
- failure to recognize a previously visited area.

Cartographer can correct moderate wheel drift with scan matching and loop
closure, but command-derived motion is not a valid sensor measurement and can
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

# WAVE ROVER ROS Driver Inconsistency Audit

This document audits the custom ROS 2 driver at
`robot_ws/src/waverover_base/waverover_base/waverover_driver.py` against the
official Waveshare WAVE ROVER documentation and the current official
Waveshare lower-controller source repositories.

The purpose is to distinguish:

- confirmed software inconsistencies;
- likely firmware/protocol mismatches that must be checked on the real robot;
- limitations of a stock WAVE ROVER chassis;
- observations that are encouraging but do not prove that odometry is measured.

No driver change should be made solely from this audit without first recording
the actual JSON emitted by the installed lower-controller firmware. Waveshare
has multiple controller and firmware variants with different feedback fields.

## Executive summary

The current driver should not yet be treated as a source of measured wheel
odometry.

The highest-priority reasons are:

1. Waveshare documents the stock WAVE ROVER as using motors **without
   encoders**. It also documents the `T:13` ROS linear/angular velocity command
   as being for the encoder-equipped UGV01.
2. The driver's expected `T:1001` fields do not match either of the current
   official Waveshare feedback formats inspected.
3. The driver republishes cached feedback with new timestamps and calculates
   velocity using a timer period rather than the actual feedback interval.
4. Raw odometry angular velocity is copied directly from the command.
5. Odometry covariance is left at zero and yaw units are guessed from value
   magnitude.

The observed result—positive angular command, positive `/odom`, and increasing
yaw—is useful evidence that the signs are consistent. It does **not** establish
that the reported motion is independent sensor feedback.

## Evidence sources

Primary Waveshare sources used for this audit:

- [WAVE ROVER Wiki](https://www.waveshare.com/wiki/WAVE_ROVER)
- [Waveshare ROS Driver `json_cmd.h` (pinned revision)](https://github.com/waveshareteam/ugv_base_ros/blob/2e7df97c331ff63e1481afb34feec52014a3632a/ROS_Driver/json_cmd.h)
- [Waveshare ROS Driver feedback implementation (pinned revision)](https://github.com/waveshareteam/ugv_base_ros/blob/2e7df97c331ff63e1481afb34feec52014a3632a/ROS_Driver/ugv_advance.h)
- [Waveshare General Driver feedback implementation (pinned revision)](https://github.com/waveshareteam/ugv_base_general/blob/d308df91b333a513f45a238bef9ba9a0a5edf64c/General_Driver/ugv_advance.h)
- [Waveshare General Driver command definitions (pinned revision)](https://github.com/waveshareteam/ugv_base_general/blob/d308df91b333a513f45a238bef9ba9a0a5edf64c/General_Driver/json_cmd.h)
- [Waveshare chassis feedback tutorial](https://www.waveshare.com/wiki/06_Get_Chassis_Feedback)
- [Waveshare sub-controller JSON command set](https://www.waveshare.com/wiki/08_Sub-controller_JSON_Command_Set)

Supplementary ROS sources:

- [`nav_msgs/Odometry` message definition](https://docs.ros.org/lunar/api/nav_msgs/html/msg/Odometry.html)
- [`robot_localization` sensor-data preparation guide](https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/doc/preparing_sensor_data.rst)
- [Cartographer ROS API](https://github.com/cartographer-project/cartographer_ros/blob/master/docs/source/ros_api.rst)

The Waveshare source-code links are pinned to the revisions inspected. Wiki and
ROS documentation can still change after this audit. The statements below
describe the sources as checked on 22 September 2026.

## Severity definitions

| Level | Meaning |
| --- | --- |
| Blocker | Do not enable or trust odometry until resolved |
| High | Can produce false, stale, or overconfident state estimates |
| Medium | Causes incorrect configuration or degraded behaviour |
| Low | Maintainability or operational problem with limited odometry impact |

## Findings

### 1. Stock WAVE ROVER hardware does not provide wheel encoders

**Severity: Blocker if this is a stock WAVE ROVER**

The official [WAVE ROVER Wiki](https://www.waveshare.com/wiki/WAVE_ROVER)
states that the WAVE ROVER's motors are without encoders. It describes `T:1`
as left/right motor speed control and recommends that command for this product.
On the same page, the `T:13` command is shown as:

```json
{"T":13,"X":0.1,"Z":0.3}
```

where `X` is linear velocity in metres per second and `Z` is angular velocity
in radians per second. The Wiki marks this command as being only for UGV01 with
encoders.

The custom driver nevertheless sends `T:13` for every ROS velocity command in
[`waverover_driver.py:308`](../robot_ws/src/waverover_base/waverover_base/waverover_driver.py#L308):

```python
self.transport.write_json({"T": 13, "X": round(linear, 4), "Z": round(angular, 4)})
```

It also expects encoder counts and accumulated distance from the controller.
Those values cannot be real wheel-encoder measurements on a stock encoderless
WAVE ROVER.

This does not prove that the project hardware has no encoders. The chassis may
have been modified, may use a different Waveshare controller, or may run custom
firmware. Hardware inspection and the emitted JSON are the deciding evidence.

**Required verification**

- Confirm the exact chassis and lower-controller model.
- Inspect each drive motor for encoder wiring in addition to motor power wires.
- With motor commands at zero, gently move the robot and check whether wheel
  counts or distance feedback changes.
- Do not stall powered motors to perform this test.

### 2. The driver's `T:1001` schema does not match official Waveshare schemas

**Severity: Blocker**

At
[`waverover_driver.py:323`](../robot_ws/src/waverover_base/waverover_base/waverover_driver.py#L323),
the driver expects these fields in a `T:1001` message:

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

The current official
[ROS Driver command header](https://github.com/waveshareteam/ugv_base_ros/blob/2e7df97c331ff63e1481afb34feec52014a3632a/ROS_Driver/json_cmd.h#L1-L8)
documents a `T:1001` message with the following shape:

```json
{
  "T": 1001,
  "L": 0,
  "R": 0,
  "gx": 0,
  "gy": 0,
  "gz": 0,
  "ax": 0,
  "ay": 0,
  "az": 0,
  "mx": 0,
  "my": 0,
  "mz": 0,
  "odl": 0,
  "odr": 0,
  "v": 11.0
}
```

The current
[General Driver `baseInfoFeedback()` implementation](https://github.com/waveshareteam/ugv_base_general/blob/d308df91b333a513f45a238bef9ba9a0a5edf64c/General_Driver/ugv_advance.h#L375-L401)
has another variant. It emits fields including `L`, `R`, `r`, `p`, `y`,
`temp`, and `v`. In that implementation, `y` comes from `icm_yaw`, which is
IMU yaw; it is not labelled wheel-odometry yaw.

Neither inspected official format contains `dist_centre`, `yaw_odom`,
`yaw_imu`, `encA`, `encB`, or `voltage` under those names.

The driver hides missing fields by keeping their previous values:

```python
self.dist_centre = float(data.get("dist_centre", self.dist_centre))
self.yaw_odom = float(data.get("yaw_odom", self.yaw_odom))
```

Consequently, a schema mismatch produces plausible-looking topics containing
unchanged zeros rather than a visible error.

**Required correction**

Capture actual `T:1001` feedback first:

```bash
ros2 topic echo /waverover/json_feedback --field data
```

Then define a parser for that exact firmware version, including units and
scaling. The parser should reject or warn about a `T:1001` message that lacks
required fields instead of silently retaining old values.

### 3. The official feedback fields do not directly provide the pose assumed by the driver

**Severity: Blocker**

The driver constructs planar pose from two assumed cumulative values:

```python
delta_dist = dist_centre - self.prev_dist_centre
self.odom_x += delta_dist * math.cos(yaw)
self.odom_y += delta_dist * math.sin(yaw)
```

See
[`waverover_driver.py:382`](../robot_ws/src/waverover_base/waverover_base/waverover_driver.py#L382).

The inspected official protocol does not define `dist_centre` or `yaw_odom`.
The ROS firmware variant instead reports left/right fields such as `odl` and
`odr`, while the General Driver exposes IMU attitude as `r`, `p`, and `y`.

Before implementing odometry, the following must be established for each
available field:

- physical source: encoder, IMU, motor controller estimate, or command;
- unit: ticks, revolutions, metres, degrees, radians, or scaled integer;
- sign convention;
- whether the value is instantaneous, incremental, or cumulative;
- reset and wrap behaviour.

It is unsafe to rename `odl`, `odr`, or `y` to the expected fields without
confirming those semantics and applying the correct conversion.

### 4. `twist.angular.z` is the commanded value, not feedback

**Severity: High**

The command callback copies `/cmd_vel.angular.z` into `self.current_angular`:

```python
self.current_angular = angular
```

The odometry publisher later does:

```python
odom_msg.twist.twist.angular.z = float(self.current_angular)
```

See
[`waverover_driver.py:288`](../robot_ws/src/waverover_base/waverover_base/waverover_driver.py#L288)
and
[`waverover_driver.py:415`](../robot_ws/src/waverover_base/waverover_base/waverover_driver.py#L415).

The official Wiki defines `T:13` `Z` as the requested steering angular
velocity. Therefore this field reports what was requested, not what the robot
measured.

The present planning EKF does not fuse this particular field: it selects only
raw odometry forward velocity and selects yaw rate from the external IMU. That
limits the current damage, but the raw odometry message remains semantically
incorrect for other consumers.

### 5. Cached feedback is republished as if it were a new measurement

**Severity: High**

The serial thread stores the latest values when a JSON line arrives. A separate
timer calls `_feedback_publish_callback()` at the configured `feedback_rate`
(100 Hz) and republishes the cached values:

```python
self.create_timer(1.0 / self.feedback_rate, self._feedback_publish_callback)
```

See
[`waverover_driver.py:250`](../robot_ws/src/waverover_base/waverover_base/waverover_driver.py#L250)
and
[`waverover_driver.py:351`](../robot_ws/src/waverover_base/waverover_base/waverover_driver.py#L351).

`_publish_odom()` then gives every publication the current ROS time, even when
no new controller feedback was received. This creates two problems:

- stale state appears current after serial feedback stops;
- the reported ROS topic rate measures the publisher timer, not the real
  controller feedback rate.

Waveshare's
[official command definitions](https://github.com/waveshareteam/ugv_base_ros/blob/2e7df97c331ff63e1481afb34feec52014a3632a/ROS_Driver/json_cmd.h#L127-L143)
document `T:131` as enabling continuous serial feedback and `T:142` as setting
the feedback-flow interval. The driver correctly sends those commands, but
should publish odometry once per newly received `T:1001` packet, or at minimum
preserve its receipt timestamp and report stale-data status.

### 6. Linear velocity uses timer time rather than measurement time

**Severity: High**

The driver calculates:

```python
dt = (now - self.prev_odom_time).nanoseconds / 1e9
linear_velocity = delta_dist / dt
```

Because this runs from the publishing timer, `dt` is not necessarily the time
between two new distance samples. If serial feedback arrives more slowly or
out of phase with the timer, the output can alternate between zero and an
artificial velocity spike.

The velocity calculation should be triggered by a new valid motion-feedback
packet and use the interval between the two packet receipt times, unless the
firmware supplies a reliable measurement timestamp.

### 7. Yaw units are guessed from magnitude

**Severity: High**

At
[`waverover_driver.py:377`](../robot_ws/src/waverover_base/waverover_base/waverover_driver.py#L377):

```python
if abs(yaw_value) > 2.0 * math.pi:
    return math.radians(yaw_value)
return yaw_value
```

This cannot reliably identify units:

- `5` degrees is incorrectly treated as `5` radians;
- an accumulated angle of `7` radians is incorrectly treated as `7` degrees.

Waveshare's command documentation explicitly labels the `T:13` angular
velocity `Z` as radians per second, but that statement does not define the unit
of every feedback-yaw field in every firmware version. Once the actual
feedback field is identified, its unit should be encoded explicitly in the
driver or a parameter. It should never be inferred from magnitude.

### 8. Odometry covariance is never populated

**Severity: High when used by `robot_localization`**

The driver publishes `nav_msgs/Odometry` without assigning either
`pose.covariance` or `twist.covariance`. They therefore remain all zero.

The [ROS odometry message](https://docs.ros.org/lunar/api/nav_msgs/html/msg/Odometry.html)
defines both pose and twist as values **with covariance**. The
[`robot_localization` guide](https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/doc/preparing_sensor_data.rst)
specifically warns that a fused variable should not have zero variance; the
filter substitutes a very small value, making the measurement unrealistically
confident.

The current planning EKF fuses `twist.linear.x` from `/odom/raw`, so at least
that variance must be measured and populated before the result is trusted.

### 9. Counter resets and implausible distance jumps are not handled

**Severity: Medium**

The driver assumes `dist_centre` is an indefinitely continuous cumulative
distance. If the lower controller restarts, resets its counter, wraps its
integer representation, or emits one malformed sample, the full jump is
integrated as physical robot motion.

The driver should reject non-finite values, detect deltas that exceed the
maximum physically possible distance for the elapsed time, and re-establish
the baseline after a controller reset.

### 10. Position integration is inaccurate while turning

**Severity: Medium**

The driver integrates the entire distance increment using the latest yaw:

```python
self.odom_x += delta_dist * math.cos(yaw)
self.odom_y += delta_dist * math.sin(yaw)
```

For a robot that translates and rotates during the same interval, a midpoint
heading or differential-drive arc integration is more accurate. Correct arc
integration also requires independently measured left/right wheel travel and
the calibrated wheel separation.

This mathematical improvement is secondary to establishing that the hardware
actually provides encoder measurements.

### 11. Battery launch parameter names do not match the node

**Severity: Medium**

The node declares:

```text
low_battery_percentage
very_low_battery_percentage
```

but
[`driver.launch.py:40`](../robot_ws/src/waverover_base/launch/driver.launch.py#L40)
passes:

```text
low_battery_percent
very_low_battery_percent
```

The supplied overrides therefore do not configure the parameters read by the
node. Identical defaults currently hide the mismatch.

### 12. `very_low_flash_rate` and `force` are unused

**Severity: Low**

`very_low_flash_rate` is declared and read, but the display toggles once per
`display_update_rate` callback. Changing `very_low_flash_rate` therefore has no
effect.

Similarly, `_send_display_text(text, force=False)` never reads `force`. These
do not affect odometry, but they indicate that the configuration interface and
implementation have diverged.

### 13. Some initialization commands are not in the current official command list

**Severity: Medium; firmware-version dependent**

The driver sends `T:161` and `T:151` during startup at
[`waverover_driver.py:261`](../robot_ws/src/waverover_base/waverover_base/waverover_driver.py#L261).
Neither command was found in the current official `json_cmd.h` files inspected
for the ROS Driver and General Driver.

This is absence-of-evidence, not proof that the commands are invalid. They may
belong to custom or older firmware. Their responses should be validated and
their intended functions documented for the firmware actually installed.

The other relevant startup commands are present in current official sources:

| Command | Documented purpose |
| --- | --- |
| `T:127` | IMU calibration step |
| `T:131` | Continuous base-feedback enable/disable |
| `T:136` | Heartbeat timeout configuration |
| `T:142` | Feedback-flow interval |
| `T:143` | Serial command echo enable/disable |

The
[official source comments](https://github.com/waveshareteam/ugv_base_ros/blob/2e7df97c331ff63e1481afb34feec52014a3632a/ROS_Driver/json_cmd.h#L112-L120)
that `T:127` IMU calibration takes about five seconds. When
`rezero_imu_on_start` is enabled, the driver waits two seconds **before**
sending `T:127`, but only 0.2 seconds afterward. The driver can therefore
continue initialization while calibration is still in progress.

### 14. Serial failures can turn into silently stale odometry

**Severity: High**

`read_json_line()` returns an empty dictionary for serial exceptions, invalid
JSON, non-JSON text, and no waiting data. The reader cannot distinguish a
normal empty poll from a disconnected controller. Because cached data is still
published by the timer, a persistent serial failure can look like valid
zero-motion odometry.

The driver should track the time of the last valid `T:1001` packet, publish a
diagnostic status, throttle parse-error logs, and stop publishing fresh-stamped
odometry when feedback is stale.

## Interpretation of the completed sign test

The observed behaviour was:

```text
positive angular command
positive /odom response
yaw increases
```

This is consistent with the ROS convention for positive counter-clockwise yaw
about the positive Z axis. It is an important sign check and should be retained
as a passing test.

It does not answer whether `/odom` is based on:

- wheel encoders;
- lower-controller IMU yaw;
- the separate external IMU;
- scan matching;
- commanded velocity;
- a combination of those sources.

In the current navigation launch, the EKF selects forward velocity from
`/odom/raw` and yaw rate from the external IMU. Therefore, increasing filtered
`/odom` yaw can be explained entirely by the external IMU even if the wheel
odometry is invalid.

## Current mapping impact

The current Cartographer configuration contains:

```lua
use_odometry = false
TRAJECTORY_BUILDER_2D.use_imu_data = false
```

Cartographer therefore does not currently consume `/odom` or the IMU as SLAM
inputs. The present map is based primarily on LiDAR scan matching. Cartographer
documents odometry as an optional input that is subscribed to only when
`use_odometry` is enabled.

Do not enable Cartographer odometry until the blocker findings above have been
resolved. Incorrect odometry pose—not merely incorrect odometry twist—can then
mislead scan matching.

## Verification checklist before enabling measured odometry

- [ ] Exact chassis and lower-controller model recorded.
- [ ] Presence or absence of physical wheel encoders confirmed.
- [ ] Installed firmware version recorded.
- [ ] At least ten real `T:1001` messages saved.
- [ ] Parser field names match the actual messages.
- [ ] Source and unit of every motion field documented.
- [ ] Motion feedback changes when the robot is moved with commands at zero.
- [ ] Straight-line distance calibrated against a tape measurement.
- [ ] Positive-yaw sign test passes.
- [ ] A measured 90-degree turn produces approximately `+1.57` or `-1.57`
      radians, as appropriate for direction.
- [ ] Feedback loss is detected and stale odometry is not fresh-stamped.
- [ ] Pose and twist covariances are nonzero and experimentally justified.
- [ ] Reset, wrap, outlier, and non-finite-value handling is tested.
- [ ] Exactly one node publishes each dynamic TF transform.

## Recommended repair order

1. Identify the exact hardware and verify whether encoders exist.
2. Capture real controller JSON and document its schema and units.
3. Replace the parser with a strict, version-appropriate parser.
4. Publish odometry only on new valid feedback and preserve measurement time.
5. Derive motion only from measured fields; remove command values from odometry.
6. Add stale-feedback diagnostics, reset/outlier detection, and covariance.
7. Calibrate distance and yaw against physical measurements.
8. Only then consider enabling odometry in Cartographer or fusing additional
   odometry variables in `robot_localization`.

For command-by-command tests, topic inspection, and rosbag recording, see
[`waverover_json_odometry_troubleshooting.md`](waverover_json_odometry_troubleshooting.md).
## Fix the jerky stop go stop teleop
### Why it jerks during teleop

The teleop has:

```
key_timeout = 0.5
```

at [keyboard_teleop.py (line 27)](/home/maxwell-guico/rb3302/rb3302_planning_navigation/robot_ws/src/waverover_base/waverover_base/keyboard_teleop.py:27).

After 0.5 seconds without another keyboard event, it does:

```
self.active_key = None
```

and publishes a zero `Twist` at [keyboard_teleop.py (line 85)](/home/maxwell-guico/rb3302/rb3302_planning_navigation/robot_ws/src/waverover_base/waverover_base/keyboard_teleop.py:85).

When holding `w`, terminal key repetition can look like:

```
w pressed → forward
keyboard-repeat delay → zero command
repeated w arrives → forward
another gap → zero
```

That exactly produces go–stop–go–stop.

Test it:

```
ros2 topic echo /cmd_vel --field linear.x
```

Hold `w`. If you see:

```
0.4
0.4
0.0
0.0
0.4
```

the teleop timeout is the cause.

Restart teleop with a longer timeout and lower initial speed:

```
ros2 run waverover_base keyboard_teleop --ros-args \
  -p key_timeout:=1.2 \
  -p linear_speed:=0.2 \
  -p angular_speed:=0.6
```

# More problems in the driver
[[Waverover inconsistencies]]
# Why solving these matters
### What the JSON mismatch affects

There are two separate data paths:

```
/cmd_vel → driver → T:13 JSON → motors
controller feedback → T:1001 JSON → parser → /odom/raw
```

The outgoing `T:13` command is valid JSON and is clearly being accepted because teleop moves the rover.

A mismatch in incoming `T:1001` field names primarily affects:

- encoder readings;
- calculated distance;
- `/odom/raw`;
- battery readings;
- EKF and Nav2 behaviour when they consume that feedback.

For example, if the firmware sends:

```
{"T":1001,"odl":120,"odr":125,"y":3.2,"v":12.1}
```

but the driver looks for:

```
dist_L, dist_R, dist_centre, yaw_odom, encA, encB, voltage
```

the driver silently retains the previous values. The robot can still respond to teleop while its odometry remains wrong or frozen.

Because your encoder system is customized, official Waveshare field names are only a comparison point. The actual custom firmware output is the authoritative schema.

Capture it with:

```
ros2 topic echo /waverover/json_feedback --field data
```

### If `/cmd_vel` is steady but the physical rover still jerks

Then inspect these next.

1. Check for competing publishers:

```
ros2 topic info /cmd_vel --verbose
```

During a teleop-only test, there should normally be one publisher: the keyboard teleop. If Nav2 or another teleop node is also publishing, the commands can fight each other.

2. Watch for driver timeout messages:

```
cmd_vel timeout, stopping robot
```

The driver timeout is 0.8 seconds at [driver.launch.py (line 14)](/home/maxwell-guico/rb3302/rb3302_planning_navigation/robot_ws/src/waverover_base/launch/driver.launch.py:14). Normally this should not trigger because teleop publishes at 30 Hz.

3. Reduce serial feedback traffic.

The driver requests feedback every 10 ms:

```
feedback_interval_ms = 10
```

That means 100 feedback messages per second. A moderately sized JSON line at 100 Hz can approach or exceed the useful capacity of a 115200-baud serial link. The official Waveshare ROS firmware uses a default extra feedback delay of approximately 50 ms in its [configuration source](https://github.com/waveshareteam/ugv_base_ros/blob/2e7df97c331ff63e1481afb34feec52014a3632a/ROS_Driver/ugv_config.h#L357).

For testing, change:

```
"feedback_rate": 20.0,
"feedback_interval_ms": 50,
```

4. Avoid sending every command twice.

Currently:

- teleop publishes at 30 Hz;
- `_cmd_vel_callback()` immediately sends every command;
- the driver heartbeat sends it again at 20 Hz.

That can produce approximately 50 outgoing motor commands per second. It is unnecessary. A cleaner design is for the callback to update the desired velocity and for the 20 Hz heartbeat to perform the regular serial transmission, with zero commands handled immediately for safety.

Start with the `/cmd_vel` echo test. If its values alternate between forward and zero, increasing `key_timeout` should address the main jerking symptom. If `/cmd_vel` remains steady, reduce the feedback rate and inspect multiple publishers next.