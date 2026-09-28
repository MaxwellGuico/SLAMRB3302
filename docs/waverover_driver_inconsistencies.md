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
