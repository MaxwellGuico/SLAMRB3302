# Project Log [5 Oct]

Source: [Project Log [5 Oct]](https://app.notion.com/p/3f0b182e8f04803c8911fc01893e152c)

---

# Moving forward

1. **Verify the drivetrain**
   Send constant commands while watching the robot on the ground:

   ```txt
   ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
   "{linear: {x: 0.0}, angular: {z: 0.5}}"
   ```

   Then test `-0.5`. Be ready to stop with `Ctrl+C`, followed by:

   ```txt
   ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
   "{linear: {x: 0.0}, angular: {z: 0.0}}"
   ```

   Both directions should produce similar rotation rates. For CCW, left wheels should reverse while right wheels move forward; CW should be the opposite.

   ### Calibrate wheel odometry separately

   After physical turning works, calculate the effective wheel separation from encoder travel and IMU-measured rotation:

   ```txt
   effective_separation =
       (delta_right - delta_left) / actual_IMU_yaw
   ```
2. **Determine whether the problem is command conversion or motors**
   Watch the JSON feedback during each test:

   ```txt
   ros2 topic echo /waverover/json_feedback
   ```

   Compare `dist_L`, `dist_R`, `encA`, and `encB`.

   - One encoder barely changes → motor, wiring, PID, PWM deadband, or encoder problem.
   - Both encoders change symmetrically but the chassis hardly rotates → wheel slip/skid geometry problem.
   - Encoder directions are wrong → sign/configuration problem.
   - Feedback claims large motion while wheels do not turn → feedback is likely command-derived rather than genuinely measured.
3. **Find the minimum reliable angular command**
   Test both signs at approximately:

   ```txt
   ±0.25, ±0.35, ±0.50, ±0.70 rad/s
   ```

   Identify the smallest magnitude where both directions start reliably. The controller must not command below this physical deadband during final alignment.
4. **Investigate the custom firmware initialization**
   Keep `T:151` disabled because it caused unexpected motion. Determine what `T:161` loads and whether it contains left/right motor calibration.
5. **Then change the EKF**
   Once the robot can physically rotate both ways, use external IMU `angular_velocity.z` in the EKF. That makes the controller see the real turn instead of believing inaccurate wheel odometry.

   Right now, your EKF:

   - Uses wheel-derived `odom/angular.z`.
   - Completely ignores the external Yesense IMU.

   That is visible in \[planning.launch.py (line 104)\](/home/rb3302-1/rb3302\_final\_proj\_code/student\_ws/src/planning\_rpp/launch/planning.launch.py:104).

   Change the configurations to:

   ```txt
   'odom0_config': [
       False, False, False,
       False, False, False,
       True,  False, False,   # wheel forward velocity
       False, False, False,   # disable wheel yaw rate
       False, False, False,
   ],

   'imu0_config': [
       False, False, False,
       False, False, False,   # don't use absolute IMU yaw initially
       False, False, False,
       False, False, True,    # use IMU angular_velocity.z
       False, False, False,
   ],
   ```

   Your Yesense driver already converts gyro degrees/s to radians/s at \[yesense\_node.cpp (line 374)\](/home/rb3302-1/rb3302\_final\_proj\_code/robot\_ws/src/yesense\_std\_ros2/src/yesense\_node.cpp:374).

   Do not fuse wheel `vyaw` and IMU `vyaw` together yet because the wheel odometry currently overestimates rotation and publishes no realistic covariance.
6. **Finally enable accurate closed-loop rotation**
   Enable `use_rotate_to_heading`, set a suitable minimum usable angular command, and tighten `yaw_goal_tolerance`.

   ### Re-enable closed-loop rotate-to-heading

   Your RPP configuration currently has:

   ```txt
   use_rotate_to_heading: false
   ```

   Change it to:

   ```txt
   use_rotate_to_heading: true
   rotate_to_heading_angular_vel: 0.5
   rotate_to_heading_min_angle: 0.35
   max_angular_accel: 0.8
   stateful: true
   ```

   ### 3. Tighten the goal tolerance

   Your goal checker currently uses its relatively loose defaults. Add:

   ```txt
   goal_checker:
     plugin: nav2_controller::SimpleGoalChecker
     xy_goal_tolerance: 0.05
     yaw_goal_tolerance: 0.05
     stateful: true
   ```

   `0.05 rad` is approximately `2.9°`. Start with this; making it much smaller may cause oscillation.

   These changes belong in \[nav2\_params.yaml (line 64)\](/home/rb3302-1/rb3302\_final\_proj\_code/student\_ws/src/planning\_rpp/config/nav2\_params.yaml:64).

   The previous “only keeps yawing” behaviour likely occurred because RPP was receiving incorrect wheel-derived heading feedback. Once the EKF follows the real IMU turn, RPP can stop when it reaches the target. Nav2’s stateful RPP behaviour is specifically designed to finish position first and then align with the goal heading. [Nav2 RPP documentation](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/controller_plugins/configuring_regulated_pp/).

The EKF/controller can correct overshoot or undershoot by continuing or reversing the command. It cannot correct a drivetrain that does not respond to one direction. Your nearly absent CW result makes drivetrain/firmware response the first priority.





# Fixing jerky movement

The strongest suspect is motor deadband caused by the `T:13` conversion.

Your RPP controller requests only:

```txt
desired_linear_vel: 0.15
regulated_linear_scaling_min_speed: 0.12
min_approach_linear_velocity: 0.05
```

For the public WAVE ROVER firmware, `main:1` uses open-loop PWM rather than encoder PID. Its approximate conversion is:

```txt
PWM = wheel_velocity × 512
```

Therefore:

| Requested velocity | Approximate PWM |
| --- | --- |
| 0.05 m/s | 26 |
| 0.12 m/s | 61 |
| 0.15 m/s | 77 |

Small DC motors may not start reliably around the lower values. RPP continually reduces velocity near turns, obstacles, and the goal. The commands therefore cross the motor’s deadband repeatedly:

```txt
command increases → motor starts suddenly
command decreases → motor stalls
controller increases command → motor starts again
```

That produces exactly the stop–go behaviour you described.

Rotation is affected even more. With the firmware’s `0.125 m` track width:

```txt
angular.z = 0.5 rad/s
wheel velocity = ±0.5 × 0.125 / 2
               = ±0.03125 m/s

PWM ≈ ±16
```

That is likely too low to rotate the chassis reliably, explaining the poor 90° results. The public WAVE ROVER documentation also says `T:13` is intended for encoder-equipped models, while normal WAVE ROVER control uses `T:1`. [WAVE ROVER documentation](https://www.waveshare.com/wiki/WAVE_ROVER), [firmware motion conversion](https://raw.githubusercontent.com/waveshareteam/ugv_base_general/main/General_Driver/movtion_module.h).

Your firmware is custom, but its behaviour strongly matches this issue.

### Confirm it first

Send a constant command without Nav2:

```txt
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.15}, angular: {z: 0.0}}"
```

Then try `0.20`, `0.25`, and `0.30 m/s`.

- Constant command is still jerky → firmware, PWM, motor, power or serial problem.
- Constant command is smooth, but Nav2 is jerky → controller/costmap/localization issue.
- `/cmd_vel` remains nonzero while the robot stops → motor deadband.
- The driver prints `cmd_vel timeout` → command frequency or processing delay.

During navigation, compare:

```txt
ros2 topic hz /cmd_vel_nav_raw
ros2 topic hz /cmd_vel
ros2 topic echo /cmd_vel
```

`/cmd_vel_nav_raw` is the RPP output; `/cmd_vel` is the smoothed command actually reaching the driver.

### Likely required fix

The proper fix is at the low-level command layer:

1. Measure the minimum command that moves both wheel banks reliably.
2. Add calibrated motor-deadband compensation.
3. Consider translating ROS velocity commands into the recommended `T:1` left/right control for this chassis, rather than sending `T:13` directly.
4. After movement is smooth, fix the EKF and closed-loop heading control.

Increasing `controller_frequency` from 20 to 30 or 40 Hz may make command interpolation finer, but it will not fix motors receiving commands below their usable PWM threshold.



Nav2 already performs closed-loop heading control, but it is not exactly a conventional PD controller, and it does not read the IMU directly.

The intended chain is:

```txt
external IMU
    ↓
robot_localization EKF
    ↓
/odom + odom → base_link TF
    ↓
Nav2 repeatedly checks current heading
    ↓
publishes angular.z until the target is reached
```

RPP and the Nav2 Spin behavior use heading error, acceleration limits, and braking calculations. Functionally, this provides closed-loop turning, although the algorithm is not simply `Kp × error + Kd × rate`.

In your current configuration, however, this is not happening correctly:

- The external IMU is completely disabled in the EKF.
- The EKF uses the inaccurate wheel-derived `angular.z`.
- `use_rotate_to_heading` is set to `false`.
- The goal checker uses its default yaw tolerance of approximately `0.25 rad` or `14°`.

To make Nav2 perform an IMU-feedback 90° rotation:

1. Fuse external IMU `angular_velocity.z` into the EKF.
2. Disable wheel-odometry `angular.z` in the EKF.
3. Set:

```txt
use_rotate_to_heading: true
```

1. Configure the goal tolerance:

```txt
goal_checker:
  plugin: nav2_controller::SimpleGoalChecker
  xy_goal_tolerance: 0.05
  yaw_goal_tolerance: 0.05
  stateful: true
```

1. Ensure the motor command can overcome its deadband.

After that, Nav2 should behave approximately like:

```txt
heading still far from target → continue rotating
heading approaching target → decelerate
heading within tolerance → publish zero
```

For testing a pure relative 90° turn, use Nav2’s `/spin` action. It uses the current TF heading in closed loop:

```txt
ros2 action send_goal /spin nav2_msgs/action/Spin \
"{target_yaw: 1.5708, time_allowance: {sec: 15, nanosec: 0}}"
```

Use `-1.5708` for clockwise.

The remaining limitation is motor deadband: Nav2 may reduce `angular.z` as it approaches the target, but if the resulting command is too weak to move the robot, it will stall just short of the desired heading. That requires low-level deadband compensation or a calibrated minimum rotation command—not a higher controller frequency.





# TLDR

Basically the commands given might be too small for the robot to start rotating which is known as deadband

The likely sequence is:

```txt
Nav2 sends a small velocity
→ command is below the motor’s deadband
→ wheels remain stationary
→ Nav2 increases or changes the command
→ command finally exceeds the threshold
→ robot suddenly moves
→ Nav2 reduces the command again
→ wheels stop
```

That creates the stop–go motion.

It also explains why rotation is especially poor: converting `angular.z = 0.5 rad/s` through the apparent WAVE ROVER geometry may produce an extremely small left/right motor command.

However, confirm it with one test before concluding:

```txt
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.15}, angular: {z: 0.0}}"
```

Try `x = 0.15`, `0.20`, `0.25`, and `0.30`.

- Constant commands are jerky → motor deadband, firmware, power, or motor control is responsible.
- Constant commands are smooth → the stop–go commands originate from Nav2, localization, or obstacle regulation.
- `/cmd_vel` remains steadily nonzero while the robot stops → strong confirmation of deadband.
- `/cmd_vel` itself repeatedly becomes zero → investigate Nav2/controller inputs instead.

So yes: **deadband is the leading hypothesis, not yet a confirmed diagnosis**. The test will identify the minimum reliable motor command that should later be used for deadband compensation.



## Another interesting thing, the motors are ran using an open-loop PWM

Honestly, the bigger problem may be the control interface rather than the motors themselves.

The WAVE ROVER appears to run these motors mostly through open-loop PWM:

```txt
requested velocity → estimated PWM → motor
```

### A better setup would have:

```txt
Nav2 velocity command
→ wheel-speed PID using encoders
→ PWM with deadband compensation
→ external IMU heading feedback
→ Nav2 closes the position/heading loop
```



The safest options are:

1. Obtain the matching ESP32 source from whoever provided the robot’s firmware.
2. Modify that source to enable wheel PID for the existing hardware profile.
3. Back up the ESP32 flash before uploading anything.
4. Compile and upload through the ESP32’s programming USB port.

The public source can be obtained from the [Waveshare General Driver repository](https://github.com/waveshareteam/ugv_base_general), but it would need your custom odometry fields ported into it.

### Required firmware change

Conceptually, replace the chassis-name check:

```txt
if (mainType == 3) {
    usePIDCompute = true;
    // encoder PID
} else {
    usePIDCompute = false;
    // open-loop PWM
}
```

with an encoder-capability setting:

```txt
if (HAS_WHEEL_ENCODERS) {
    usePIDCompute = true;
    // encoder PID
} else {
    usePIDCompute = false;
    // open-loop PWM
}
```

Configure the real:

```txt
WHEEL_D
ONE_CIRCLE_PULSES
TRACK_WIDTH
SET_MOTOR_DIR
```

The control should then be approximately:

```txt
left_target  = linear_x - angular_z * track_width / 2.0;
right_target = linear_x + angular_z * track_width / 2.0;

left_speed  = left_tick_change  * metres_per_tick / dt;
right_speed = right_tick_change * metres_per_tick / dt;

left_pwm  = left_feedforward  + left_PI_output;
right_pwm = right_feedforward + right_PI_output;
```

Deadband compensation should be added as feedforward rather than setting every small PID output to zero.

### Alternative using only your current ROS files

You can implement the wheel PI loops inside `waverover_driver.py`:

```txt
/cmd_vel
→ calculate target left/right wheel speed
→ compare with encA/encB measured speeds
→ PI controller
→ send T:11 direct left/right PWM
```

This avoids reflashing the ESP32, but it is less robust because Linux, Python and serial communication introduce timing jitter. A 50–100 Hz ESP32 control loop is preferable.

So the recommendation is:

- Do not switch blindly to `main:3`.
- Obtain or reconstruct the matching custom firmware.
- Enable encoder PID while preserving your actual hardware profile and custom JSON format.
- Use a ROS-side PI controller only as a reversible prototype if the firmware source cannot be obtained.



# TLDR OVERALL

Do these in order:

1. Test constant straight commands with Nav2 off: `0.15`, `0.20`, `0.25`, `0.30 m/s`.
2. Test constant rotation commands in both directions: `angular.z = ±0.5`, then increase carefully if the motors do not respond.
3. While testing, record:
   - `/cmd_vel`
   - `encA`, `encB`
   - `dist_L`, `dist_R`
   - External IMU yaw/gyro
   - Actual physical distance/angle
4. Determine:
   - Minimum command that reliably starts each motor side.
   - Whether both directions respond equally.
   - Whether encoders reflect actual wheel movement.
   - Whether wheels turn while the chassis slips.
   - Whether odometry changes when wheels are stationary.
5. Do not switch to `main:3` yet. Obtain the matching custom ESP32 firmware source and identify what `T:151` and `T:161` do.
6. If firmware is available, enable and tune encoder wheel-speed PI/PID for the existing hardware profile, with deadband compensation.
7. Configure the EKF to use:
   - Wheel encoders for forward velocity.
   - External IMU gyro for angular velocity.
   - Disable wheel-derived angular velocity initially.
8. Re-enable Nav2 `use_rotate_to_heading`, set the yaw tolerance to approximately `0.05 rad`, and test the `/spin` action for ±90°.

In short:

```txt
Verify motors and encoders
→ measure deadband
→ enable wheel-speed PID
→ fix EKF using external IMU
→ tune Nav2 turning
```
