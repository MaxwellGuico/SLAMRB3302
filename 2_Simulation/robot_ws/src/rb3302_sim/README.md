# RB3302 Gazebo simulation

This package runs the RB3302 skid-steer model in the 6-by-4 project course. It
replaces the physical base, LiDAR, and IMU drivers while retaining their ROS
topic contract:

- `/cmd_vel` (`geometry_msgs/msg/Twist`)
- `/odom/raw` (`nav_msgs/msg/Odometry`)
- `/imu_data_ros` (`sensor_msgs/msg/Imu`)
- `/scan` (`sensor_msgs/msg/LaserScan`)
- `/joint_states` (`sensor_msgs/msg/JointState`)
- `/clock` (`rosgraph_msgs/msg/Clock`)

The public odometry and IMU topics are deliberately not Gazebo ground truth.
The `sensor_realism` node models wheel-size error, left/right mismatch,
skid-turn error, encoder quantization, IMU bias and random walk, white noise,
non-zero covariance, delivery latency/jitter, and rare dropped samples. Clean
reference measurements remain available as `/sim/odom/ideal` and
`/sim/imu/ideal` so a run can be diagnosed without weakening the sensor model.

The defaults in `config/realistic_sensors.yaml` use the 100 Hz base-feedback
and 200 Hz H30 IMU settings from the latest `irl` branch. The IMU covariance
also matches that branch's declared `0.10 rad`, `0.03 rad/s`, and `0.35 m/s^2`
uncertainty.

## Build

Generated workspace directories are not stored in Git. Build the simulation
locally after cloning:

```bash
REPO_ROOT="$(git rev-parse --show-toplevel)"
cd "$REPO_ROOT/2_Simulation/robot_ws"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select robot_model rb3302_sim
source install/setup.bash
```

## Run

Start Gazebo with its GUI:

```bash
ros2 launch rb3302_sim sim.launch.py
```

Runs are repeatable by default. Change the seed to sample different startup
biases, drift, skid error, jitter, and packet loss:

```bash
ros2 launch rb3302_sim sim.launch.py random_seed:=7319
```

For an A/B comparison, pass ideal measurements through the same public topics:

```bash
ros2 launch rb3302_sim sim.launch.py realism:=false
```

For a headless run:

```bash
ros2 launch rb3302_sim sim.launch.py gz_args:='-r -s -v 3'
```

The robot starts in landmark square `S`, at `(0.30, 0.30)`, facing east. The
spawn can be changed with the `x`, `y`, and `yaw` launch arguments.

## Verify

While the simulation is running, source the same overlay in a second terminal
and run:

```bash
source /opt/ros/jazzy/setup.bash
REPO_ROOT="$(git rev-parse --show-toplevel)"
source "$REPO_ROOT/2_Simulation/robot_ws/install/setup.bash"
ros2 run rb3302_sim smoke_test
```

The test waits for all simulated sensor streams, commands a short straight
motion and rotation, and checks odometry, uncertainty covariance, and frame
IDs.

## Tune the uncertainty

Edit `config/realistic_sensors.yaml`, rebuild `rb3302_sim`, and change only one
error family at a time. The most useful parameters for measured-course tuning
are:

- `odom_linear_scale`: straight-line distance scale.
- `odom_wheel_mismatch_rad_per_m`: systematic curvature on straight runs.
- `odom_angular_scale`: effective track-width/skid error during turns.
- `odom_turn_slip_stddev_fraction`: turn-to-turn variation from floor contact.
- `imu_gyro_bias_stddev` and `imu_gyro_bias_random_walk`: heading drift.
- latency, jitter, and dropout parameters: serial/USB transport behaviour.

Use the same `random_seed` while tuning so a parameter change is compared
against the same stochastic run. Once tuned, test several seeds to make sure
navigation is robust to more than one favourable error sample.

## Run the navigation stack

The current host first needs the packages declared by `planning_rpp`. From the
`2_Simulation` folder, build the navigation package once:

```bash
source /opt/ros/jazzy/setup.bash
source robot_ws/install/setup.bash
cd student_ws
colcon build --symlink-install --packages-select planning_rpp
cd ..
```

After building and sourcing `planning_rpp`, launch it in another terminal with
the map and Nav2 settings synced from the latest IRL branch:

```bash
REPO_ROOT="$(git rev-parse --show-toplevel)"
source /opt/ros/jazzy/setup.bash
source "$REPO_ROOT/2_Simulation/robot_ws/install/setup.bash"
source "$REPO_ROOT/2_Simulation/student_ws/install/setup.bash"
ros2 launch planning_rpp planning.launch.py \
  use_sim_time:=true \
  map:="$REPO_ROOT/2_Simulation/student_ws/src/planning_rpp/maps/rb3302_map.yaml" \
  params_file:="$REPO_ROOT/2_Simulation/student_ws/src/planning_rpp/config/nav2_params_irl.yaml" \
  rviz:=true \
  set_initial_pose:=false
```

The IRL launch intentionally leaves the navigation lifecycle inactive until
an initial pose is sent. In RViz, use **2D Pose Estimate** at the matching
location and heading; `initial_pose_nav_starter.py` then requests stationary
AMCL updates and activates Nav2 after `map -> base_link` becomes available.

Do not run `robot_init` at the same time; it starts the physical serial, LiDAR,
and IMU drivers and would duplicate the simulation topics.
