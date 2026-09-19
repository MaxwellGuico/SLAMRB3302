# RB3302 Gazebo simulation

This package runs the RB3302 differential-drive model in an idealized version
of the 6-by-4 project course. It replaces the physical base, LiDAR, and IMU
drivers while retaining their ROS topic contract:

- `/cmd_vel` (`geometry_msgs/msg/Twist`)
- `/odom/raw` (`nav_msgs/msg/Odometry`)
- `/imu_data_ros` (`sensor_msgs/msg/Imu`)
- `/scan` (`sensor_msgs/msg/LaserScan`)
- `/joint_states` (`sensor_msgs/msg/JointState`)
- `/clock` (`rosgraph_msgs/msg/Clock`)

## Build

Generated workspace directories are not stored in Git. Build the simulation
locally after cloning:

```bash
REPO_ROOT="$(git rev-parse --show-toplevel)"
cd "$REPO_ROOT/robot_ws"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select robot_model rb3302_sim
source install/setup.bash
```

## Run

Start Gazebo with its GUI:

```bash
ros2 launch rb3302_sim sim.launch.py
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
source "$REPO_ROOT/robot_ws/install/setup.bash"
ros2 run rb3302_sim smoke_test
```

The test waits for all simulated sensor streams, commands a short straight
motion and rotation, and checks odometry and frame IDs.

## Run the navigation stack

The current host first needs the packages declared by `planning_rpp`. From the
repository root, build the navigation package once:

```bash
source /opt/ros/jazzy/setup.bash
source robot_ws/install/setup.bash
cd student_ws
colcon build --symlink-install --packages-select planning_rpp
cd ..
```

After building and sourcing `planning_rpp`, launch it in another terminal:

```bash
REPO_ROOT="$(git rev-parse --show-toplevel)"
source /opt/ros/jazzy/setup.bash
source "$REPO_ROOT/robot_ws/install/setup.bash"
source "$REPO_ROOT/student_ws/install/setup.bash"
ros2 launch planning_rpp planning.launch.py \
  use_sim_time:=true \
  map:="$REPO_ROOT/student_ws/maps/rb3302_course_ideal.yaml" \
  set_initial_pose:=true \
  initial_pose_x:=0.30 \
  initial_pose_y:=0.30 \
  initial_pose_yaw:=0.0
```

Do not run `robot_init` at the same time; it starts the physical serial, LiDAR,
and IMU drivers and would duplicate the simulation topics.
