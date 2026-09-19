# RB3302 Planning and Navigation

This repository contains the RB3302 robot packages, an idealized Gazebo
simulation of the project course, and a Nav2 stack using AMCL, NavFn, and the
Regulated Pure Pursuit controller.

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic / ROS-Gazebo integration

Install the main binary dependencies:

```bash
sudo apt update
sudo apt install \
  ros-jazzy-ros-gz \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-rviz-plugins \
  ros-jazzy-robot-localization
```

## Build

From the repository root:

```bash
source /opt/ros/jazzy/setup.bash

cd robot_ws
colcon build --symlink-install --packages-select robot_model rb3302_sim
cd ../student_ws
colcon build --symlink-install --packages-select planning_rpp
cd ..
```

The generated `build/`, `install/`, and `log/` directories are intentionally
not stored in Git. Each teammate should build them locally.

## Run the simulation

Terminal 1, from the repository root:

```bash
source /opt/ros/jazzy/setup.bash
source robot_ws/install/setup.bash
ros2 launch rb3302_sim sim.launch.py
```

Terminal 2, from the repository root:

```bash
source /opt/ros/jazzy/setup.bash
source robot_ws/install/setup.bash
source student_ws/install/setup.bash

ros2 launch planning_rpp planning.launch.py \
  use_sim_time:=true \
  map:="$PWD/student_ws/maps/rb3302_course_ideal.yaml" \
  set_initial_pose:=true \
  initial_pose_x:=0.30 \
  initial_pose_y:=0.30 \
  initial_pose_yaw:=0.0
```

The robot starts at landmark `S`, at `(0.30, 0.30)`, facing east. Use RViz's
Nav2 Goal tool to send a goal. Do not launch `robot_init` during simulation,
because it starts the physical robot drivers on the same topics.

## Smoke test

With Gazebo running, use a third terminal from the repository root:

```bash
source /opt/ros/jazzy/setup.bash
source robot_ws/install/setup.bash
ros2 run rb3302_sim smoke_test
```

The smoke test checks the simulated sensor streams, commands a short motion,
and verifies odometry and frame IDs.

More details are available in
[`robot_ws/src/rb3302_sim/README.md`](robot_ws/src/rb3302_sim/README.md) and
[`student_ws/README.md`](student_ws/README.md).
