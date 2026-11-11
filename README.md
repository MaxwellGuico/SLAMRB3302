# RB3302 – SLAM, Planning and Navigation

RB3302 final project: a Waverover robot that drives to four letter points
(`A`–`X`) in order, using AMCL, NavFn A* and Regulated Pure Pursuit (Nav2).

```text
rb3302/
├── 1_Project_Logs/   daily project logs (Project Log - YYYY-MM-DD.md)
├── 2_Simulation/     Gazebo simulation (only robot_ws/src and student_ws/src are in git)
└── 3_IRL/            code that runs on the physical robot
```

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic (simulation only)

```bash
sudo apt update
sudo apt install \
  ros-jazzy-ros-gz \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-rviz-plugins \
  ros-jazzy-robot-localization \
  ros-jazzy-cartographer-ros \
  python3-colcon-common-extensions \
  python3-rosdep
```

Any missing dependencies can be installed per workspace with:

```bash
sudo rosdep init   # first time only
rosdep update
rosdep install --from-paths src --ignore-src -r -y   # run inside a workspace
```

## Get the code

```bash
git clone https://github.com/MaxwellGuico/SLAMRB3302.git ~/rb3302
cd ~/rb3302
```

`build/`, `install/` and `log/` folders are not stored in git. Build them
locally with the steps below.

---

## Simulation

### Build

```bash
cd ~/rb3302/2_Simulation
source /opt/ros/jazzy/setup.bash

cd robot_ws && colcon build && cd ..
source robot_ws/install/setup.bash
cd student_ws && colcon build && cd ..
```

### Run

Every terminal starts with:

```bash
cd ~/rb3302/2_Simulation
source /opt/ros/jazzy/setup.bash
source robot_ws/install/setup.bash
source student_ws/install/setup.bash
```

**Terminal 1 – Gazebo:**

```bash
ros2 launch rb3302_sim sim.launch.py
```

**Terminal 2 – Nav2 + RViz:**

```bash
ros2 launch planning_rpp planning.launch.py \
  use_sim_time:=true \
  map:=$PWD/student_ws/src/planning_rpp/maps/rb3302_map.yaml \
  params_file:=$PWD/student_ws/src/planning_rpp/config/nav2_params_irl.yaml \
  rviz:=true \
  set_initial_pose:=false
```

Set the initial pose in RViz with **2D Pose Estimate**.

**Terminal 3 – four-letter mission:**

```bash
ros2 run planning_rpp letter_waypoints A F K X --ros-args -p use_sim_time:=true
```

**Optional – log viewer:**

```bash
ros2 run rqt_console rqt_console
```

Other maps are in `student_ws/src/planning_rpp/maps/`. For example,
`rb3302_course_ideal.yaml` is an idealised course.

---

## Real robot (IRL)

The robot expects the code at `/home/ubuntu/rb3302_final_proj_code`.

### Copy to the robot

```bash
scp -r ~/rb3302/3_IRL/rb3302_final_proj_code ubuntu@<robot-ip>:~/
```

### Build (on the robot)

```bash
source /opt/ros/jazzy/setup.bash
cd ~/rb3302_final_proj_code/robot_ws && colcon build --symlink-install
source install/setup.bash
cd ~/rb3302_final_proj_code/student_ws && colcon build --symlink-install
```

### Run (on the robot)

```bash
cd ~/rb3302_final_proj_code
./robot_init.sh   # Terminal 1: lidar, IMU, base driver, robot model
./rpp.sh          # Terminal 2: Nav2 with Regulated Pure Pursuit
./nav_rviz.sh     # Optional: RViz with the navigation layout
./kill.sh         # Clean up leftover ROS processes after a crash
```

Mapping with Cartographer, saving maps, A* planning and the letter-waypoint
mission scripts are covered in
[`3_IRL/rb3302_final_proj_code/student_ws/README.md`](3_IRL/rb3302_final_proj_code/student_ws/README.md).

Recorded debug runs (`*_debug*/`, `.mcap` bags) and `run_logs/` are kept in
`3_IRL/rb3302_final_proj_code/` for reference.
