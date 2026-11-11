# 2_Simulation – Gazebo Simulation of the RB3302 Course

A Gazebo Harmonic copy of the 6 × 4 project course and the Wave Rover, used to
develop and test the navigation stack before running it on the real robot.
It publishes the **same ROS topics as the real hardware**, so the same Nav2
setup runs unchanged in either. Only `use_sim_time` differs.

Only `robot_ws/src` and `student_ws/src` (plus this README) are in git.
Build them locally; see the [main README](../README.md#simulation).

```text
robot_ws/src/
  rb3302_sim     - Gazebo world, ROS–Gazebo bridge, sensor_realism, smoke_test
  robot_model    - URDF with Gazebo sensors and diff-drive plugin
student_ws/src/
  planning_rpp   - Nav2 launch + params, maps, letter_waypoints mission script
```

---

## How it works

```text
 rb3302_course.sdf (walls, 3.6 m × 2.4 m, 0.60 m cells)
        │
        ▼
 Gazebo ── DiffDrive plugin ◄── /cmd_vel ◄───────────────────────────┐
   │  gpu_lidar (720 samples, 10 Hz) ──► /scan                       │
   │  IMU (200 Hz)       ──► /sim/imu/ideal  ─┐                      │
   │  wheel odometry     ──► /sim/odom/ideal ─┤                      │
   ▼                                           ▼                      │
 ros_gz_bridge (/clock, /scan, ...)      sensor_realism              │
                                         adds realistic errors       │
                                           │            │            │
                                       /odom/raw   /imu_data_ros     │
                                           └─────┬──────┘            │
                                                 ▼                   │
                       student_ws: EKF → AMCL → Nav2 (A* + Shim/RPP) ─┘
                                                 ▲
                              letter_waypoints A F K X
```

- **World (`worlds/rb3302_course.sdf`)**: the course walls built from the
  project brief's dimensions. The robot spawns in cell `S` at (0.30, 0.30),
  facing east.
- **`sensor_realism`**: Gazebo's sensors are perfect, and the real robot's are
  not. This node sits between them and Nav2 and deliberately degrades the
  data (see [Sim-to-real gap](#sim-to-real-gap)). The clean values stay
  available on `/sim/odom/ideal` and `/sim/imu/ideal` for comparison.
- **`smoke_test`**: checks that every sensor stream is alive, drives a short
  line and turn, and checks odometry, covariance and frame IDs.
- **Navigation**: the same `planning_rpp` stack as on the robot.
  - `nav2_params.yaml` is tuned for the ideal map.
  - `nav2_params_irl.yaml` mirrors the robot settings so they can be tried in
    sim first.
- **Maps** (`planning_rpp/maps/`):
  - `rb3302_course_ideal.yaml`: a perfect grid drawn from the brief.
  - `rb3302_map.yaml`: the real SLAM map from the robot.
  - `letter_waypoints --map-profile ideal|irl` picks the matching letter
    coordinates.

---

## Sim-to-real gap

A perfect simulator makes navigation look easier than it is. Turns are exact,
odometry never drifts, and the IMU has no bias. Tuning against that would
produce settings that fail on the robot. To narrow the gap, `sensor_realism`
injects the kinds of errors seen on the real Wave Rover
(`config/realistic_sensors.yaml`):

| Error modelled | Real cause | Value used |
| --- | --- | --- |
| Distance scale error | Wheel size not exactly nominal | +1.2 % |
| Turn under-reporting | Skid-steer wheels slip sideways in turns | 94 % of real turn, ±4.5 % per turn |
| Curving on straight runs | Left/right wheel mismatch | 0.012 rad per metre |
| Encoder quantisation and noise | Finite encoder resolution | 0.5 mm steps, 4 mm/√m noise |
| Gyro bias and drift | IMU bias that wanders over time | 0.010 rad/s bias + random walk |
| IMU mounted upside-down | Yesense installed with forward-right-down axes | yaw inverted, as on the robot |
| Late or missing messages | USB/serial transport | odometry 18 ± 5 ms, IMU 7 ± 2 ms, rare dropouts |

Runs are repeatable with `random_seed` (default 3302). Try several seeds so
the tuning is not relying on one lucky sample. Use `realism:=false` to compare
against ideal sensors.

### What the simulation still does not capture

Even with these errors, moving to the real robot exposed problems the
simulation could not show. Each needed its own fix (see the
[`robot_ws`](../3_IRL/rb3302_final_proj_code/robot_ws/README.md) and
[`student_ws`](../3_IRL/rb3302_final_proj_code/student_ws/README.md) READMEs):

| Not modelled in sim | What happened on the real robot |
| --- | --- |
| **Motor deadband / open-loop PWM** | Gazebo's DiffDrive executes any command exactly. The real motors ignore small commands, causing stop–go motion and very slow turns. It needed a ×3 angular gain in the driver. |
| **Firmware odometry bugs** | The real firmware reported the commanded turn rate as measured, and its distance counter never went backwards. The sim publishes correct, signed odometry. |
| **Sensor geometry errors** | The real URDF had the LiDAR offset sign wrong (−0.052 m instead of +0.052 m), causing a 5–10 cm stopping error. In sim, Gazebo and TF read the same URDF, so a wrong offset is self-consistent and invisible. |
| **Onboard computing limits** | On the robot, the controller fell from 20 Hz to ~1 Hz and the EKF from 50 Hz to ~17 Hz under load. A desktop running Gazebo does not hit this. |
| **Real map vs. ideal grid** | The SLAM map is cropped, offset and slightly compressed. Letter coordinates had to be re-sampled from the 24 physical cell centres. |
| **Hardware start-up issues** | The IMU sometimes failed to come online, and leftover nodes from a crashed run kept publishing. |
| **The environment** | Floor traction, battery level, other teams' robots and the TAs' obstacles (one full cell). |

As a result, the final settings diverged:

| Setting | Simulation | Real robot |
| --- | --- | --- |
| Cruise speed (`desired_linear_vel`) | 0.12 m/s | 0.30 m/s |
| Local inflation radius / cost scaling | 0.22 m / 10.0 | 0.40 m / 3.0 |
| Driver angular gain | none | ×3.0 |
| Final approach | Nav2 only | Nav2 + precision `Spin`/`DriveOnHeading` |

The simulation model also still differs from the real robot in two places.
Its URDF uses a 0.288 m wheel spacing (real: 12.5 cm) and the old −0.052 m
LiDAR offset (real: +0.052 m).

The simulation was most useful for logic: the behaviour tree, replanning
around new obstacles, the mission script and the Rotation Shim. Speeds,
turning and final accuracy had to be tuned on the robot.

---

## Challenges and how they were overcome

| # | Challenge | What we found | How it was overcome |
| --- | --- | --- | --- |
| 1 | **Mission failed at the first letter (action status 6)** | Goals asked for a fixed east-facing heading. Turning on the spot next to a wall was predicted as a collision. | `yaw_goal_tolerance: 3.20` (any heading). Scoring only checks position. |
| 2 | **Planner intermittently failed in narrow corridors** | Inflation 0.25 m × scaling 10.0 left only a near-lethal-cost channel through valid gaps. | Global inflation 0.20 m with scaling 15.0. Dijkstra was also tried, but the problem was the costmap, not the search, so A* was kept. |
| 3 | **Goal rejected right after launch** | The Nav2 action server is visible before its lifecycle node is active. | The mission script retries until a timeout. |
| 4 | **Robot kept yawing and never drove** | RPP's built-in `use_rotate_to_heading` could loop on rotation. | Left it off and added the **Rotation Shim controller** to handle large heading changes. |
| 5 | **180° transitions (e.g. `X → S → X → S`) failed** | With the goal behind the robot near a wall, RPP tried a forward U-turn. | The Rotation Shim turns in place first, then hands over to RPP. |
| 6 | **First obstacle test failed** | An obstacle blocked the only nearby opening, so there was no safe detour. | Correct, safe behaviour. A second test with a passable obstacle replanned and finished (0.58 m sideways detour). |
| 7 | **Blocked waypoints were silently skipped** | `stop_on_failure: false` moved on without telling anyone. | Set to `true` in sim. The robot version instead announces the skip, as the brief requires. |
| 8 | **Ideal sensors gave over-optimistic results** | Perfect odometry hid drift and turn-slip problems. | Added `sensor_realism` (see above). |

### Verified in simulation

- `A → F → K → X` with an obstacle present: 127 s.
- All 24 letters were visited across six missions, including repeated letters
  and `X → S → X → S`.
- Live obstacle with an alternate route: replanned and passed in both
  directions.
- Invalid letters and counts were rejected. A missing Nav2 server timed out
  cleanly.

---

## Usage

Build and run commands are in the [main README](../README.md#simulation). In
short, with both workspaces built and sourced:

```bash
ros2 launch rb3302_sim sim.launch.py                       # Gazebo
ros2 launch planning_rpp planning.launch.py use_sim_time:=true rviz:=true \
  map:=$PWD/student_ws/src/planning_rpp/maps/rb3302_course_ideal.yaml \
  set_initial_pose:=true initial_pose_x:=0.30 initial_pose_y:=0.30
ros2 run planning_rpp letter_waypoints A F K X --map-profile ideal \
  --ros-args -p use_sim_time:=true
```

Useful launch options for `sim.launch.py`:

| Option | Effect |
| --- | --- |
| `random_seed:=7319` | Different sample of sensor errors |
| `realism:=false` | Ideal sensors on the same topics |
| `gz_args:='-r -s -v 3'` | Headless (no Gazebo window) |
| `x:= y:= yaw:=` | Change spawn pose |

Sensor check: `ros2 run rb3302_sim smoke_test`.

Don't run `robot_init` alongside the simulation, because it publishes the
same topics from the real drivers.
