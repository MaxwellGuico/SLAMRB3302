#!/usr/bin/env bash
# Open RViz with the navigation config (map, costmaps, plan, robot, 2D Goal Pose).
#   ./nav_rviz.sh             uses planning_rpp/rviz/navigation.rviz
#   ./nav_rviz.sh my.rviz     uses another config file
# Reads the config straight from the source tree, so no colcon build is needed.

source /opt/ros/jazzy/setup.bash

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG="${1:-$HERE/student_ws/src/planning_rpp/rviz/navigation.rviz}"

if [ ! -f "$CONFIG" ]; then
  echo "RViz config not found: $CONFIG" >&2
  exit 1
fi

echo "Opening RViz with $CONFIG"
exec ros2 run rviz2 rviz2 -d "$CONFIG" --ros-args -p use_sim_time:=false
