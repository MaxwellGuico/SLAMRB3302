#!/usr/bin/env bash
# Kill leftover ROS 2 processes after a crash so the next launch starts clean.
#   ./kill.sh        SIGINT first, then SIGKILL anything still alive after 3 s
#   ./kill.sh -9     SIGKILL straight away
# Only touches processes owned by the current user.

PATTERN='ros2 (launch|run)|/opt/ros/|_ws/install/|rviz2|component_container|lifecycle_manager|controller_server|planner_server|bt_navigator|behavior_server|waypoint_follower|velocity_smoother|map_server|amcl|ekf_node|robot_state_publisher|rplidar|letter_waypoint'

list() { pgrep -u "$USER" -f -a -E "$PATTERN" | grep -v -E "pgrep|kill\.sh"; }

if ! list >/dev/null; then
  echo "No ROS 2 processes running."
else
  echo "Stopping:"; list | sed 's/^/  /'
  if [ "$1" != "-9" ]; then
    pkill -INT -u "$USER" -f -E "$PATTERN"
    for _ in 1 2 3 4 5 6; do list >/dev/null || break; sleep 0.5; done
  fi
  if list >/dev/null; then
    echo "Force killing what is left:"; list | sed 's/^/  /'
    pkill -KILL -u "$USER" -f -E "$PATTERN"
  fi
fi

# Clear the cached node/topic list so `ros2 node list` doesn't show ghosts.
command -v ros2 >/dev/null && ros2 daemon stop >/dev/null 2>&1
echo "Done."
