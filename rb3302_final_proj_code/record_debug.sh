#!/usr/bin/env bash
set -e

# Usage: ./record_debug.sh 01
# Output: /home/ubuntu/rb3302_final_proj_code/rosbags/01-DD-MM-YY
if [[ $# -ne 1 || ! "$1" =~ ^[0-9]{1,2}$ ]]; then
  echo "Usage: $0 <run number 01-99>" >&2
  exit 1
fi

run_number=$((10#$1))
if (( run_number < 1 )); then
  echo "Run number must be between 01 and 99." >&2
  exit 1
fi

printf -v run_id '%02d' "$run_number"
bag_root="/home/ubuntu/rb3302_final_proj_code/rosbags"
bag_dir="$bag_root/${run_id}-$(date +%d-%m-%y)"

if [[ -e "$bag_dir" ]]; then
  echo "Recording already exists: $bag_dir" >&2
  echo "Choose a different run number." >&2
  exit 1
fi

source /opt/ros/jazzy/setup.bash
mkdir -p "$bag_root"

echo "Recording to: $bag_dir"
echo "Press Ctrl+C after the final waypoint to stop and save."

exec ros2 bag record -o "$bag_dir" --include-hidden-topics --topics \
  /tf \
  /tf_static \
  /odom \
  /odometry/filtered \
  /amcl_pose \
  /plan \
  /cmd_vel \
  /cmd_vel_smoothed \
  /navigate_to_pose/_action/feedback \
  /navigate_to_pose/_action/status \
  /rosout
