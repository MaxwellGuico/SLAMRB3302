cmake_minimum_required(VERSION 3.8)
project(planning_rpp)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS 
    scripts/initial_pose_nav_starter.py
    scripts/yaw_diagnostic.py
    scripts/letter_waypoint.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
