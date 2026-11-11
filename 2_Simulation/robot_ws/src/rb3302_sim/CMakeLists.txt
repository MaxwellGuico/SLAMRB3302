cmake_minimum_required(VERSION 3.8)
project(rb3302_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

# Keep conventional ROS executable names while retaining descriptive source
# filenames in the repository.
install(PROGRAMS scripts/smoke_test.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME smoke_test
)
install(PROGRAMS scripts/sensor_realism.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sensor_realism
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
