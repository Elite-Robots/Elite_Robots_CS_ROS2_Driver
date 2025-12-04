cmake_minimum_required(VERSION 3.5)
project(eli_cs_robot_simulation_gz)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/gz_control.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
