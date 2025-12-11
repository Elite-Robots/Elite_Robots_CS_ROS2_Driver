cmake_minimum_required(VERSION 3.8)
project(elite_moveit_kinematics)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(elite_kinematics REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/elite_moveit_kinematics.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  pluginlib
  rclcpp
  geometry_msgs
  tf2
  tf2_geometry_msgs
  moveit_core
  moveit_msgs
  kdl_parser
  elite_kinematics
)

# MoveIt looks for kinematics plugins under the "moveit_core" resource namespace.
pluginlib_export_plugin_description_file(moveit_core kinematics_plugin_description.xml)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES kinematics_plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
