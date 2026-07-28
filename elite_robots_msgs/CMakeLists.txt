cmake_minimum_required(VERSION 3.8)
project(elite_robots_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)


rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "srv/GetTaskStatus.srv"
  "srv/GetRobotMode.srv"
  "srv/GetSafetyMode.srv"
  "srv/SetIO.srv"
  "srv/SetSpeedSliderFraction.srv"
  "srv/SetPayload.srv"
  "msg/RobotMode.msg"
  "msg/TaskStatus.msg"
  "msg/SafetyMode.msg"
  "msg/Analog.msg"
  "msg/IOState.msg"
  "msg/ToolData.msg"
  DEPENDENCIES
  geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
