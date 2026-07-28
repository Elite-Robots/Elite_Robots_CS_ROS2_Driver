cmake_minimum_required(VERSION 3.5)
project(elite_robots_calibration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(elite-cs-series-sdk 1.1.0 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

set(YAML_CPP_INCLUDE_DIRS ${YAML_CPP_INCLUDE_DIR})

add_library(calibration
  src/calibration.cpp
  src/calibration_consumer.cpp
)
target_include_directories(
  calibration
  PUBLIC
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)
target_link_libraries(
  calibration
  elite-cs-series-sdk
  ${YAML_CPP_LIBRARIES}
)
ament_target_dependencies(calibration
  rclcpp
)

add_executable(calibration_correction
  src/calibration_correction.cpp
)
target_include_directories(calibration_correction
  PRIVATE
    include
)
target_link_libraries(calibration_correction
  calibration
)

install(TARGETS calibration_correction
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
        DESTINATION include
)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
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

ament_export_include_directories(
        include
)
ament_export_dependencies(
        rclcpp
)

ament_package()
