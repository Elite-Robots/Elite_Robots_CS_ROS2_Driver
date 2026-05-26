cmake_minimum_required(VERSION 3.5)
project(eli_cs_robot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes rviz urdf urdf_5f test
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(cs_urdf_xacro test/test_cs_urdf_xacro.py)
  ament_add_pytest_test(view_cs_launch test/test_view_cs_launch.py)
endif()

ament_package()
