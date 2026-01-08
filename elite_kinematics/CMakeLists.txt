cmake_minimum_required(VERSION 3.8)
project(elite_kinematics)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(orocos_kdl REQUIRED)

# 构建共享库
add_library(kinematics_example1 SHARED
  src/kinematics_example1.cpp
)

# 查找头文件
target_include_directories(kinematics_example1 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(kinematics_example1
  pluginlib
  orocos_kdl
)

# 导出 pluginlib 插件描述
pluginlib_export_plugin_description_file(
  elite_kinematics
  kinematics_plugin.xml
)

install(TARGETS kinematics_example1
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


install(DIRECTORY include/
  DESTINATION include
)

install(FILES kinematics_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(kinematics_example1)
ament_export_dependencies(pluginlib orocos_kdl)

ament_package()
