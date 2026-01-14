[中文](./README_CN.md)

# Elite CS Robot Ros2 Driver

This driver is developed on top of `Elite_Robots_CS_SDK` and support some key cobot functionalities like: motion, set digital io. In addition the ExternalControl EliCOs makes it possible to include ROS2 behaviors in the robot program.

## Requirements
- Elite_Robots_CS_SDK
- ROS2 - humble
- Ubuntu22.04


## Packages in the Repository
- `eli_common_interface` - some common service or message interface.
- `eli_dashboard_interface` - package defining messages used by dashboard node.
- `eli_cs_controllers` - implementations of controllers specific for Elite CS robots.
- `eli_cs_robot_calibration` - tool for extracting calibration information from a real robot.
- `eli_cs_robot_description` - description files and meshes for Elite CS Robots manipulators. 
- `eli_cs_robot_driver` - driver / hardware interface for communication with Elite CS robots.
- `eli_cs_robot_simulation_gz` - Example files and configurations for Gazebo simulation of Elite CS Robots' manipulators.
- `eli_cs_robot_moveit_config` - example MoveIt configuration for Elite CS robots.

## Getting Started
For getting started, you'll basically need follow steps:
1. Install the `elite-cs-series-sdk` from [source](https://github.com/Elite-Robots/Elite_Robots_CS_SDK).

2. Ensure your ros environment. And recommended to run the following command to resolve the dependency issue:
    ```bash
    sudo apt update
    rosdep install --ignore-src --rosdistro $ROS_DISTRO --from-paths src -y
    ```
3. **Compile driver**
    ```bash
    # create a workspace
    mkdir -p elite_ros_ws/src
    # move source code to worksapce
    mv Elite_Robots_CS_ROS2_Driver  elite_ros_ws/src
    cd elite_ros_ws
    # compile
    colcon build
    ```
4. **Install**
    ```bash
    . install/setup.bash
    ```

5. **Start the driver. See the [usage](eli_cs_robot_driver/doc/Usage.md) documentation for details**
    ```bash
    ros2 launch eli_cs_robot_driver elite_control.launch.py robot_ip:=<robot ip> local_ip:=<your pc ip> cs_type:=cs66
    ```

6. Unless started in [headless mode](doc/ROS2Interface.md#headless_mode): Run the task which contain ExternalControl node by pressing play on the teach pendant.

> tips:
> - If the compilation fails, you can check that the version of the dependency package matches. [dependency list](doc/DependencyList.md)
> - If use real robot, ensure your robot FB1 and FB2 connected to network.
