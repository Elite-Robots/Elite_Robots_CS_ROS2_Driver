[英文](./README.md)

# Elite CS Robot Ros2 Driver

此驱动基于 `Elite_Robots_CS_SDK` 开发，支持一些关键的机器人功能，如：运动、设置数字IO。此外，ExternalControl EliCOs 是一个实现这些行为的重要的机器人插件。

## Requirements
- Elite_Robots_CS_SDK
- ROS2 - humble
- Ubuntu22.04


## 此仓库中的包
- `eli_common_interface` - 定义了一些共用的服务或消息接口。
- `eli_dashboard_interface` - 定义了dashboard节点会用到的消息接口。 
- `eli_cs_controllers` - Elite CS 机器人控制器的具体实现。
- `eli_cs_robot_calibration` - 从真实机器人上获取标定数据的工具。
- `eli_cs_robot_description` - Elite CS 机器人的描述文件和模型。
- `eli_cs_robot_driver` - 与机器人通信的硬件接口、驱动。
- `eli_cs_robot_simulation_gz` - Gazebo仿真的Elite CS机器人的配置文件和示例。
- `eli_cs_robot_moveit_config` - Elite CS机器人的MoveIt配置与示例。

## Getting Started
遵循下面的步骤以编译这个项目：
1. 使用源码编译安装`elite-cs-series-sdk`[（elite-cs-series-sdk项目仓库）](https://github.com/Elite-Robots/Elite_Robots_CS_SDK)。 

2. 使用下面的指令确保你的ros环境满足要求。并推荐使用下面的指令来解决依赖问题：
    ```bash
    sudo apt update
    rosdep install --ignore-src --rosdistro $ROS_DISTRO --from-paths src -y
    ```
3. **参考下面指令编译项目**
    ```bash
    # create a workspace
    mkdir -p elite_ros_ws/src
    # move source code to worksapce
    mv Elite_Robots_CS_ROS2_Driver  elite_ros_ws/src
    cd elite_ros_ws
    # compile
    colcon build
    ```
4. **安装此项目**
    ```bash
    . install/setup.bash
    ```

5. **使用下面指令来启动机器人的ros驱动. 更多详细内容可以参考 [usage](eli_cs_robot_driver/doc/Usage_CN.md) 文档**
    ```bash
    ros2 launch eli_cs_robot_driver elite_control.launch.py robot_ip:=<robot ip> local_ip:=<your pc ip> cs_type:=cs66
    ```

6. 如果不以[“headless mode”](doc/ROS2Interface_CN.md#headless_mode)启动: 在机器人的任务树中挂上 ExternalControl 节点并按下示教器上的 play 键。

> tips:
> - 如果编译失败了，可以查看[依赖表](doc/DependencyList.md)核对一下版本信息。 
> - 如果使用的是真机，请确保机器人的FB1和FB2都被接入网络中。
