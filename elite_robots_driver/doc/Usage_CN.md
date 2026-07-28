# Usage

## Launch files
在`elite_robots_driver`包中有一个启动文件来启动CS系列机器人的驱动程序：
- elite_control.launch.py - 启动了 [ros2_control](https://control.ros.org/humble/index.html) 节点，此节点包括： hardware interface、joint state broadcaster以及一个controller。 如果使用的是真机，此驱动程序同样会启动dashboard_client和primary_client。

使用 `ros2 launch elite_robots_driver elite_control.launch.py --show-args`指令能看到启动文件的参数以及解释。比较经常实用的参数以及解释如下:
- `cs_type` (mandatory) - 机器人类型（`cs63`、`cs66`、`cs66a`、`cs68`、`cs612`、`cs616`、`cs618f`、`cs620`、`cs625`、`cs520h`、`ls65`）。如果类型以 `h` 结尾（五轴），launch 会自动使用五轴 URDF 和控制器配置。
- `robot_ip` (mandatory) - 机器人FB1的IP（确保能连通）。
- `local_ip` (mandatory) - 外部控制器的IP（确保机器人能连通）。
- `use_fake_hardware` (default: false ) - 使用来自[ros2_control](https://control.ros.org/humble/index.html)的简单硬件模拟器。用于测试启动文件、描述等。见下面的解释。
- `fake_sensor_commands` (default: false ) - 允许为硬件模拟器设置传感器值。用于控制器的离线测试。
- `initial_joint_controller`(default: scaled_joint_trajectory_controller) - 用于控制机器人关节的控制器。可选的控制器有:
    - joint_trajectory_controller
    - scaled_joint_trajectory_controller

> 注意：以下的控制器也会被启动：joint_state_broadcaster、speed_scaling_state_broadcaster、force_torque_sensor_broadcaster、io_and_status_controller。

> 注意：使用`ros2 control list_controllers`指令可以查看ros2正在使用的控制器。因此必须安装ros2controlcli包，可以使用指令：`sudo apt-get install ros-${ROS_DISTRO}-ros2controlcli`安装。

## 测试驱动的指令

可以使用的CS机器人类型 - `cs63`, `cs66`, `cs66a`, `cs68`, `cs612`, `cs616`, `cs618f`, `cs620`, `cs625`, `cs520h`, `ls65`

> ***注意:***  
> **这是一个非常基础的测试，如果使用的是真实的机器人，在运行前请确保周围环境安全。**

### 1. 启动硬件，模拟器或者虚拟硬件

- 使用的是仿真或者真机:

```bash
ros2 launch elite_robots_driver elite_control.launch.py robot_ip:=xxx.xxx.xxx.xxx local_ip:=yyy.yyy.yyy.yyy cs_type:=zzzz
```
查看参数详细内容的指令 : `ros2 launch elite_robots_driver elite_control.launch.py --show-args`.

启动launch文件后，运行机器人的任务，机器人的任务中应当包含ExternalControl EliCOs插件节点。

- 使用虚拟硬件:
```bash
ros2 launch elite_robots_driver elite_control.launch.py robot_ip:=xxx.xxx.xxx.xxx local_ip:=yyy.yyy.yyy.yyy cs_type:=zzzz use_fake_hardware:=true
```

### 2. 向控制器发送指令

在运行其他指令之前，首先使用`ros2 control list_controllers`指令来检查控制器的状态（记得使用前文提到的命令安装“ros2controlcli”包）。

- 通过使用一个来自ros2_controllers_test_nodes的demo节点，发送一些目标点给“Joint Trajectory Controller”。指令如下（在另一个终端中运行）：
```bash
ros2 launch elite_robots_driver example_scaled_joint_trajectory_controller.launch.py
```

稍等片刻，机器人应该会移动。

设置“initial_joint_controller”参数以测试其他控制器, 当使用虚拟硬件时，下面是一个例子:
```bash
ros2 launch elite_robots_driver elite_control.launch.py robot_ip:=xxx.xxx.xxx.xxx local_ip:=yyy.yyy.yyy.yyy cs_type:=zzzz initial_joint_controller:=joint_trajectory_controller use_fake_hardware:=true
```

然后在另一个终端中运行demo节点：
```bash
ros2 launch elite_robots_driver example_joint_trajectory_controller.launch.py
```

稍等片刻，机器人应该会移动。

如果你想写一个自己的ros2节点来控制机器人运动, 下面是一个python的例子可以用作参考：

下面的示例适用于六轴机器人。对于 `cs520h`，只使用 `shoulder_pan_joint`、`shoulder_lift_joint`、`elbow_joint`、`wrist_1_joint`、`wrist_2_joint` 这五个关节，并且每个目标点只提供五个位置值。
```python
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        pos1 =  [0.785, -1.57, 0.785, 0.785, 0.785, 0.785]
        pos2 = [0.0, -1.57, 0.0, 0.0, 0.0, 0.0]
        pos3 = [0.0, -1.57, 0.0, 0.0, -0.785, 0.0]
        pos4 = [0.0, -1.57, 0.0, 0.0, 0.0, 0.0]
        self.point_list_ = [pos1, pos2, pos3, pos4]
        self.pos_count_ = 0
        # Create a publisher and publish data to topic /scaled_joint_trajectory_controller/joint_trajectory
        self.publisher_ = self.create_publisher(JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)
        # Every 8 second update target position
        self.timer = self.create_timer(8.0, self.publish_trajectory)

    def publish_trajectory(self):
        traj = JointTrajectory()
        # In cs.ros2_control.xacro the joint name
        traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        point = JointTrajectoryPoint()

        # Update target position
        if self.pos_count_ >= len(self.point_list_):
            self.pos_count_ = 0
        point.positions = self.point_list_[self.pos_count_]
        self.pos_count_ += 1

        point.time_from_start.sec = 3
        traj.points.append(point)

        self.publisher_.publish(traj)
        self.get_logger().info('Publishing trajectory command: ' + str(point.positions))

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```

### 3. 仅使用机器人的描述包
如果你只想测试机器人的描述包，你可以使用以下命令：
```bash
    ros2 launch elite_robots_description view_cs.launch.py cs_type:=zzzz
```

## 使用MoveIt!
[MoveIt!](https://moveit.ros.org/) 在此驱动中已经支持。  
要使用示例MoveIt-setup测试驱动程序，首先使用以下命令启动驱动程序：
```bash
ros2 launch elite_robots_driver elite_control.launch.py robot_ip:=xxx.xxx.xxx.xxx local_ip:=yyy.yyy.yyy.yyy cs_type:=zzzz launch_rviz:=false
```

接着启动MoveIt!节点:
```bash
ros2 launch elite_robots_moveit_config elite_moveit.launch.py cs_type:=zzzz launch_rviz:=true
```

现在你应该能够在rviz2中使用MoveIt插件来规划和执行机器人的轨迹，如[这里](https://moveit.picknik.ai/main/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html)所述。

对于 `cs520h`，MoveIt launch 会自动使用五轴 SRDF、关节限制和控制器列表。


## 机器人坐标系

尽管大多数tf坐标系来自URDF并由robot_state_publisher发布，但有几点需要注意：
- 基坐标系是机器人控制器视角下的机器人基座坐标系。
- tool0_controller是由机器人控制器发布的工具坐标系。如果在教示器（Teach Pendant，TP）上没有配置额外的工具，这应该与tool0相等，因为URDF使用了特定机器人的校准。如果在TP上配置了工具，则额外的变换会显示在base -> tool0之间。

## 用户脚本
此驱动程序包含一个名为script_sender的节点，可用于直接将脚本片段发送到机器人，当机器人处于远程控制模式时。

它在驱动程序的启动文件中默认启动。要使用它，只需发布一条消息即可：
```bash
# Display a popup box on the teach pendant.
ros2 topic pub /script_sender/script_command std_msgs/msg/String '{data: popup("hello")}' --once
```
请注意，在该接口上运行脚本（即将脚本代码发布到该接口）会停止机器人上正在运行的任何程序。因此，由驱动程序启动的运动解释程序会被停止，并且需要重新启动。如果使用headless mode，需要调用resend_external_script服务来重新发送控制脚本；如果使用有ExternalControl节点的任务则在教示器上按下播放按钮，以重新启动ExternalControl任务。

## 多行脚本

当您想定义多行脚本时，请确保检查您的消息中换行符是否正确解释。驱动程序会按原样打印发送到机器人脚本的内容。当从命令行发送多行脚本时，您可以在每个语句之间使用空行：
```bash
ros2 topic pub --once /script_sender/script_command std_msgs/msg/String '{data:
"def my_prog():  

    \tset_standard_digital_out(1, True)  

    \tmovej([0, -1.57, 0, -1.57, 0, 0], a=1.2, v=0.25, r=0)  

    \ttextmsg(\"motion finished\")  

end"}'
```

## 不会中断正在运行任务的脚本

为了防止中断主脚本，您可以将某些命令作为辅脚本发送（sec脚本）。
```bash
ros2 topic pub --once /script_sender/script_command std_msgs/msg/String '{data:
"sec my_prog():  

    \tpopup(\"This is a hello message\")

end"}'
```
