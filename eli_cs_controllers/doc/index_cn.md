# eli_cs_controllers

该包包含了专门为Elite CS机器人系列设计的`ros2_control`控制器和硬件接口。目前包含以下内容：

* **speed_scaling_interface** 用于读取当前速度缩放值并传递给控制器。
* **scaled_joint_command_interface**，结合速度缩放值提供对关节值和命令的访问。
* **speed_scaling_state_controller**，将机器人报告的当前执行速度发布到主题接口。值为介于0到1之间的浮动值。
* **scaled_joint_trajectory_controller**，与 *joint_trajectory_controller* 类似，但它使用机器人报告的速度缩放值来减少轨迹的执行进度。

## 关于此包

该包包含了在默认的 `ros2_control` 集合中不可用的控制器。它们是为了支持Elite CS机器人系列提供的更多功能而创建的。

## 控制器描述

该包提供了几个特定的控制器，接下来的章节将对这些控制器进行详细说明。

### eli_cs_controllers/speed_scaling_state_broadcaster

此控制器发布机器人报告的当前实际执行速度。值为介于0到1之间的浮动值。

### position_controllers/ScaledJointTrajectoryController 和 velocity_controllers/ScaledJointTrajectoryController

这些控制器的工作方式类似于知名的 [joint_trajectory_controller](https://control.ros.org/master/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html)。

然而，它们被扩展为专门处理机器人的执行速度。因为默认的 `joint_trajectory_controller` 会根据配置的时间约束来插值轨迹（即：总是假设机器人支持最大速度和加速度），这可能由于多种原因导致显著的路径偏差：

* 机器人的速度滑块可能没有设置为100%，因此从ROS发送的运动命令会被缩小，导致执行速度变慢。
* 机器人可能会根据配置的安全限制缩小运动范围，导致比预期更慢的运动，因此可能无法在控制周期内到达目标。
* 运动可能根本无法执行，例如因为机器人被停止或处于保护停止状态。
* 发送到机器人的运动命令可能无法被解释，例如因为机器人控制器上没有运行 [`ExternalControl`]() 程序节点。
* 解释运动命令的程序可能被暂停。

上述所有情况都由缩放轨迹版本解决。轨迹执行可以透明地通过教示器上的速度滑块进行缩放，而不会导致额外的路径偏差。暂停程序或按下停止按钮会导致 `speed_scaling` 为0，这意味着轨迹不会继续，直到程序继续运行。通过这种方式，轨迹执行可以显式地暂停和继续。

### freedrive_controller

此控制器为启动、停止freedrive的控制器。使用时，需要先把位置控制器和速度控制器置为inactive后再启动此控制器。

此控制器订阅了`~/enable_freedrive`的topic，数据类型为`[std_msgs/msg/Bool]`。以命令行举例，当此控制器被激活后，使用下面指令可以开始freedrive模式：
```bash
ros2 topic pub --rate 2 /freedrive_controller/enable_freedrive std_msgs/msg/Bool "{data: true}"
```
同理下面的指令就是结束freedrive模式
```bash
ros2 topic pub --rate 2 /freedrive_controller/enable_freedrive std_msgs/msg/Bool "{data: false}"
```

topic的发布频率务必大于1HZ，否则将结束freedrive模式。
