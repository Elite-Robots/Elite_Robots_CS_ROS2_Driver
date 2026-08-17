# elite_robots_controllers

此包包含了专门为Elite CS机器人系列设计的`ros2_control`控制器和硬件接口。目前包含以下内容：

* **speed_scaling_interface**，用于读取当前速度缩放值并传递给控制器。
* **scaled_joint_command_interface**，结合速度缩放值提供对关节值和命令的访问。
* **speed_scaling_state_controller**，将机器人报告的当前执行速度发布到主题接口。值为介于0到1之间的浮动值。
* **scaled_joint_trajectory_controller**，与 *joint_trajectory_controller* 类似，但它使用机器人报告的速度缩放值来减少轨迹进度。

更多详细信息请参见 [doc/index_cn.md](doc/index_cn.md)
