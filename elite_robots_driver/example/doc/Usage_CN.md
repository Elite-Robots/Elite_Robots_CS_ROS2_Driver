# Elite CS Driver 示例说明

本目录包含一些通过 ROS 2 驱动发送运动命令的小示例，方便快速验证机器人控制。

## 运行示例前

先启动 driver。请根据实际情况替换 `cs63a` 和 `172.16.100.82`：

```bash
ros2 launch elite_robots_driver elite_control.launch.py \
  cs_type:=cs63a \
  robot_ip:=172.16.100.82 \
  launch_rviz:=false \
  headless_mode:=true
```

如果要运行 MoveIt 位姿示例，还需要启动 MoveIt：

```bash
ros2 launch elite_robots_moveit_config elite_moveit.launch.py \
  cs_type:=cs63a \
  launch_rviz:=true
```

发送运动前建议确认控制器状态：

```bash
ros2 control list_controllers
```

## MoveIt 笛卡尔位姿示例

功能：通过 MoveIt 规划到指定笛卡尔位姿，并可选择是否执行规划出的轨迹。

位姿格式：

```text
x y z rx ry rz
```

`x y z` 单位是米，`rx ry rz` 是 RPY 欧拉角，单位是弧度。

C++：

```bash
ros2 run elite_robots_driver example_moveit_pose_goal --ros-args \
  -p target_pose:="[0.4,0.0,0.523,0.3,-1.0,0.5]" \
  -p execute:=true
```

Python：

```bash
ros2 run elite_robots_driver example_moveit_pose_goal_py \
  --target-pose "[0.4,0.0,0.523,0.3,-1.0,0.5]" \
  --execute
```

只规划不控制机器人运动：

```bash
ros2 run elite_robots_driver example_moveit_pose_goal --ros-args \
  -p target_pose:="[0.4,0.0,0.523,0.3,-1.0,0.5]"
```

## 安全关节运动示例

功能：通过 `FollowJointTrajectory` 发送一个关节目标。示例会读取当前 `/joint_states`，根据 `max_vel` 自动计算运动时间，然后发送单点轨迹。

目标格式：

```text
shoulder_pan shoulder_lift elbow wrist_1 wrist_2 wrist_3
```

所有数值都是关节角，单位是弧度。

C++：

```bash
ros2 run elite_robots_driver example_safe_joint_move --ros-args \
  -p target:="[0.0,-1.2,0.0,-1.57,0.0,0.0]" \
  -p max_vel:=0.5 \
  -p min_time:=2.0 \
  -p controller:="scaled_joint_trajectory_controller"
```

Python：

```bash
ros2 run elite_robots_driver example_safe_joint_move_py --ros-args \
  -p target:="[0.0,-1.2,0.0,-1.57,0.0,0.0]" \
  -p max_vel:=0.5 \
  -p min_time:=2.0 \
  -p controller:="scaled_joint_trajectory_controller"
```

也可以使用 launch 启动：

```bash
ros2 launch elite_robots_driver example_safe_joint_move.launch.py \
  cpp_py:=cpp \
  target:="[0.0,-1.2,0.0,-1.57,0.0,0.0]" \
  max_vel:=0.5 \
  min_time:=2.0 \
  controller:=scaled_joint_trajectory_controller
```

设置 `cpp_py:=py` 可以运行 Python 版本。

## 控制器测试发布示例

这些示例使用 `ros2_controllers_test_nodes` 发布测试命令。它们会直接向控制器发送运动命令，使用前请确认目标点安全。

普通关节轨迹控制器：

```bash
ros2 launch elite_robots_driver example_joint_trajectory_controller.launch.py
```

带速度缩放的关节轨迹控制器：

```bash
ros2 launch elite_robots_driver example_scaled_joint_trajectory_controller.launch.py
```

对于 `cs66a`，可以指定模型名来使用对应的测试目标配置：

```bash
ros2 launch elite_robots_driver example_scaled_joint_trajectory_controller.launch.py \
  robot_model:=cs66a
```

速度控制器：

```bash
ros2 launch elite_robots_driver example_forward_velocity_controller.launch.py
```

## 常用反馈命令

```bash
ros2 topic echo /joint_states --once
ros2 topic echo /tcp_pose_broadcaster/pose --once
ros2 action list | grep follow_joint_trajectory
```
