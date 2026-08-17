# Elite CS Driver Examples

This folder contains small examples for sending motion commands through the ROS 2 driver.

## Before Running Examples

Start the driver first. Replace `cs63a` and `172.16.100.82` with your robot type and IP address:

```bash
ros2 launch elite_robots_driver elite_control.launch.py \
  cs_type:=cs63a \
  robot_ip:=172.16.100.82 \
  launch_rviz:=false \
  headless_mode:=true
```

For MoveIt pose examples, also start MoveIt:

```bash
ros2 launch elite_robots_moveit_config elite_moveit.launch.py \
  cs_type:=cs63a \
  launch_rviz:=true
```

Check the active controller before sending motion:

```bash
ros2 control list_controllers
```

## MoveIt Cartesian Pose Goal

Function: plans to a Cartesian pose with MoveIt and optionally executes the planned trajectory.

Pose format:

```text
x y z rx ry rz
```

`x y z` are in meters. `rx ry rz` are RPY angles in radians.

C++:

```bash
ros2 run elite_robots_driver example_moveit_pose_goal --ros-args \
  -p target_pose:="[0.4,0.0,0.523,0.3,-1.0,0.5]" \
  -p execute:=true
```

Python:

```bash
ros2 run elite_robots_driver example_moveit_pose_goal_py \
  --target-pose "[0.4,0.0,0.523,0.3,-1.0,0.5]" \
  --execute
```

Use planning only without moving the robot:

```bash
ros2 run elite_robots_driver example_moveit_pose_goal --ros-args \
  -p target_pose:="[0.4,0.0,0.523,0.3,-1.0,0.5]"
```

## Safe Joint Move

Function: sends one joint target through `FollowJointTrajectory`. The example reads the current `/joint_states`, computes a duration from `max_vel`, and sends a single-point trajectory.

Target format:

```text
shoulder_pan shoulder_lift elbow wrist_1 wrist_2 wrist_3
```

All values are joint positions in radians.

C++:

```bash
ros2 run elite_robots_driver example_safe_joint_move --ros-args \
  -p target:="[0.0,-1.2,0.0,-1.57,0.0,0.0]" \
  -p max_vel:=0.5 \
  -p min_time:=2.0 \
  -p controller:="scaled_joint_trajectory_controller"
```

Python:

```bash
ros2 run elite_robots_driver example_safe_joint_move_py --ros-args \
  -p target:="[0.0,-1.2,0.0,-1.57,0.0,0.0]" \
  -p max_vel:=0.5 \
  -p min_time:=2.0 \
  -p controller:="scaled_joint_trajectory_controller"
```

Launch wrapper:

```bash
ros2 launch elite_robots_driver example_safe_joint_move.launch.py \
  cpp_py:=cpp \
  target:="[0.0,-1.2,0.0,-1.57,0.0,0.0]" \
  max_vel:=0.5 \
  min_time:=2.0 \
  controller:=scaled_joint_trajectory_controller
```

Use `cpp_py:=py` to run the Python version.

## Controller Test Publishers

These examples use `ros2_controllers_test_nodes` to publish test commands. Use them carefully because they send motion commands directly to the active controller.

Joint trajectory controller:

```bash
ros2 launch elite_robots_driver example_joint_trajectory_controller.launch.py
```

Scaled joint trajectory controller:

```bash
ros2 launch elite_robots_driver example_scaled_joint_trajectory_controller.launch.py
```

For `cs66a`, pass the model name to use the matching test goal config:

```bash
ros2 launch elite_robots_driver example_scaled_joint_trajectory_controller.launch.py \
  robot_model:=cs66a
```

Forward velocity controller:

```bash
ros2 launch elite_robots_driver example_forward_velocity_controller.launch.py
```

## Useful Feedback Topics

```bash
ros2 topic echo /joint_states --once
ros2 topic echo /tcp_pose_broadcaster/pose --once
ros2 action list | grep follow_joint_trajectory
```
