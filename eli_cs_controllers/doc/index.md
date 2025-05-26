# eli_cs_controllers

This package contains controllers and hardware interface for `ros2_control` that are special to the Elite CS robot family. Currently this contains:


* A **speed_scaling_interface** to read the value of the current speed scaling into controllers.
* A **scaled_joint_command_interface** that provides access to joint values and commands in combination with the speed scaling value.
* A **speed_scaling_state_controller** that publishes the current execution speed as reported by the robot to a topic interface. Values are floating points between 0 and 1.
* A **scaled_joint_trajectory_controller** that is similar to the *joint_trajectory_controller*, but it uses the speed scaling reported by the robot to reduce progress in the trajectory.

## About this package

This package contains controllers not being available in the default `ros2_control` set. They are created to support more features offered by the Elite CS robot family. 

## Controller description

This packages offers a couple of specific controllers that will be explained in the following sections.

### eli_cs_controllers/speed_scaling_state_broadcaster


This controller publishes the current actual execution speed as reported by the robot. Values are floating points between 0 and 1.


### position_controllers/ScaledJointTrajectoryController and velocity_controllers/ScaledJointTrajectoryController

These controllers work similar to the well-known [joint_trajectory_controller](https://control.ros.org/master/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html)

However, they are extended to handle the robot's execution speed specifically. Because the default ``joint_trajectory_controller`` would interpolate the trajectory with the configured time constraints (ie: always assume maximum velocity and acceleration supported by the robot), this could lead to significant path deviation due to multiple reasons:

* The speed slider on the robot might not be at 100%, so motion commands sent from ROS would effectively get scaled down resulting in a slower execution.
* The robot could scale down motions based on configured safety limits resulting in a slower motion than expected and therefore not reaching the desired target in a control cycle.
* Motions might not be executed at all, e.g. because the robot is stopped or in a protective stop
* Motion commands sent to the robot might not be interpreted, e.g. because there is no [`ExternalControl]() program node running on the robot controller.
* The program interpreting motion commands could be paused.

All of the cases mentioned above are addressed by the scaled trajectory versions. Trajectory execution can be transparently scaled down using the speed slider on the teach pendant without leading to additional path deviations. Pausing the program or hitting the stop effectively leads to ``speed_scaling`` being 0 meaning the trajectory will not be continued until the program is continued. This way, trajectory executions can be explicitly paused and continued.

### freedrive_controller

This controller manages the activation and deactivation of freedrive mode. Before enabling this controller, both the position controller and velocity controller must be set to inactive state.

The controller subscribes to the `~/enable_freedrive` topic (message type: `[std_msgs/msg/Bool]`).

To activate freedrive mode (after controller activation):
```bash
ros2 topic pub --rate 2 /freedrive_controller/enable_freedrive std_msgs/msg/Bool "{data: true}"
```

To deactivate freedrive mode:
```bash
ros2 topic pub --rate 2 /freedrive_controller/enable_freedrive std_msgs/msg/Bool "{data: false}"
```

Always ensure position/velocity controllers are inactive before enabling freedrive mode.

The topic publish rate must be maintained above 1Hz, otherwise the freedrive mode will automatically terminate. 
