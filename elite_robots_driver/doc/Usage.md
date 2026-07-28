# Usage

## Launch files
For starting the driver there are a main launch files in the `elite_robots_driver` package.
- elite_control.launch.py - starts [ros2_control](https://control.ros.org/humble/index.html) node including hardware interface, joint state broadcaster and a controller. This launch file also starts dashboard_client and primary_client if real robot is used.

The arguments for launch files can be listed using `ros2 launch elite_robots_driver elite_control.launch.py --show-args`. The most relevant arguments are the following:
- `cs_type` (mandatory) - a type of used Elite CS robot (`cs63`, `cs66`, `cs66a`, `cs68`, `cs612`, `cs616`, `cs618f`, `cs620`, `cs625`, `cs520h`, `ls65`). If the type ends with `h` (5-axis), the launch automatically uses the 5-axis URDF and controller configuration.
- `robot_ip` (mandatory) - the used robot ip which the root can be reached.
- `local_ip` (mandatory) - the external controller ip address which robot can readched.
- `use_fake_hardware` (default: false ) - use simple hardware emulator from [ros2_control](https://control.ros.org/humble/index.html). Useful for testing launch files, descriptions, etc. See explanation below.
- `fake_sensor_commands` (default: false ) - enables setting sensor values for the hardware emulators. Useful for offline testing of controllers.
- `initial_joint_controller`(default: scaled_joint_trajectory_controller) - controller for robot joints to be started. Available controllers:
    - joint_trajectory_controller
    - scaled_joint_trajectory_controller

> Note : joint_state_broadcaster, speed_scaling_state_broadcaster, force_torque_sensor_broadcaster, and io_and_status_controller will always start.

> Note : list all loaded controllers using `ros2 control list_controllers command`. For this, the package ros2controlcli must be installed. Command: `sudo apt-get install ros-${ROS_DISTRO}-ros2controlcli`.

## Example Commands for Testing the Driver

Allowed CS robots - Type strings: `cs63`, `cs66`, `cs66a`, `cs68`, `cs612`, `cs616`, `cs618f`, `cs620`, `cs625`, `cs520h`, `ls65`

> ***NOTE:***  
> **These tests is a very basic. Look at the code and make sure that the robot is able to perform the motions safely before running this on a real robot!**

### 1. Start hardware, simulator or fake hardware

- To do test with hardware or simulator, use:

```bash
ros2 launch elite_robots_driver elite_control.launch.py robot_ip:=xxx.xxx.xxx.xxx local_ip:=yyy.yyy.yyy.yyy cs_type:=zzzz
```
For more details check the argument documentation with : `ros2 launch elite_robots_driver elite_control.launch.py --show-args`.

After starting the launch file start the ExternalControl EliCOs task from the pendant, as described above.

- To use fake hardware, use the `use_fake_hardware` argument, like:
```bash
ros2 launch elite_robots_driver elite_control.launch.py robot_ip:=xxx.xxx.xxx.xxx local_ip:=yyy.yyy.yyy.yyy cs_type:=zzzz use_fake_hardware:=true
```

### 2. Sending commands to controllers

Before running any commands, first check the controllers’ state using `ros2 control list_controllers` (Remember to install the ros2controlcli package as mentioned above).

- Send some goal to the Joint Trajectory Controller by using a demo node from ros2_controllers_test_nodes package by starting the following command in another terminal:
```bash
ros2 launch elite_robots_driver example_scaled_joint_trajectory_controller.launch.py
```

After a few seconds the robot should move.

To test another controller, simply define it using initial_joint_controller argument, for example when using fake hardware:
```bash
ros2 launch elite_robots_driver elite_control.launch.py robot_ip:=xxx.xxx.xxx.xxx local_ip:=yyy.yyy.yyy.yyy cs_type:=zzzz initial_joint_controller:=joint_trajectory_controller use_fake_hardware:=true
```

And send the command using demo node:
```bash
ros2 launch elite_robots_driver example_joint_trajectory_controller.launch.py
```

After a few seconds the robot should move(or jump when using emulation).

In case you want to write your own ROS node to move the robot, there is an example python node included that you can use as a start.

The following example is for 6-axis robots. For `cs520h`, use only the five joints `shoulder_pan_joint`, `shoulder_lift_joint`, `elbow_joint`, `wrist_1_joint`, and `wrist_2_joint`, and provide five position values per point.
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

### 3. Using only robot description
If you just want to test description of the Elite CS robots, e.g., you can use the following command:
```bash
    ros2 launch elite_robots_description view_cs.launch.py cs_type:=zzzz
```

## Using MoveIt
[MoveIt!](https://moveit.ros.org/) support is built-in into this driver already.
To test the driver with the example MoveIt-setup, first start the driver using:
```bash
ros2 launch elite_robots_driver elite_control.launch.py robot_ip:=xxx.xxx.xxx.xxx local_ip:=yyy.yyy.yyy.yyy cs_type:=zzzz launch_rviz:=false
```

And then start the MoveIt! nodes using:
```bash
ros2 launch elite_robots_moveit_config elite_moveit.launch.py cs_type:=zzzz launch_rviz:=true
```

Now you should be able to use the MoveIt Plugin in rviz2 to plan and execute trajectories with the robot as explained [here](https://moveit.picknik.ai/main/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html).

For `cs520h`, the MoveIt launch automatically uses the 5-axis SRDF, joint limits, and controller list.


## Robot frames

While most tf frames come from the URDF and are published from the robot_state_publisher, there are a couple of things to know:
- The base frame is the robot’s base as the robot controller sees it.
- The tool0_controller is the tool frame as published from the robot controller. If there is no additional tool configured on the Teach pendant (TP), this should be equivalent to tool0 given that the URDF uses the specific robot’s calibration. If a tool is configured on the TP, then the additional transformation will show in base -> tool0.

## Custom script
The driver’s package contains a script_sender node that allows sending script snippets directly to the robot when the robot is in remote control mode.

It gets started in the driver’s launch files by default. To use it, simply publish a message to its interface:
```bash
# Display a popup box on the teach pendant.
ros2 topic pub /script_sender/script_command std_msgs/msg/String '{data: popup("hello")}' --once
```

Be aware, that running a script on this interface (meaning publishing script code to that interface) stops any running program on the robot. Thus, the motion-interpreting program that is started by the driver gets stopped and has to be restarted again. Depending whether you use headless mode or not, you’ll have to call the resend_external_script service or press the play button on the teach panel to start the ExternalControl task again.

## Multi-line script
When you want to define multi-line script, make sure to check that newlines are correctly interpreted from your message. For this purpose the driver prints the script as it is being sent to the robot. When sending a multi-line script from the command line, you can use an empty line between each statement:
```bash
ros2 topic pub --once /script_sender/script_command std_msgs/msg/String '{data:
"def my_prog():  

    \tset_standard_digital_out(1, True)  

    \tmovej([0, -1.57, 0, -1.57, 0, 0], a=1.2, v=0.25, r=0)  

    \ttextmsg(\"motion finished\")  

end"}'
```

## Non-interrupting script

To prevent interrupting the main script, you can send certain commands as secondary script.
```bash
ros2 topic pub --once /script_sender/script_command std_msgs/msg/String '{data:
"sec my_prog():  

    \tpopup(\"This is a hello message\")

end"}'
```
