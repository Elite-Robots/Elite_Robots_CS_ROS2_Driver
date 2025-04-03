<style>
p {
  text-indent: 2em;
}
</style>
# ROS2 Interface

## elite ros2_control nodes

This is the actual driver node containing the ros2-control stack. Interfaces documented here refer to the robot’s hardware interface. Controller-specific API elements might be present for the individual controllers outside of this package.

### Parameters

Note that parameters are passed through the ros2_control xacro definition.

#### *headless_mode*
Start robot in headless mode. This does not require the ExternalControl EliCOs to be running on the robot, but this will send the script to the robot directly. If the robot has remote control enabled, you need to set the mode to remote mode.

#### *input_recipe_filename*
Path to the file containing the recipe used for requesting RTSI inputs.
#### *output_recipe_filename*
Path to the file containing the recipe used for requesting RTSI outputs.

#### *script_sender_port*
The driver will offer an interface to receive the script on this port.

#### *reverse_port*
Port that will be opened to communicate between the driver and the robot controller.

#### *script_command_port*
Port that will be opened to communicate between the driver and the robot controller.

#### *trajectory_port*
Port that will be opened to communicate between the driver and the robot controller.

#### *robot_ip*
The robot’s IP address.

#### *local_ip*
The robot controller's IP address.

#### *servoj_gain*
Temporarily unavailable.

#### *servoj_lookahead_time*
Specify lookahead_time parameter of underlying servoj command. This will be used whenever position control is active. A higher value will result in smoother trajectories, but will also introduce a higher delay between the commands sent from ROS and the motion being executed on the robot. Unit: seconds. Range: [0.03 - 0.2]

#### *tool_voltage*
Tool voltage that will be set as soon as the task on the robot is started.

## dashboard_client

### Services

#### *power_on*[(std_srvs/Trigger)](https://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html)
Power on the robot motors. To fully start the robot, call ‘brake_release’ afterwards.

#### *power_off*[(std_srvs/Trigger)](https://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html)
Power off the robot motors.

#### *brake_release*[(std_srvs/Trigger)](https://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html)
Service to release the brakes.

#### *play*[(std_srvs/Trigger)](https://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html)
Run the robot current task.

#### *pause*[(std_srvs/Trigger)](https://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html)
Pause the running task.

#### *stop*[(std_srvs/Trigger)](https://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html)
Stop the running task.

#### *shutdown*[(std_srvs/Trigger)](https://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html)
Shutdown the robot shutdown.

#### *reboot*[(std_srvs/Trigger)](https://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html)
Reboot the robot system.

#### *unlock_protective_stop*[(std_srvs/Trigger)](https://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html)
Cancel the protective stop. 


#### *close_safety_dialog*[(std_srvs/Trigger)](https://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html)
Close the pop-up dialogue of the safe mode.

#### *quit*[(std_srvs/Trigger)](https://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html)
Quit and disconnect.

#### *popup*[eli_dashboard_interface/srv/Popup](/eli_dashboard_interface/srv/Popup.srv)
Pop up a message box with the specified texts or close a message box of the latest popup commands.

#### *log*[(eli_dashboard_interface/srv/Log)](/eli_dashboard_interface/srv/Log.srv)
Add the log information.

#### *get_task_status*[(eli_common_interface/srv/GetTaskStatus)](/eli_common_interface/srv/GetTaskStatus.srv)
Get current task state.

#### *is_task_saved*[(eli_dashboard_interface/srv/IsSaved)](/eli_dashboard_interface/srv/IsSaved.srv)
Is current task saved.

#### *is_configuration_saved*[(eli_dashboard_interface/srv/IsSaved)](/eli_dashboard_interface/srv/IsSaved.srv)
Is current configuration saved.

#### *robot_mode*[(eli_common_interface/srv/GetRobotMode)](/eli_common_interface/srv/GetRobotMode.srv)
The robot current mode.

#### *get_safety_mode*[(eli_common_interface/srv/GetSafetyMode)](/eli_common_interface/srv/GetSafetyMode.srv)
The robot current safety mode.

#### *get_task_path*[(std_srvs/Trigger)](https://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html)
Got the relative path of the currently loaded task file (It will be deemed no file loading if any untitled task is not saved locally. Unsaved files do not have a clear path.)

#### *load_configure*[(eli_dashboard_interface/srv/Load)](/eli_dashboard_interface/srv/Load.srv)
Loaded the configuration file in accordance with the path.

#### *load_task*[(eli_dashboard_interface/srv/Load)](/eli_dashboard_interface/srv/Load.srv)
Loaded the task file in accordance with the path.

#### *connect*[(std_srvs/Trigger)](https://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html)
Connect to robot.

#### *restart_safety*[(std_srvs/Trigger)](https://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html)
Restart robot safety system.

#### *custom_request*[(eli_dashboard_interface/srv/CustomRequest)](/eli_dashboard_interface/srv/CustomRequest.srv)
Send custom request and receive robot response

## GPIO Controller
The controller provide ditital IO publisher, tool data publisher and the like. Also provide service which can set robot IO state.

### Publisher

#### *io_states*[(eli_common_interface/msg/IOState)](/eli_common_interface/msg/IOState.msg)
The standard ,configure and tool digital IO state.

#### *tool_data*[(eli_common_interface/msg/ToolData)](/eli_common_interface/msg/ToolData.msg)
The tool data containing mode, output voltage and so on.

#### *robot_mode*[(eli_common_interface/msg/RobotMode)](/eli_common_interface/msg/RobotMode.msg)
The robot mode.

#### *safety_mode*[(eli_common_interface/msg/SafetyMode)](/eli_common_interface/msg/SafetyMode.msg)
The robot safety mode.

#### *robot_task_running*[(std_msgs/msg/Bool)](https://docs.ros.org/en/api/std_msgs/html/msg/Bool.html)
If running task and robot connected, the value is true. Else is false.

### Services

#### *set_io*[(eli_common_interface/srv/SetIO)](/eli_common_interface/srv/SetIO.srv)
Set the standard, configure, tool digital IO.

#### *set_speed_slider*[(eli_common_interface/srv/SetSpeedSliderFraction)](/eli_common_interface/srv/SetSpeedSliderFraction.srv)
Set the robot speed slider.

#### *resend_external_script*[(std_srvs/srv/Trigger)](https://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html)
If in headless mode, resend the external script to robot.

#### *hand_back_control*[(std_srvs/srv/Trigger)](https://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html)
Exit external control script hand back control.

#### *set_payload*[(eli_common_interface/srv/SetPayload)](/eli_common_interface/srv/SetPayload.srv)
Set robot payload.

#### *zero_ftsensor*[(std_srvs/srv/Trigger)](https://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html)
Zero the ft sensor.


