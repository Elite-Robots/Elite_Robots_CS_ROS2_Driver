<style>
p {
  text-indent: 2em;
}
</style>

# ROS2 接口

## elite ros2_control 节点

这是包含 ros2-control 堆栈的实际驱动程序节点。此处文档中提到的接口指的是机器人的硬件接口。对于不同的控制器，可能会有一些专门的 API，这些 API 不包含在当前的包里。

### 参数

请注意，参数是通过 ros2_control xacro 定义传递的。

#### *headless_mode*
以无头模式启动机器人。此模式不需要机器人上运行 ExternalControl EliCOs，但会直接将脚本发送到机器人。如果机器人启用了远程控制，您需要将模式设置为远程模式。

#### *input_recipe_filename*
包含请求 RTSI 输入的配方文件路径。

#### *output_recipe_filename*
包含请求 RTSI 输出的配方文件路径。

#### *script_sender_port*
驱动程序将提供一个接口，通过此端口接收脚本。

#### *reverse_port*
用于驱动程序与机器人控制器之间通信的端口。

#### *script_command_port*
用于驱动程序与机器人控制器之间通信的端口。

#### *trajectory_port*
用于驱动程序与机器人控制器之间通信的端口。

#### *robot_ip*
机器人的 IP 地址。

#### *local_ip*
机器人控制器的 IP 地址。

#### *servoj_gain*
暂时不可用。

#### *servoj_lookahead_time*
指定底层 servoj 命令的 lookahead_time 参数。每当位置控制处于活动状态时，将使用此值。较高的值将导致更平滑的轨迹，但也会引入更高的延迟，即 ROS 发送的命令与机器人执行的动作之间的延迟。单位：秒。范围：[0.03 - 0.2]

#### *tool_voltage*
工具电压将在机器人任务启动时设置。

## dashboard_client

### 服务

#### *power_on*[(std_srvs/Trigger)](https://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html)
开启机器人电机。要完全启动机器人，之后需调用 `brake_release`。

#### *power_off*[(std_srvs/Trigger)](https://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html)
关闭机器人电机。

#### *brake_release*[(std_srvs/Trigger)](https://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html)
释放刹车的服务。

#### *play*[(std_srvs/Trigger)](https://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html)
运行机器人当前任务。

#### *pause*[(std_srvs/Trigger)](https://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html)
暂停当前任务。

#### *stop*[(std_srvs/Trigger)](https://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html)
停止当前任务。

#### *shutdown*[(std_srvs/Trigger)](https://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html)
关闭机器人。

#### *reboot*[(std_srvs/Trigger)](https://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html)
重启机器人系统。

#### *unlock_protective_stop*[(std_srvs/Trigger)](https://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html)
取消保护停止。

#### *close_safety_dialog*[(std_srvs/Trigger)](https://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html)
关闭安全模式的弹出对话框。

#### *quit*[(std_srvs/Trigger)](https://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html)
退出并断开连接。

#### *popup*[eli_dashboard_interface/srv/Popup](/eli_dashboard_interface/srv/Popup.srv)
弹出消息框并显示指定的文本，或者关闭最新的弹出消息框。

#### *log*[(eli_dashboard_interface/srv/Log)](/eli_dashboard_interface/srv/Log.srv)
添加日志信息。

#### *get_task_status*[(eli_common_interface/srv/GetTaskStatus)](/eli_common_interface/srv/GetTaskStatus.srv)
获取当前任务的状态。

#### *is_task_saved*[(eli_dashboard_interface/srv/IsSaved)](/eli_dashboard_interface/srv/IsSaved.srv)
当前任务是否已保存。

#### *is_configuration_saved*[(eli_dashboard_interface/srv/IsSaved)](/eli_dashboard_interface/srv/IsSaved.srv)
当前配置是否已保存。

#### *robot_mode*[(eli_common_interface/srv/GetRobotMode)](/eli_common_interface/srv/GetRobotMode.srv)
获取机器人当前模式。

#### *get_safety_mode*[(eli_common_interface/srv/GetSafetyMode)](/eli_common_interface/srv/GetSafetyMode.srv)
获取机器人当前的安全模式。

#### *get_task_path*[(std_srvs/Trigger)](https://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html)
获取当前加载任务文件的相对路径（如果任务未保存，则视为未加载文件，未保存的文件没有明确路径）。

#### *load_configure*[(eli_dashboard_interface/srv/Load)](/eli_dashboard_interface/srv/Load.srv)
加载指定路径的配置文件。

#### *load_task*[(eli_dashboard_interface/srv/Load)](/eli_dashboard_interface/srv/Load.srv)
加载指定路径的任务文件。

#### *connect*[(std_srvs/Trigger)](https://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html)
连接到机器人。

#### *restart_safety*[(std_srvs/Trigger)](https://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html)
重启安全系统。

#### *custom_request*[(eli_dashboard_interface/srv/CustomRequest)](/eli_dashboard_interface/srv/CustomRequest.srv)
发送自定义指令并且接收回复。

## GPIO 控制器
控制器提供数字 IO 发布器、工具数据发布器等功能，同时还提供可以设置机器人 IO 状态的服务。

### 发布器

#### *io_states*[(eli_common_interface/msg/IOState)](/eli_common_interface/msg/IOState.msg)
标准配置和工具数字 IO 状态。

#### *tool_data*[(eli_common_interface/msg/ToolData)](/eli_common_interface/msg/ToolData.msg)
包含模式、输出电压等的工具数据。

#### *robot_mode*[(eli_common_interface/msg/RobotMode)](/eli_common_interface/msg/RobotMode.msg)
机器人的模式。

#### *safety_mode*[(eli_common_interface/msg/SafetyMode)](/eli_common_interface/msg/SafetyMode.msg)
机器人的安全模式。

#### *robot_task_running*[(std_msgs/msg/Bool)](https://docs.ros.org/en/api/std_msgs/html/msg/Bool.html)
如果任务正在运行且机器人已连接，值为 true，否则为 false。

### 服务

#### *set_io*[(eli_common_interface/srv/SetIO)](/eli_common_interface/srv/SetIO.srv)
设置标准配置、工具数字 IO。

#### *set_speed_slider*[(eli_common_interface/srv/SetSpeedSliderFraction)](/eli_common_interface/srv/SetSpeedSliderFraction.srv)
设置机器人速度滑块。

#### *resend_external_script*[(std_srvs/srv/Trigger)](https://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html)
如果处于无头模式，则重新发送外部脚本到机器人。

#### *hand_back_control*[(std_srvs/srv/Trigger)](https://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html)
退出外部控制脚本并交还控制。

#### *set_payload*[(eli_common_interface/srv/SetPayload)](/eli_common_interface/srv/SetPayload.srv)
设置机器人负载。

#### *zero_ftsensor*[(std_srvs/srv/Trigger)](https://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html)
归零 ft 传感器。
