#include "elite_robots_controllers/gpio_controller.hpp"

#include <string>

namespace ELITE_CS_CONTROLLER {
controller_interface::CallbackReturn GPIOController::on_init() {
    try {
        initMsgs();
        // Create the parameter listener and get the parameters
        param_listener_ = std::make_shared<gpio_controller::ParamListener>(get_node());
        params_ = param_listener_->get_params();
    } catch (const std::exception& e) {
        fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
        return CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration GPIOController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    const std::string tf_prefix = params_.tf_prefix;

    for (size_t i = 0; i < STANDARD_DIG_GPIO_NUM; i++) {
        config.names.emplace_back(tf_prefix + "gpio/standard_digital_output_cmd_" + std::to_string(i));
    }

    for (size_t i = 0; i < CONF_DIG_GPIO_NUM; i++) {
        config.names.emplace_back(tf_prefix + "gpio/configure_digital_output_cmd_" + std::to_string(i));
    }

    for (size_t i = 0; i < TOOL_DIG_GPIO_NUM; i++) {
        config.names.emplace_back(tf_prefix + "gpio/tool_digital_output_cmd_" + std::to_string(i));
    }

    for (size_t i = 0; i < STANARD_ANALOG_IO_NUM; i++) {
        config.names.emplace_back(tf_prefix + "gpio/standard_analog_output_cmd_" + std::to_string(i));
        config.names.emplace_back(tf_prefix + "gpio/standard_analog_output_type_cmd_" + std::to_string(i));
    }
    
    config.names.emplace_back(tf_prefix + "gpio/tool_voltage_cmd");

    config.names.emplace_back(tf_prefix + "gpio/io_async_success");

    config.names.emplace_back(tf_prefix + "speed_scaling/target_speed_fraction_cmd");

    config.names.emplace_back(tf_prefix + "speed_scaling/target_speed_fraction_async_success");

    config.names.emplace_back(tf_prefix + "resend_external_script/resend_external_script_cmd");

    config.names.emplace_back(tf_prefix + "resend_external_script/resend_external_script_async_success");

    // payload stuff
    config.names.emplace_back(tf_prefix + "payload/mass");
    config.names.emplace_back(tf_prefix + "payload/cog.x");
    config.names.emplace_back(tf_prefix + "payload/cog.y");
    config.names.emplace_back(tf_prefix + "payload/cog.z");
    config.names.emplace_back(tf_prefix + "payload/payload_async_success");

    // FTS sensor
    config.names.emplace_back(tf_prefix + "zero_ftsensor/zero_ftsensor_cmd");
    config.names.emplace_back(tf_prefix + "zero_ftsensor/zero_ftsensor_async_success");

    // hand back control --> make program return
    config.names.emplace_back(tf_prefix + "hand_back_control/hand_back_control_cmd");
    config.names.emplace_back(tf_prefix + "hand_back_control/hand_back_control_async_success");

    return config;
}

controller_interface::InterfaceConfiguration ELITE_CS_CONTROLLER::GPIOController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    const std::string tf_prefix = params_.tf_prefix;

    // digital io
    for (size_t i = 0; i < STANDARD_DIG_GPIO_NUM; i++) {
        config.names.emplace_back(tf_prefix + "gpio/standard_digital_output_" + std::to_string(i));
    }
    for (size_t i = 0; i < CONF_DIG_GPIO_NUM; i++) {
        config.names.emplace_back(tf_prefix + "gpio/configure_digital_output_" + std::to_string(i));
    }
    for (size_t i = 0; i < TOOL_DIG_GPIO_NUM; i++) {
        config.names.emplace_back(tf_prefix + "gpio/tool_digital_output_" + std::to_string(i));
    }

    for (size_t i = 0; i < STANDARD_DIG_GPIO_NUM; i++) {
        config.names.emplace_back(tf_prefix + "gpio/standard_digital_input_" + std::to_string(i));
    }
    for (size_t i = 0; i < CONF_DIG_GPIO_NUM; i++) {
        config.names.emplace_back(tf_prefix + "gpio/configure_digital_input_" + std::to_string(i));
    }
    for (size_t i = 0; i < TOOL_DIG_GPIO_NUM; i++) {
        config.names.emplace_back(tf_prefix + "gpio/tool_digital_input_" + std::to_string(i));
    }

    // analog io
    for (size_t i = 0; i < STANARD_ANALOG_IO_NUM; i++) {
        config.names.emplace_back(tf_prefix + "gpio/standard_analog_output_type_" + std::to_string(i));
        config.names.emplace_back(tf_prefix + "gpio/standard_analog_output_" + std::to_string(i));
    }

    for (size_t i = 0; i < STANARD_ANALOG_IO_NUM; i++) {
        config.names.emplace_back(tf_prefix + "gpio/standard_analog_input_type_" + std::to_string(i));
        config.names.emplace_back(tf_prefix + "gpio/standard_analog_input_" + std::to_string(i));
    }

    // tool
    config.names.emplace_back(tf_prefix + "gpio/tool_mode");
    config.names.emplace_back(tf_prefix + "gpio/tool_output_voltage");
    config.names.emplace_back(tf_prefix + "gpio/tool_output_current");
    config.names.emplace_back(tf_prefix + "gpio/tool_temperature");

    config.names.emplace_back(tf_prefix + "gpio/tool_analog_input_type");
    config.names.emplace_back(tf_prefix + "gpio/tool_analog_input");
    config.names.emplace_back(tf_prefix + "gpio/tool_analog_output_type");
    config.names.emplace_back(tf_prefix + "gpio/tool_analog_output");
    
    // robot
    config.names.emplace_back(tf_prefix + "gpio/robot_mode");
    for (size_t i = 0; i < ROBOT_STATUS_BITS_NUM; ++i) {
        config.names.emplace_back(tf_prefix + "gpio/robot_status_bit_" + std::to_string(i));
    }

    // safety
    config.names.emplace_back(tf_prefix + "gpio/safety_mode");
    for (size_t i = 0; i < SAFETY_STATUS_BITS_NUM; ++i) {
        config.names.emplace_back(tf_prefix + "gpio/safety_status_bit_" + std::to_string(i));
    }
    config.names.emplace_back(tf_prefix + "system_interface/initialized");

    // task running
    config.names.emplace_back(tf_prefix + "gpio/task_running");

    return config;
}

controller_interface::return_type ELITE_CS_CONTROLLER::GPIOController::update(const rclcpp::Time& /*time*/,
                                                                         const rclcpp::Duration& /*period*/) {
    publishIO();
    publishToolData();
    publishRobotMode();
    publishSafetyMode();
    publishTaskRunning();
    return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn ELITE_CS_CONTROLLER::GPIOController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    const auto logger = get_node()->get_logger();

    if (!param_listener_) {
        RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
        return controller_interface::CallbackReturn::ERROR;
    }

    // update the dynamic map parameters
    param_listener_->refresh_dynamic_parameters();

    // get parameters from the listener in case they were updated
    params_ = param_listener_->get_params();

    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void GPIOController::publishIO() {
    for (size_t i = 0; i < STANDARD_DIG_GPIO_NUM; ++i) {
        io_msg_.standard_out[i] = static_cast<bool>(state_interfaces_[i].get_value());
        io_msg_.standard_in[i] = static_cast<bool>(state_interfaces_[i + (int)StateOffset::STANDARD_DIG_IN].get_value());
    }
    for (size_t i = 0; i < CONF_DIG_GPIO_NUM; ++i) {
        io_msg_.config_out[i] = static_cast<bool>(state_interfaces_[i + (int)StateOffset::CONFIGURE_DIG_OUT].get_value());
        io_msg_.config_in[i] = static_cast<bool>(state_interfaces_[i + (int)StateOffset::CONFIGURE_DIG_IN].get_value());
    }
    for (size_t i = 0; i < TOOL_DIG_GPIO_NUM; ++i) {
        io_msg_.tool_out[i] = static_cast<bool>(state_interfaces_[i + (int)StateOffset::TOOL_DIG_OUT].get_value());
        io_msg_.tool_in[i] = static_cast<bool>(state_interfaces_[i + (int)StateOffset::TOOL_DIG_IN].get_value());
    }

    io_msg_.standard_analog_in[0].type = (uint8_t)state_interfaces_[(int)StateOffset::STANDARD_ANALOG_IN_TYPE1].get_value();
    io_msg_.standard_analog_in[0].value = state_interfaces_[(int)StateOffset::STANDARD_ANALOG_IN1].get_value();

    io_msg_.standard_analog_in[1].type = (uint8_t)state_interfaces_[(int)StateOffset::STANDARD_ANALOG_IN_TYPE2].get_value();
    io_msg_.standard_analog_in[1].value = state_interfaces_[(int)StateOffset::STANDARD_ANALOG_IN2].get_value();

    io_msg_.standard_analog_out[0].type = state_interfaces_[(int)StateOffset::STANDARD_ANALOG_OUT_TYPE1].get_value();
    io_msg_.standard_analog_out[0].value = state_interfaces_[(int)StateOffset::STANDARD_ANALOG_OUT1].get_value();

    io_msg_.standard_analog_out[1].type = state_interfaces_[(int)StateOffset::STANDARD_ANALOG_OUT_TYPE2].get_value();
    io_msg_.standard_analog_out[1].value = state_interfaces_[(int)StateOffset::STANDARD_ANALOG_OUT2].get_value();

    io_pub_->publish(io_msg_);
}

void GPIOController::publishToolData() {
    tool_data_msg_.mode = static_cast<uint8_t>(state_interfaces_[(int)StateOffset::TOOL_MODE].get_value());
    tool_data_msg_.output_voltage = state_interfaces_[(int)StateOffset::TOOL_OUTPUT_VOLTAGE].get_value();
    tool_data_msg_.output_current = state_interfaces_[(int)StateOffset::TOOL_OUTPUT_CURRENT].get_value();
    tool_data_msg_.temperature = state_interfaces_[(int)StateOffset::TOOL_TEMPERATURE].get_value();
    tool_data_msg_.analog_input_type = (uint8_t)state_interfaces_[(int)StateOffset::TOOL_ANALOG_INPUT_TYPE].get_value();
    tool_data_msg_.analog_input = state_interfaces_[(int)StateOffset::TOOL_ANALOG_INPUT].get_value();
    tool_data_msg_.analog_output_type = (uint8_t)state_interfaces_[(int)StateOffset::TOOL_ANALOG_OUTPUT_TYPE].get_value();
    tool_data_msg_.analog_output = state_interfaces_[(int)StateOffset::TOOL_ANALOG_OUTPUT].get_value();
    tool_data_pub_->publish(tool_data_msg_);
}

void GPIOController::publishRobotMode() {
    auto robot_mode = static_cast<int8_t>(state_interfaces_[(int)StateOffset::ROBOT_MODE].get_value());

    if (robot_mode_msg_.mode != robot_mode) {
        robot_mode_msg_.mode = robot_mode;
        robot_mode_pub_->publish(robot_mode_msg_);
    }
}

void GPIOController::publishSafetyMode() {
    auto safety_mode = static_cast<uint8_t>(state_interfaces_[(int)StateOffset::SAFETY_MODE].get_value());

    if (safety_mode_msg_.mode != safety_mode) {
        safety_mode_msg_.mode = safety_mode;
        safety_mode_pub_->publish(safety_mode_msg_);
    }
}

void GPIOController::publishTaskRunning() {
    auto task_running_value = static_cast<uint8_t>(state_interfaces_[(int)StateOffset::TASK_RUNNING].get_value());
    bool task_running = task_running_value == 1.0 ? true : false;
    if (task_running_msg_.data != task_running) {
        task_running_msg_.data = task_running;
        task_state_pub_->publish(task_running_msg_);
    }
}

controller_interface::CallbackReturn ELITE_CS_CONTROLLER::GPIOController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    while (state_interfaces_[(int)StateOffset::INITIALIZED_FLAG].get_value() == 0.0) {
        RCLCPP_INFO(get_node()->get_logger(), "Waiting for system interface to initialize...");
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    try {
        // register publisher
        io_pub_ = get_node()->create_publisher<elite_robots_msgs::msg::IOState>("~/io_states", rclcpp::SystemDefaultsQoS());

        tool_data_pub_ = get_node()->create_publisher<elite_robots_msgs::msg::ToolData>("~/tool_data", rclcpp::SystemDefaultsQoS());

        robot_mode_pub_ =
            get_node()->create_publisher<elite_robots_msgs::msg::RobotMode>("~/robot_mode", rclcpp::SystemDefaultsQoS());

        safety_mode_pub_ =
            get_node()->create_publisher<elite_robots_msgs::msg::SafetyMode>("~/safety_mode", rclcpp::SystemDefaultsQoS());

        auto task_state_pub_qos = rclcpp::SystemDefaultsQoS();
        task_state_pub_qos.transient_local();
        task_state_pub_ = get_node()->create_publisher<std_msgs::msg::Bool>("~/robot_task_running", task_state_pub_qos);

        set_io_srv_ = get_node()->create_service<elite_robots_msgs::srv::SetIO>(
            "~/set_io", std::bind(&GPIOController::setIO, this, std::placeholders::_1, std::placeholders::_2));

        set_speed_slider_srv_ = get_node()->create_service<elite_robots_msgs::srv::SetSpeedSliderFraction>(
            "~/set_speed_slider", std::bind(&GPIOController::setSpeedSlider, this, std::placeholders::_1, std::placeholders::_2));

        resend_external_script_srv_ = get_node()->create_service<std_srvs::srv::Trigger>(
            "~/resend_external_script",
            std::bind(&GPIOController::resendRobotControlScript, this, std::placeholders::_1, std::placeholders::_2));

        hand_back_control_srv_ = get_node()->create_service<std_srvs::srv::Trigger>(
            "~/hand_back_control", std::bind(&GPIOController::handBackControl, this, std::placeholders::_1, std::placeholders::_2));

        set_payload_srv_ = get_node()->create_service<elite_robots_msgs::srv::SetPayload>(
            "~/set_payload", std::bind(&GPIOController::setPayload, this, std::placeholders::_1, std::placeholders::_2));

        tare_sensor_srv_ = get_node()->create_service<std_srvs::srv::Trigger>(
            "~/zero_ftsensor", std::bind(&GPIOController::zeroFTSensor, this, std::placeholders::_1, std::placeholders::_2));
    } catch (...) {
        return LifecycleNodeInterface::CallbackReturn::ERROR;
    }
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ELITE_CS_CONTROLLER::GPIOController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    try {
        // reset publisher
        io_pub_.reset();
        tool_data_pub_.reset();
        robot_mode_pub_.reset();
        safety_mode_pub_.reset();
        task_state_pub_.reset();
        set_io_srv_.reset();
        set_speed_slider_srv_.reset();
    } catch (...) {
        return LifecycleNodeInterface::CallbackReturn::ERROR;
    }
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

bool GPIOController::setIO(elite_robots_msgs::srv::SetIO::Request::SharedPtr req, elite_robots_msgs::srv::SetIO::Response::SharedPtr resp) {
    if (req->fun == req->FUN_SET_DIGITAL_OUT && req->pin >= 0 && req->pin <= STANDARD_DIG_GPIO_NUM) {
        // io async success
        command_interfaces_[(int)CommandOffset::IO_ASYNC_SUCCESS].set_value(ASYNC_WAITING);
        command_interfaces_[req->pin].set_value(static_cast<double>(req->state));

        RCLCPP_INFO(get_node()->get_logger(), "Setting standard digital output '%d' to state: '%1.0f'.", req->pin, req->state);

        if (!waitForAsyncCommand([&]() { return command_interfaces_[(int)CommandOffset::IO_ASYNC_SUCCESS].get_value(); })) {
            RCLCPP_WARN(get_node()->get_logger(),
                        "Could not verify that io was set. (This might happen when using the "
                        "mocked interface)");
        }

        resp->success = static_cast<bool>(command_interfaces_[(int)CommandOffset::IO_ASYNC_SUCCESS].get_value());
        return resp->success;
    } else if(req->fun == req->FUN_SET_CONFIGURE_OUT && req->pin >= 0 && req->pin <= CONF_DIG_GPIO_NUM) {
        // io async success
        command_interfaces_[(int)CommandOffset::IO_ASYNC_SUCCESS].set_value(ASYNC_WAITING);
        command_interfaces_[req->pin + (int)CommandOffset::CONFIGURE_DIG_OUT].set_value(static_cast<double>(req->state));

        RCLCPP_INFO(get_node()->get_logger(), "Setting config digital output '%d' to state: '%1.0f'.", req->pin, req->state);

        if (!waitForAsyncCommand([&]() { return command_interfaces_[(int)CommandOffset::IO_ASYNC_SUCCESS].get_value(); })) {
            RCLCPP_WARN(get_node()->get_logger(),
                        "Could not verify that io was set. (This might happen when using the "
                        "mocked interface)");
        }

        resp->success = static_cast<bool>(command_interfaces_[(int)CommandOffset::IO_ASYNC_SUCCESS].get_value());
        return resp->success;
    }  else if(req->fun == req->FUN_SET_TOOL_DIG_OUT && req->pin >= 0 && req->pin <= TOOL_DIG_GPIO_NUM) {
        // io async success
        command_interfaces_[(int)CommandOffset::IO_ASYNC_SUCCESS].set_value(ASYNC_WAITING);
        command_interfaces_[req->pin + (int)CommandOffset::TOOL_DIG_OUT].set_value(static_cast<double>(req->state));

        RCLCPP_INFO(get_node()->get_logger(), "Setting tool digital output '%d' to state: '%1.0f'.", req->pin, req->state);

        if (!waitForAsyncCommand([&]() { return command_interfaces_[(int)CommandOffset::IO_ASYNC_SUCCESS].get_value(); })) {
            RCLCPP_WARN(get_node()->get_logger(),
                        "Could not verify that io was set. (This might happen when using the "
                        "mocked interface)");
        }

        resp->success = static_cast<bool>(command_interfaces_[(int)CommandOffset::IO_ASYNC_SUCCESS].get_value());
        return resp->success;
    } else if (req->fun == req->FUN_SET_ANALOG_OUT && req->pin >= 0 && req->pin <= STANDARD_DIG_GPIO_NUM) {
        // io async success
        command_interfaces_[(int)CommandOffset::IO_ASYNC_SUCCESS].set_value(ASYNC_WAITING);
        if(req->pin == 0) {
            command_interfaces_[(int)CommandOffset::STANDARD_ANALOG_OUT].set_value(static_cast<double>(req->state));
            command_interfaces_[(int)CommandOffset::STANDARD_ANALOG_OUT + 1].set_value(static_cast<double>(req->analog_type));
        } else {
            command_interfaces_[(int)CommandOffset::STANDARD_ANALOG_OUT + 2].set_value(static_cast<double>(req->state));
            command_interfaces_[(int)CommandOffset::STANDARD_ANALOG_OUT + 3].set_value(static_cast<double>(req->analog_type));
        }
        RCLCPP_INFO(get_node()->get_logger(), "Setting analog output '%d' to type: '%d' state: '%.4f'.", req->pin, req->analog_type, req->state);
        

        if (!waitForAsyncCommand([&]() { return command_interfaces_[(int)CommandOffset::IO_ASYNC_SUCCESS].get_value(); })) {
            RCLCPP_WARN(get_node()->get_logger(),
                        "Could not verify that io was set. (This might happen when using the "
                        "mocked interface)");
        }

        resp->success = static_cast<bool>(command_interfaces_[(int)CommandOffset::IO_ASYNC_SUCCESS].get_value());
        return resp->success;
    } else if (req->fun == req->FUN_SET_TOOL_VOLTAGE) {
        command_interfaces_[(int)CommandOffset::IO_ASYNC_SUCCESS].set_value(ASYNC_WAITING);
        command_interfaces_[(int)CommandOffset::TOOL_VOLTAGE].set_value(static_cast<double>(req->state));

        RCLCPP_INFO(get_node()->get_logger(), "Setting tool voltage to: '%1.0f'.", req->state);

        if (!waitForAsyncCommand([&]() { return command_interfaces_[(int)CommandOffset::IO_ASYNC_SUCCESS].get_value(); })) {
            RCLCPP_WARN(get_node()->get_logger(),
                        "Could not verify that io was set. (This might happen when using the "
                        "mocked interface)");
        }

        resp->success = static_cast<bool>(command_interfaces_[(int)CommandOffset::IO_ASYNC_SUCCESS].get_value());
        return resp->success;
    } else {
        resp->success = false;
        return false;
    }
}

bool GPIOController::setSpeedSlider(elite_robots_msgs::srv::SetSpeedSliderFraction::Request::SharedPtr req,
                                    elite_robots_msgs::srv::SetSpeedSliderFraction::Response::SharedPtr resp) {
    if (req->speed_slider_fraction >= 0.01 && req->speed_slider_fraction <= 1.0) {
        RCLCPP_INFO(get_node()->get_logger(), "Setting speed slider to %.2f%%.", req->speed_slider_fraction * 100.0);
        // reset success flag
        command_interfaces_[(int)CommandOffset::TARGET_SPEED_FRACTION_SUCCESS].set_value(ASYNC_WAITING);
        // set commanding value for speed slider
        command_interfaces_[(int)CommandOffset::TARGET_SPEED_FRACTION].set_value(
            static_cast<double>(req->speed_slider_fraction));

        if (!waitForAsyncCommand(
                [&]() { return command_interfaces_[(int)CommandOffset::TARGET_SPEED_FRACTION_SUCCESS].get_value(); })) {
            RCLCPP_WARN(get_node()->get_logger(),
                        "Could not verify that target speed fraction was set. (This might happen "
                        "when using the mocked interface)");
        }
        resp->success = static_cast<bool>(command_interfaces_[(int)CommandOffset::TARGET_SPEED_FRACTION_SUCCESS].get_value());
    } else {
        RCLCPP_WARN(get_node()->get_logger(),
                    "The desired speed slider fraction must be within range (0; 1.0]. Request "
                    "ignored.");
        resp->success = false;
        return false;
    }
    return true;
}

bool GPIOController::resendRobotControlScript(std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
                                        std_srvs::srv::Trigger::Response::SharedPtr resp) {
    // reset success flag
    command_interfaces_[(int)CommandOffset::RESEND_ROBOT_CONTROL_SCRIPT_SUCCESS].set_value(ASYNC_WAITING);
    // call the service in the hardware
    command_interfaces_[(int)CommandOffset::RESEND_ROBOT_CONTROL_SCRIPT].set_value(1.0);

    if (!waitForAsyncCommand(
            [&]() { return command_interfaces_[(int)CommandOffset::RESEND_ROBOT_CONTROL_SCRIPT_SUCCESS].get_value(); })) {
        RCLCPP_WARN(get_node()->get_logger(),
                    "Could not verify that program was sent. (This might happen when using the "
                    "mocked interface)");
    }
    resp->success = static_cast<bool>(command_interfaces_[(int)CommandOffset::RESEND_ROBOT_CONTROL_SCRIPT_SUCCESS].get_value());

    if (resp->success) {
        RCLCPP_INFO(get_node()->get_logger(), "Successfully resent robot program");
    } else {
        RCLCPP_ERROR(get_node()->get_logger(), "Could not resend robot program");
        return false;
    }

    return true;
}

bool GPIOController::handBackControl(std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
                                     std_srvs::srv::Trigger::Response::SharedPtr resp) {
    // reset success flag
    command_interfaces_[(int)CommandOffset::HAND_BACK_CONTROL_SUCCESS].set_value(ASYNC_WAITING);
    // call the service in the hardware
    command_interfaces_[(int)CommandOffset::HAND_BACK_CONTROL].set_value(1.0);

    if (!waitForAsyncCommand(
            [&]() { return command_interfaces_[(int)CommandOffset::HAND_BACK_CONTROL_SUCCESS].get_value(); })) {
        RCLCPP_WARN(get_node()->get_logger(),
                    "Could not verify that hand_back_control was correctly triggered. (This "
                    "might happen when using the mocked interface)");
    }
    resp->success = static_cast<bool>(command_interfaces_[(int)CommandOffset::HAND_BACK_CONTROL_SUCCESS].get_value());

    if (resp->success) {
        RCLCPP_INFO(get_node()->get_logger(), "Deactivated control");
    } else {
        RCLCPP_ERROR(get_node()->get_logger(), "Could not deactivate control");
        return false;
    }

    return true;
}

bool GPIOController::setPayload(const elite_robots_msgs::srv::SetPayload::Request::SharedPtr req,
                                elite_robots_msgs::srv::SetPayload::Response::SharedPtr resp) {
    // reset success flag
    command_interfaces_[(int)CommandOffset::PAYLOAD_SUCCESS].set_value(ASYNC_WAITING);

    command_interfaces_[(int)CommandOffset::MASS].set_value(req->mass);
    command_interfaces_[(int)CommandOffset::COG_X].set_value(req->center_of_gravity.x);
    command_interfaces_[(int)CommandOffset::COG_Y].set_value(req->center_of_gravity.y);
    command_interfaces_[(int)CommandOffset::COG_Z].set_value(req->center_of_gravity.z);

    if (!waitForAsyncCommand([&]() { return command_interfaces_[(int)CommandOffset::PAYLOAD_SUCCESS].get_value(); })) {
        RCLCPP_WARN(get_node()->get_logger(),
                    "Could not verify that payload was set. (This might happen when using the "
                    "mocked interface)");
    }

    resp->success = static_cast<bool>(command_interfaces_[(int)CommandOffset::PAYLOAD_SUCCESS].get_value());

    if (resp->success) {
        RCLCPP_INFO(get_node()->get_logger(), "Payload has been set successfully");
    } else {
        RCLCPP_ERROR(get_node()->get_logger(), "Could not set the payload");
        return false;
    }

    return true;
}

bool GPIOController::zeroFTSensor(std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
                                  std_srvs::srv::Trigger::Response::SharedPtr resp) {
    // reset success flag
    command_interfaces_[(int)CommandOffset::ZERO_FTSENSOR_SUCCESS].set_value(ASYNC_WAITING);
    // call the service in the hardware
    command_interfaces_[(int)CommandOffset::ZERO_FTSENSOR].set_value(1.0);

    if (!waitForAsyncCommand([&]() { return command_interfaces_[(int)CommandOffset::ZERO_FTSENSOR_SUCCESS].get_value(); })) {
        RCLCPP_WARN(get_node()->get_logger(),
                    "Could not verify that FTS was zeroed. (This might happen when using the "
                    "mocked interface)");
    }

    resp->success = static_cast<bool>(command_interfaces_[(int)CommandOffset::ZERO_FTSENSOR_SUCCESS].get_value());

    if (resp->success) {
        RCLCPP_INFO(get_node()->get_logger(), "Successfully zeroed the force torque sensor");
    } else {
        RCLCPP_ERROR(get_node()->get_logger(), "Could not zero the force torque sensor");
        return false;
    }

    return true;
}

void GPIOController::initMsgs() {
    io_msg_.standard_in.resize(STANDARD_DIG_GPIO_NUM);
    io_msg_.standard_out.resize(STANDARD_DIG_GPIO_NUM);
    io_msg_.config_in.resize(CONF_DIG_GPIO_NUM);
    io_msg_.config_out.resize(CONF_DIG_GPIO_NUM);
    io_msg_.tool_in.resize(TOOL_DIG_GPIO_NUM);
    io_msg_.tool_out.resize(TOOL_DIG_GPIO_NUM);
    io_msg_.standard_analog_in.resize(STANARD_ANALOG_IO_NUM);
    io_msg_.standard_analog_out.resize(STANARD_ANALOG_IO_NUM);
}

bool GPIOController::waitForAsyncCommand(std::function<double(void)> get_value) {
    const auto maximum_retries = params_.check_io_successfull_retries;
    int retries = 0;
    while (get_value() == ASYNC_WAITING) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        retries++;

        if (retries > maximum_retries) return false;
    }
    return true;
}

}  // namespace ELITE_CS_CONTROLLER

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ELITE_CS_CONTROLLER::GPIOController, controller_interface::ControllerInterface)
