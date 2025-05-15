#include "eli_cs_robot_driver/hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <Elite/EliteException.hpp>
#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <bitset>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <fstream>
#include <sstream>
#include <cstring>

namespace ELITE_CS_ROBOT_ROS_DRIVER {

using namespace std::chrono;

EliteCSPositionHardwareInterface::~EliteCSPositionHardwareInterface() {
    // If the controller manager is shutdown via Ctrl + C the on_deactivate methods won't be called.
    // We therefore need to make sure to actually deactivate the communication
    on_cleanup(rclcpp_lifecycle::State());
}

hardware_interface::CallbackReturn EliteCSPositionHardwareInterface::on_init(const hardware_interface::HardwareInfo& system_info) {
    if (hardware_interface::SystemInterface::on_init(system_info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    info_ = system_info;

    // initialize
    joint_positions_ = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    joint_velocities_ = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    joint_efforts_ = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    ft_sensor_measurements_ = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    tcp_pose_ = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    position_commands_ = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    velocity_commands_ = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    stop_modes_ = {StoppingInterface::NONE, StoppingInterface::NONE, StoppingInterface::NONE,
                   StoppingInterface::NONE, StoppingInterface::NONE, StoppingInterface::NONE};
    start_modes_ = {};
    position_controller_running_ = false;
    velocity_controller_running_ = false;
    runtime_state_ = ELITE::TaskStatus::STOPPED;
    controllers_initialized_ = false;
    system_interface_initialized_ = 0.0;
    resend_external_script_cmd_ = NO_NEW_CMD;
    is_robot_connected_ = false;
    is_last_power_on_ = false;

    for (size_t i = 0; i < STANDARD_DIG_GPIO_NUM; i++) {
        standard_dig_out_bits_cmd_[i] = NO_NEW_CMD;
    }
    for (size_t i = 0; i < CONF_DIG_GPIO_NUM; i++) {
        conf_dig_out_bits_cmd_[i] = NO_NEW_CMD;
    }
    for (size_t i = 0; i < TOOL_DIG_GPIO_NUM; i++) {
        tool_dig_out_bits_cmd_[i] = NO_NEW_CMD;
    }

    for (const hardware_interface::ComponentInfo& joint : info_.joints) {
        // position and velocity
        if (joint.command_interfaces.size() != 2) {
            RCLCPP_FATAL(rclcpp::get_logger("EliteCSPositionHardwareInterface"),
                         "Joint '%s' has %zu command interfaces found. 2 expected.", joint.name.c_str(),
                         joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(rclcpp::get_logger("EliteCSPositionHardwareInterface"),
                         "Joint '%s' have %s command interfaces found as first command interface. '%s' expected.",
                         joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(rclcpp::get_logger("EliteCSPositionHardwareInterface"),
                         "Joint '%s' have %s command interfaces found as second command interface. '%s' expected.",
                         joint.name.c_str(), joint.command_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 3) {
            RCLCPP_FATAL(rclcpp::get_logger("EliteCSPositionHardwareInterface"), "Joint '%s' has %zu state interface. 3 expected.",
                         joint.name.c_str(), joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(rclcpp::get_logger("EliteCSPositionHardwareInterface"),
                         "Joint '%s' have %s state interface as first state interface. '%s' expected.", joint.name.c_str(),
                         joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(rclcpp::get_logger("EliteCSPositionHardwareInterface"),
                         "Joint '%s' have %s state interface as second state interface. '%s' expected.", joint.name.c_str(),
                         joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
            RCLCPP_FATAL(rclcpp::get_logger("EliteCSPositionHardwareInterface"),
                         "Joint '%s' have %s state interface as third state interface. '%s' expected.", joint.name.c_str(),
                         joint.state_interfaces[2].name.c_str(), hardware_interface::HW_IF_EFFORT);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> EliteCSPositionHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION,
                                                                         &joint_positions_[i]));

        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
                                                                         &joint_velocities_[i]));

        state_interfaces.emplace_back(
            hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joint_efforts_[i]));
    }

    // Obtain the tf_prefix from the urdf so that we can have the general interface multiple times
    // NOTE using the tf_prefix at this point is some kind of workaround. One should actually go through the list of gpio
    // state interface in info_ and match them accordingly
    const std::string tf_prefix = info_.hardware_parameters.at("tf_prefix");
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(tf_prefix + "speed_scaling", "speed_scaling_factor", &speed_scaling_combined_));

    for (auto& sensor : info_.sensors) {
        for (uint j = 0; j < sensor.state_interfaces.size(); j++) {
            state_interfaces.emplace_back(
                hardware_interface::StateInterface(sensor.name, sensor.state_interfaces[j].name, &ft_sensor_measurements_[j]));
        }
    }


    for (size_t i = 0; i < STANDARD_DIG_GPIO_NUM; i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "standard_digital_output_" + std::to_string(i),
                                                                         &standard_dig_out_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "standard_digital_input_" + std::to_string(i),
                                                                         &standard_dig_in_[i]));
    }

    for (size_t i = 0; i < CONF_DIG_GPIO_NUM; i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio",
                                                                         "configure_digital_output_" + std::to_string(i),
                                                                         &config_dig_out_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio",
                                                                         "configure_digital_input_" + std::to_string(i),
                                                                         &config_dig_in_[i]));
    }

    for (size_t i = 0; i < TOOL_DIG_GPIO_NUM; i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio",
                                                                         "tool_digital_output_" + std::to_string(i),
                                                                         &tool_dig_out_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio",
                                                                         "tool_digital_input_" + std::to_string(i),
                                                                         &tool_dig_in_[i]));
    }

    for (size_t i = 0; i < SAFETY_STATUS_BITS_NUM; i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            tf_prefix + "gpio", "safety_status_bit_" + std::to_string(i), &safety_status_bits_copy_[i]));
    }


    for (size_t i = 0; i < ROBOT_STATUS_BITS_NUM; i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            tf_prefix + "gpio", "robot_status_bit_" + std::to_string(i), &robot_status_bits_copy_[i]));
    }

    for (size_t i = 0; i < STANARD_ANALOG_IO_NUM; i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            tf_prefix + "gpio", "standard_analog_input_type_" + std::to_string(i), &standard_analog_input_types_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
            tf_prefix + "gpio", "standard_analog_input_" + std::to_string(i), &standard_analog_input_[i]));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            tf_prefix + "gpio", "standard_analog_output_" + std::to_string(i), &standard_analog_output_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            tf_prefix + "gpio", "standard_analog_output_type_" + std::to_string(i), &standard_analog_output_types_[i]));
    }

    state_interfaces.emplace_back(hardware_interface::StateInterface(
            tf_prefix + "gpio", "tool_analog_input_type", &tool_analog_input_type_copy_));
    
    state_interfaces.emplace_back(hardware_interface::StateInterface(
            tf_prefix + "gpio", "tool_analog_output_type", &tool_analog_output_type_copy_));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
            tf_prefix + "gpio", "tool_analog_input", &tool_analog_input_));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
            tf_prefix + "gpio", "tool_analog_output", &tool_analog_output_));

    state_interfaces.emplace_back(
        hardware_interface::StateInterface(tf_prefix + "gpio", "tool_output_voltage", &tool_output_voltage_copy_));

    state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "robot_mode", &robot_mode_copy_));

    state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "safety_mode", &safety_mode_copy_));

    state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "tool_mode", &tool_mode_copy_));

    state_interfaces.emplace_back(
        hardware_interface::StateInterface(tf_prefix + "gpio", "tool_output_current", &tool_output_current_));

    state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "tool_temperature", &tool_temperature_));

    state_interfaces.emplace_back(
        hardware_interface::StateInterface(tf_prefix + "system_interface", "initialized", &system_interface_initialized_));

    state_interfaces.emplace_back(
        hardware_interface::StateInterface(tf_prefix + "gpio", "task_running", &robot_task_running_copy_));

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> EliteCSPositionHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
    }
    // Obtain the tf_prefix from the urdf so that we can have the general interface multiple times
    // NOTE using the tf_prefix at this point is some kind of workaround. One should actually go through the list of gpio
    // command interface in info_ and match them accordingly
    const std::string tf_prefix = info_.hardware_parameters.at("tf_prefix");

    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(tf_prefix + "gpio", "io_async_success", &io_async_success_));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(tf_prefix + "speed_scaling", "target_speed_fraction_cmd",
                                                                         &target_speed_fraction_cmd_));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        tf_prefix + "speed_scaling", "target_speed_fraction_async_success", &scaling_async_success_));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(tf_prefix + "resend_external_script",
                                                                         "resend_external_script_cmd", &resend_external_script_cmd_));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        tf_prefix + "resend_external_script", "resend_external_script_async_success", &resend_external_script_async_success_));
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(tf_prefix + "hand_back_control", "hand_back_control_cmd", &hand_back_control_cmd_));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        tf_prefix + "hand_back_control", "hand_back_control_async_success", &hand_back_control_async_success_));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(tf_prefix + "payload", "mass", &payload_mass_));
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(tf_prefix + "payload", "cog.x", &payload_center_of_gravity_[0]));
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(tf_prefix + "payload", "cog.y", &payload_center_of_gravity_[1]));
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(tf_prefix + "payload", "cog.z", &payload_center_of_gravity_[2]));
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(tf_prefix + "payload", "payload_async_success", &payload_async_success_));

    for (size_t i = 0; i < STANDARD_DIG_GPIO_NUM; ++i) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            tf_prefix + "gpio", "standard_digital_output_cmd_" + std::to_string(i), &standard_dig_out_bits_cmd_[i]));
    }

    for (size_t i = 0; i < CONF_DIG_GPIO_NUM; i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            tf_prefix + "gpio", "configure_digital_output_cmd_" + std::to_string(i), &conf_dig_out_bits_cmd_[i]));
    }

    for (size_t i = 0; i < TOOL_DIG_GPIO_NUM; i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            tf_prefix + "gpio", "tool_digital_output_cmd_" + std::to_string(i), &tool_dig_out_bits_cmd_[i]));
    }
    

    for (size_t i = 0; i < STANARD_ANALOG_IO_NUM; ++i) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            tf_prefix + "gpio", "standard_analog_output_cmd_" + std::to_string(i), &standard_analog_output_cmd_[i]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            tf_prefix + "gpio", "standard_analog_output_type_cmd_" + std::to_string(i), &standard_analog_output_types_cmd_[i]));
    }

    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(tf_prefix + "gpio", "tool_voltage_cmd", &tool_voltage_cmd_));

    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(tf_prefix + "zero_ftsensor", "zero_ftsensor_cmd", &zero_ftsensor_cmd_));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(tf_prefix + "zero_ftsensor", "zero_ftsensor_async_success",
                                                                         &zero_ftsensor_async_success_));

    return command_interfaces;
}

hardware_interface::CallbackReturn EliteCSPositionHardwareInterface::on_configure(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    RCLCPP_INFO(rclcpp::get_logger("EliteCSPositionHardwareInterface"), "Starting ...please wait...");

    // The robot's IP address.
    const std::string robot_ip = info_.hardware_parameters["robot_ip"];
    // Path to the script code that will be sent to the robot
    const std::string script_filename = info_.hardware_parameters["script_filename"];
    // Path to the file containing the recipe used for requesting RTSI outputs.
    const std::string output_recipe_filename = info_.hardware_parameters["output_recipe_filename"];
    // Path to the file containing the recipe used for requesting RTSI inputs.
    const std::string input_recipe_filename = info_.hardware_parameters["input_recipe_filename"];
    // Start robot in headless mode. This does not require the 'External Control' EliteCOs to be running
    // on the robot, but this will send the script to the robot directly. This requires the robot to 
    // run in 'remote-control' mode.
    const bool headless_mode =
        (info_.hardware_parameters["headless_mode"] == "true") || (info_.hardware_parameters["headless_mode"] == "True");
    // Port that will be opened to communicate between the driver and the robot controller.
    const int reverse_port = stoi(info_.hardware_parameters["reverse_port"]);
    // The driver will offer an interface to receive the task's script on this port.
    const int script_sender_port = stoi(info_.hardware_parameters["script_sender_port"]);

    // The ip address of the host the driver runs on
    std::string local_ip = info_.hardware_parameters["local_ip"];

    // Port (on the host pc) of the trajectory interface
    const int trajectory_port = stoi(info_.hardware_parameters["trajectory_port"]);

    // Port (on the host PC) that will be used to forward script commands from the driver to the robot
    const int script_command_port = stoi(info_.hardware_parameters["script_command_port"]);

    // Specify gain for servoing to position in joint space.
    // A higher gain can sharpen the trajectory.
    const int servoj_gain = stoi(info_.hardware_parameters["servoj_gain"]);
    // Specify lookahead time for servoing to position in joint space.
    // A longer lookahead time can smooth the trajectory.
    const double servoj_lookahead_time = stod(info_.hardware_parameters["servoj_lookahead_time"]);
    // Time of servoj run
    const double servoj_time = stod(info_.hardware_parameters["servoj_time"]);

    // const bool use_tool_communication = (info_.hardware_parameters["use_tool_communication"] == "true") ||
    //                                     (info_.hardware_parameters["use_tool_communication"] == "True");

    // Obtain the tf_prefix which is needed for the logging handler so that log messages from different arms are
    // distiguishable in the log
    const std::string tf_prefix = info_.hardware_parameters.at("tf_prefix");
    RCLCPP_INFO(rclcpp::get_logger("EliteCSPositionHardwareInterface"), "Initializing driver...");
    try {
        ELITE::EliteDriverConfig driver_config;
        driver_config.robot_ip = robot_ip;
        driver_config.local_ip = local_ip;
        driver_config.script_file_path = script_filename;
        driver_config.headless_mode = headless_mode;
        driver_config.script_sender_port = script_sender_port;
        driver_config.reverse_port = reverse_port;
        driver_config.trajectory_port = trajectory_port;
        driver_config.script_command_port = script_command_port;
        driver_config.servoj_time = servoj_time;
        driver_config.servoj_lookahead_time = servoj_lookahead_time;
        driver_config.servoj_gain = servoj_gain;
        eli_driver_ = std::make_unique<ELITE::EliteDriver>(driver_config);
        if (rtsiInit(robot_ip, output_recipe_filename, input_recipe_filename)) {
            RCLCPP_INFO(rclcpp::get_logger("EliteCSPositionHardwareInterface"), "RTSI init: 'success'.");
        } else {
            RCLCPP_FATAL(rclcpp::get_logger("EliteCSPositionHardwareInterface"), "RTSI init: 'fail'.");
            return hardware_interface::CallbackReturn::ERROR;
        }
    } catch (ELITE::EliteException& e) {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("EliteCSPositionHardwareInterface"), e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Timeout before the reverse interface will be dropped by the robot
    recv_timeout_ = std::stof(info_.hardware_parameters["robot_receive_timeout"]);

    async_thread_alive_ = true;
    async_thread_ = std::make_unique<std::thread>([&](){
        asyncThread();
    });

    RCLCPP_INFO(rclcpp::get_logger("EliteCSPositionHardwareInterface"), "System successfully started!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EliteCSPositionHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    RCLCPP_INFO(rclcpp::get_logger("EliteCSPositionHardwareInterface"), "Activating HW interface");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EliteCSPositionHardwareInterface::on_cleanup(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    RCLCPP_INFO(rclcpp::get_logger("EliteCSPositionHardwareInterface"), "Stopping ...please wait...");

    if (async_thread_) {
        async_thread_alive_ = false;
        async_thread_->join();
        async_thread_.reset();
    }
    rtsi_interface_.reset();
    eli_driver_.reset();

    RCLCPP_INFO(rclcpp::get_logger("EliteCSPositionHardwareInterface"), "System successfully stopped!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<std::string> EliteCSPositionHardwareInterface::readRecipe(const std::string& recipe_file) {
    std::vector<std::string> recipe;
    std::ifstream file(recipe_file);
    if (file.fail()) {
        std::stringstream msg;
        msg << "Opening file '" << recipe_file << "' failed with error: " << strerror(errno);
        throw ELITE::EliteException(ELITE::EliteException::Code::FILE_OPEN_FAIL, msg.str());
    }

    if (file.peek() == std::ifstream::traits_type::eof()) {
        std::stringstream msg;
        msg << "The recipe '" << recipe_file << "' file is empty exiting ";
        throw ELITE::EliteException(ELITE::EliteException::Code::FILE_OPEN_FAIL, msg.str());
    }

    std::string line;
    while (std::getline(file, line)) {
        recipe.push_back(line);
    }

    return recipe;
}

bool EliteCSPositionHardwareInterface::rtsiInit(const std::string& ip, const std::string& output_file, const std::string& input_file) {
    rtsi_interface_ = std::make_unique<ELITE::RtsiClientInterface>();
    rtsi_interface_->connect(ip);
    
    if (!rtsi_interface_->negotiateProtocolVersion()) {
        RCLCPP_FATAL(rclcpp::get_logger("EliteCSPositionHardwareInterface"), "RTSI check protocol version: 'fail'.");
        return false;
    }
    
    std::vector<std::string> output_recipe = readRecipe(output_file);
    std::vector<std::string> input_recipe = readRecipe(input_file);
    
    rtsi_out_recipe_ = rtsi_interface_->setupOutputRecipe(output_recipe);
    rtsi_in_recipe_ = rtsi_interface_->setupInputRecipe(input_recipe);

    if (!rtsi_interface_->start()) {
        RCLCPP_FATAL(rclcpp::get_logger("EliteCSPositionHardwareInterface"), "RTSI start data sync: 'fail'.");
        return false;
    }
    return true;
}

void EliteCSPositionHardwareInterface::asyncThread() {
    while (async_thread_alive_) {
        updateAsyncIO();
        std::this_thread::sleep_for(40000000ns);
    }
}

hardware_interface::return_type EliteCSPositionHardwareInterface::read(const rclcpp::Time& time, const rclcpp::Duration& period) {
    (void)period;
    (void)time;
    if (!rtsi_interface_->receiveData(rtsi_out_recipe_, true)) {
        RCLCPP_FATAL(rclcpp::get_logger("EliteCSPositionHardwareInterface"), "RTSI receive data: 'fail'.");
        return hardware_interface::return_type::ERROR;
    }
    rtsi_out_recipe_->getValue("actual_joint_positions", joint_positions_);
    rtsi_out_recipe_->getValue("actual_joint_speeds", joint_velocities_);
    rtsi_out_recipe_->getValue("actual_joint_current", joint_efforts_);
    rtsi_out_recipe_->getValue("speed_scaling", speed_scaling_combined_);
    rtsi_out_recipe_->getValue("actual_TCP_force", ft_sensor_measurements_);
    rtsi_out_recipe_->getValue("actual_TCP_pose", tcp_pose_);
    rtsi_out_recipe_->getValue("standard_analog_output0", standard_analog_output_[0]);
    rtsi_out_recipe_->getValue("standard_analog_output1", standard_analog_output_[1]);
    rtsi_out_recipe_->getValue("standard_analog_input0", standard_analog_input_[0]);
    rtsi_out_recipe_->getValue("standard_analog_input1", standard_analog_input_[1]);
    rtsi_out_recipe_->getValue("tool_analog_input", tool_analog_input_);
    rtsi_out_recipe_->getValue("tool_analog_output", tool_analog_output_);
    rtsi_out_recipe_->getValue("tool_output_voltage", tool_output_voltage_copy_);
    rtsi_out_recipe_->getValue("tool_output_current", tool_output_current_);
    rtsi_out_recipe_->getValue("tool_temperature", tool_temperature_);
    
    uint32_t runtime_state = 0;
    rtsi_out_recipe_->getValue("runtime_state", runtime_state);
    runtime_state_ = static_cast<ELITE::TaskStatus>(runtime_state);

    uint32_t tool_mode = 0;
    rtsi_out_recipe_->getValue("tool_mode", tool_mode);
    tool_mode_copy_ = static_cast<double>(tool_mode);

    uint32_t tool_analog_input_type = 0;
    rtsi_out_recipe_->getValue("tool_analog_input_types", tool_analog_input_type);
    tool_analog_input_type_copy_ = static_cast<double>(tool_analog_input_type);

    uint32_t tool_analog_output_type = 0;
    rtsi_out_recipe_->getValue("tool_analog_output_types", tool_analog_output_type);
    tool_analog_output_type_copy_ = static_cast<double>(tool_analog_output_type);

    int32_t robot_mode = 0;
    rtsi_out_recipe_->getValue("robot_mode", robot_mode);
    robot_mode_copy_ = static_cast<double>(robot_mode);

    int32_t safety_mode = 0;
    rtsi_out_recipe_->getValue("safety_status", safety_mode);
    safety_mode_copy_ = static_cast<double>(safety_mode);

    uint32_t analog_types = 0;
    rtsi_out_recipe_->getValue("analog_io_types", analog_types);
    std::bitset<4> analog_types_bits = analog_types;
    standard_analog_input_types_[0] = analog_types_bits[0];
    standard_analog_input_types_[1] = analog_types_bits[1];
    standard_analog_output_types_[0] = analog_types_bits[2];
    standard_analog_output_types_[1] = analog_types_bits[3];

    uint32_t robot_status = 0;
    rtsi_out_recipe_->getValue("robot_status_bits", robot_status);
    std::bitset<4> robot_status_bits = robot_status;
    robot_status_bits_copy_[0] = robot_status_bits[0]; // is power on
    robot_status_bits_copy_[1] = robot_status_bits[1]; // is program running
    robot_status_bits_copy_[2] = robot_status_bits[2]; // is freedrive button pressed
    robot_status_bits_copy_[3] = robot_status_bits[3]; // no use

    uint32_t safety_status = 0;
    rtsi_out_recipe_->getValue("robot_status_bits", robot_status);
    std::bitset<11> safety_status_bits = safety_status;
    for (size_t i = 0; i < SAFETY_STATUS_BITS_NUM; i++) {
        safety_status_bits_copy_[i] = safety_status_bits[i];
    }

    uint32_t input_dig_temp = 0;
    rtsi_out_recipe_->getValue("actual_digital_input_bits", input_dig_temp);
    std::bitset<ALL_DIG_GPIO_NUM> input_dig = input_dig_temp;

    uint32_t output_dig_temp = 0;
    rtsi_out_recipe_->getValue("actual_digital_output_bits", output_dig_temp);
    std::bitset<ALL_DIG_GPIO_NUM> output_dig = output_dig_temp;

    for (size_t i = 0; i < STANDARD_DIG_GPIO_NUM; i++) {
        standard_dig_in_[i] = input_dig[i];
        standard_dig_out_[i] = output_dig[i];
    }
    for (size_t i = 0; i < CONF_DIG_GPIO_NUM; i++) {
        config_dig_in_[i] = input_dig[i + STANDARD_DIG_GPIO_NUM];
        config_dig_out_[i] = output_dig[i + STANDARD_DIG_GPIO_NUM];
    }
    for (size_t i = 0; i < TOOL_DIG_GPIO_NUM; i++) {
        tool_dig_in_[i] = input_dig[i + (STANDARD_DIG_GPIO_NUM + CONF_DIG_GPIO_NUM)];
        tool_dig_out_[i] = output_dig[i + (STANDARD_DIG_GPIO_NUM + CONF_DIG_GPIO_NUM)];
    }

    is_robot_connected_ = eli_driver_->isRobotConnected();
    
    // If power off to power on, init some commands
    if (robot_status_bits[0] && !is_last_power_on_) {
        
        if (!rtsi_interface_->receiveData(rtsi_out_recipe_, true)) {
            RCLCPP_FATAL(rclcpp::get_logger("EliteCSPositionHardwareInterface"), "RTSI receive data: 'fail'.");
            return hardware_interface::return_type::ERROR;
        }
        rtsi_out_recipe_->getValue("actual_joint_positions", joint_positions_);

        position_commands_ = joint_positions_;
        velocity_commands_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
        target_speed_fraction_cmd_ = NO_NEW_CMD;
        resend_external_script_cmd_ = NO_NEW_CMD;
        zero_ftsensor_cmd_ = NO_NEW_CMD;
        hand_back_control_cmd_ = NO_NEW_CMD;
        is_last_power_on_ = true;
        RCLCPP_INFO(rclcpp::get_logger("EliteCSPositionHardwareInterface"), "Power off to power on");
    } else if(!robot_status_bits[0] && is_last_power_on_) {
        is_last_power_on_ = false;
        RCLCPP_INFO(rclcpp::get_logger("EliteCSPositionHardwareInterface"), "Power off");
    }

    // required transforms
    extractToolPose();
    transformForceTorque();

    system_interface_initialized_ = 1.0;
    robot_task_running_copy_ = ((runtime_state_ == ELITE::TaskStatus::PLAYING) &&
                                is_robot_connected_ );

    return hardware_interface::return_type::OK;

}

hardware_interface::return_type EliteCSPositionHardwareInterface::write(const rclcpp::Time& time, const rclcpp::Duration& period) {
    (void)time;
    (void)period;
    // If there is no interpreting task running on the robot, we do not want to send anything.
    if (runtime_state_ == ELITE::TaskStatus::PLAYING && is_robot_connected_) {
        if (position_controller_running_) {
            eli_driver_->writeServoj(position_commands_, recv_timeout_ * 1000);

        } else if (velocity_controller_running_) {
            eli_driver_->writeSpeedj(velocity_commands_, recv_timeout_ * 1000);

        } else {
            eli_driver_->writeIdle(1000);
        }
    }

    return hardware_interface::return_type::OK;
}

bool EliteCSPositionHardwareInterface::updateStandardIO(bool *is_update) {
    // Standard digital output
    std::bitset<STANDARD_DIG_GPIO_NUM> standard_digital_output_mask_bits = 0;
    std::bitset<STANDARD_DIG_GPIO_NUM> standard_digital_output_bits = 0;
    bool need_update = false;
    for (size_t i = 0; i < STANDARD_DIG_GPIO_NUM; i++) {    
        if (!std::isnan(standard_dig_out_bits_cmd_[i]) && rtsi_in_recipe_ != nullptr) {
            standard_digital_output_mask_bits[i] = true;
            standard_digital_output_bits[i] = static_cast<bool>(standard_dig_out_bits_cmd_[i]);
            need_update = true;
            *is_update = true;
        }
        standard_dig_out_bits_cmd_[i] = NO_NEW_CMD;
    }
    if (need_update) {
        uint16_t standard_digital_output_mask = static_cast<uint16_t>(standard_digital_output_mask_bits.to_ulong());
        if(!rtsi_in_recipe_->setValue("standard_digital_output_mask", standard_digital_output_mask)) {
            return false;
        }

        uint16_t standard_digital_output = static_cast<uint16_t>(standard_digital_output_bits.to_ulong());
        if(!rtsi_in_recipe_->setValue("standard_digital_output", standard_digital_output)) {
            return false;
        }
    }
    return true;
}

bool EliteCSPositionHardwareInterface::updateConfigIO(bool *is_update) {
    // Configure digital output
    std::bitset<CONF_DIG_GPIO_NUM> config_digital_output_mask_bits = 0;
    std::bitset<CONF_DIG_GPIO_NUM> config_digital_output_bits = 0;
    bool need_update = false;
    for (size_t i = 0; i < CONF_DIG_GPIO_NUM; i++) {
        if (!std::isnan(conf_dig_out_bits_cmd_[i]) && rtsi_in_recipe_ != nullptr) {
            config_digital_output_mask_bits[i] = true;
            config_digital_output_bits[i] = static_cast<bool>(conf_dig_out_bits_cmd_[i]);
            need_update = true;
            *is_update = true;
        }
        conf_dig_out_bits_cmd_[i] = NO_NEW_CMD;
    }
    if(need_update) {
        uint8_t config_digital_output_mask = static_cast<uint8_t>(config_digital_output_mask_bits.to_ulong());
        if (!rtsi_in_recipe_->setValue("configurable_digital_output_mask", config_digital_output_mask)) {
            return false;
        }

        uint8_t config_digital_output = static_cast<uint8_t>(config_digital_output_bits.to_ulong());
        rtsi_in_recipe_->setValue("configurable_digital_output", config_digital_output);
    }
    return true;
}

bool EliteCSPositionHardwareInterface::updateToolDigital(bool *is_update) {
    // Tool digital IO
    std::bitset<CONF_DIG_GPIO_NUM> tool_digital_output_mask_bits = 0;
    std::bitset<CONF_DIG_GPIO_NUM> tool_digital_output_bits = 0;
    bool need_update = false;
    for (size_t i = 0; i < TOOL_DIG_GPIO_NUM; i++) {
        if (!std::isnan(tool_dig_out_bits_cmd_[i]) && rtsi_in_recipe_ != nullptr) {
            tool_digital_output_mask_bits[i] = true;
            tool_digital_output_bits[i] = static_cast<bool>(tool_dig_out_bits_cmd_[i]);
            need_update = true;
            *is_update = true;
        }
        tool_dig_out_bits_cmd_[i] = NO_NEW_CMD;
    }
    if (need_update) {
        uint8_t tool_digital_output_mask = static_cast<uint8_t>(tool_digital_output_mask_bits.to_ulong());
        if(!rtsi_in_recipe_->setValue("tool_digital_output_mask", tool_digital_output_mask)) {
            return false;
        }

        uint8_t tool_digital_output = static_cast<uint8_t>(tool_digital_output_bits.to_ulong());
        if(!rtsi_in_recipe_->setValue("tool_digital_output", tool_digital_output)) {
            return false;
        }
    }
    return true;
}

bool EliteCSPositionHardwareInterface::updateStandardAnalog(bool *is_update) {
    // Standard analog output
    std::bitset<STANARD_ANALOG_IO_NUM> standard_analog_output_type_bit = 0;
    std::bitset<STANARD_ANALOG_IO_NUM> standard_analog_output_mask_bit = 0;
    bool need_update = false;
    for (size_t i = 0; i < STANARD_ANALOG_IO_NUM; ++i) {
        if (!std::isnan(standard_analog_output_types_cmd_[i])) {
            standard_analog_output_type_bit[i] = static_cast<bool>(standard_analog_output_types_cmd_[i]);
            standard_analog_output_types_cmd_[i] = NO_NEW_CMD;
            need_update = true;
            *is_update = true;
        }
        if (!std::isnan(standard_analog_output_cmd_[i])) {
            standard_analog_output_mask_bit[i] = true;
            if(!rtsi_in_recipe_->setValue("standard_analog_output_" + std::to_string(i), standard_analog_output_cmd_[i])) {
                return false;
            }
            standard_analog_output_cmd_[i] = NO_NEW_CMD;
            need_update = true;
            *is_update = true;
        }
    }
    if (need_update) {
        int32_t standard_analog_output_type = static_cast<int32_t>(standard_analog_output_type_bit.to_ulong());
        if(!rtsi_in_recipe_->setValue("standard_analog_output_type", standard_analog_output_type)) {
            return false;
        }

        int32_t standard_analog_output_mask = static_cast<int32_t>(standard_analog_output_mask_bit.to_ulong());
        if(rtsi_in_recipe_->setValue("standard_analog_output_mask", standard_analog_output_mask)) {
            return false;
        }
    }
    return true;
}

bool EliteCSPositionHardwareInterface::updateToolVoltage(bool *is_update) {
    // Tool voltage
    bool ret = true;
    if (!std::isnan(tool_voltage_cmd_) && rtsi_interface_ != nullptr) {
        ret = eli_driver_->setToolVoltage(static_cast<ELITE::ToolVoltage>(tool_voltage_cmd_));
        tool_voltage_cmd_ = NO_NEW_CMD;
        *is_update = true;
    }
    return ret;
}

void EliteCSPositionHardwareInterface::updateAsyncIO() {
    if (rtsi_interface_ && !rtsi_interface_->isStarted() && rtsi_in_recipe_) {
        return;
    }

    bool is_io_update = false;
    io_async_success_ = updateStandardIO(&is_io_update) ? io_async_success_ : false;
    io_async_success_ = updateConfigIO(&is_io_update) ? io_async_success_ : false;
    io_async_success_ = updateToolDigital(&is_io_update) ? io_async_success_ : false;
    io_async_success_ = updateStandardAnalog(&is_io_update) ? io_async_success_ : false;
    io_async_success_ = updateToolVoltage(&is_io_update) ? io_async_success_ : false;

    if (is_io_update && io_async_success_ != false) {
        io_async_success_ = true;
    }
    

    if (!std::isnan(target_speed_fraction_cmd_) && rtsi_interface_ != nullptr) {
        if (rtsi_in_recipe_->setValue("speed_slider_mask", (int)1) &&
            rtsi_in_recipe_->setValue("speed_slider_fraction", target_speed_fraction_cmd_)) {
                scaling_async_success_ = true;
        }
        target_speed_fraction_cmd_ = NO_NEW_CMD;
    }

    if (!std::isnan(resend_external_script_cmd_) && eli_driver_ != nullptr) {
        try {
            eli_driver_->sendScript("stop task\n");
            while (runtime_state_ != ELITE::TaskStatus::STOPPED) {
                std::this_thread::sleep_for(1ms);
            }
            resend_external_script_async_success_ = eli_driver_->sendExternalControlScript();
            RCLCPP_INFO(rclcpp::get_logger("EliteCSPositionHardwareInterface"), 
                "Send external control: %s", resend_external_script_async_success_ ? "success" : "fail");
        } catch (const ELITE::EliteException& e) {
            RCLCPP_ERROR(rclcpp::get_logger("EliteCSPositionHardwareInterface"), "Service Call failed: '%s'", e.what());
        }
        resend_external_script_cmd_ = NO_NEW_CMD;
    }

    if (!std::isnan(hand_back_control_cmd_) && eli_driver_ != nullptr) {
        eli_driver_->stopControl();
        hand_back_control_async_success_ = true;
        hand_back_control_cmd_ = NO_NEW_CMD;
    }

    if (!std::isnan(payload_mass_) && !std::isnan(payload_center_of_gravity_[0]) && !std::isnan(payload_center_of_gravity_[1]) &&
        !std::isnan(payload_center_of_gravity_[2]) && eli_driver_ != nullptr) {
            payload_async_success_ = eli_driver_->setPayload(payload_mass_, payload_center_of_gravity_);
            payload_mass_ = NO_NEW_CMD;
            payload_center_of_gravity_ = {NO_NEW_CMD, NO_NEW_CMD, NO_NEW_CMD};
    }

    if (!std::isnan(zero_ftsensor_cmd_) && eli_driver_ != nullptr) {
        zero_ftsensor_async_success_ = eli_driver_->zeroFTSensor();
        zero_ftsensor_cmd_ = NO_NEW_CMD;
    }
    if (rtsi_interface_->isConnected()) {
        rtsi_interface_->send(rtsi_in_recipe_);
    }
}


void EliteCSPositionHardwareInterface::transformForceTorque() {
    tcp_force_.setValue(ft_sensor_measurements_[0], ft_sensor_measurements_[1], ft_sensor_measurements_[2]);
    tcp_torque_.setValue(ft_sensor_measurements_[3], ft_sensor_measurements_[4], ft_sensor_measurements_[5]);

    tf2::Quaternion rotation_quat;
    tf2::fromMsg(tcp_transform_.transform.rotation, rotation_quat);
    tcp_force_ = tf2::quatRotate(rotation_quat.inverse(), tcp_force_);
    tcp_torque_ = tf2::quatRotate(rotation_quat.inverse(), tcp_torque_);

    ft_sensor_measurements_ = {tcp_force_.x(),  tcp_force_.y(),  tcp_force_.z(),
                               tcp_torque_.x(), tcp_torque_.y(), tcp_torque_.z()};
}

void EliteCSPositionHardwareInterface::extractToolPose() {
    double tcp_angle = std::sqrt(std::pow(tcp_pose_[3], 2) + std::pow(tcp_pose_[4], 2) + std::pow(tcp_pose_[5], 2));

    tf2::Vector3 rotation_vec(tcp_pose_[3], tcp_pose_[4], tcp_pose_[5]);
    tf2::Quaternion rotation;
    if (tcp_angle > 1e-16) {
        rotation.setRotation(rotation_vec.normalized(), tcp_angle);
    } else {
        rotation.setValue(0.0, 0.0, 0.0, 1.0);  // default Quaternion is 0,0,0,0 which is invalid
    }
    tcp_transform_.transform.translation.x = tcp_pose_[0];
    tcp_transform_.transform.translation.y = tcp_pose_[1];
    tcp_transform_.transform.translation.z = tcp_pose_[2];

    tcp_transform_.transform.rotation = tf2::toMsg(rotation);
}

hardware_interface::return_type EliteCSPositionHardwareInterface::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces) {
    hardware_interface::return_type ret_val = hardware_interface::return_type::OK;

    start_modes_.clear();
    stop_modes_.clear();

    // Starting interfaces
    // add start interface per joint in tmp var for later check
    for (const auto& key : start_interfaces) {
        for (auto i = 0u; i < info_.joints.size(); i++) {
            if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
                start_modes_.push_back(hardware_interface::HW_IF_POSITION);
            }
            if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) {
                start_modes_.push_back(hardware_interface::HW_IF_VELOCITY);
            }
        }
    }
    // set new mode to all interfaces at the same time
    if (start_modes_.size() != 0 && start_modes_.size() != 6) {
        ret_val = hardware_interface::return_type::ERROR;
    }

    // all start interfaces must be the same - can't mix position and velocity control
    if (start_modes_.size() != 0 && !std::equal(start_modes_.begin() + 1, start_modes_.end(), start_modes_.begin())) {
        ret_val = hardware_interface::return_type::ERROR;
    }

    // Stopping interfaces
    // add stop interface per joint in tmp var for later check
    for (const auto& key : stop_interfaces) {
        for (auto i = 0u; i < info_.joints.size(); i++) {
            if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
                stop_modes_.push_back(StoppingInterface::STOP_POSITION);
            }
            if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) {
                stop_modes_.push_back(StoppingInterface::STOP_VELOCITY);
            }
        }
    }
    // stop all interfaces at the same time
    if (stop_modes_.size() != 0 &&
        (stop_modes_.size() != 6 || !std::equal(stop_modes_.begin() + 1, stop_modes_.end(), stop_modes_.begin()))) {
        ret_val = hardware_interface::return_type::ERROR;
    }

    controllers_initialized_ = true;
    return ret_val;
}

hardware_interface::return_type EliteCSPositionHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces) {
    (void)start_interfaces;
    (void)stop_interfaces;
    hardware_interface::return_type ret_val = hardware_interface::return_type::OK;

    if (stop_modes_.size() != 0 &&
        std::find(stop_modes_.begin(), stop_modes_.end(), StoppingInterface::STOP_POSITION) != stop_modes_.end()) {
        position_controller_running_ = false;
        position_commands_ = joint_positions_;
    } else if (stop_modes_.size() != 0 &&
               std::find(stop_modes_.begin(), stop_modes_.end(), StoppingInterface::STOP_VELOCITY) != stop_modes_.end()) {
        velocity_controller_running_ = false;
        velocity_commands_ = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    }

    if (start_modes_.size() != 0 &&
        std::find(start_modes_.begin(), start_modes_.end(), hardware_interface::HW_IF_POSITION) != start_modes_.end()) {
        velocity_controller_running_ = false;
        position_commands_ = joint_positions_;
        position_controller_running_ = true;

    } else if (start_modes_.size() != 0 &&
               std::find(start_modes_.begin(), start_modes_.end(), hardware_interface::HW_IF_VELOCITY) != start_modes_.end()) {
        position_controller_running_ = false;
        velocity_commands_ = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
        velocity_controller_running_ = true;
    }

    start_modes_.clear();
    stop_modes_.clear();

    return ret_val;
}
}  // namespace ELITE_CS_ROBOT_ROS_DRIVER

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ELITE_CS_ROBOT_ROS_DRIVER::EliteCSPositionHardwareInterface, hardware_interface::SystemInterface)
