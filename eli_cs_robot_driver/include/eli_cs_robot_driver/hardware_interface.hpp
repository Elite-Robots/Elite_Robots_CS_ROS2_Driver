#ifndef __ELITE_CS_ROBOT_ROS_DRIVER__HARDWARE_INTERFACE_HPP__
#define __ELITE_CS_ROBOT_ROS_DRIVER__HARDWARE_INTERFACE_HPP__

// System
#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

// ROS
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// ros2_control hardware_interface
#include <hardware_interface/visibility_control.h>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>

#include <Elite/EliteDriver.hpp>
#include <Elite/RtsiIOInterface.hpp>

namespace ELITE_CS_ROBOT_ROS_DRIVER {

// Other command mode
constexpr char ELITE_HW_IF_FREEDRIVE[] = "freedrive_mode";


struct Quaternion {
    Quaternion() : x(0), y(0), z(0), w(0) { }

    void set(const tf2::Quaternion& q) {
        x = q.x();
        y = q.y();
        z = q.z();
        w = q.w();
    }

    double x;
    double y;
    double z;
    double w;
};

/**
 * @brief Handles the interface between the ROS system and the main driver.
 *
 */
class EliteCSPositionHardwareInterface : public hardware_interface::SystemInterface {
   public:
    RCLCPP_SHARED_PTR_DEFINITIONS(EliteCSPositionHardwareInterface)
    virtual ~EliteCSPositionHardwareInterface();

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& system_info) final;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() final;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() final;

    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) final;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) final;
    hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) final;

    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) final;
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) final;

    hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                                const std::vector<std::string>& stop_interfaces) final;

    hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                                const std::vector<std::string>& stop_interfaces) final;

   protected:
    ELITE::EliteDriverConfig driver_config_;
    std::unique_ptr<ELITE::EliteDriver> eli_driver_;
    std::unique_ptr<ELITE::RtsiIOInterface> rtsi_interface_;
    std::unique_ptr<std::thread> async_thread_;
    bool async_thread_alive_;
    ELITE::TaskStatus runtime_state_;
    // resources switching aux vars
    std::vector<std::vector<std::string>> stop_modes_;
    std::vector<std::vector<std::string>> start_modes_;
    bool position_controller_running_;
    bool velocity_controller_running_;
    bool controllers_initialized_;

    bool is_robot_connected_;
    bool is_last_power_on_;

    // param
    double recv_timeout_;

    static constexpr int STANDARD_DIG_GPIO_NUM = 16;
    static constexpr int CONF_DIG_GPIO_NUM = 8;
    static constexpr int TOOL_DIG_GPIO_NUM = 4;
    static constexpr int SAFETY_STATUS_BITS_NUM = 11;
    static constexpr int STANARD_ANALOG_IO_NUM = 2;
    static constexpr int ROBOT_STATUS_BITS_NUM = 4;
    static constexpr int ALL_DIG_GPIO_NUM = STANDARD_DIG_GPIO_NUM + CONF_DIG_GPIO_NUM + TOOL_DIG_GPIO_NUM;
    static constexpr double NO_NEW_CMD = std::numeric_limits<double>::quiet_NaN();

    // command
    ELITE::vector6d_t position_commands_;
    ELITE::vector6d_t velocity_commands_;
    ELITE::vector6d_t joint_positions_;
    ELITE::vector6d_t joint_velocities_;
    ELITE::vector6d_t joint_efforts_;
    ELITE::vector6d_t ft_sensor_measurements_;
    ELITE::vector6d_t tcp_pose_;
    tf2::Quaternion tcp_rotation_quat_;
    Quaternion tcp_rotation_quat_buffer_;
    double tool_analog_input_;
    double tool_analog_output_;
    std::array<double, STANARD_ANALOG_IO_NUM> standard_analog_input_types_;
    std::array<double, STANARD_ANALOG_IO_NUM> standard_analog_input_;
    std::array<double, STANARD_ANALOG_IO_NUM> standard_analog_output_types_;
    std::array<double, STANARD_ANALOG_IO_NUM> standard_analog_output_;
    double speed_scaling_combined_;
    double tool_output_voltage_copy_;
    double tool_output_current_;
    double tool_temperature_;

    // copy of non double values
    std::array<double, STANDARD_DIG_GPIO_NUM> standard_dig_out_;
    std::array<double, STANDARD_DIG_GPIO_NUM> standard_dig_in_;
    std::array<double, CONF_DIG_GPIO_NUM> config_dig_out_;
    std::array<double, CONF_DIG_GPIO_NUM> config_dig_in_;
    std::array<double, TOOL_DIG_GPIO_NUM> tool_dig_out_;
    std::array<double, TOOL_DIG_GPIO_NUM> tool_dig_in_;

    std::array<double, SAFETY_STATUS_BITS_NUM> safety_status_bits_copy_;
    std::array<double, ROBOT_STATUS_BITS_NUM> robot_status_bits_copy_;

    double tool_analog_input_type_copy_;
    double tool_analog_output_type_copy_;
    double robot_mode_copy_;
    double safety_mode_copy_;
    double tool_mode_copy_;
    double system_interface_initialized_;
    double robot_task_running_copy_;
    double io_async_success_;
    double target_speed_fraction_cmd_;
    double scaling_async_success_;
    double resend_external_script_cmd_;
    double resend_external_script_async_success_;
    double hand_back_control_cmd_;
    double hand_back_control_async_success_;

    // payload stuff
    ELITE::vector3d_t payload_center_of_gravity_;
    double payload_mass_;
    double payload_async_success_;

    std::array<double, STANDARD_DIG_GPIO_NUM> standard_dig_out_bits_cmd_;
    std::array<double, CONF_DIG_GPIO_NUM> conf_dig_out_bits_cmd_;
    std::array<double, CONF_DIG_GPIO_NUM> tool_dig_out_bits_cmd_;
    std::array<double, STANARD_ANALOG_IO_NUM> standard_analog_output_cmd_;
    std::array<double, STANARD_ANALOG_IO_NUM> standard_analog_output_types_cmd_;
    double tool_voltage_cmd_;
    double zero_ftsensor_cmd_;
    double zero_ftsensor_async_success_;

    // Freedrive interface values
    bool freedrive_activated_;
    bool freedrive_controller_running_;
    double freedrive_async_success_;
    double freedrive_start_cmd_;
    double freedrive_end_cmd_;

    // transform stuff
    tf2::Vector3 tcp_force_;
    tf2::Vector3 tcp_torque_;
    geometry_msgs::msg::TransformStamped tcp_transform_;

    void asyncThread();
    void updateAsyncIO();
    bool updateStandardIO();
    bool updateConfigIO();
    bool updateToolDigital();
    bool updateStandardAnalog();
    bool updateToolVoltage();

    void extractToolPose();
    void transformForceTorque();
    bool rtsiInit(const std::string& ip);

    template <const char*... Args>
    bool containsAnyOfString(const std::vector<std::string>& input) {
        return std::any_of(input.begin(), input.end(), [&](const std::string& item) { return ((item == Args) || ...); });
    }

    template <typename T>
    void removeVectorElements(std::vector<T>& vec, const T& key) {
        vec.erase(std::remove_if(vec.begin(), vec.end(), [&](const T& item) { return item == key; }), vec.end());
    }
};
}  // namespace ELITE_CS_ROBOT_ROS_DRIVER

#endif
