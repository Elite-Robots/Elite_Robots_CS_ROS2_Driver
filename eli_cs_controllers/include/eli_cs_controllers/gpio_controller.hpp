#ifndef _ELITE_CS_CONTROLLER__GPIO_CONTROLLER_HPP_
#define _ELITE_CS_CONTROLLER__GPIO_CONTROLLER_HPP_

#include "eli_common_interface/msg/io_state.hpp"
#include "eli_common_interface/msg/tool_data.hpp"
#include "eli_common_interface/msg/robot_mode.hpp"
#include "eli_common_interface/msg/safety_mode.hpp"
#include "eli_common_interface/srv/set_io.hpp"
#include "eli_common_interface/srv/set_speed_slider_fraction.hpp"
#include "eli_common_interface/srv/set_payload.hpp"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <std_srvs/srv/trigger.hpp>
#include <controller_interface/controller_interface.hpp>
#include <eli_cs_controllers/gpio_controller_parameters.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/bool.hpp>

namespace ELITE_CS_CONTROLLER {



class GPIOController : public controller_interface::ControllerInterface {
public:
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    CallbackReturn on_init() override;

    

private:
    static constexpr int STANDARD_DIG_GPIO_NUM = 16;
    static constexpr int CONF_DIG_GPIO_NUM = 8;
    static constexpr int TOOL_DIG_GPIO_NUM = 4;
    static constexpr int SAFETY_STATUS_BITS_NUM = 11;
    static constexpr int STANARD_ANALOG_IO_NUM = 2;
    static constexpr int ROBOT_STATUS_BITS_NUM = 4;
    static constexpr int ALL_DIG_GPIO_NUM = STANDARD_DIG_GPIO_NUM + CONF_DIG_GPIO_NUM + TOOL_DIG_GPIO_NUM;
    
    enum class StateOffset {
        STANDARD_DIG_OUT = 0,
        CONFIGURE_DIG_OUT = STANDARD_DIG_GPIO_NUM,
        TOOL_DIG_OUT = CONFIGURE_DIG_OUT + CONF_DIG_GPIO_NUM,
        STANDARD_DIG_IN = ALL_DIG_GPIO_NUM,
        CONFIGURE_DIG_IN = STANDARD_DIG_IN + STANDARD_DIG_GPIO_NUM,
        TOOL_DIG_IN = CONFIGURE_DIG_IN + CONF_DIG_GPIO_NUM,
        STANDARD_ANALOG_OUT_TYPE1 = TOOL_DIG_IN + TOOL_DIG_GPIO_NUM,
        STANDARD_ANALOG_OUT1,
        STANDARD_ANALOG_OUT_TYPE2,
        STANDARD_ANALOG_OUT2,
        STANDARD_ANALOG_IN_TYPE1,
        STANDARD_ANALOG_IN1,
        STANDARD_ANALOG_IN_TYPE2,
        STANDARD_ANALOG_IN2,
        TOOL_MODE,
        TOOL_OUTPUT_VOLTAGE,
        TOOL_OUTPUT_CURRENT,
        TOOL_TEMPERATURE,
        TOOL_ANALOG_INPUT_TYPE,
        TOOL_ANALOG_INPUT,
        TOOL_ANALOG_OUTPUT_TYPE,
        TOOL_ANALOG_OUTPUT,
        ROBOT_MODE,
        ROBOT_STATUS_BITS,
        SAFETY_MODE = ROBOT_STATUS_BITS + ROBOT_STATUS_BITS_NUM,
        SAFETY_STATUS_BITS,
        INITIALIZED_FLAG = SAFETY_STATUS_BITS + SAFETY_STATUS_BITS_NUM,
        TASK_RUNNING
    };

    enum class CommandOffset {
        STANDARD_DIG_OUT = 0,
        CONFIGURE_DIG_OUT = STANDARD_DIG_GPIO_NUM,
        TOOL_DIG_OUT = CONFIGURE_DIG_OUT + CONF_DIG_GPIO_NUM,
        STANDARD_ANALOG_OUT = TOOL_DIG_OUT + TOOL_DIG_GPIO_NUM,
        TOOL_VOLTAGE = STANDARD_ANALOG_OUT + (STANARD_ANALOG_IO_NUM * 2),
        IO_ASYNC_SUCCESS,
        TARGET_SPEED_FRACTION,
        TARGET_SPEED_FRACTION_SUCCESS,
        RESEND_ROBOT_CONTROL_SCRIPT,
        RESEND_ROBOT_CONTROL_SCRIPT_SUCCESS,
        MASS,
        COG_X,
        COG_Y,
        COG_Z,
        PAYLOAD_SUCCESS,
        ZERO_FTSENSOR,
        ZERO_FTSENSOR_SUCCESS,
        HAND_BACK_CONTROL,
        HAND_BACK_CONTROL_SUCCESS
    };

    bool setIO(eli_common_interface::srv::SetIO::Request::SharedPtr req, eli_common_interface::srv::SetIO::Response::SharedPtr resp);

    bool setSpeedSlider(eli_common_interface::srv::SetSpeedSliderFraction::Request::SharedPtr req,
                        eli_common_interface::srv::SetSpeedSliderFraction::Response::SharedPtr resp);

    bool resendRobotControlScript(std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr resp);

    bool handBackControl(std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr resp);

    bool setPayload(const eli_common_interface::srv::SetPayload::Request::SharedPtr req, eli_common_interface::srv::SetPayload::Response::SharedPtr resp);

    bool zeroFTSensor(std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr resp);

    void publishIO();

    void publishToolData();

    void publishRobotMode();

    void publishSafetyMode();

    void publishTaskRunning();

   protected:
    void initMsgs();

    bool first_pass_;

    // services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resend_external_script_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr hand_back_control_srv_;
    rclcpp::Service<eli_common_interface::srv::SetSpeedSliderFraction>::SharedPtr set_speed_slider_srv_;
    rclcpp::Service<eli_common_interface::srv::SetIO>::SharedPtr set_io_srv_;
    rclcpp::Service<eli_common_interface::srv::SetPayload>::SharedPtr set_payload_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr tare_sensor_srv_;

    std::shared_ptr<rclcpp::Publisher<eli_common_interface::msg::IOState>> io_pub_;
    std::shared_ptr<rclcpp::Publisher<eli_common_interface::msg::ToolData>> tool_data_pub_;
    std::shared_ptr<rclcpp::Publisher<eli_common_interface::msg::RobotMode>> robot_mode_pub_;
    std::shared_ptr<rclcpp::Publisher<eli_common_interface::msg::SafetyMode>> safety_mode_pub_;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>> task_state_pub_;

    eli_common_interface::msg::IOState io_msg_;
    eli_common_interface::msg::ToolData tool_data_msg_;
    eli_common_interface::msg::RobotMode robot_mode_msg_;
    eli_common_interface::msg::SafetyMode safety_mode_msg_;
    std_msgs::msg::Bool task_running_msg_;

    // Parameters from ROS for gpio_controller
    std::shared_ptr<gpio_controller::ParamListener> param_listener_;
    gpio_controller::Params params_;

    static constexpr double ASYNC_WAITING = 2.0;

    /**
     * @brief wait until a command interface isn't in state ASYNC_WAITING anymore or until the parameter maximum_retries
     * have been reached
     */
    bool waitForAsyncCommand(std::function<double(void)> get_value);
};
}  // namespace ELITE_CS_CONTROLLER

#endif
