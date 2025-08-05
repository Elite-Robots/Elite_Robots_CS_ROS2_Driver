#ifndef _ELITE_CS_CONTROLLER__SPEED_SCALING_STATE_BROADCASTER_HPP_
#define _ELITE_CS_CONTROLLER__SPEED_SCALING_STATE_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include <eli_cs_controllers/speed_scaling_state_broadcaster_parameters.hpp>
#include "std_msgs/msg/float64.hpp"

namespace ELITE_CS_CONTROLLER {
class SpeedScalingStateBroadcaster : public controller_interface::ControllerInterface {
public:
    SpeedScalingStateBroadcaster();

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    controller_interface::CallbackReturn on_init() override;

protected:
    std::vector<std::string> sensor_names_;
    double publish_rate_;

    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> speed_scaling_state_publisher_;
    std_msgs::msg::Float64 speed_scaling_state_msg_;

    // Parameters from ROS for SpeedScalingStateBroadcaster
    std::shared_ptr<speed_scaling_state_broadcaster::ParamListener> param_listener_;
    speed_scaling_state_broadcaster::Params params_;
};
} // namespace ELITE_CS_CONTROLLER
#endif
