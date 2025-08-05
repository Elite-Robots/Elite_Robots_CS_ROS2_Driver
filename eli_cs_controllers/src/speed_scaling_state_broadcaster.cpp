#include "eli_cs_controllers/speed_scaling_state_broadcaster.hpp"

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/qos_event.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcpputils/split.hpp"
#include "rcutils/logging_macros.h"


namespace ELITE_CS_CONTROLLER {
SpeedScalingStateBroadcaster::SpeedScalingStateBroadcaster() {}

controller_interface::CallbackReturn SpeedScalingStateBroadcaster::on_init() {
    try {
        // Create the parameter listener and get the parameters
        param_listener_ = std::make_shared<speed_scaling_state_broadcaster::ParamListener>(get_node());
        params_ = param_listener_->get_params();

        RCLCPP_INFO(get_node()->get_logger(), "Loading Elite SpeedScalingStateBroadcaster with tf_prefix: %s",
                    params_.tf_prefix.c_str());

    } catch (std::exception& e) {
        fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration SpeedScalingStateBroadcaster::command_interface_configuration() const {
    return controller_interface::InterfaceConfiguration{controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration SpeedScalingStateBroadcaster::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    const std::string tf_prefix = params_.tf_prefix;
    config.names.push_back(tf_prefix + "speed_scaling/speed_scaling_factor");
    return config;
}

controller_interface::CallbackReturn SpeedScalingStateBroadcaster::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
    if (!param_listener_) {
        RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
        return controller_interface::CallbackReturn::ERROR;
    }

    // update the dynamic map parameters
    param_listener_->refresh_dynamic_parameters();

    // get parameters from the listener in case they were updated
    params_ = param_listener_->get_params();

    publish_rate_ = params_.state_publish_rate;

    RCLCPP_INFO(get_node()->get_logger(), "Publisher rate set to : %.1f Hz", publish_rate_);

    try {
        speed_scaling_state_publisher_ =
            get_node()->create_publisher<std_msgs::msg::Float64>("~/speed_scaling", rclcpp::SystemDefaultsQoS());
    } catch (const std::exception& e) {
        // get_node() may throw, logging raw here
        fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SpeedScalingStateBroadcaster::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SpeedScalingStateBroadcaster::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type SpeedScalingStateBroadcaster::update(const rclcpp::Time& /*time*/,
                                                                       const rclcpp::Duration& period) {
    if (publish_rate_ > 0.0 && period > rclcpp::Duration(1.0 / publish_rate_, 0.0)) {
        // Speed scaling is the only interface of the controller
        speed_scaling_state_msg_.data = state_interfaces_[0].get_value() * 100.0;

        // publish
        speed_scaling_state_publisher_->publish(speed_scaling_state_msg_);
    }
    return controller_interface::return_type::OK;
}

}  // namespace ELITE_CS_CONTROLLER

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ELITE_CS_CONTROLLER::SpeedScalingStateBroadcaster, controller_interface::ControllerInterface)
