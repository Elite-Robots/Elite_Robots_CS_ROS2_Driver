#include "eli_cs_controllers/freedrive_controller.hpp"

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/logging.hpp>

namespace ELITE_CS_CONTROLLER {

controller_interface::CallbackReturn FreedriveController::on_init() {
    try {
        // Create the parameter listener and get the parameters
        freedrive_param_listener_ = std::make_shared<freedrive_controller::ParamListener>(get_node());
        freedrive_params_ = freedrive_param_listener_->get_params();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
        return CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration FreedriveController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    const std::string tf_prefix = freedrive_params_.tf_prefix;

    // Get the command interfaces needed for freedrive mode from the hardware interface
    config.names.emplace_back(tf_prefix + "freedrive_mode/freedrive_async_success");
    config.names.emplace_back(tf_prefix + "freedrive_mode/freedrive_start_cmd");
    config.names.emplace_back(tf_prefix + "freedrive_mode/freedrive_end_cmd");

    return config;
}

controller_interface::InterfaceConfiguration FreedriveController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::NONE;
    return config;
}

controller_interface::CallbackReturn FreedriveController::on_configure(const rclcpp_lifecycle::State& previous_state) {
    auto cmd_cb = [&](const std_msgs::msg::Bool::SharedPtr msg) {
        if (get_node()->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
            if (msg->data) {
                if ((!is_freedrive_active_) && (!is_new_request_)) {
                    is_freedrive_active_ = true;
                    is_new_request_ = true;
                }
            } else {
                if (is_freedrive_active_ && (!is_new_request_)) {
                    is_freedrive_active_ = false;
                    is_new_request_ = true;
                }
            }
        }
    };
    freedrive_sub_ = get_node()->create_subscription<std_msgs::msg::Bool>("~/enable_freedrive", 10, cmd_cb);

    if (!freedrive_param_listener_) {
        RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during configuration");
        return controller_interface::CallbackReturn::ERROR;
    }

    // Update the dynamic map parameters
    freedrive_param_listener_->refresh_dynamic_parameters();

    // Get parameters from the listener in case they were updated
    freedrive_params_ = freedrive_param_listener_->get_params();

    return ControllerInterface::on_configure(previous_state);
}

bool FreedriveController::setReferenceWrapper(
    std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>& wapper, const std::string& interface_name) {
    auto it = std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
                           [&](auto& interface) { return (interface.get_name() == interface_name); });
    if (it != command_interfaces_.end()) {
        wapper = *it;
    } else {
        RCLCPP_ERROR(get_node()->get_logger(), "Did not find '%s' in command interfaces.", interface_name.c_str());
        return false;
    }
    return true;
}

controller_interface::CallbackReturn FreedriveController::on_activate(const rclcpp_lifecycle::State& state) {
    is_new_request_ = false;
    is_freedrive_active_ = false;

    if (!setReferenceWrapper(async_success_interface_, freedrive_params_.tf_prefix + "freedrive_mode/freedrive_async_success")) {
        return controller_interface::CallbackReturn::ERROR;
    }

    if (!setReferenceWrapper(start_interface_, freedrive_params_.tf_prefix + "freedrive_mode/freedrive_start_cmd")) {
        return controller_interface::CallbackReturn::ERROR;
    }

    if (!setReferenceWrapper(end_interface_, freedrive_params_.tf_prefix + "freedrive_mode/freedrive_end_cmd")) {
        return controller_interface::CallbackReturn::ERROR;
    }

    return ControllerInterface::on_activate(state);
}

controller_interface::CallbackReturn FreedriveController::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/) {
    if (end_interface_.has_value()) {
        end_interface_->get().set_value(1.0);
        return controller_interface::CallbackReturn::SUCCESS;
    } else {
        RCLCPP_ERROR(get_node()->get_logger(), "Not init to end command interface.");
        return controller_interface::CallbackReturn::ERROR;
    }
}

controller_interface::CallbackReturn FreedriveController::on_deactivate(const rclcpp_lifecycle::State&) {
    is_freedrive_active_ = false;
    return CallbackReturn::SUCCESS;
}

controller_interface::return_type FreedriveController::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
    if (is_new_request_) {
        if (is_freedrive_active_) {
            if (end_interface_.has_value() && end_interface_->get().get_value() == 1.0) {
                RCLCPP_INFO(get_node()->get_logger(), "Freedrive mode end by hardware, ending request.");
                is_freedrive_active_ = false;
                return controller_interface::return_type::OK;
            } else {
                RCLCPP_INFO(get_node()->get_logger(), "Received command to start Freedrive Mode.");

                // Set command interface to enable
                start_interface_->get().set_value(1.0);
            }

        } else {
            RCLCPP_INFO(get_node()->get_logger(), "Received command to stop Freedrive Mode.");

            end_interface_->get().set_value(1.0);
        }
        is_new_request_ = false;
    }

    return controller_interface::return_type::OK;
}

}  // namespace ELITE_CS_CONTROLLER

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ELITE_CS_CONTROLLER::FreedriveController, controller_interface::ControllerInterface)
