#ifndef __ELITE_CS_CONTROLLERS_FREEDRIVE_CONTROLLER_HPP__
#define __ELITE_CS_CONTROLLERS_FREEDRIVE_CONTROLLER_HPP__

#include <eli_cs_controllers/freedrive_controller_parameters.hpp>

#include <atomic>
#include <controller_interface/controller_interface.hpp>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/duration.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>
#include <thread>
#include <vector>

namespace ELITE_CS_CONTROLLER {

class FreedriveController : public controller_interface::ControllerInterface {
   public:
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    // Change the input for the update function
    controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    CallbackReturn on_init() override;

   private:
    std::shared_ptr<freedrive_controller::ParamListener> freedrive_param_listener_;
    freedrive_controller::Params freedrive_params_;

    mutable std::chrono::seconds timeout_;
    rclcpp::TimerBase::SharedPtr freedrive_timer_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr freedrive_sub_;

    std::atomic<bool> is_new_request_;
    std::atomic<bool> is_freedrive_active_;

    std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> async_success_interface_;
    std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> start_interface_;
    std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> end_interface_;

    bool setReferenceWrapper(std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>& wapper,
                             const std::string& interface_name);
   
    void startTimer();
};

}  // namespace ELITE_CS_CONTROLLER

#endif
