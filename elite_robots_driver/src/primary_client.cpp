#include "elite_robots_driver/primary_client.hpp"

namespace ELITE_CS_ROBOT_ROS_DRIVER {

PrimaryClient::PrimaryClient(const rclcpp::NodeOptions& options) : Node("primary_client", options) {
    this->declare_parameter<std::string>("robot_ip", "192.168.51.244");

    if (connect()) {
        RCLCPP_INFO(this->get_logger(), "Connect to robot primary port success");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Connect to robot primary port fail");
    }

    power_on_service_ = createTriggerService("~/power_on", [&]() -> bool { return client_.powerOn(); });
    power_off_service_ = createTriggerService("~/power_off", [&]() -> bool { return client_.powerOff(); });
    pause_service_ = createTriggerService("~/pause", [&]() -> bool { return client_.pauseProgram(); });
    stop_service_ = createTriggerService("~/stop", [&]() -> bool { return client_.stopProgram(); });
    brake_release_service_ = createTriggerService("~/brake_release", [&]() -> bool { return client_.brakeRelease(); });
    unlock_protective_stop_service_ =
        createTriggerService("~/unlock_protective_stop", [&]() -> bool { return client_.unlockProtectiveStop(); });
    safety_system_restart_service_ = createTriggerService("~/restart_safety", [&]() -> bool { return client_.safetySystemRestart(); });

    connect_service_ = createTriggerService("~/connect", [&]() -> bool { return connect(); });
}

PrimaryClient::~PrimaryClient() {}

rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr PrimaryClient::createTriggerService(const std::string& name,
                                                                                       std::function<bool()> func) {
    return this->create_service<std_srvs::srv::Trigger>(
        name, [func](const std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr resp) {
            (void)req;
            try {
                resp->success = func();
            } catch (const ELITE::EliteException& e) {
                resp->success = false;
                resp->message = e.what();
            }
        });
}

bool PrimaryClient::connect() {
    try {
        const std::string robot_ip = this->get_parameter("robot_ip").as_string();
        return client_.connect(robot_ip);
    } catch (const ELITE::EliteException& e) {
        RCLCPP_ERROR(this->get_logger(), "Connect robot primary port exception: %s", e.what());
    }
    return false;
}

}  // namespace ELITE_CS_ROBOT_ROS_DRIVER

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ELITE_CS_ROBOT_ROS_DRIVER::PrimaryClient)
