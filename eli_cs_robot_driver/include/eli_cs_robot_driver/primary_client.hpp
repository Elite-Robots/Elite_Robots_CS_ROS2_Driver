#ifndef __ELITE_CS_ROBOT_ROS_DRIVER__PRIMARY_CLIENT_HPP__
#define __ELITE_CS_ROBOT_ROS_DRIVER__PRIMARY_CLIENT_HPP__

#include <Elite/EliteException.hpp>
#include <Elite/PrimaryPortInterface.hpp>

#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace ELITE_CS_ROBOT_ROS_DRIVER {

class PrimaryClient : public rclcpp::Node {
   private:
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr connect_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr power_on_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr power_off_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr brake_release_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr unlock_protective_stop_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr safety_system_restart_service_;

    ELITE::PrimaryPortInterface client_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr createTriggerService(const std::string& name, std::function<bool()> func);
    bool connect();

   public:
    PrimaryClient(const rclcpp::NodeOptions& options);
    ~PrimaryClient();
};

}  // namespace ELITE_CS_ROBOT_ROS_DRIVER

#endif
