#ifndef __ELITE_CS_ROBOT_ROS_DRIVER__DASHBOARD_CLIENT_HPP__
#define __ELITE_CS_ROBOT_ROS_DRIVER__DASHBOARD_CLIENT_HPP__

#include <Elite/DashboardClient.hpp>
#include <Elite/EliteException.hpp>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "elite_robots_dashboard_msgs/srv/load.hpp"
#include "elite_robots_dashboard_msgs/srv/popup.hpp"
#include "elite_robots_dashboard_msgs/srv/log.hpp"
#include "elite_robots_msgs/srv/get_task_status.hpp"
#include "elite_robots_dashboard_msgs/srv/is_saved.hpp"
#include "elite_robots_msgs/srv/get_robot_mode.hpp"
#include "elite_robots_msgs/srv/get_safety_mode.hpp"
#include "elite_robots_dashboard_msgs/srv/custom_request.hpp"

namespace ELITE_CS_ROBOT_ROS_DRIVER {

class DashboardClient : public rclcpp::Node {
   private:
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr connect_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr power_on_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr power_off_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr play_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr brake_release_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr close_safety_dialog_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr unlock_protective_stop_service_;
    rclcpp::Service<elite_robots_dashboard_msgs::srv::Load>::SharedPtr load_configure_service_;
    rclcpp::Service<elite_robots_dashboard_msgs::srv::Load>::SharedPtr load_task_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_task_path_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr shutdown_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reboot_service_;
    rclcpp::Service<elite_robots_dashboard_msgs::srv::Popup>::SharedPtr popup_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr quit_service_;
    rclcpp::Service<elite_robots_dashboard_msgs::srv::Log>::SharedPtr add_log_service_;
    rclcpp::Service<elite_robots_msgs::srv::GetTaskStatus>::SharedPtr task_status_service_;
    rclcpp::Service<elite_robots_dashboard_msgs::srv::IsSaved>::SharedPtr is_task_saved_service_;
    rclcpp::Service<elite_robots_dashboard_msgs::srv::IsSaved>::SharedPtr is_configuration_saved_service_;
    rclcpp::Service<elite_robots_msgs::srv::GetRobotMode>::SharedPtr robot_mode_service_;
    rclcpp::Service<elite_robots_msgs::srv::GetSafetyMode>::SharedPtr safety_mode_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr safety_system_restart_service_;
    rclcpp::Service<elite_robots_dashboard_msgs::srv::CustomRequest>::SharedPtr custom_request_service_;

    ELITE::DashboardClient client_;
    bool is_connected_ = false;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr createTriggerService(const std::string& name, std::function<bool()> func) {
        return this->create_service<std_srvs::srv::Trigger>(name, [&, func](const std_srvs::srv::Trigger::Request::SharedPtr req,
                                                                            std_srvs::srv::Trigger::Response::SharedPtr resp) {
            (void)req;
            try {
                resp->success = func();
            } catch (const ELITE::EliteException& e) {
                resp->success = false;
                resp->message = e.what();
            }
        });
    }

   public:
    DashboardClient(const rclcpp::NodeOptions& options);
    ~DashboardClient();
    bool isConnected() const;
};


}


#endif
