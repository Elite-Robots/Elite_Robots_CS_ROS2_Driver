#include "eli_cs_robot_driver/dashboard_client.hpp"

namespace ELITE_CS_ROBOT_ROS_DRIVER {

DashboardClient::DashboardClient(const rclcpp::NodeOptions& options) : Node("dashboard_client", options) {
    this->declare_parameter<std::string>("robot_ip", "192.168.51.244");

    bool is_connect_success = false;
    try {
        RCLCPP_INFO(rclcpp::get_logger("EliteCSDashboardInterface"), "Connecting to robot ...");
        std::string robot_ip = this->get_parameter("robot_ip").as_string();
        is_connect_success = client_.connect(robot_ip);
    } catch (const ELITE::EliteException& e) {
        is_connect_success = false;
    }
    if (is_connect_success) {
        RCLCPP_INFO(rclcpp::get_logger("EliteCSDashboardInterface"), "Connect to robot success");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("EliteCSDashboardInterface"), "Connect to robot fail");
    }

    power_on_service_ = createTriggerService("~/power_on", [&]() -> bool { return client_.powerOn(); });
    power_off_service_ = createTriggerService("~/power_off", [&]() -> bool { return client_.powerOff(); });
    play_service_ = createTriggerService("~/play", [&]() -> bool { return client_.playProgram(); });
    pause_service_ = createTriggerService("~/pause", [&]() -> bool { return client_.pauseProgram(); });
    stop_service_ = createTriggerService("~/stop", [&]() -> bool { return client_.stopProgram(); });
    brake_release_service_ = createTriggerService("~/brake_release", [&]() -> bool { return client_.brakeRelease(); });
    shutdown_service_ = createTriggerService("~/shutdown", [&]()->bool{ client_.shutdown(); return true; });
    reboot_service_ = createTriggerService("~/reboot", [&]()->bool{ client_.reboot(); return true; });
    unlock_protective_stop_service_ =
        createTriggerService("~/unlock_protective_stop", [&]() -> bool { return client_.unlockProtectiveStop(); });
    close_safety_dialog_service_ =
        createTriggerService("~/close_safety_dialog", [&]() -> bool { return client_.closeSafetyDialog(); });

    quit_service_ = createTriggerService("~/quit", [&]()->bool{ client_.quit(); return true; });

    safety_system_restart_service_ = createTriggerService("~/restart_safety", [&]()->bool{ return client_.safetySystemRestart(); });

    popup_service_ = this->create_service<eli_dashboard_interface::srv::Popup>(
        "~/popup",
        [&](const eli_dashboard_interface::srv::Popup::Request::SharedPtr req,
            eli_dashboard_interface::srv::Popup::Response::SharedPtr resp) {
                try {
                    resp->success = client_.popup(req->arg, req->message);
                } catch(const ELITE::EliteException& e) {
                    resp->success = false;
                    resp->message = e.what();
                }
            }
    );

    add_log_service_ = this->create_service<eli_dashboard_interface::srv::Log>(
        "~/log",
        [&](const eli_dashboard_interface::srv::Log::Request::SharedPtr req,
            eli_dashboard_interface::srv::Log::Response::SharedPtr resp){
                try {
                    resp->success = client_.log(req->message);
                } catch (const ELITE::EliteException& e) {
                    resp->success = false;
                    resp->message = e.what();
                }
        }
    );

    task_status_service_ = this->create_service<eli_common_interface::srv::GetTaskStatus>(
        "~/get_task_status",
        [&](const eli_common_interface::srv::GetTaskStatus::Request::SharedPtr req,
            eli_common_interface::srv::GetTaskStatus::Response::SharedPtr resp){
                (void)req;
                try{
                    resp->status.status = (int8_t)client_.getTaskStatus();
                    resp->success = true;
                } catch (const ELITE::EliteException& e) {
                    resp->success = false;
                    resp->message = e.what();
                }
            }
    );

    is_task_saved_service_ = this->create_service<eli_dashboard_interface::srv::IsSaved>(
        "~/is_task_saved",
        [&](const eli_dashboard_interface::srv::IsSaved::Request::SharedPtr req,
            eli_dashboard_interface::srv::IsSaved::Response::SharedPtr resp){
                (void)req;
                try{
                    resp->is_saved = client_.isTaskSaved();
                    resp->success = true;
                } catch (const ELITE::EliteException& e) {
                    resp->success = false;
                    resp->message = e.what();
                }
        }
    );

    is_configuration_saved_service_  = this->create_service<eli_dashboard_interface::srv::IsSaved>(
        "~/is_configuration_saved",
        [&](const eli_dashboard_interface::srv::IsSaved::Request::SharedPtr req,
            eli_dashboard_interface::srv::IsSaved::Response::SharedPtr resp){
                (void)req;
                try{
                    resp->is_saved = !client_.isConfigurationModify();
                    resp->success = true;
                } catch (const ELITE::EliteException& e) {
                    resp->success = false;
                    resp->message = e.what();
                }
        }
    );

    robot_mode_service_ = this->create_service<eli_common_interface::srv::GetRobotMode>(
        "~/robot_mode",
        [&](const eli_common_interface::srv::GetRobotMode::Request::SharedPtr req,
            eli_common_interface::srv::GetRobotMode::Response::SharedPtr resp) {
                (void)req;
                try {
                    resp->mode.mode = (int8_t)client_.robotMode();
                    resp->success = true;
                } catch (const ELITE::EliteException& e) {
                    resp->success = false;
                    resp->message = e.what();
                }
            }
    );

    safety_mode_service_ = this->create_service<eli_common_interface::srv::GetSafetyMode>(
        "~/get_safety_mode",
        [&](const eli_common_interface::srv::GetSafetyMode::Request::SharedPtr req,
            eli_common_interface::srv::GetSafetyMode::Response::SharedPtr resp ) {
                (void)req;
                try {
                    resp->mode.mode = (int8_t)client_.safetyMode();
                    resp->success = true;
                } catch (const ELITE::EliteException& e) {
                    resp->success = false;
                    resp->message = e.what();
                }
            }
    );

    get_task_path_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/get_task_path",
        [&](const std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr resp) {
            (void)req;
            try {
                resp->success = true;
                resp->message = client_.getTaskPath();
            } catch (const ELITE::EliteException& e) {
                resp->success = false;
                resp->message = e.what();
            }
        });

    load_configure_service_ = this->create_service<eli_dashboard_interface::srv::Load>(
        "~/load_configure", [&](const eli_dashboard_interface::srv::Load::Request::SharedPtr req,
                                eli_dashboard_interface::srv::Load::Response::SharedPtr resp) {
            try {
                resp->success = client_.loadConfiguration(req->filename);
                resp->answer = "Load configure: " + req->filename;
            } catch (const ELITE::EliteException& e) {
                resp->success = false;
                resp->answer = e.what();
            }
        });

    load_task_service_ = this->create_service<eli_dashboard_interface::srv::Load>(
        "~/load_task", [&](const eli_dashboard_interface::srv::Load::Request::SharedPtr req,
                           eli_dashboard_interface::srv::Load::Response::SharedPtr resp) {
            try {
                resp->success = client_.loadTask(req->filename);
                resp->answer = "Load Task: " + req->filename;
            } catch (const ELITE::EliteException& e) {
                resp->success = false;
                resp->answer = e.what();
            }
        });

    connect_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/connect", [&](const std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr resp) {
            (void)req;
            try {
                std::string robot_ip = this->get_parameter("robot_ip").as_string();
                resp->success = client_.connect(robot_ip);
            } catch (const ELITE::EliteException& e) {
                resp->success = false;
                resp->message = e.what();
            }
        });
    
    custom_request_service_ = this->create_service<eli_dashboard_interface::srv::CustomRequest>(
        "~/custom_request", [&](const eli_dashboard_interface::srv::CustomRequest::Request::SharedPtr req,
                             eli_dashboard_interface::srv::CustomRequest::Response::SharedPtr resp) {
            try {
                resp->response = client_.sendAndReceive(req->request);
            } catch(const ELITE::EliteException& e) {
                resp->response = e.what();
            }
        }
    );
}

DashboardClient::~DashboardClient() {}

}  // namespace ELITE_CS_ROBOT_ROS_DRIVER

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ELITE_CS_ROBOT_ROS_DRIVER::DashboardClient)
