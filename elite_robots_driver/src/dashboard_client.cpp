#include "elite_robots_driver/dashboard_client.hpp"
#include "elite_robots_msgs/msg/task_status.hpp"

namespace ELITE_CS_ROBOT_ROS_DRIVER {

namespace {
int8_t toRosTaskStatus(const ELITE::TaskStatus sdk_status) {
    // SDK enum order: UNKNOWN=0, PLAYING=1, PAUSED=2, STOPPED=3
    // ROS msg constants: UNKNOWN=-1, STOPPED=0, PAUSED=1, PLAYING=2
    switch (sdk_status) {
        case ELITE::TaskStatus::STOPPED:
            return elite_robots_msgs::msg::TaskStatus::STOPPED;
        case ELITE::TaskStatus::PAUSED:
            return elite_robots_msgs::msg::TaskStatus::PAUSED;
        case ELITE::TaskStatus::PLAYING:
            return elite_robots_msgs::msg::TaskStatus::PLAYING;
        case ELITE::TaskStatus::UNKNOWN:
        default:
            return elite_robots_msgs::msg::TaskStatus::UNKNOWN;
    }
}
}  // namespace

DashboardClient::DashboardClient(const rclcpp::NodeOptions& options) : Node("dashboard_client", options) {
    this->declare_parameter<std::string>("robot_ip", "192.168.51.244");

    try {
        RCLCPP_INFO(rclcpp::get_logger("EliteCSDashboardInterface"), "Connecting to robot ...");
        std::string robot_ip = this->get_parameter("robot_ip").as_string();
        is_connected_ = client_.connect(robot_ip);
    } catch (const ELITE::EliteException& e) {
        is_connected_ = false;
    }
    if (is_connected_) {
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

    popup_service_ = this->create_service<elite_robots_dashboard_msgs::srv::Popup>(
        "~/popup",
        [&](const elite_robots_dashboard_msgs::srv::Popup::Request::SharedPtr req,
            elite_robots_dashboard_msgs::srv::Popup::Response::SharedPtr resp) {
                try {
                    resp->success = client_.popup(req->arg, req->message);
                } catch(const ELITE::EliteException& e) {
                    resp->success = false;
                    resp->message = e.what();
                }
            }
    );

    add_log_service_ = this->create_service<elite_robots_dashboard_msgs::srv::Log>(
        "~/log",
        [&](const elite_robots_dashboard_msgs::srv::Log::Request::SharedPtr req,
            elite_robots_dashboard_msgs::srv::Log::Response::SharedPtr resp){
                try {
                    resp->success = client_.log(req->message);
                } catch (const ELITE::EliteException& e) {
                    resp->success = false;
                    resp->message = e.what();
                }
        }
    );

    task_status_service_ = this->create_service<elite_robots_msgs::srv::GetTaskStatus>(
        "~/get_task_status",
        [&](const elite_robots_msgs::srv::GetTaskStatus::Request::SharedPtr req,
            elite_robots_msgs::srv::GetTaskStatus::Response::SharedPtr resp){
                (void)req;
                try{
                    resp->status.status = toRosTaskStatus(client_.getTaskStatus());
                    resp->success = true;
                } catch (const ELITE::EliteException& e) {
                    resp->success = false;
                    resp->message = e.what();
                }
            }
    );

    is_task_saved_service_ = this->create_service<elite_robots_dashboard_msgs::srv::IsSaved>(
        "~/is_task_saved",
        [&](const elite_robots_dashboard_msgs::srv::IsSaved::Request::SharedPtr req,
            elite_robots_dashboard_msgs::srv::IsSaved::Response::SharedPtr resp){
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

    is_configuration_saved_service_  = this->create_service<elite_robots_dashboard_msgs::srv::IsSaved>(
        "~/is_configuration_saved",
        [&](const elite_robots_dashboard_msgs::srv::IsSaved::Request::SharedPtr req,
            elite_robots_dashboard_msgs::srv::IsSaved::Response::SharedPtr resp){
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

    robot_mode_service_ = this->create_service<elite_robots_msgs::srv::GetRobotMode>(
        "~/robot_mode",
        [&](const elite_robots_msgs::srv::GetRobotMode::Request::SharedPtr req,
            elite_robots_msgs::srv::GetRobotMode::Response::SharedPtr resp) {
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

    safety_mode_service_ = this->create_service<elite_robots_msgs::srv::GetSafetyMode>(
        "~/get_safety_mode",
        [&](const elite_robots_msgs::srv::GetSafetyMode::Request::SharedPtr req,
            elite_robots_msgs::srv::GetSafetyMode::Response::SharedPtr resp ) {
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

    load_configure_service_ = this->create_service<elite_robots_dashboard_msgs::srv::Load>(
        "~/load_configure", [&](const elite_robots_dashboard_msgs::srv::Load::Request::SharedPtr req,
                                elite_robots_dashboard_msgs::srv::Load::Response::SharedPtr resp) {
            try {
                resp->success = client_.loadConfiguration(req->filename);
                resp->answer = "Load configure: " + req->filename;
            } catch (const ELITE::EliteException& e) {
                resp->success = false;
                resp->answer = e.what();
            }
        });

    load_task_service_ = this->create_service<elite_robots_dashboard_msgs::srv::Load>(
        "~/load_task", [&](const elite_robots_dashboard_msgs::srv::Load::Request::SharedPtr req,
                           elite_robots_dashboard_msgs::srv::Load::Response::SharedPtr resp) {
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
                is_connected_ = client_.connect(robot_ip);
                resp->success = is_connected_;
            } catch (const ELITE::EliteException& e) {
                is_connected_ = false;
                resp->success = false;
                resp->message = e.what();
            }
        });
    
    custom_request_service_ = this->create_service<elite_robots_dashboard_msgs::srv::CustomRequest>(
        "~/custom_request", [&](const elite_robots_dashboard_msgs::srv::CustomRequest::Request::SharedPtr req,
                             elite_robots_dashboard_msgs::srv::CustomRequest::Response::SharedPtr resp) {
            try {
                resp->response = client_.sendAndReceive(req->request);
            } catch(const ELITE::EliteException& e) {
                resp->response = e.what();
            }
        }
    );
}

DashboardClient::~DashboardClient() {}

bool DashboardClient::isConnected() const {
    return is_connected_;
}

}  // namespace ELITE_CS_ROBOT_ROS_DRIVER

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ELITE_CS_ROBOT_ROS_DRIVER::DashboardClient)
