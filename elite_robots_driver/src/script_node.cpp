#include "elite_robots_driver/script_node.hpp"


namespace ELITE_CS_ROBOT_ROS_DRIVER {

ScriptNode::ScriptNode(const rclcpp::NodeOptions& options) : Node("script_sender", options) {
    this->declare_parameter<std::string>("robot_ip");
    std::string robot_ip = this->get_parameter("robot_ip").as_string();

    // Create a primary instance and connect to robot then send a hello script
    primary_sender_ = std::make_unique<ELITE::PrimaryPortInterface>();
    try {
        if(primary_sender_->connect(robot_ip)) {
            auto hello_script = std::string("sec hello():\n\ttextmsg(\"script_sender connected\")\nend");
            primary_sender_->sendScript(hello_script);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Connect robot primary port fail.");
        }
        
    } catch(const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Connect robot primary port fail.");
    }

    script_sub_ = this->create_subscription<std_msgs::msg::String>(
        "~/script_command", 1, [&](const std_msgs::msg::String::SharedPtr msg) {
            if (primary_sender_->sendScript(msg->data)) {
                RCLCPP_INFO(this->get_logger(), "Sent script to robot:\n%s", msg->data.c_str());
                return true;
            }
            RCLCPP_ERROR(this->get_logger(), "Couldn't sent script to robot:\n%s", msg->data.c_str());
            return false;
        });
    
}

ScriptNode::~ScriptNode() { }

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ELITE_CS_ROBOT_ROS_DRIVER::ScriptNode)

