#ifndef __ELITE_CS_ROBOT_ROS_DRIVER__SCRIPT_NODE_HPP__
#define __ELITE_CS_ROBOT_ROS_DRIVER__SCRIPT_NODE_HPP__

#include <Elite/PrimaryPortInterface.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>

namespace ELITE_CS_ROBOT_ROS_DRIVER {

class ScriptNode : public rclcpp::Node {
private:
    std::unique_ptr<ELITE::PrimaryPortInterface> primary_sender_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr script_sub_;
public:
    ScriptNode(const rclcpp::NodeOptions& options);
    ~ScriptNode();

};


}


#endif