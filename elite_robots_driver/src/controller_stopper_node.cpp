#include "elite_robots_driver/controller_stopper.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("controller_stopper_node");

    bool headless_mode = node->declare_parameter<bool>("headless_mode", false);
    node->get_parameter<bool>("headless_mode", headless_mode);
    bool joint_controller_active = node->declare_parameter<bool>("joint_controller_active", true);
    node->get_parameter<bool>("joint_controller_active", joint_controller_active);

    // If headless mode is not active, but the joint controllers are we should stop the joint controllers during startup
    // of the node
    bool stop_controllers_on_startup = false;
    if (joint_controller_active == true && headless_mode == false) {
        stop_controllers_on_startup = true;
    }

    ControllerStopper stopper(node, stop_controllers_on_startup);

    rclcpp::spin(node);

    return 0;
}
