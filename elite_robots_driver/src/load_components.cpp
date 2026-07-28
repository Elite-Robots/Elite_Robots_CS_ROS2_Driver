#include "elite_robots_driver/dashboard_client.hpp"
#include "elite_robots_driver/primary_client.hpp"
#include "elite_robots_driver/script_node.hpp"

#include "rclcpp/rclcpp.hpp"
#include <memory>

int main(int argc, char **argv) {

    // Force flush of the stdout buffer.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Initialize any global resources needed by the middleware and the client library.
    // This will also parse command line arguments one day (as of Beta 1 they are not used).
    // You must call this before using any other part of the ROS system.
    // This should be called once per process.
    rclcpp::init(argc, argv);

    // Create an executor that will be responsible for execution of callbacks for a set of nodes.
    // With this version, all callbacks will be called from within this thread (the main one).
    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;

    auto dashboard_node = std::make_shared<ELITE_CS_ROBOT_ROS_DRIVER::DashboardClient>(options);
    if (dashboard_node->isConnected()) {
        exec.add_node(dashboard_node);
    } else {
        RCLCPP_WARN(rclcpp::get_logger("eli_components_loader"), "Skip dashboard_client because dashboard connection failed.");
        dashboard_node.reset();
    }

    auto primary_node = std::make_shared<ELITE_CS_ROBOT_ROS_DRIVER::PrimaryClient>(options);
    exec.add_node(primary_node);

    auto script_node = std::make_shared<ELITE_CS_ROBOT_ROS_DRIVER::ScriptNode>(options);
    exec.add_node(script_node);

    // spin will block until work comes in, execute work as it becomes available, and keep blocking.
    // It will only be interrupted by Ctrl-C.
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
