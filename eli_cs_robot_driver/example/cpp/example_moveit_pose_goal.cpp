// SPDX-License-Identifier: Apache-2.0
// Plan to a Cartesian pose with MoveIt, then optionally execute it.
//
// Usage examples:
//   # Plan only, keep current orientation, move tool0 +2 cm along base_link X:
//   ros2 run eli_cs_robot_driver example_moveit_pose_goal --ros-args -p offset:="[0.02,0.0,0.0]"
//
//   # Execute the plan:
//   ros2 run eli_cs_robot_driver example_moveit_pose_goal --ros-args -p execute:=true -p offset:="[0.02,0.0,0.0]"
//
//   # Absolute pose target: x y z rx ry rz in reference_frame.
//   # rx ry rz is RPY [rad], matching the teach pendant when its pose format is RPY [rad].
//   ros2 run eli_cs_robot_driver example_moveit_pose_goal --ros-args
//     -p execute:=true -p target_pose:="[0.4,0.0,0.35,0.0,3.14159,0.0]"

#include <algorithm>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace {

void logPose(const rclcpp::Logger& logger, const std::string& label, const geometry_msgs::msg::Pose& pose) {
    RCLCPP_INFO(
        logger, "%s position=(%.4f, %.4f, %.4f), orientation=(%.4f, %.4f, %.4f, %.4f)", label.c_str(), pose.position.x,
        pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
}

geometry_msgs::msg::Quaternion quaternionFromRpy(double roll, double pitch, double yaw) {
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    q.normalize();
    return tf2::toMsg(q);
}

}  // namespace

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("moveit_pose_goal_example");
    auto logger = node->get_logger();

    const std::string group = node->declare_parameter<std::string>("group", "cs_manipulator");
    const std::string ee_link = node->declare_parameter<std::string>("ee_link", "tool0");
    const std::string reference_frame = node->declare_parameter<std::string>("reference_frame", "base_link");
    const bool execute = node->declare_parameter<bool>("execute", false);
    const double planning_time = node->declare_parameter<double>("planning_time", 5.0);
    const double velocity_scale = node->declare_parameter<double>("velocity_scale", 0.1);
    const double acceleration_scale = node->declare_parameter<double>("acceleration_scale", 0.1);
    const std::vector<double> offset =
        node->declare_parameter<std::vector<double>>("offset", std::vector<double>{0.02, 0.0, 0.0});
    const std::vector<double> target_pose = node->declare_parameter<std::vector<double>>("target_pose", std::vector<double>{});

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });

    bool ok = false;
    try {
        moveit::planning_interface::MoveGroupInterface move_group(node, group);
        move_group.setEndEffectorLink(ee_link);
        move_group.setPoseReferenceFrame(reference_frame);
        move_group.setPlanningTime(planning_time);
        move_group.setMaxVelocityScalingFactor(std::clamp(velocity_scale, 0.01, 1.0));
        move_group.setMaxAccelerationScalingFactor(std::clamp(acceleration_scale, 0.01, 1.0));
        move_group.setStartStateToCurrentState();

        geometry_msgs::msg::Pose current = move_group.getCurrentPose(ee_link).pose;
        geometry_msgs::msg::Pose target = current;

        if (!target_pose.empty()) {
            if (target_pose.size() != 6) {
                RCLCPP_ERROR(logger, "Parameter target_pose must contain 6 values: x y z rx ry rz");
                throw std::runtime_error("invalid target_pose size");
            }
            target.position.x = target_pose[0];
            target.position.y = target_pose[1];
            target.position.z = target_pose[2];
            target.orientation = quaternionFromRpy(target_pose[3], target_pose[4], target_pose[5]);
            RCLCPP_INFO(
                logger, "Target RPY=(%.4f, %.4f, %.4f) rad converted to quaternion", target_pose[3], target_pose[4], target_pose[5]);
        } else {
            if (offset.size() != 3) {
                RCLCPP_ERROR(logger, "Parameter offset must contain 3 values: dx dy dz");
                throw std::runtime_error("invalid offset size");
            }
            target.position.x += offset[0];
            target.position.y += offset[1];
            target.position.z += offset[2];
        }

        RCLCPP_INFO(logger, "Planning group=%s, ee_link=%s, reference_frame=%s", group.c_str(), ee_link.c_str(), reference_frame.c_str());
        logPose(logger, "Current pose", current);
        logPose(logger, "Target pose", target);

        move_group.setPoseTarget(target, ee_link);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        const auto plan_result = move_group.plan(plan);
        if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(logger, "MoveIt planning failed");
            throw std::runtime_error("planning failed");
        }

        RCLCPP_INFO(logger, "MoveIt planning succeeded");
        if (!execute) {
            RCLCPP_WARN(logger, "execute=false, plan was not sent to the robot. Set execute:=true to move.");
            ok = true;
        } else {
            RCLCPP_WARN(logger, "Executing trajectory on the robot");
            const auto exec_result = move_group.execute(plan);
            ok = exec_result == moveit::core::MoveItErrorCode::SUCCESS;
            if (!ok) {
                RCLCPP_ERROR(logger, "MoveIt execution failed");
            } else {
                RCLCPP_INFO(logger, "MoveIt execution succeeded");
            }
        }
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(logger, "Exception: %s", ex.what());
    }

    executor.cancel();
    if (spinner.joinable()) {
        spinner.join();
    }
    rclcpp::shutdown();
    return ok ? 0 : 1;
}
