// SPDX-License-Identifier: Apache-2.0
// Send a single joint target with a duration computed from a max joint velocity.
// Usage example:
//   ros2 run eli_cs_robot_driver safe_joint_move_cpp --ros-args
//     -p target:="[0.0,-1.2,0.0,-1.57,0.0,0.0]" -p max_vel:=0.5 -p min_time:=2.0
//     -p controller:="scaled_joint_trajectory_controller"

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace {
const std::vector<std::string> JOINTS = {
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
};
}  // namespace

class SafeJointMoveNode : public rclcpp::Node {
   public:
    using FollowJT = control_msgs::action::FollowJointTrajectory;
    using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJT>;

    SafeJointMoveNode() : rclcpp::Node("safe_joint_move_cpp") {
        target_ = this->declare_parameter<std::vector<double>>("target", std::vector<double>{0.0, -1.2, 0.0, -1.57, 0.0, 0.0});
        max_vel_ = this->declare_parameter<double>("max_vel", 0.5);
        min_time_ = this->declare_parameter<double>("min_time", 2.0);
        controller_ = this->declare_parameter<std::string>("controller", "scaled_joint_trajectory_controller");

        client_ = rclcpp_action::create_client<FollowJT>(this, "/" + controller_ + "/follow_joint_trajectory");

        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&SafeJointMoveNode::jointCallback, this, std::placeholders::_1));
    }

    bool execute() {
        if (target_.size() != JOINTS.size()) {
            RCLCPP_ERROR(get_logger(), "Target size %zu does not match joint count %zu", target_.size(), JOINTS.size());
            return false;
        }

        // Wait for first joint state to avoid jumps.
        RCLCPP_INFO(get_logger(), "Waiting for /joint_states...");
        const auto deadline = this->now() + rclcpp::Duration::from_seconds(5.0);
        rclcpp::Rate r(50.0);
        while (rclcpp::ok() && !current_pos_) {
            rclcpp::spin_some(shared_from_this());
            if (this->now() > deadline) {
                break;
            }
            r.sleep();
        }
        if (!current_pos_) {
            RCLCPP_ERROR(get_logger(), "No /joint_states received; aborting");
            return false;
        }

        // Compute required duration based on max joint delta and max_vel.
        double max_delta = 0.0;
        for (size_t i = 0; i < JOINTS.size(); ++i) {
            max_delta = std::max(max_delta, std::abs(target_[i] - current_pos_.value()[i]));
        }
        double duration = min_time_;
        if (max_vel_ > 0.0) {
            duration = std::max(duration, max_delta / max_vel_);
        }
        RCLCPP_INFO(get_logger(), "Max delta %.3f rad, duration %.3f s (max_vel %.3f rad/s)", max_delta, duration, max_vel_);

        if (!client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(get_logger(), "follow_joint_trajectory action server not available");
            return false;
        }

        FollowJT::Goal goal;
        goal.trajectory.joint_names = JOINTS;

        trajectory_msgs::msg::JointTrajectoryPoint start;
        start.positions = current_pos_.value();
        start.time_from_start = rclcpp::Duration(0, 0);
        goal.trajectory.points.push_back(start);

        trajectory_msgs::msg::JointTrajectoryPoint target_pt;
        target_pt.positions = target_;
        target_pt.time_from_start = rclcpp::Duration::from_seconds(duration);
        goal.trajectory.points.push_back(target_pt);

        auto goal_future = client_->async_send_goal(goal);
        if (rclcpp::spin_until_future_complete(shared_from_this(), goal_future) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(get_logger(), "Failed to send goal");
            return false;
        }
        auto goal_handle = goal_future.get();
        if (!goal_handle || goal_handle->get_status() != rclcpp_action::GoalStatus::STATUS_ACCEPTED) {
            RCLCPP_ERROR(get_logger(), "Goal rejected by controller");
            return false;
        }

        auto result_future = client_->async_get_result(goal_handle);
        if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(get_logger(), "Failed to get result");
            return false;
        }
        auto result = result_future.get();
        if (result.result->error_code != 0) {
            RCLCPP_ERROR(get_logger(), "Controller returned error_code=%d", result.result->error_code);
            return false;
        }
        RCLCPP_INFO(get_logger(), "Trajectory executed successfully.");
        return true;
    }

   private:
    void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::vector<double> vals;
        vals.reserve(JOINTS.size());
        std::unordered_map<std::string, double> m;
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (i < msg->position.size()) {
                m[msg->name[i]] = msg->position[i];
            }
        }
        for (const auto& j : JOINTS) {
            auto it = m.find(j);
            if (it == m.end()) {
                return;  // wait for complete set
            }
            vals.push_back(it->second);
        }
        current_pos_ = vals;
    }

    std::vector<double> target_;
    double max_vel_{0.5};
    double min_time_{2.0};
    std::string controller_;

    rclcpp_action::Client<FollowJT>::SharedPtr client_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    std::optional<std::vector<double>> current_pos_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SafeJointMoveNode>();
    bool ok = node->execute();
    rclcpp::shutdown();
    return ok ? 0 : 1;
}
