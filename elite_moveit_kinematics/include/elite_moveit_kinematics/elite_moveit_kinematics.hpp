#pragma once
#include <moveit/kinematics_base/kinematics_base.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <array>
#include <geometry_msgs/msg/pose.hpp>
#include <memory>
#include <moveit_msgs/moveit_msgs/msg/move_it_error_codes.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include "elite_kinematics/kinematics_example1.hpp"

namespace elite_moveit {

class EliteKinematicsPlugin_example1 : public kinematics::KinematicsBase {
   public:
    EliteKinematicsPlugin_example1();
    ~EliteKinematicsPlugin_example1() override = default;

    bool initialize(const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModel& robot_model, const std::string& group_name,
                    const std::string& base_frame, const std::vector<std::string>& tip_frames, double search_res) override;

    bool getPositionFK(const std::vector<std::string>& link_names, const std::vector<double>& joint_angles,
                       std::vector<geometry_msgs::msg::Pose>& poses) const override;

    bool getPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                       std::vector<double>& solution, moveit_msgs::msg::MoveItErrorCodes& error_code,
                       const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

    bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                          std::vector<double>& solution, moveit_msgs::msg::MoveItErrorCodes& error_code,
                          const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

    bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                          const std::vector<double>& consistency_limits, std::vector<double>& solution,
                          moveit_msgs::msg::MoveItErrorCodes& error_code,
                          const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

    bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                          std::vector<double>& solution, const IKCallbackFn& solution_callback,
                          moveit_msgs::msg::MoveItErrorCodes& error_code,
                          const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

    bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                          const std::vector<double>& consistency_limits, std::vector<double>& solution,
                          const IKCallbackFn& solution_callback, moveit_msgs::msg::MoveItErrorCodes& error_code,
                          const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

    bool supportsGroup(const moveit::core::JointModelGroup* jmg, std::string* error) const override;

    const std::vector<std::string>& getJointNames() const override { return joint_names_; }
    const std::vector<std::string>& getLinkNames() const override { return link_names_; }

   private:
    void loadDhParameters(const rclcpp::Node::SharedPtr& node);
    static void matToPose(const MAT4X4 mat, geometry_msgs::msg::Pose& pose);
    static void poseToMat(const geometry_msgs::msg::Pose& pose, MAT4X4 mat);

    std::shared_ptr<Kinematics> kin_;
    std::vector<std::string> joint_names_;
    std::vector<std::string> link_names_;
    std::array<double, AXIS_COUNT> dh_a_{};
    std::array<double, AXIS_COUNT> dh_d_{};
    std::array<double, AXIS_COUNT> dh_alpha_{};
};

}  // namespace elite_moveit
