#include "elite_moveit_kinematics/elite_moveit_kinematics.hpp"
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_model/robot_model.h>
#include <urdf/model.h>
#include <chrono>
#include <cmath>
#include <kdl_parser/kdl_parser.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>

namespace elite_moveit {

EliteKinematicsPlugin_example1::EliteKinematicsPlugin_example1() { kin_ = std::make_shared<Kinematics>(); }

bool EliteKinematicsPlugin_example1::initialize(const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModel& robot_model,
                                       const std::string& group_name, const std::string& base_frame,
                                       const std::vector<std::string>& tip_frames, double search_res) {
    (void)search_res;

    if (tip_frames.empty()) {
        RCLCPP_ERROR(node->get_logger(), "No tip frames provided for group '%s'", group_name.c_str());
        return false;
    }

    RCLCPP_WARN(node->get_logger(), "Elite MoveIt IK Plugin initialized for group: %s", group_name.c_str());

    setValues(robot_model.getName(), group_name, base_frame, tip_frames, search_res);

    const moveit::core::JointModelGroup* jmg = robot_model.getJointModelGroup(group_name);
    if (jmg == nullptr) {
        RCLCPP_ERROR(node->get_logger(), "JointModelGroup '%s' not found", group_name.c_str());
        return false;
    }

    joint_names_ = jmg->getVariableNames();
    link_names_ = tip_frames;

    if (joint_names_.size() != AXIS_COUNT) {
        RCLCPP_ERROR(node->get_logger(), "Expected %d joints but group '%s' has %zu", AXIS_COUNT, group_name.c_str(),
                     joint_names_.size());
        return false;
    }

    // 优先使用 URDF 直接构造 KDL Chain
    urdf::ModelInterfaceSharedPtr urdf_model = robot_model.getURDF();
    KDL::Tree tree;
    if (!urdf_model || !kdl_parser::treeFromUrdfModel(*urdf_model, tree)) {
        RCLCPP_ERROR(node->get_logger(), "Failed to build KDL tree from URDF");
        return false;
    }

    KDL::Chain chain;
    if (!tree.getChain(base_frame, tip_frames.front(), chain)) {
        RCLCPP_ERROR(node->get_logger(), "Failed to extract KDL chain from %s to %s", base_frame.c_str(),
                     tip_frames.front().c_str());
        return false;
    }
    RCLCPP_INFO(node->get_logger(), "Built KDL chain base:%s tip:%s joints:%u", base_frame.c_str(), tip_frames.front().c_str(),
                chain.getNrOfJoints());

    kin_->updateKdlChain(chain);
    return true;
}

void EliteKinematicsPlugin_example1::loadDhParameters(const rclcpp::Node::SharedPtr& node) {
    auto declare_or_get = [&](const std::string& name, std::array<double, AXIS_COUNT>& target) {
        std::vector<double> defaults(AXIS_COUNT, 0.0);
        node->declare_parameter(name, defaults);
        std::vector<double> values = node->get_parameter(name).as_double_array();
        if (values.size() != AXIS_COUNT) {
            RCLCPP_WARN(node->get_logger(), "Parameter '%s' size %zu != AXIS_COUNT (%d), trunc/pad will apply", name.c_str(),
                        values.size(), AXIS_COUNT);
            values.resize(AXIS_COUNT, 0.0);
        }
        for (size_t i = 0; i < AXIS_COUNT; ++i) {
            target[i] = values[i];
        }
    };

    declare_or_get(group_name_ + ".dh_a", dh_a_);
    declare_or_get(group_name_ + ".dh_d", dh_d_);
    declare_or_get(group_name_ + ".dh_alpha", dh_alpha_);
}

void EliteKinematicsPlugin_example1::matToPose(const MAT4X4 mat, geometry_msgs::msg::Pose& pose) {
    pose.position.x = mat[0][3];
    pose.position.y = mat[1][3];
    pose.position.z = mat[2][3];

    tf2::Matrix3x3 rot(mat[0][0], mat[0][1], mat[0][2], mat[1][0], mat[1][1], mat[1][2], mat[2][0], mat[2][1], mat[2][2]);
    tf2::Quaternion q;
    rot.getRotation(q);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
}

void EliteKinematicsPlugin_example1::poseToMat(const geometry_msgs::msg::Pose& pose, MAT4X4 mat) {
    tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    tf2::Matrix3x3 rot(q);

    mat[0][0] = rot[0][0];
    mat[0][1] = rot[0][1];
    mat[0][2] = rot[0][2];
    mat[0][3] = pose.position.x;
    mat[1][0] = rot[1][0];
    mat[1][1] = rot[1][1];
    mat[1][2] = rot[1][2];
    mat[1][3] = pose.position.y;
    mat[2][0] = rot[2][0];
    mat[2][1] = rot[2][1];
    mat[2][2] = rot[2][2];
    mat[2][3] = pose.position.z;
    mat[3][0] = 0.0;
    mat[3][1] = 0.0;
    mat[3][2] = 0.0;
    mat[3][3] = 1.0;
}

bool EliteKinematicsPlugin_example1::getPositionFK(const std::vector<std::string>& link_names, const std::vector<double>& joint_angles,
                                          std::vector<geometry_msgs::msg::Pose>& poses) const {
    if (link_names.size() != 1 || link_names.front() != link_names_.front()) {
        return false;
    }
    if (joint_angles.size() != AXIS_COUNT) {
        return false;
    }

    JOINTS joints{};
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        joints[i] = joint_angles[i];
    }

    MAT4X4 mat{};
    kin_->forward(joints, mat);

    poses.resize(1);
    matToPose(mat, poses[0]);
    return true;
}

bool EliteKinematicsPlugin_example1::getPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                                          std::vector<double>& solution, moveit_msgs::msg::MoveItErrorCodes& error_code,
                                          const kinematics::KinematicsQueryOptions& options) const {
    RCLCPP_INFO(rclcpp::get_logger("EliteIK"), "getPositionIK called");
    if (ik_seed_state.size() != AXIS_COUNT) {
        error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE;
        return false;
    }

    MAT4X4 target{};
    poseToMat(ik_pose, target);
    RCLCPP_WARN(rclcpp::get_logger("EliteIK"), "Elite IK solve called, target xyz=[%.3f %.3f %.3f]", ik_pose.position.x,
                ik_pose.position.y, ik_pose.position.z);

    JOINTS seed{};
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        seed[i] = ik_seed_state[i];
    }

    JOINTS out{};
    int ret = kin_->inverse(target, seed, out);
    if (ret != 0) {
        RCLCPP_WARN(rclcpp::get_logger("EliteIK"), "IK failed ret=%d target xyz=[%.3f %.3f %.3f]", ret, ik_pose.position.x,
                    ik_pose.position.y, ik_pose.position.z);
        error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
        return false;
    }

    solution.assign(out, out + AXIS_COUNT);
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    return true;
}

bool EliteKinematicsPlugin_example1::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                                             double /*timeout*/, const std::vector<double>& /*consistency_limits*/,
                                             std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                             moveit_msgs::msg::MoveItErrorCodes& error_code,
                                             const kinematics::KinematicsQueryOptions& options) const {
    RCLCPP_INFO(rclcpp::get_logger("EliteIK"), "searchPositionIK called");
    bool ok = getPositionIK(ik_pose, ik_seed_state, solution, error_code, options);
    if (ok && solution_callback) {
        solution_callback(ik_pose, solution, error_code);
    }
    return ok;
}

bool EliteKinematicsPlugin_example1::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                                             double timeout, std::vector<double>& solution,
                                             moveit_msgs::msg::MoveItErrorCodes& error_code,
                                             const kinematics::KinematicsQueryOptions& options) const {
    std::vector<double> limits;
    return searchPositionIK(ik_pose, ik_seed_state, timeout, limits, solution, IKCallbackFn(), error_code, options);
}

bool EliteKinematicsPlugin_example1::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                                             double timeout, const std::vector<double>& consistency_limits,
                                             std::vector<double>& solution, moveit_msgs::msg::MoveItErrorCodes& error_code,
                                             const kinematics::KinematicsQueryOptions& options) const {
    return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, IKCallbackFn(), error_code, options);
}

bool EliteKinematicsPlugin_example1::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                                             double timeout, std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                             moveit_msgs::msg::MoveItErrorCodes& error_code,
                                             const kinematics::KinematicsQueryOptions& options) const {
    std::vector<double> limits;
    return searchPositionIK(ik_pose, ik_seed_state, timeout, limits, solution, solution_callback, error_code, options);
}

bool EliteKinematicsPlugin_example1::supportsGroup(const moveit::core::JointModelGroup* jmg, std::string* error) const {
    if (jmg == nullptr) {
        if (error) {
            *error = "JointModelGroup is null";
        }
        return false;
    }
    if (jmg->getVariableCount() != AXIS_COUNT) {
        if (error) {
            *error = "Joint count mismatch";
        }
        return false;
    }
    return true;
}

}  // namespace elite_moveit

// PLUGINLIB_EXPORT_CLASS(elite_moveit::EliteKinematicsPlugin_example1, kinematics::KinematicsBase)
//  register EliteKinematicsPlugin_example1 as a KinematicsBase implementation
#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(elite_moveit::EliteKinematicsPlugin_example1, kinematics::KinematicsBase)
