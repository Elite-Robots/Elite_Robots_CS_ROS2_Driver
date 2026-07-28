#include "cs_calibration/calibration.hpp"

#include <vector>

namespace cs_calibration {
Calibration::Calibration(const DHRobot& robot_parameters) : robot_parameters_(robot_parameters) { buildChain(); }

Calibration::~Calibration() {}

Eigen::Matrix4d Calibration::calcForwardKinematics(const Eigen::Matrix<double, 6, 1>& joint_values, const size_t link_nr) {
    // Currently ignore input and calculate for zero vector input
    Eigen::Matrix4d output = Eigen::Matrix4d::Identity();

    std::vector<Eigen::Matrix4d> simplified_chain = getSimplified();
    for (size_t i = 0; i < link_nr; ++i) {
        Eigen::Matrix4d rotation = Eigen::Matrix4d::Identity();
        rotation.topLeftCorner(3, 3) = Eigen::AngleAxisd(joint_values(i), Eigen::Vector3d::UnitZ()).toRotationMatrix();
        output *= simplified_chain[i] * rotation;
    }

    return output;
}

void Calibration::buildChain() {
    chain_.clear();
    for (size_t i = 0; i < robot_parameters_.segments_.size(); ++i) {
        Eigen::Matrix4d seg1_mat = Eigen::Matrix4d::Identity();
        seg1_mat.topLeftCorner(3, 3) =
            Eigen::AngleAxisd(robot_parameters_.segments_[i].theta_, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        seg1_mat(2, 3) = robot_parameters_.segments_[i].d_;

        chain_.push_back(seg1_mat);

        Eigen::Matrix4d seg2_mat = Eigen::Matrix4d::Identity();
        seg2_mat.topLeftCorner(3, 3) =
            Eigen::AngleAxisd(robot_parameters_.segments_[i].alpha_, Eigen::Vector3d::UnitX()).toRotationMatrix();
        seg2_mat(0, 3) = robot_parameters_.segments_[i].a_;

        chain_.push_back(seg2_mat);
    }
}

std::vector<Eigen::Matrix4d> Calibration::getSimplified() const {
    std::vector<Eigen::Matrix4d> simplified_chain;
    simplified_chain.push_back(chain_[0]);
    for (size_t i = 1; i < chain_.size() - 1; i += 2) {
        simplified_chain.emplace_back(chain_[i] * chain_[i + 1]);
    }
    simplified_chain.push_back(chain_.back());
    return simplified_chain;
}

YAML::Node Calibration::toYaml() const {
    YAML::Node node;

    std::vector<Eigen::Matrix4d> chain = getSimplified();

    for (std::size_t i = 0; i < link_names_.size(); ++i) {
        YAML::Node link;
        link["x"] = chain[i](0, 3);
        link["y"] = chain[i](1, 3);
        link["z"] = chain[i](2, 3);
        Eigen::Matrix3d rot = chain[i].topLeftCorner(3, 3);
        Eigen::Vector3d rpy = rot.eulerAngles(2, 1, 0);
        link["roll"] = rpy[2];
        link["pitch"] = rpy[1];
        link["yaw"] = rpy[0];
        node["kinematics"][link_names_[i]] = link;
    }

    return node;
}
}  // namespace cs_calibration
