/**
 * @author Chen Shichao
 * @date 2024-06-27
 */

#ifndef CS_CALIBRATION__CALIBRATION_HPP_
#define CS_CALIBRATION__CALIBRATION_HPP_

#include <Eigen/Dense>
#include <fstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

namespace cs_calibration
{
/**
 * @brief An internal representation of a DH-parametrized link. Each segment consists of parameters a, d, alpha and theta.
 */
struct DHSegment
{
  double d_;
  double a_;
  double theta_;
  double alpha_;

  /**
   * @brief Creates an element with defined elements.
   */
  DHSegment(const double d, const double a, const double theta, const double alpha)
    : d_(d), a_(a), theta_(theta), alpha_(alpha)
  {
  }

  /**
   * @brief Creates a Segment with all elements equal to zero.
   */
  DHSegment() : d_(0), a_(0), theta_(0), alpha_(0)
  {
  }
};

/**
 * @brief Internal representation of a robot based on DH parameters.
 */
struct DHRobot
{
  std::vector<DHSegment> segments_;

  /**
   * @brief Create a new robot representation giving a set of DHSegment objects
   */
  explicit DHRobot(const std::vector<DHSegment>& segments)
  {
    segments_= segments;
  }

  DHRobot() = default;
};

/**
 * @brief Class that handles the calibration for ELITE CS Robot
 */
class Calibration
{
public:
  explicit Calibration(const DHRobot& robot);
  ~Calibration();

  /**
   * @brief Get the transformation matrix representation of the chain as constructed by the
   * DH parameters.
   *
   * This will contain twice as many transformation matrices as joints, as for each set of DH
   * parameters two matrices are generated. If you'd like to receive one matrix per joint instead,
   * use the getSimplified() function instead.
   *
   * @returns A vector of 4x4 transformation matrices, two for each joint going from the base to the
   * tcp.
   */
  std::vector<Eigen::Matrix4d> getChain()
  {
    return chain_;
  }

  /**
   * @brief Get the transformation matrix representation of the chain, where each joint is
   * represented by one matrix.
   *
   * @returns Vector of 4x4 transformation matrices, one for each joint going from the base to the tcp.
   */
  std::vector<Eigen::Matrix4d> getSimplified() const;

  /**
   * @brief Generates a yaml representation of all transformation matrices as returned by
   * getSimplified()
   *
   * @returns A YAML tree representing all transformation matrices.
   */
  YAML::Node toYaml() const;

  /**
   * @brief Calculates the forwart kinematics given a joint configuration with respect to the base link.
   *
   * @param joint_values Joint values for which the forward kinematics should be calculated.
   * @param link_nr If given, the cartesian position for this joint (starting at 1) is returned. By default the 6th joint is used.
   *
   * @returns Transformation matrix giving the full pose of the requested link in base coordinates.
   */
  Eigen::Matrix4d calcForwardKinematics(const Eigen::Matrix<double, 6, 1>& joint_values, const size_t link_nr = 6);

private:

  // Builds the chain from robot_parameters_
  void buildChain();

  DHRobot robot_parameters_;
  std::vector<std::string> link_names_ = { "shoulder", "upperarm", "forearm", "wrist_1", "wrist_2", "wrist_3" };

  std::vector<Eigen::Matrix4d> chain_;
};
}  // namespace cs_calibration
#endif  // CS_CALIBRATION__CALIBRATION_HPP_
