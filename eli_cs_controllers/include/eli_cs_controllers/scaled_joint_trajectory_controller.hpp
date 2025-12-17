#ifndef _ELITE_CS_CONTROLLER__SCALED_JOINT_TRAJECTORY_CONTROLLER_HPP_
#define _ELITE_CS_CONTROLLER__SCALED_JOINT_TRAJECTORY_CONTROLLER_HPP_
#include <limits>
#include <memory>
#include <vector>
#include "angles/angles.h"
#include "eli_cs_controllers/scaled_joint_trajectory_controller_parameters.hpp"
#include "joint_trajectory_controller/joint_trajectory_controller.hpp"
#include "joint_trajectory_controller/trajectory.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "realtime_tools/realtime_buffer.hpp"

namespace ELITE_CS_CONTROLLER {

class ScaledJointTrajectoryController : public joint_trajectory_controller::JointTrajectoryController {
   public:
    ScaledJointTrajectoryController() = default;
    ~ScaledJointTrajectoryController() override = default;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
    controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    CallbackReturn on_init() override;

   protected:
    struct TimeData {
        TimeData() : time(0.0), period(rclcpp::Duration::from_nanoseconds(0)), uptime(0.0) {}
        rclcpp::Time time;
        rclcpp::Duration period;
        rclcpp::Time uptime;
    };

   private:
    double scaling_factor_{1.0};
    bool has_scaling_interface_{false};
    size_t scaling_state_interface_index_{std::numeric_limits<size_t>::max()};

    realtime_tools::RealtimeBuffer<TimeData> time_data_;

    std::shared_ptr<scaled_joint_trajectory_controller::ParamListener> scaled_param_listener_;
    scaled_joint_trajectory_controller::Params scaled_params_;

    double read_and_clamp_scaling_factor_();

    template <typename T>
    void assign_interface_from_point(const T& joint_interface, const std::vector<double>& trajectory_point_interface) {
        for (size_t index = 0; index < dof_; ++index) {
            joint_interface[index].get().set_value(trajectory_point_interface[index]);
        }
    }
};

}  // namespace ELITE_CS_CONTROLLER

#endif
