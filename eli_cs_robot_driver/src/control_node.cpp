// Copyright 2020 ROS2-Control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <errno.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "controller_manager/controller_manager.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_helpers.hpp"

using namespace std::chrono_literals;

namespace
{
// Reference: https://man7.org/linux/man-pages/man2/sched_setparam.2.html
// This value is used when configuring the main loop to use SCHED_FIFO scheduling
// We use a midpoint RT priority to allow maximum flexibility to users
int const kSchedPriority = 50;

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  const std::string manager_node_name = "controller_manager";

  // Jazzy: use controller_manager helper for NodeOptions so parameters are declared correctly
  rclcpp::NodeOptions cm_node_options = controller_manager::get_cm_node_options();

  // Pass through ROS arguments to the controller manager node options
  std::vector<std::string> node_arguments = cm_node_options.arguments();
  for (int i = 1; i < argc; ++i)
  {
    if (node_arguments.empty() && std::string(argv[i]) != "--ros-args")
    {
      // Reject non-ROS args in a simple way
      continue;
    }
    node_arguments.push_back(argv[i]);
  }
  cm_node_options.arguments(node_arguments);

  auto cm = std::make_shared<controller_manager::ControllerManager>(
    executor, manager_node_name, /*namespace=*/"", cm_node_options);

  const bool use_sim_time = cm->get_parameter_or("use_sim_time", false);

  // Keep master behavior: decide defaults based on realtime kernel presence
  const bool has_realtime = realtime_tools::has_realtime_kernel();

  // lock_memory default: true if realtime kernel, otherwise false (unless user sets it)
  const bool lock_memory = cm->get_parameter_or<bool>("lock_memory", has_realtime);
  if (lock_memory)
  {
    const auto lock_result = realtime_tools::lock_memory();
    if (!lock_result.first)
    {
      RCLCPP_WARN(cm->get_logger(), "Unable to lock the memory: '%s'", lock_result.second.c_str());
    }
  }

  RCLCPP_INFO(cm->get_logger(), "update rate is %d Hz", cm->get_update_rate());

  // Jazzy template supports overruns handling
  const bool manage_overruns = cm->get_parameter_or<bool>("overruns.manage", true);
  RCLCPP_INFO(
    cm->get_logger(), "Overruns handling is : %s", manage_overruns ? "enabled" : "disabled");

  const int thread_priority = cm->get_parameter_or<int>("thread_priority", kSchedPriority);
  RCLCPP_INFO(
    cm->get_logger(), "Spawning %s RT thread with scheduler priority: %d", cm->get_name(),
    thread_priority);

  std::thread cm_thread(
    [cm, thread_priority, use_sim_time, manage_overruns, has_realtime]()
    {
      // CPU affinity: keep Jazzy template behavior (supports int or int_array)
      rclcpp::Parameter cpu_affinity_param;
      if (cm->get_parameter("cpu_affinity", cpu_affinity_param))
      {
        std::vector<int> cpus;
        if (cpu_affinity_param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        {
          cpus = {static_cast<int>(cpu_affinity_param.as_int())};
        }
        else if (cpu_affinity_param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY)
        {
          const auto arr = cpu_affinity_param.as_integer_array();
          cpus.reserve(arr.size());
          std::for_each(arr.begin(), arr.end(), [&cpus](int cpu) {
            cpus.push_back(static_cast<int>(cpu));
          });
        }

        if (!cpus.empty())
        {
          const auto affinity_result = realtime_tools::set_current_thread_affinity(cpus);
          if (!affinity_result.first)
          {
            RCLCPP_WARN(
              cm->get_logger(), "Unable to set the CPU affinity : '%s'",
              affinity_result.second.c_str());
          }
        }
      }

      // Keep master behavior: only try SCHED_FIFO if realtime kernel is present
      if (has_realtime)
      {
        if (!realtime_tools::configure_sched_fifo(thread_priority))
        {
          RCLCPP_WARN(
            cm->get_logger(),
            "Could not enable FIFO RT scheduling policy: with error number <%i>(%s). See "
            "[https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html] "
            "for details on how to enable realtime scheduling.",
            errno, strerror(errno));
        }
        else
        {
          RCLCPP_INFO(
            cm->get_logger(), "Successful set up FIFO RT scheduling policy with priority %i.",
            thread_priority);
        }
      }
      else
      {
        RCLCPP_WARN(
          cm->get_logger(),
          "No real-time kernel detected on this system. See "
          "[https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html] "
          "for details on how to enable realtime scheduling.");
      }

      // Jazzy: wait for clock availability (important for sim time)
      cm->get_clock()->wait_until_started();

      const auto period = std::chrono::nanoseconds(1'000'000'000 / cm->get_update_rate());

      // Initialize timing using trigger clock (Jazzy pattern)
      rclcpp::Time previous_time = cm->get_trigger_clock()->now();
      std::this_thread::sleep_for(period);

      std::chrono::steady_clock::time_point next_iteration_time{std::chrono::steady_clock::now()};

      while (rclcpp::ok())
      {
        const auto current_time = cm->get_trigger_clock()->now();
        const auto measured_period = current_time - previous_time;
        previous_time = current_time;

        // Execute update loop
        cm->read(cm->get_trigger_clock()->now(), measured_period);
        cm->update(cm->get_trigger_clock()->now(), measured_period);
        cm->write(cm->get_trigger_clock()->now(), measured_period);

        // wait until we hit the end of the period
        if (use_sim_time)
        {
          try
          {
            cm->get_clock()->sleep_until(current_time + rclcpp::Duration(period));
          }
          catch (const std::runtime_error & e)
          {
            RCLCPP_ERROR(
              cm->get_logger(),
              "sleep_until failed with error: %s. Exiting control loop and aborting....", e.what());
            break;
          }
        }
        else
        {
          next_iteration_time += period;
          const auto time_now = std::chrono::steady_clock::now();

          if (manage_overruns && next_iteration_time < time_now)
          {
            const double time_diff_ms =
              static_cast<double>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(time_now - next_iteration_time)
                  .count()) /
              1.e6;

            const double cm_period_ms = 1.e3 / static_cast<double>(cm->get_update_rate());
            const int overrun_count = static_cast<int>(std::ceil(time_diff_ms / cm_period_ms));

            RCLCPP_WARN_THROTTLE(
              cm->get_logger(), *cm->get_clock(), 1000,
              "Overrun detected! The controller manager missed its desired rate of %d Hz. The loop "
              "took %f ms (missed cycles : %d).",
              cm->get_update_rate(), time_diff_ms + cm_period_ms, overrun_count + 1);

            next_iteration_time += (overrun_count * period);
          }

          std::this_thread::sleep_until(next_iteration_time);
        }
      }
    });

  executor->add_node(cm);
  executor->spin();

  cm_thread.join();
  rclcpp::shutdown();
  return 0;
}
