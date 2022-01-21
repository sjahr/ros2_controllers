// Copyright 2020 PAL Robotics S.L.
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

#ifndef EFFORT_CONTROLLERS__JOINT_GROUP_EFFORT_CONTROLLER_HPP_
#define EFFORT_CONTROLLERS__JOINT_GROUP_EFFORT_CONTROLLER_HPP_

#include <string>

#include "controller_interface/controller_interface.hpp"
#include "effort_controllers/visibility_control.h"
#include "forward_command_controller/forward_command_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

#include "control_toolbox/pid.hpp"

namespace effort_controllers
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * \brief Forward command controller for a set of effort controlled joints (linear or angular).
 *
 * This class forwards the commanded efforts down to a set of joints.
 *
 * \param joints Names of the joints to control.
 *
 * Subscribes to:
 * - \b command (std_msgs::msg::Float64MultiArray) : The effort commands to apply.
 */
class JointGroupEffortController : public forward_command_controller::ForwardCommandController
{
public:
  EFFORT_CONTROLLERS_PUBLIC
  JointGroupEffortController();

  EFFORT_CONTROLLERS_PUBLIC
  CallbackReturn on_init() override;

  /**
   * @brief command_interface_configuration This controller requires the velocity
   * state interfaces for the controlled joints
   */
  EFFORT_CONTROLLERS_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  EFFORT_CONTROLLERS_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  EFFORT_CONTROLLERS_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  EFFORT_CONTROLLERS_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  EFFORT_CONTROLLERS_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  bool read_state_from_hardware(std::vector<double> & joint_velocities);

  //bool read_state_from_command_interfaces(std::vector<double> & joint_velocities);

protected:
  std::vector<std::string> joint_names_;
  std::vector<std::string> state_interface_types_;

  std::vector<std::unique_ptr<control_toolbox::Pid>> pids_;
  std::chrono::steady_clock::time_point last_update_time_;
  std::vector<double> velocity_ff_;

  bool has_velocity_state_interface_ = false;

  // To reduce number of variables and to make the code shorter the interfaces are ordered in types
  // as the following constants
  const std::vector<std::string> allowed_interface_types_ = {
    hardware_interface::HW_IF_POSITION,
    hardware_interface::HW_IF_VELOCITY,
  };

  template <typename T>
  using InterfaceReferences = std::vector<std::vector<std::reference_wrapper<T>>>;
  InterfaceReferences<hardware_interface::LoanedStateInterface> joint_state_interface_;
};

}  // namespace effort_controllers

#endif  // EFFORT_CONTROLLERS__JOINT_GROUP_EFFORT_CONTROLLER_HPP_
