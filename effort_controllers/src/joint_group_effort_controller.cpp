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

#include <string>

#include "controller_interface/helpers.hpp"
#include "effort_controllers/joint_group_effort_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/parameter.hpp"

namespace effort_controllers
{
JointGroupEffortController::JointGroupEffortController()
: forward_command_controller::ForwardCommandController()
{
  logger_name_ = "joint effort controller";
  interface_name_ = hardware_interface::HW_IF_EFFORT;
}

CallbackReturn JointGroupEffortController::on_init()
{
  auto ret = forward_command_controller::ForwardCommandController::on_init();
  if (ret != CallbackReturn::SUCCESS)
  {
    return ret;
  }

  try
  {
    // Explicitly set the interface parameter declared by the forward_command_controller
    // to match the value set in the JointGroupEffortController constructor.
    get_node()->set_parameter(
      rclcpp::Parameter("interface_name", hardware_interface::HW_IF_EFFORT));
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);
  auto_declare<std::vector<std::string>>("joints", joint_names_);
  for (const auto & joint_name : joint_names_)
  {
    // Init PID gains from ROS parameter server
    const std::string prefix = "gains." + joint_name;
    const auto k_p = auto_declare<double>(prefix + ".p", 0.0);
    const auto k_i = auto_declare<double>(prefix + ".i", 0.0);
    const auto k_d = auto_declare<double>(prefix + ".d", 0.0);
    const auto i_clamp = auto_declare<double>(prefix + ".i_clamp", 0.0);
    const auto velocity_ff = auto_declare<double>("velocity_ff." + joint_name, 0.0);
    // Initialize PID
    pids_.push_back(std::make_unique<control_toolbox::Pid>(k_p, k_i, k_d, i_clamp, -i_clamp));
    velocity_ff_.push_back(velocity_ff);
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointGroupEffortController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  // Load command interfaces
  auto ret = ForwardCommandController::on_activate(previous_state);
  if (ret != CallbackReturn::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to call on_activate for ForwardCommandController");
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(node_->get_logger(), "Got command interface");

  // Load state interface
  for (const auto & interface : state_interface_types_)
  {
    auto it =
      std::find(allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    auto index = std::distance(allowed_interface_types_.begin(), it);
    if (!controller_interface::get_ordered_interfaces(
          state_interfaces_, joint_names_, interface, joint_state_interface_[index]))
    {
      RCLCPP_ERROR(
        node_->get_logger(), "Expected %zu '%s' state interfaces, got %zu.", joint_names_.size(),
        interface.c_str(), joint_state_interface_[index].size());
      return CallbackReturn::ERROR;
    }
  }

  // Hold position till new command arrives
  std::vector<double> zero_vel;
  zero_vel.resize(joint_names_.size());
  for (auto & joint_vel : zero_vel)
  {
    joint_vel = 0.0;
  }
  auto zero_velocities = std::make_shared<std_msgs::msg::Float64MultiArray>();
  zero_velocities->data = zero_vel;
  rt_command_ptr_.writeFromNonRT(zero_velocities);
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
JointGroupEffortController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(joint_names_.size() * state_interface_types_.size());
  for (const auto & joint_name : joint_names_)
  {
    for (const auto & interface_type : state_interface_types_)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }
  return conf;
}

controller_interface::return_type JointGroupEffortController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto joint_commands = rt_command_ptr_.readFromRT();

  // no command received yet
  if (!joint_commands || !(*joint_commands))
  {
    return controller_interface::return_type::OK;
  }

  if ((*joint_commands)->data.size() != command_interfaces_.size())
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *node_->get_clock(), 1000,
      "command size (%zu) does not match number of interfaces (%zu)",
      (*joint_commands)->data.size(), command_interfaces_.size());
    return controller_interface::return_type::ERROR;
  }

  // Get current joint velocities
  std::vector<double> current_joint_velocities;
  read_state_from_hardware(current_joint_velocities);

  const auto period = std::chrono::steady_clock::now() - last_update_time_;
  for (size_t index = 0; index < command_interfaces_.size(); ++index)
  {
    const double command =
      ((*joint_commands)->data[index] * velocity_ff_[index]) +
      pids_[index]->computeCommand(
        (*joint_commands)->data[index] - current_joint_velocities[index], period.count());
    command_interfaces_[index].set_value(command);
  }
  last_update_time_ = std::chrono::steady_clock::now();

  return controller_interface::return_type::OK;
}

CallbackReturn JointGroupEffortController::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  ForwardCommandController::on_configure(previous_state);
  const auto logger = node_->get_logger();

  // update parameters
  joint_names_ = node_->get_parameter("joints").as_string_array();

  for (const auto & pid : pids_)
  {
    pid->reset();
  }

  if (joint_names_.empty())
  {
    RCLCPP_WARN(logger, "'joints' parameter is empty.");
  }

  for (const auto & joint_name : joint_names_)
  {
    // Init PID gains from ROS parameter server
    const std::string prefix = "gains." + joint_name;
    const auto k_p = auto_declare<double>(prefix + ".p", 0.0);
    const auto k_i = auto_declare<double>(prefix + ".i", 0.0);
    const auto k_d = auto_declare<double>(prefix + ".d", 0.0);
    const auto i_clamp = auto_declare<double>(prefix + ".i_clamp", 0.0);
    const auto velocity_ff = auto_declare<double>("velocity_ff." + joint_name, 0.0);
    // Initialize PID
    pids_.push_back(std::make_unique<control_toolbox::Pid>(k_p, k_i, k_d, i_clamp, -i_clamp));
    velocity_ff_.push_back(velocity_ff);
  }

  // Read always state interfaces from the parameter because they can be used
  // independently from the controller's type.
  // Specialized, child controllers should set its default value.
  state_interface_types_ = node_->get_parameter("state_interfaces").as_string_array();

  if (state_interface_types_.empty())
  {
    RCLCPP_ERROR(logger, "'state_interfaces' parameter is empty.");
    return CallbackReturn::FAILURE;
  }
  RCLCPP_ERROR(logger, "Successfully configured joint_group_effort_controller :)");

  joint_state_interface_.resize(allowed_interface_types_.size());
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointGroupEffortController::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  auto ret = ForwardCommandController::on_deactivate(previous_state);

  for (size_t index = 0; index < allowed_interface_types_.size(); ++index)
  {
    joint_state_interface_[index].clear();
  }
  release_interfaces();
  // stop all joints
  for (auto & command_interface : command_interfaces_)
  {
    command_interface.set_value(0.0);
  }

  return ret;
}

bool JointGroupEffortController::read_state_from_hardware(std::vector<double> & joint_velocities)
{
  const auto joint_num = joint_names_.size();
  joint_velocities.resize(joint_num);
  auto assign_point_from_interface = [&, joint_num](
                                       std::vector<double> & state, const auto & joint_inteface) {
    for (size_t index = 0; index < joint_num; ++index)
    {
      state[index] = joint_inteface[index].get().get_value();
    }
  };

  // Assign values from the hardware
  // velocity and acceleration states are optional
  if (has_velocity_state_interface_)
  {
    assign_point_from_interface(joint_velocities, joint_state_interface_[1]);
    // Acceleration is used only in combination with velocity
  }
  else
  {
    // Cannot read velocities
    return false;
  }
  return true;
}

/*
bool JointGroupEffortController::read_state_from_command_interfaces(std::vector<double>& joint_velocities)
{
  bool has_values = true;

  const auto joint_num = joint_names_.size();
  joint_velocities.resize(joint_num);
  auto assign_point_from_interface =
    [&, joint_num](std::vector<double> & state, const auto & joint_inteface) {
      for (size_t index = 0; index < joint_num; ++index)
      {
        state[index] = joint_inteface[index].get().get_value();
      }
    };

  auto interface_has_values = [](const auto & joint_interface) {
    return std::find_if(joint_interface.begin(), joint_interface.end(), [](const auto & interface) {
             return std::isnan(interface.get().get_value());
           }) == joint_interface.end();
  };

  // Assign values from the command interfaces as state. Therefore needs check for both.
  // velocity and acceleration states are optional
  if (has_velocity_state_interface_)
  {
    if (has_velocity_command_interface_ && interface_has_values(joint_command_interface_[1]))
    {
      assign_point_from_interface(joint_velocities, joint_command_interface_[1]);
    }
    else
    {
      joint_velocities.clear();
      has_values = false;
    }
  }
  return has_values;
}*/

}  // namespace effort_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  effort_controllers::JointGroupEffortController, controller_interface::ControllerInterface)
