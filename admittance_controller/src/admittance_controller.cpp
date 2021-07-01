// Copyright (c) 2021, PickNik, Inc.
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
//
/// \authors: Denis Stogl, Andy Zelenak

#include "admittance_controller/admittance_controller.hpp"

#include <chrono>
#include <functional>
#include <limits>
#include <math.h>
#include <string>
#include <vector>

#include "admittance_controller/admittance_rule_impl.hpp"
#include "angles/angles.h"
#include "controller_interface/helpers.hpp"
#include "filters/filter_chain.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "joint_limits/joint_limits.hpp"
#include "joint_limits/joint_limits_rosparam.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

constexpr size_t ROS_LOG_THROTTLE_PERIOD = 1 * 1000;  // Milliseconds to throttle logs inside loops

namespace admittance_controller
{
AdmittanceController::AdmittanceController()
: controller_interface::ControllerInterface()
{
}

controller_interface::return_type AdmittanceController::init(const std::string & controller_name)
{
  auto ret = ControllerInterface::init(controller_name);
  if (ret != controller_interface::return_type::OK) {
    return ret;
  }

  try {
    // TODO: use variables as parameters
    get_node()->declare_parameter<std::vector<std::string>>("joints", {});
    get_node()->declare_parameter<std::vector<std::string>>("command_interfaces", {});
    get_node()->declare_parameter<std::vector<std::string>>("state_interfaces", {});
    get_node()->declare_parameter<std::string>("ft_sensor_name", "");
    get_node()->declare_parameter<bool>("use_joint_commands_as_input", false);
    get_node()->declare_parameter<bool>("open_loop_control", false);

    get_node()->declare_parameter<std::string>("IK.base", "");
    get_node()->declare_parameter<std::string>("IK.tip", "");
    // TODO(destogl): enable when IK-plugin support is added
//     get_node()->declare_parameter<std::string>("IK.plugin", "");
    get_node()->declare_parameter<std::string>("IK.group_name", "");

    get_node()->declare_parameter<std::string>("control_frame", "");
    get_node()->declare_parameter<std::string>("endeffector_frame", "");
    get_node()->declare_parameter<std::string>("fixed_world_frame", "");
    get_node()->declare_parameter<std::string>("sensor_frame", "");

    // TODO(destogl): enable when force/position control is implemented
//     get_node()->declare_parameter<bool>("admitance.unified_mode", false);
    get_node()->declare_parameter<bool>("admittance.selected_axes.x", false);
    get_node()->declare_parameter<bool>("admittance.selected_axes.y", false);
    get_node()->declare_parameter<bool>("admittance.selected_axes.z", false);
    get_node()->declare_parameter<bool>("admittance.selected_axes.rx", false);
    get_node()->declare_parameter<bool>("admittance.selected_axes.ry", false);
    get_node()->declare_parameter<bool>("admittance.selected_axes.rz", false);

    get_node()->declare_parameter<double>("admittance.mass.x", std::numeric_limits<double>::quiet_NaN());
    get_node()->declare_parameter<double>("admittance.mass.y", std::numeric_limits<double>::quiet_NaN());
    get_node()->declare_parameter<double>("admittance.mass.z", std::numeric_limits<double>::quiet_NaN());
    get_node()->declare_parameter<double>("admittance.mass.rx", std::numeric_limits<double>::quiet_NaN());
    get_node()->declare_parameter<double>("admittance.mass.ry", std::numeric_limits<double>::quiet_NaN());
    get_node()->declare_parameter<double>("admittance.mass.rz", std::numeric_limits<double>::quiet_NaN());

    get_node()->declare_parameter<double>("admittance.damping.x", std::numeric_limits<double>::quiet_NaN());
    get_node()->declare_parameter<double>("admittance.damping.y", std::numeric_limits<double>::quiet_NaN());
    get_node()->declare_parameter<double>("admittance.damping.z", std::numeric_limits<double>::quiet_NaN());
    get_node()->declare_parameter<double>("admittance.damping.rx", std::numeric_limits<double>::quiet_NaN());
    get_node()->declare_parameter<double>("admittance.damping.ry", std::numeric_limits<double>::quiet_NaN());
    get_node()->declare_parameter<double>("admittance.damping.rz", std::numeric_limits<double>::quiet_NaN());

    get_node()->declare_parameter<double>("admittance.damping_ratio.x", std::numeric_limits<double>::quiet_NaN());
    get_node()->declare_parameter<double>("admittance.damping_ratio.y", std::numeric_limits<double>::quiet_NaN());
    get_node()->declare_parameter<double>("admittance.damping_ratio.z", std::numeric_limits<double>::quiet_NaN());
    get_node()->declare_parameter<double>("admittance.damping_ratio.rx", std::numeric_limits<double>::quiet_NaN());
    get_node()->declare_parameter<double>("admittance.damping_ratio.ry", std::numeric_limits<double>::quiet_NaN());
    get_node()->declare_parameter<double>("admittance.damping_ratio.rz", std::numeric_limits<double>::quiet_NaN());

    get_node()->declare_parameter<double>("admittance.stiffness.x", std::numeric_limits<double>::quiet_NaN());
    get_node()->declare_parameter<double>("admittance.stiffness.y", std::numeric_limits<double>::quiet_NaN());
    get_node()->declare_parameter<double>("admittance.stiffness.z", std::numeric_limits<double>::quiet_NaN());
    get_node()->declare_parameter<double>("admittance.stiffness.rx", std::numeric_limits<double>::quiet_NaN());
    get_node()->declare_parameter<double>("admittance.stiffness.ry", std::numeric_limits<double>::quiet_NaN());
    get_node()->declare_parameter<double>("admittance.stiffness.rz", std::numeric_limits<double>::quiet_NaN());

  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::return_type::ERROR;
  }

  admittance_ = std::make_unique<admittance_controller::AdmittanceRule>();

  return controller_interface::return_type::OK;
}

CallbackReturn AdmittanceController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto error_if_empty = [&](const auto & parameter, const char * parameter_name) {
    if (parameter.empty()) {
      RCLCPP_ERROR(get_node()->get_logger(), "'%s' parameter was empty", parameter_name);
      return true;
    }
    return false;
  };

  auto get_string_array_param_and_error_if_empty = [&](
    std::vector<std::string> & parameter, const char * parameter_name) {
    parameter = get_node()->get_parameter(parameter_name).as_string_array();
    return error_if_empty(parameter, parameter_name);
  };

  auto get_string_param_and_error_if_empty = [&](
    std::string & parameter, const char * parameter_name) {
    parameter = get_node()->get_parameter(parameter_name).as_string();
    return error_if_empty(parameter, parameter_name);
  };

  // TODO(destogl): If we would use C++20 than we can use templates here
  auto get_bool_param_and_error_if_empty = [&](
    bool & parameter, const char * parameter_name) {
    parameter = get_node()->get_parameter(parameter_name).get_value<bool>();
    return false; // TODO(destogl): how to check "if_empty" for bool?
  };

  auto get_double_param_and_error_if_empty = [&](
    double & parameter, const char * parameter_name) {
    parameter = get_node()->get_parameter(parameter_name).get_value<double>();
    if (std::isnan(parameter)) {
      RCLCPP_ERROR(get_node()->get_logger(), "'%s' parameter was not set", parameter_name);
      return true;
    }
    return false;
  };

  auto get_double_params_and_error_if_both_empty_or_set = [&](
    double & parameter, const char * parameter_name,
    double & parameter2, const char * parameter_name2) {
    parameter = get_node()->get_parameter(parameter_name).get_value<double>();
    parameter2 = get_node()->get_parameter(parameter_name2).get_value<double>();
    if (std::isnan(parameter) == std::isnan(parameter2)) {
      RCLCPP_ERROR(get_node()->get_logger(), "'%s' and '%s' parameters were both '%s'set",
                   parameter_name, parameter_name2, (std::isnan(parameter) ? "not " : ""));
      return true;
    }
    return false;
    };

  if (
    get_string_array_param_and_error_if_empty(joint_names_, "joints") ||
    get_string_array_param_and_error_if_empty(command_interface_types_, "command_interfaces") ||
    get_string_array_param_and_error_if_empty(state_interface_types_, "state_interfaces") ||
    get_string_param_and_error_if_empty(ft_sensor_name_, "ft_sensor_name") ||
    get_bool_param_and_error_if_empty(use_joint_commands_as_input_, "use_joint_commands_as_input") ||
    get_bool_param_and_error_if_empty(admittance_->open_loop_control_, "open_loop_control") ||
    get_string_param_and_error_if_empty(admittance_->ik_base_frame_, "IK.base") ||
    get_string_param_and_error_if_empty(admittance_->ik_tip_frame_, "IK.tip") ||
    get_string_param_and_error_if_empty(admittance_->ik_group_name_, "IK.group_name") ||
    get_string_param_and_error_if_empty(admittance_->endeffector_frame_, "endeffector_frame") ||
    get_string_param_and_error_if_empty(admittance_->control_frame_, "control_frame") ||
    get_string_param_and_error_if_empty(admittance_->fixed_world_frame_, "fixed_world_frame") ||
    get_string_param_and_error_if_empty(admittance_->sensor_frame_, "sensor_frame") ||

    get_bool_param_and_error_if_empty(admittance_->selected_axes_[0], "admittance.selected_axes.x") ||
    get_bool_param_and_error_if_empty(admittance_->selected_axes_[1], "admittance.selected_axes.y") ||
    get_bool_param_and_error_if_empty(admittance_->selected_axes_[2], "admittance.selected_axes.z") ||
    get_bool_param_and_error_if_empty(admittance_->selected_axes_[3], "admittance.selected_axes.rx") ||
    get_bool_param_and_error_if_empty(admittance_->selected_axes_[4], "admittance.selected_axes.ry") ||
    get_bool_param_and_error_if_empty(admittance_->selected_axes_[5], "admittance.selected_axes.rz") ||

    get_double_param_and_error_if_empty(admittance_->mass_[0], "admittance.mass.x") ||
    get_double_param_and_error_if_empty(admittance_->mass_[1], "admittance.mass.y") ||
    get_double_param_and_error_if_empty(admittance_->mass_[2], "admittance.mass.z") ||
    get_double_param_and_error_if_empty(admittance_->mass_[3], "admittance.mass.rx") ||
    get_double_param_and_error_if_empty(admittance_->mass_[4], "admittance.mass.ry") ||
    get_double_param_and_error_if_empty(admittance_->mass_[5], "admittance.mass.rz") ||

    get_double_params_and_error_if_both_empty_or_set(
      admittance_->damping_[0], "admittance.damping.x",
      admittance_->damping_ratio_[0], "admittance.damping_ratio.x") ||
    get_double_params_and_error_if_both_empty_or_set(
      admittance_->damping_[1], "admittance.damping.y",
      admittance_->damping_ratio_[1], "admittance.damping_ratio.y") ||
    get_double_params_and_error_if_both_empty_or_set(
      admittance_->damping_[2], "admittance.damping.z",
      admittance_->damping_ratio_[2], "admittance.damping_ratio.z") ||
    get_double_params_and_error_if_both_empty_or_set(
      admittance_->damping_[3], "admittance.damping.rx",
      admittance_->damping_ratio_[3], "admittance.damping_ratio.rx") ||
    get_double_params_and_error_if_both_empty_or_set(
      admittance_->damping_[4], "admittance.damping.ry",
      admittance_->damping_ratio_[4], "admittance.damping_ratio.ry") ||
    get_double_params_and_error_if_both_empty_or_set(
      admittance_->damping_[5], "admittance.damping.rz",
      admittance_->damping_ratio_[5], "admittance.damping_ratio.rz") ||

    get_double_param_and_error_if_empty(admittance_->stiffness_[0], "admittance.stiffness.x") ||
    get_double_param_and_error_if_empty(admittance_->stiffness_[1], "admittance.stiffness.y") ||
    get_double_param_and_error_if_empty(admittance_->stiffness_[2], "admittance.stiffness.z") ||
    get_double_param_and_error_if_empty(admittance_->stiffness_[3], "admittance.stiffness.rx") ||
    get_double_param_and_error_if_empty(admittance_->stiffness_[4], "admittance.stiffness.ry") ||
    get_double_param_and_error_if_empty(admittance_->stiffness_[5], "admittance.stiffness.rz")
    )
  {
    return CallbackReturn::ERROR;
  }

  // Convert the damping ratio (if given) to mass/spring/damper representation
  admittance_->convert_damping_ratio_to_damping();

  try {
    admittance_->filter_chain_ =
    std::make_unique<filters::FilterChain<geometry_msgs::msg::WrenchStamped>>(
      "geometry_msgs::msg::WrenchStamped");
  } catch (const std::exception & e) {
    fprintf(
      stderr, "Exception thrown during filter chain creation at configure stage with message : %s \n",
      e.what());
    return CallbackReturn::ERROR;
  }

  if (!admittance_->filter_chain_->configure("input_wrench_filter_chain",
    get_node()->get_node_logging_interface(), get_node()->get_node_parameters_interface()))
  {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Could not configure sensor filter chain, please check if the "
                 "parameters are provided correctly.");
    return CallbackReturn::ERROR;
  }

  // Check if only allowed interface types are used and initialize storage to avoid memory
  // allocation during activation
  joint_command_interface_.resize(allowed_interface_types_.size());
  for (const auto & interface : command_interface_types_) {
    auto it = std::find(
      allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    if (it == allowed_interface_types_.end()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Command interface type '" + interface + "' not allowed!");
      return CallbackReturn::ERROR;
    }
  }

  if (controller_interface::interface_list_contains_interface_type(
    command_interface_types_, hardware_interface::HW_IF_POSITION)) {
    has_position_command_interface_ = true;
  }
  if (controller_interface::interface_list_contains_interface_type(
    command_interface_types_, hardware_interface::HW_IF_VELOCITY)) {
    has_velocity_command_interface_ = true;
  }

  // Check if only allowed interface types are used and initialize storage to avoid memory
  // allocation during activation
  joint_state_interface_.resize(allowed_interface_types_.size());
  for (const auto & interface : state_interface_types_) {
    auto it = std::find(
      allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    if (it == allowed_interface_types_.end()) {
      RCLCPP_ERROR(get_node()->get_logger(), "State interface type '" + interface + "' not allowed!");
      return CallbackReturn::ERROR;
    }
  }

  if (!controller_interface::interface_list_contains_interface_type(
    state_interface_types_, hardware_interface::HW_IF_POSITION)) {
    RCLCPP_ERROR(get_node()->get_logger(), "State interface type '" + std::string(hardware_interface::HW_IF_POSITION) + "' has to be always present allowed!");
    return CallbackReturn::ERROR;
  }

  if (controller_interface::interface_list_contains_interface_type(
    state_interface_types_, hardware_interface::HW_IF_VELOCITY)) {
    has_velocity_state_interface_ = true;
  }

  auto get_interface_list = [](const std::vector<std::string> & interface_types) {
    std::stringstream ss_command_interfaces;
    for (size_t index = 0; index < interface_types.size(); ++index) {
      if (index != 0) {
        ss_command_interfaces << " ";
      }
      ss_command_interfaces << interface_types[index];
    }
    return ss_command_interfaces.str();
  };

  // Print output so users can be sure the interface setup is correct
  RCLCPP_INFO(
    get_node()->get_logger(), "Command interfaces are [%s] and and state interfaces are [%s].",
    get_interface_list(command_interface_types_).c_str(),
              get_interface_list(state_interface_types_).c_str());

  auto num_joints = joint_names_.size();

  // Initialize joint limits
  joint_limits_.resize(num_joints);
  for (auto i = 0ul; i < num_joints; ++i) {
    joint_limits::declare_parameters(joint_names_[i], get_node());
    joint_limits::get_joint_limits(joint_names_[i], get_node(), joint_limits_[i]);
    RCLCPP_INFO(get_node()->get_logger(), "Joint '%s':\n  has position limits: %s [%e, %e]"
                "\n  has velocity limits: %s [%e]\n  has acceleration limits: %s [%e]",
                joint_names_[i].c_str(), joint_limits_[i].has_position_limits ? "true" : "false",
                joint_limits_[i].min_position, joint_limits_[i].max_position,
                joint_limits_[i].has_velocity_limits ? "true" : "false",
                joint_limits_[i].max_velocity,
                joint_limits_[i].has_acceleration_limits ? "true" : "false",
                joint_limits_[i].max_acceleration
               );
  }

  // Initialize FTS semantic semantic_component
  force_torque_sensor_ = std::make_unique<semantic_components::ForceTorqueSensor>(
    semantic_components::ForceTorqueSensor(ft_sensor_name_));

  // Subscribers and callbacks
  if (admittance_->unified_mode_) {
    auto callback_input_force = [&](const std::shared_ptr<ControllerCommandWrenchMsg> msg)
      -> void
      {
        input_wrench_command_.writeFromNonRT(msg);
      };
    input_wrench_command_subscriber_ = get_node()->create_subscription<ControllerCommandWrenchMsg>(
      "~/force_commands", rclcpp::SystemDefaultsQoS(), callback_input_force);
  }

  auto callback_input_joint = [&](const std::shared_ptr<ControllerCommandJointMsg> msg)
  -> void
  {
    input_joint_command_.writeFromNonRT(msg);
  };
  input_joint_command_subscriber_ = get_node()->create_subscription<ControllerCommandJointMsg>(
    "~/joint_commands", rclcpp::SystemDefaultsQoS(), callback_input_joint);

  auto callback_input_pose = [&](const std::shared_ptr<ControllerCommandPoseMsg> msg)
  -> void
  {
    input_pose_command_.writeFromNonRT(msg);
  };
  input_pose_command_subscriber_ = get_node()->create_subscription<ControllerCommandPoseMsg>(
    "~/pose_commands", rclcpp::SystemDefaultsQoS(), callback_input_pose);

  // TODO(destogl): Add subscriber for velocity scaling

  // State publisher
  s_publisher_ = get_node()->create_publisher<ControllerStateMsg>(
    "~/state", rclcpp::SystemDefaultsQoS());
  state_publisher_ = std::make_unique<ControllerStatePublisher>(s_publisher_);

  // Initialize state message
  state_publisher_->lock();
  state_publisher_->msg_.joint_names = joint_names_;
  state_publisher_->msg_.actual_joint_state.positions.resize(num_joints, 0.0);
  state_publisher_->msg_.desired_joint_state.positions.resize(num_joints, 0.0);
  state_publisher_->msg_.error_joint_state.positions.resize(num_joints, 0.0);
  state_publisher_->unlock();

  // Configure AdmittanceRule
  admittance_->configure(get_node());

  last_commanded_state_.positions.resize(num_joints);
  last_commanded_state_.velocities.resize(num_joints, 0.0);
  last_commanded_state_.accelerations.resize(num_joints, 0.0);

  if (use_joint_commands_as_input_) {
    RCLCPP_INFO(get_node()->get_logger(), "Using Joint input mode.");
  } else {
    RCLCPP_INFO(get_node()->get_logger(), "Using Cartesian input mode.");
  }
  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration AdmittanceController::command_interface_configuration()
const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.reserve(joint_names_.size() * command_interface_types_.size());
  for (const auto & joint : joint_names_) {
    for (const auto & interface : command_interface_types_) {
      command_interfaces_config.names.push_back(joint + "/" + interface);
    }
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration AdmittanceController::state_interface_configuration()
const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.reserve(6 + joint_names_.size() * state_interface_types_.size());

  state_interfaces_config.names = force_torque_sensor_->get_state_interface_names();

  for (const auto & joint : joint_names_) {
    for (const auto & interface : state_interface_types_) {
      state_interfaces_config.names.push_back(joint + "/" + interface);
    }
  }

  return state_interfaces_config;
}

CallbackReturn AdmittanceController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  const auto num_joints = joint_names_.size();

  // order all joints in the storage
  for (const auto & interface : command_interface_types_) {
    auto it = std::find(
      allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    auto index = std::distance(allowed_interface_types_.begin(), it);
    if (!controller_interface::get_ordered_interfaces(
      command_interfaces_, joint_names_, interface, joint_command_interface_[index]))
    {
      RCLCPP_ERROR(
        node_->get_logger(), "Expected %u '%s' command interfaces, got %u.",
                   num_joints, interface.c_str(), joint_command_interface_[index].size());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
  }
  for (const auto & interface : state_interface_types_) {
    auto it = std::find(
      allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    auto index = std::distance(allowed_interface_types_.begin(), it);
    if (!controller_interface::get_ordered_interfaces(
      state_interfaces_, joint_names_, interface, joint_state_interface_[index]))
    {
      RCLCPP_ERROR(
        node_->get_logger(), "Expected %u '%s' state interfaces, got %u.",
                   num_joints, interface.c_str(), joint_state_interface_[index].size());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
  }

  // Initialize interface of the FTS semantic semantic component
  force_torque_sensor_->assign_loaned_state_interfaces(state_interfaces_);

  // Initialize Admittance Rule from current states
  admittance_->reset();
  previous_time_ = get_node()->now();

  read_state_from_hardware(last_commanded_state_);
  // Handle restart of controller by reading last_commanded_state_ from commands if not nan
  read_state_from_command_interfaces(last_commanded_state_);

  // Set initial command values - initialize all to simplify update
  std::shared_ptr<ControllerCommandWrenchMsg> msg_wrench = std::make_shared<ControllerCommandWrenchMsg>();
  msg_wrench->header.frame_id = admittance_->control_frame_;
  input_wrench_command_.writeFromNonRT(msg_wrench);

  std::shared_ptr<ControllerCommandJointMsg> msg_joint = std::make_shared<ControllerCommandJointMsg>();
  msg_joint->joint_names = joint_names_;
  msg_joint->points.reserve(1);

  trajectory_msgs::msg::JointTrajectoryPoint trajectory_point;
  trajectory_point.positions.reserve(num_joints);
  trajectory_point.velocities.resize(num_joints, 0.0);
  // FIXME(destogl): ATTENTION: This does not work properly, so using velocity mode and commenting positions out!
//   for (auto index = 0u; index < num_joints; ++index) {
//     trajectory_point.positions.emplace_back(joint_state_interface_[0][index].get().get_value());
//   }
  msg_joint->points.emplace_back(trajectory_point);

  input_joint_command_.writeFromNonRT(msg_joint);

  std::shared_ptr<ControllerCommandPoseMsg> msg_pose = std::make_shared<ControllerCommandPoseMsg>();
  msg_pose->header.frame_id = admittance_->control_frame_;
  if (admittance_->get_pose_of_control_frame_in_base_frame(*msg_pose) !=
      controller_interface::return_type::OK)
  {
    RCLCPP_ERROR(node_->get_logger(),
                 "Can not find transform from '%s' to '%s' needed in the update loop",
                 admittance_->ik_base_frame_, admittance_->control_frame_);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  input_pose_command_.writeFromNonRT(msg_pose);

  return CallbackReturn::SUCCESS;
}

CallbackReturn AdmittanceController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Stop the robot
  for (auto index = 0ul; index < joint_names_.size(); ++index) {
    joint_command_interface_[0][index].get().set_value(
      joint_command_interface_[0][index].get().get_value());
  }

  for (auto index = 0ul; index < allowed_interface_types_.size(); ++index) {
    joint_command_interface_[index].clear();
    joint_state_interface_[index].clear();
  }
  release_interfaces();

  force_torque_sensor_->release_interfaces();

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type AdmittanceController::update()
{
  // get input commands
  auto input_wrench_cmd = input_wrench_command_.readFromRT();
  auto input_joint_cmd = input_joint_command_.readFromRT();
  auto input_pose_cmd = input_pose_command_.readFromRT();

  // Position has to always be there
  auto num_joints = joint_state_interface_[0].size();
  trajectory_msgs::msg::JointTrajectoryPoint current_joint_states;
  current_joint_states.positions.resize(num_joints, 0.0);
  trajectory_msgs::msg::JointTrajectoryPoint desired_joint_states;
  desired_joint_states.positions.resize(num_joints);
  desired_joint_states.velocities.resize(num_joints);

  read_state_from_hardware(current_joint_states);

  if (admittance_->open_loop_control_) {
    // TODO(destogl): This may not work in every case.
    // Please add checking which states are available and which not!
    current_joint_states = last_commanded_state_;
  }

  geometry_msgs::msg::Wrench ft_values;
  force_torque_sensor_->get_values_as_message(ft_values);

  auto duration_since_last_call = get_node()->now() - previous_time_;

  // TODO(destogl): Enable this when unified mode is used
//   if (admittance_->unified_mode_) {
  //     admittance_->update(current_joint_states, ft_values, **input_pose_cmd, **input_wrench_cmd, duration_since_last_call, desired_joint_states);
//   } else {

  // TODO(destogl): refactor this into different admittance controllers: 1. Pose input, Joint State input and Unified mode (is there need for switching between unified and non-unified mode?)
  if (use_joint_commands_as_input_) {
    std::array<double, 6> joint_deltas;
    // If there are no positions, expect velocities
    // TODO(destogl): add error handling
    if ((*input_joint_cmd)->points[0].positions.empty()) {
      for (auto index = 0u; index < num_joints; ++index) {
        joint_deltas[index] = (*input_joint_cmd)->points[0].velocities[index] * duration_since_last_call.seconds();
      }
    } else {
      for (auto index = 0u; index < num_joints; ++index) {
        // TODO(destogl): ATTENTION: This does not work properly, deltas are getting neutralized and robot is not moving on external forces
        // TODO(anyone): Is here OK to use shortest_angular_distance?
        joint_deltas[index] = angles::shortest_angular_distance(current_joint_states.positions[index], (*input_joint_cmd)->points[0].positions[index]);
      }
    }

    admittance_->update(current_joint_states, ft_values, joint_deltas, duration_since_last_call, desired_joint_states);
  } else {
    admittance_->update(current_joint_states, ft_values, **input_pose_cmd, duration_since_last_call, desired_joint_states);
  }
//   }
  previous_time_ = get_node()->now();

  // TODO: Replace with limit plugin
  if(current_joint_states.velocities.empty()) {
    // First update() after activating does not have velocity available, assume 0
    current_joint_states.velocities.resize(num_joints, 0.0);
  }

  // Clamp velocities to limits
  for (auto index = 0u; index < num_joints; ++index) {
    if(joint_limits_[index].has_velocity_limits) {
      if(std::abs(desired_joint_states.velocities[index]) > joint_limits_[index].max_velocity) {
        RCLCPP_WARN_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), ROS_LOG_THROTTLE_PERIOD, "Joint(s) would exceed velocity limits, limiting");
        desired_joint_states.velocities[index] = copysign(joint_limits_[index].max_velocity, desired_joint_states.velocities[index]);
        double accel = (desired_joint_states.velocities[index] - current_joint_states.velocities[index]) / duration_since_last_call.seconds();
        // Recompute position
        desired_joint_states.positions[index] = current_joint_states.positions[index] + current_joint_states.velocities[index] * duration_since_last_call.seconds() + 0.5 * accel * duration_since_last_call.seconds() * duration_since_last_call.seconds();
      }
    }
  }

  // Clamp acclerations to limits
  for (auto index = 0u; index < num_joints; ++index) {
    if(joint_limits_[index].has_acceleration_limits) {
      double accel = (desired_joint_states.velocities[index] - current_joint_states.velocities[index]) / duration_since_last_call.seconds();
      if(std::abs(accel) > joint_limits_[index].max_acceleration) {
        RCLCPP_WARN_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), ROS_LOG_THROTTLE_PERIOD, "Joint(s) would exceed acceleration limits, limiting");
        desired_joint_states.velocities[index] = current_joint_states.velocities[index] + copysign(joint_limits_[index].max_acceleration, accel) * duration_since_last_call.seconds();
        // Recompute position
        desired_joint_states.positions[index] = current_joint_states.positions[index] + current_joint_states.velocities[index] * duration_since_last_call.seconds() + 0.5 * copysign(joint_limits_[index].max_acceleration, accel) * duration_since_last_call.seconds() * duration_since_last_call.seconds();
      }
    }
  }

  // Check that stopping distance is within joint limits
  // - In joint mode, slow down only joints whose stopping distance isn't inside joint limits, at maximum decel
  // - In Cartesian mode, slow down all joints at maximum decel if any don't have stopping distance within joint limits
  bool position_limit_triggered = false;
  for (auto index = 0u; index < num_joints; ++index) {
    if(joint_limits_[index].has_acceleration_limits) {
      // delta_x = (v2*v2 - v1*v1) / (2*a)
      // stopping_distance = (- v1*v1) / (2*max_acceleration)
      // Here we assume we will not trigger velocity limits while maximally decelerating. This is a valid assumption if we are not currently at a velocity limit since we are just coming to a rest.
      double stopping_distance = std::abs((- desired_joint_states.velocities[index] * desired_joint_states.velocities[index]) / (2 * joint_limits_[index].max_acceleration));
      // Check that joint limits are beyond stopping_distance and desired_velocity is towards that limit
      // TODO: Should we consider sign on acceleration here?
      if ((desired_joint_states.velocities[index] < 0 &&
           (current_joint_states.positions[index] - joint_limits_[index].min_position < stopping_distance)) ||
          (desired_joint_states.velocities[index] > 0 &&
           (joint_limits_[index].max_position - current_joint_states.positions[index] < stopping_distance))) {
        RCLCPP_WARN_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), ROS_LOG_THROTTLE_PERIOD, "Joint(s) would exceed position limits, limiting");
        position_limit_triggered = true;

        // We will limit all joints
        break;
      }
    }
  }

  if (position_limit_triggered) {
    // In Cartesian admittance mode, stop all joints if one would exceed limit
    for (auto index = 0u; index < num_joints; ++index) {
      if(joint_limits_[index].has_acceleration_limits) {
        // Compute accel to stop
        // Here we aren't explicitly maximally decelerating, but for joints near their limits this should still result in max decel being used
        double accel_to_stop = -current_joint_states.velocities[index] / duration_since_last_call.seconds();
        double limited_accel = copysign(std::min(std::abs(accel_to_stop), joint_limits_[index].max_acceleration), accel_to_stop);

        desired_joint_states.velocities[index] = current_joint_states.velocities[index] + limited_accel * duration_since_last_call.seconds();
        // Recompute position
        desired_joint_states.positions[index] = current_joint_states.positions[index] + current_joint_states.velocities[index] * duration_since_last_call.seconds() + 0.5 * limited_accel * duration_since_last_call.seconds() * duration_since_last_call.seconds();
      }
    }
  }
  // End limit plugin

  // Write new joint angles to the robot
  for (auto index = 0u; index < num_joints; ++index) {
    if (has_position_command_interface_) {
      joint_command_interface_[0][index].get().set_value(desired_joint_states.positions[index]);
    }
    if (has_velocity_command_interface_) {
      joint_command_interface_[1][index].get().set_value(desired_joint_states.velocities[index]);
    }
  }
  last_commanded_state_ = desired_joint_states;

  // Publish controller state
  state_publisher_->lock();
  state_publisher_->msg_.input_wrench_command = **input_wrench_cmd;
  state_publisher_->msg_.input_pose_command = **input_pose_cmd;
  state_publisher_->msg_.input_joint_command = **input_joint_cmd;

  state_publisher_->msg_.desired_joint_state = desired_joint_states;
  state_publisher_->msg_.actual_joint_state = current_joint_states;
  for (auto index = 0u; index < num_joints; ++index) {
    state_publisher_->msg_.error_joint_state.positions[index] = angles::shortest_angular_distance(
      current_joint_states.positions[index], desired_joint_states.positions[index]);
  }
  admittance_->get_controller_state(state_publisher_->msg_);
  state_publisher_->unlockAndPublish();

  return controller_interface::return_type::OK;
}

void AdmittanceController::read_state_from_hardware(
  trajectory_msgs::msg::JointTrajectoryPoint & state)
{
  const auto num_joints = joint_names_.size();
  auto assign_point_from_interface = [&, num_joints](
    std::vector<double> & trajectory_point_interface, const auto & joint_interface)
  {
    for (auto index = 0ul; index < num_joints; ++index) {
      trajectory_point_interface[index] = joint_interface[index].get().get_value();
    }
  };

  // Assign values from the hardware
  // Position states always exist
  assign_point_from_interface(state.positions, joint_state_interface_[0]);
  // velocity and acceleration states are optional
  if (has_velocity_state_interface_) {
    assign_point_from_interface(state.velocities, joint_state_interface_[1]);
    // Acceleration is used only in combination with velocity
    // TODO(destogl): enable acceleration and remove next line
    state.accelerations.clear();
//     if (has_acceleration_state_interface_) {
//       assign_point_from_interface(state.accelerations, joint_state_interface_[2]);
//     } else {
//       // Make empty so the property is ignored during interpolation
//       state.accelerations.clear();
//     }
  } else {
    // Make empty so the property is ignored during interpolation
    state.velocities.clear();
    state.accelerations.clear();
  }
}

bool AdmittanceController::read_state_from_command_interfaces(
  trajectory_msgs::msg::JointTrajectoryPoint & output_state)
{
  bool has_values = true;
  const auto num_joints = joint_names_.size();
  trajectory_msgs::msg::JointTrajectoryPoint state = output_state;

  auto assign_point_from_interface = [&, num_joints](
    std::vector<double> & trajectory_point_interface, const auto & joint_interface)
    {
      for (auto index = 0ul; index < num_joints; ++index) {
        trajectory_point_interface[index] = joint_interface[index].get().get_value();
      }
    };

  auto interface_has_values = [](const auto & joint_interface)
    {
      return std::find_if(
        joint_interface.begin(), joint_interface.end(),
        [](const auto & interface) {return std::isnan(interface.get().get_value());}) ==
             joint_interface.end();
    };

  // Assign values from the command interfaces as state. Therefore needs check for both.
  // Position state interface has to exist always
  if (has_position_command_interface_ && interface_has_values(joint_command_interface_[0])) {
    assign_point_from_interface(state.positions, joint_command_interface_[0]);
  } else {
    state.positions.clear();
    has_values = false;
  }
  // velocity and acceleration states are optional
  if (has_velocity_state_interface_) {
    if (has_velocity_command_interface_ && interface_has_values(joint_command_interface_[1])) {
      assign_point_from_interface(state.velocities, joint_command_interface_[1]);
      //TODO(destogl): enable this line under to be sure if positions are not existing and velocities
      // are existing to still update the output_state; !commented because not tested!
//       has_values = true;
    } else {
      state.velocities.clear();
      has_values = false;
    }
  }
  else {
    state.velocities.clear();
  }

// TODO(destogl): Enable this
//   // Acceleration is used only in combination with velocity
//   if (has_acceleration_state_interface_) {
//     if (has_acceleration_command_interface_ && interface_has_values(joint_command_interface_[2])) {
//       assign_point_from_interface(state.accelerations, joint_command_interface_[2]);
//     } else {
//       state.accelerations.clear();
//       has_values = false;
//     }
//   } else {
//     state.accelerations.clear();
//   }

  if (has_values) {
    output_state = state;
  }

  return has_values;
}

}  // namespace admittance_controller

#include "pluginlib/class_list_macros.hpp"
#include <angles/angles.h>
#include <angles/angles.h>

PLUGINLIB_EXPORT_CLASS(
  admittance_controller::AdmittanceController,
  controller_interface::ControllerInterface)
