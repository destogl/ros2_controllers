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
/// \author: Denis Stogl

#include "admittance_controller/admittance_rule.hpp"

#include "angles/angles.h"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/utilities.hpp"
#include "tf2/utils.h"
#include <tf2_eigen/tf2_eigen.h>

namespace {  // Utility namespace

template<typename Type>
void convert_message_to_array(const geometry_msgs::msg::Pose & msg, Type & vector_out)
{
  vector_out[0] = msg.position.x;
  vector_out[1] = msg.position.y;
  vector_out[2] = msg.position.z;
  tf2::Quaternion q;
  tf2::fromMsg(msg.orientation, q);
  q.normalize();
  tf2::Matrix3x3(q).getRPY(vector_out[3], vector_out[4], vector_out[5]);
  for (auto i = 3u; i < 6; ++i) {
    vector_out[i] = angles::normalize_angle(vector_out[i]);
  }
}

template<typename Type>
void convert_message_to_array(
  const geometry_msgs::msg::PoseStamped & msg, Type & vector_out)
{
  convert_message_to_array(msg.pose, vector_out);
}

template<typename Type>
void convert_message_to_array(const geometry_msgs::msg::Transform & msg, Type & vector_out)
{
  vector_out[0] = msg.translation.x;
  vector_out[1] = msg.translation.y;
  vector_out[2] = msg.translation.z;
  tf2::Quaternion q;
  tf2::fromMsg(msg.rotation, q);
  q.normalize();
  tf2::Matrix3x3(q).getRPY(vector_out[3], vector_out[4], vector_out[5]);
  for (auto i = 3u; i < 6; ++i) {
    vector_out[i] = angles::normalize_angle(vector_out[i]);
  }
}

template<typename Type>
void convert_message_to_array(const geometry_msgs::msg::TransformStamped & msg, Type & vector_out)
{
  convert_message_to_array(msg.transform, vector_out);
}

template<typename Type>
void convert_message_to_array(
  const geometry_msgs::msg::Wrench & msg, Type & vector_out)
{
  vector_out[0] = msg.force.x;
  vector_out[1] = msg.force.y;
  vector_out[2] = msg.force.z;
  vector_out[3] = msg.torque.x;
  vector_out[4] = msg.torque.y;
  vector_out[5] = msg.torque.z;
}

template<typename Type>
void convert_message_to_array(
  const geometry_msgs::msg::WrenchStamped & msg, Type & vector_out)
{
  convert_message_to_array(msg.wrench, vector_out);
}

template<typename Type>
void convert_array_to_message(const Type & vector, geometry_msgs::msg::Pose & msg_out)
{
  msg_out.position.x = vector[0];
  msg_out.position.y = vector[1];
  msg_out.position.z = vector[2];

  tf2::Quaternion q;
  q.setRPY(vector[3], vector[4], vector[5]);

  msg_out.orientation.x = q.x();
  msg_out.orientation.y = q.y();
  msg_out.orientation.z = q.z();
  msg_out.orientation.w = q.w();
}

template<typename Type>
void convert_array_to_message(const Type & vector, geometry_msgs::msg::PoseStamped & msg_out)
{
  convert_array_to_message(vector, msg_out.pose);
}

// template<typename Type>
// void convert_array_to_message(const Type & vector, geometry_msgs::msg::Wrench & msg_out)
// {
//   msg_out.force.x = vector[0];
//   msg_out.force.y = vector[1];
//   msg_out.force.z = vector[2];
//   msg_out.torque.x = vector[3];
//   msg_out.torque.y = vector[4];
//   msg_out.torque.z = vector[5];
// }

// template<typename Type>
// void convert_array_to_message(const Type & vector, geometry_msgs::msg::WrenchStamped & msg_out)
// {
//   convert_array_to_message(vector, msg_out.wrench);
// }

template<typename Type>
void convert_array_to_message(const Type & vector, geometry_msgs::msg::Transform & msg_out)
{
  msg_out.translation.x = vector[0];
  msg_out.translation.y = vector[1];
  msg_out.translation.z = vector[2];

  tf2::Quaternion q;
  q.setRPY(vector[3], vector[4], vector[5]);

  msg_out.rotation.x = q.x();
  msg_out.rotation.y = q.y();
  msg_out.rotation.z = q.z();
  msg_out.rotation.w = q.w();
}

template<typename Type>
void convert_array_to_message(const Type & vector, geometry_msgs::msg::TransformStamped & msg_out)
{
  convert_array_to_message(vector, msg_out.transform);
}

}  // utility namespace

namespace admittance_controller
{

controller_interface::return_type AdmittanceRule::configure(rclcpp::Node::SharedPtr node)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize variables used in the update loop
  measured_wrench_.header.frame_id = sensor_frame_;

  relative_desired_pose_.header.frame_id = control_frame_;

  identity_transform_.transform.rotation.w = 1;

  relative_desired_joint_state_vec_.reserve(6);

  // Initialize IK
  ik_ = std::make_shared<IncrementalKinematics>(node, ik_group_name_);

  return controller_interface::return_type::OK;
}

controller_interface::return_type AdmittanceRule::reset()
{
  measured_wrench_control_frame_arr_.fill(0.0);
  target_pose_ik_base_frame_arr_.fill(0.0);
  current_pose_ik_base_frame_arr_.fill(0.0);
  angles_error_.fill(0.0);
  desired_velocity_arr_.fill(0.0);
  desired_velocity_previous_arr_.fill(0.0);
  desired_acceleration_previous_arr_.fill(0.0);

  get_pose_of_control_frame_in_base_frame(current_pose_ik_base_frame_);
  feedforward_pose_ik_base_frame_ = current_pose_ik_base_frame_;

  if (open_loop_control_) {
    get_pose_of_control_frame_in_base_frame(desired_pose_ik_base_frame_);
  }

  // Initialize ik_tip and tool_frame transformations - those are fixed transformations
  tf2::Stamped<tf2::Transform> tf2_transform;
  try {
    auto transform = tf_buffer_->lookupTransform(ik_tip_frame_, control_frame_, tf2::TimePointZero);
    tf2::fromMsg(transform, tf2_transform);
    ik_tip_to_control_frame_tf_ = tf2_transform;
    control_frame_to_ik_tip_tf_ = tf2_transform.inverse();
  } catch (const tf2::TransformException & e) {
    // TODO(destogl): Use RCLCPP_ERROR_THROTTLE
    RCLCPP_ERROR(rclcpp::get_logger("AdmittanceRule"), "LookupTransform failed from '" +
    control_frame_ + "' to '" + ik_tip_frame_ + "'.");
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

// Update with target Cartesian pose - the main update method!
controller_interface::return_type AdmittanceRule::update(
  const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_state,
  const geometry_msgs::msg::Wrench & measured_wrench,
  const geometry_msgs::msg::PoseStamped & target_pose,
  const rclcpp::Duration & period,
  trajectory_msgs::msg::JointTrajectoryPoint & desired_joint_state
)
{
  // FIXME: What if there is open loop control used? Will this work?
  // Convert inputs to ik_base frame (assumed stationary)
  transform_message_to_ik_base_frame(target_pose, target_pose_ik_base_frame_);

  // TODO(andyz): what if there is a hardware offset?
  if (!open_loop_control_) {
    get_pose_of_control_frame_in_base_frame(current_pose_ik_base_frame_);
  } else {
    current_pose_ik_base_frame_ = desired_pose_ik_base_frame_;
  }

  // Convert all data to arrays for simpler calculation
  convert_message_to_array(target_pose_ik_base_frame_, target_pose_ik_base_frame_arr_);
  convert_message_to_array(current_pose_ik_base_frame_, current_pose_ik_base_frame_arr_);

  std::array<double, 6> pose_error;
  // Estimate feedforward acceleration from target_pose_ik_base_frame_arr_ and previous
  std::array<double, 6> feedforward_acceleration;

  for (auto i = 0u; i < 6; ++i) {
    pose_error[i] = current_pose_ik_base_frame_arr_[i] - target_pose_ik_base_frame_arr_[i];
    if (i >= 3) {
      pose_error[i] = angles::normalize_angle(pose_error[i]);
    }

    // Estimate feedforward acceleration
    feedforward_acceleration[i] = (feedforward_velocity_ik_base_frame_[i] - prev_feedforward_velocity_ik_base_frame_[i]) / period.seconds();
  }
  prev_feedforward_velocity_ik_base_frame_ = feedforward_velocity_ik_base_frame_;

  process_wrench_measurements(measured_wrench);

  calculate_admittance_rule(measured_wrench_control_frame_arr_, pose_error, feedforward_acceleration, period,
                            relative_desired_pose_arr_);

  // This works in all cases because not current TF data are used
  // Do clean conversion to the goal pose using transform and not messing with Euler angles
  convert_array_to_message(relative_desired_pose_arr_, relative_desired_pose_);
  tf2::doTransform(current_pose_ik_base_frame_, desired_pose_ik_base_frame_, relative_desired_pose_);

  return calculate_desired_joint_state(current_joint_state, period, desired_joint_state);
}

// Update from target joint deltas
controller_interface::return_type AdmittanceRule::update(
  const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_state,
  const geometry_msgs::msg::Wrench & measured_wrench,
  const std::array<double, 6> & target_joint_deltas,
  const rclcpp::Duration & period,
  trajectory_msgs::msg::JointTrajectoryPoint & desired_joint_state)
{
  std::vector<double> target_joint_deltas_vec(target_joint_deltas.begin(), target_joint_deltas.end());
  std::vector<double> target_ik_tip_deltas_vec(6);

  // Get feed-forward cartesian deltas in the ik_base frame.
  // Since ik_base is MoveIt's working frame, the transform is identity.
  identity_transform_.header.frame_id = ik_base_frame_;
  ik_->update_robot_state(current_joint_state);
  // FIXME: Do we need if here? Can we simply use if (!ik_->...)?
  if (ik_->convertJointDeltasToCartesianDeltas(target_joint_deltas_vec, identity_transform_, target_ik_tip_deltas_vec)) {
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("AdmittanceRule"),
                 "Conversion of joint deltas to Cartesian deltas failed. Sending current joint"
                 " values to the robot.");
    desired_joint_state = current_joint_state;
    std::fill(desired_joint_state.velocities.begin(), desired_joint_state.velocities.end(), 0.0);
    return controller_interface::return_type::ERROR;
  }

  for (auto i = 0u; i < 6; ++i) {
    feedforward_velocity_ik_base_frame_[i] = target_ik_tip_deltas_vec.at(i) / period.seconds();
  }

  // Add deltas to previously-desired pose to get the next desired pose
  // FIXME: Why not use convert_to_array method?
  // FIXME: (?) Does this variable have a wrong name? Shouldn't it be target_pose_ik_base_frame?
  feedforward_pose_ik_base_frame_.pose.position.x += target_ik_tip_deltas_vec.at(0);
  feedforward_pose_ik_base_frame_.pose.position.y += target_ik_tip_deltas_vec.at(1);
  feedforward_pose_ik_base_frame_.pose.position.z += target_ik_tip_deltas_vec.at(2);

  tf2::Quaternion q(feedforward_pose_ik_base_frame_.pose.orientation.x, feedforward_pose_ik_base_frame_.pose.orientation.y, feedforward_pose_ik_base_frame_.pose.orientation.z, feedforward_pose_ik_base_frame_.pose.orientation.w);
  tf2::Quaternion q_rot;
  q_rot.setRPY(target_ik_tip_deltas_vec.at(3), target_ik_tip_deltas_vec.at(4), target_ik_tip_deltas_vec.at(5));
  q = q_rot * q;
  q.normalize();
  feedforward_pose_ik_base_frame_.pose.orientation.w = q.w();
  feedforward_pose_ik_base_frame_.pose.orientation.x = q.x();
  feedforward_pose_ik_base_frame_.pose.orientation.y = q.y();
  feedforward_pose_ik_base_frame_.pose.orientation.z = q.z();

  return update(current_joint_state, measured_wrench, feedforward_pose_ik_base_frame_, period, desired_joint_state);
}

controller_interface::return_type AdmittanceRule::update(
  const trajectory_msgs::msg::JointTrajectoryPoint & /*current_joint_state*/,
  const geometry_msgs::msg::Wrench & /*measured_wrench*/,
  const geometry_msgs::msg::PoseStamped & /*target_pose*/,
  const geometry_msgs::msg::WrenchStamped & /*target_force*/,
  const rclcpp::Duration & /*period*/,
  trajectory_msgs::msg::JointTrajectoryPoint & /*desired_joint_state*/
)
{
  // TODO(destogl): Implement this update
  //  transform_message_to_ik_base_frame(**input_force_cmd, force_cmd_ctrl_frame_);
  // TODO(destogl) reuse things from other update

  return controller_interface::return_type::OK;
}

controller_interface::return_type AdmittanceRule::get_controller_state(
  control_msgs::msg::AdmittanceControllerState & state_message)
{
  //   state_message.input_wrench_control_frame = target_wrench_control_frame_;
  state_message.input_pose_control_frame = target_pose_ik_base_frame_;
  state_message.measured_wrench = measured_wrench_;
  state_message.measured_wrench_filtered = measured_wrench_filtered_;
  state_message.measured_wrench_control_frame = measured_wrench_control_frame_;

  try {
    auto transform = tf_buffer_->lookupTransform(endeffector_frame_, control_frame_, tf2::TimePointZero);
    tf2::doTransform(measured_wrench_control_frame_, measured_wrench_endeffector_frame_, transform);
  } catch (const tf2::TransformException & e) {
    // TODO(destogl): Use RCLCPP_ERROR_THROTTLE
    RCLCPP_ERROR(rclcpp::get_logger("AdmittanceRule"), "LookupTransform failed from '" +
    control_frame_ + "' to '" + endeffector_frame_ + "'.");
  }

  state_message.measured_wrench_endeffector_frame = measured_wrench_endeffector_frame_;

  state_message.current_pose = current_pose_ik_base_frame_;
  state_message.desired_pose = desired_pose_ik_base_frame_;
  state_message.relative_desired_pose = relative_desired_pose_;

  return controller_interface::return_type::OK;
}

controller_interface::return_type AdmittanceRule::get_pose_of_control_frame_in_base_frame(geometry_msgs::msg::PoseStamped & pose)
{
  try {
    auto transform = tf_buffer_->lookupTransform(ik_base_frame_, control_frame_, tf2::TimePointZero);

    pose.header = transform.header;
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation= transform.transform.rotation;
  } catch (const tf2::TransformException & e) {
    // TODO(destogl): Use RCLCPP_ERROR_THROTTLE
    RCLCPP_ERROR(rclcpp::get_logger("AdmittanceRule"), "LookupTransform failed from '" +
    control_frame_ + "' to '" + ik_base_frame_ + "'.");
    return controller_interface::return_type::ERROR;
  }
  return controller_interface::return_type::OK;
}

void AdmittanceRule::process_wrench_measurements(
  const geometry_msgs::msg::Wrench & measured_wrench
)
{
  // TODO(andyz): Implement gravity comp. For now, just pass the measured wrench through
  measured_wrench_.wrench = measured_wrench;
  measured_wrench_filtered_ = measured_wrench_;

  // // get current states, and transform those into controller frame
  // measured_wrench_.wrench = measured_wrench;
  // try {
  //   auto transform_to_world = tf_buffer_->lookupTransform(fixed_world_frame_,  measured_wrench_.header.frame_id, tf2::TimePointZero);
  //   auto transform_to_sensor = tf_buffer_->lookupTransform(measured_wrench_.header.frame_id, fixed_world_frame_, tf2::TimePointZero);

  //   geometry_msgs::msg::WrenchStamped measured_wrench_transformed;
  //   tf2::doTransform(measured_wrench_, measured_wrench_transformed, transform_to_world);

  //   geometry_msgs::msg::Vector3Stamped cog_transformed;
  //   for (const auto & params : gravity_compensation_params_) {
  //     auto transform_cog = tf_buffer_->lookupTransform(fixed_world_frame_,  params.cog_.header.frame_id, tf2::TimePointZero);
  //     tf2::doTransform(params.cog_, cog_transformed, transform_cog);

  //     measured_wrench_transformed.wrench.force.z += params.force_;
  //     measured_wrench_transformed.wrench.torque.x += (params.force_ * cog_transformed.vector.y);
  //     measured_wrench_transformed.wrench.torque.y -= (params.force_ * cog_transformed.vector.x);
  //   }

  //   tf2::doTransform(measured_wrench_transformed, measured_wrench_filtered_, transform_to_sensor);

  // } catch (const tf2::TransformException & e) {
  //   // TODO(destogl): Use RCLCPP_ERROR_THROTTLE
  //   RCLCPP_ERROR(rclcpp::get_logger("AdmittanceRule"), "LookupTransform failed between '" + fixed_world_frame_ + "' and '" + measured_wrench_.header.frame_id + "' or '<a cog frame>'.");
  //   // If transform error just use measured force
  //   measured_wrench_filtered_ = measured_wrench_;
  // }

  transform_message_to_ik_base_frame(measured_wrench_filtered_, measured_wrench_control_frame_);
  convert_message_to_array(measured_wrench_control_frame_, measured_wrench_control_frame_arr_);

  // If at least one measured force is nan set all to 0
  if (std::find_if(measured_wrench_control_frame_arr_.begin(), measured_wrench_control_frame_arr_.end(), [](const auto value){ return std::isnan(value); }) != measured_wrench_control_frame_arr_.end()) {
    measured_wrench_control_frame_arr_.fill(0.0);
  }
}

void AdmittanceRule::calculate_admittance_rule(
  const std::array<double, 6> & measured_wrench,
  const std::array<double, 6> & pose_error,
  const std::array<double, 6> & feedforward_acceleration,
  const rclcpp::Duration & period,
  std::array<double, 6> & desired_relative_pose
)
{
  // Compute admittance control law: F = M*a + D*v + S*(x - x_d)
  for (auto i = 0u; i < 6; ++i) {
    if (selected_axes_[i]) {
      // TODO(destogl): check if velocity is measured from hardware
      const double acceleration = feedforward_acceleration[i] + (1 / mass_[i]) *
      (measured_wrench[i] - damping_[i] * desired_velocity_arr_[i] - stiffness_[i] * pose_error[i]);

      desired_velocity_arr_[i] += (desired_acceleration_previous_arr_[i] + acceleration) * 0.5 * period.seconds();

      desired_relative_pose[i] = (desired_velocity_previous_arr_[i] + desired_velocity_arr_[i]) * 0.5 * period.seconds();

      desired_acceleration_previous_arr_[i] = acceleration;
      desired_velocity_previous_arr_[i] = desired_velocity_arr_[i];

//       RCLCPP_INFO(rclcpp::get_logger("AR"), "Adm. Rule [%zu]: (%e = %e - D(%.1f)*%e - S(%.1f)*%e)", i,
//                   acceleration, measured_wrench[i], damping_[i], desired_velocity_arr_[i],
//                   stiffness_[i], pose_error[i]);

//       RCLCPP_INFO(rclcpp::get_logger("AR"), "Adm. acc [%zu]: %e = %e - (1/M(%.1f)) (%e - D(%.1f)*%e - S(%.1f)*%e)",
//                   i, acceleration, feedforward_acceleration[i], mass_[i], measured_wrench[i], damping_[i], desired_velocity_arr_[i], stiffness_[i], pose_error[i]);
    }
  }
}

controller_interface::return_type AdmittanceRule::calculate_desired_joint_state(
  const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_state,
  const rclcpp::Duration & period,
  trajectory_msgs::msg::JointTrajectoryPoint & desired_joint_state
)
{
  convert_array_to_message(relative_desired_pose_arr_, relative_desired_pose_);

  // Since ik_base is MoveIt's working frame, the transform is identity.
  identity_transform_.header.frame_id = ik_base_frame_;

  // Use Jacobian-based IK
  std::vector<double> relative_desired_pose_vec(relative_desired_pose_arr_.begin(), relative_desired_pose_arr_.end());
  if (ik_->convertCartesianDeltasToJointDeltas(
    relative_desired_pose_vec, identity_transform_, relative_desired_joint_state_vec_)){
    for (auto i = 0u; i < desired_joint_state.positions.size(); ++i) {
      desired_joint_state.positions[i] = current_joint_state.positions[i] + relative_desired_joint_state_vec_[i];
      desired_joint_state.velocities[i] = relative_desired_joint_state_vec_[i] / period.seconds();
      //       RCLCPP_INFO(rclcpp::get_logger("AR"), "joint states [%zu]: %f + %f = %f", i, current_joint_state.positions[i], relative_desired_joint_state_vec_[i], desired_joint_state.positions[i]);
    }
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("AdmittanceRule"), "Conversion of Cartesian deltas to joint deltas failed. Sending current joint values to the robot.");
      desired_joint_state = current_joint_state;
      std::fill(desired_joint_state.velocities.begin(), desired_joint_state.velocities.end(), 0.0);
      return controller_interface::return_type::ERROR;
    }

    return controller_interface::return_type::OK;
}

}  // namespace admittance_controller
