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

#ifndef ADMITTANCE_CONTROLLER__ADMITTANCE_RULE_HPP_
#define ADMITTANCE_CONTROLLER__ADMITTANCE_RULE_HPP_

#include "admittance_controller/incremental_kinematics.hpp"
#include "control_msgs/msg/admittance_controller_state.hpp"
#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace admittance_controller
{
class AdmittanceRule
{
public:
  AdmittanceRule() = default;

  controller_interface::return_type configure(rclcpp::Node::SharedPtr node);

  controller_interface::return_type update(
    const std::vector<double> & current_joint_state,
    const geometry_msgs::msg::Wrench & measured_force,
    const geometry_msgs::msg::PoseStamped & target_pose,
    const geometry_msgs::msg::WrenchStamped & target_force,
    const rclcpp::Duration & period,
    std::array<double, 6> desired_joint_states);

  controller_interface::return_type update(
    const std::array<double, 6> & /*current_joint_state*/,
    const geometry_msgs::msg::Wrench & measured_force,
    const geometry_msgs::msg::PoseStamped & target_pose,
    const rclcpp::Duration & period,
    std::array<double, 6> desired_joint_states);

//   controller_interface::return_type update(
//     const geometry_msgs::msg::WrenchStamped & measured_force,
//     const geometry_msgs::msg::PoseStamped & target_pose,
//     const geometry_msgs::msg::PoseStamped & current_pose,
//     const rclcpp::Duration & period,
//     geometry_msgs::msg::TransformStamped & relative_desired_pose_vec
//   );

  controller_interface::return_type get_controller_state(
    control_msgs::msg::AdmittanceControllerState & state_message
  );

  controller_interface::return_type reset();

public:
  // IK related parameters
  std::string ik_base_frame_;
  std::string ik_tip_frame_;
  std::string ik_group_name_;

  // Controller frames
  std::string control_frame_;
  std::string endeffector_frame_;
  std::string fixed_world_frame_;
  std::string sensor_frame_;

  // Admittance parameters
  //   bool unified_mode_ = false;
  std::array<bool, 6> selected_axes_;
  std::array<double, 6> mass_;
  std::array<double, 6> damping_;
  std::array<double, 6> stiffness_;

protected:
  // IK variables
  std::shared_ptr<IncrementalKinematics> ik_;
  Eigen::VectorXd delta_x_;
  Eigen::VectorXd delta_theta_;

  // Transformation variables
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  geometry_msgs::msg::WrenchStamped measured_force_;
  geometry_msgs::msg::WrenchStamped measured_force_control_frame_;

  geometry_msgs::msg::PoseStamped current_origin_;
  geometry_msgs::msg::PoseStamped current_pose_;
  geometry_msgs::msg::PoseStamped current_pose_control_frame_;

  geometry_msgs::msg::WrenchStamped target_force_control_frame_;
  geometry_msgs::msg::PoseStamped target_pose_control_frame_;

  geometry_msgs::msg::PoseStamped desired_pose_;
  geometry_msgs::msg::TransformStamped relative_desired_pose_;

  // Pre-reserved update-loop variables
  std::array<double, 6> measured_force_control_frame_vec_;
  std::array<double, 6> target_pose_control_frame_vec_;
  std::array<double, 6> current_pose_control_frame_vec_;

  std::array<double, 3> angles_error_;

  std::array<double, 6> relative_desired_pose_vec_;
  std::array<double, 6> desired_velocity_vec_;
  std::array<double, 6> desired_velocity_previous_vec_;
  std::array<double, 6> desired_acceleration_previous_vec_;

  std::vector<double> relative_desired_joint_state_vec_;

private:
  // TODO: implement doTransform for WrenchStamped
  template<typename MsgType>
//   using MsgType = geometry_msgs::msg::PoseStamped;
  void transform_message_to_control_frame(const MsgType & message_in, MsgType & message_out)
  {
    if (control_frame_ != message_in.header.frame_id) {
      geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
        control_frame_, message_in.header.frame_id, tf2::TimePointZero);
      tf2::doTransform(message_in, message_out, transform);
    } else {
      message_out = message_in;
    }
  }

//   using MsgType2 = geometry_msgs::msg::WrenchStamped;
//   void transform_message_to_control_frame(const MsgType2 & message_in, MsgType2 & message_out)
//   {
//     if (control_frame_ != message_in.header.frame_id) {
//       geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
//         control_frame_, message_in.header.frame_id, tf2::TimePointZero);
//
//       geometry_msgs::msg::Vector3Stamped vec_in;
//       geometry_msgs::msg::Vector3Stamped vec_out;
//
//       vec_in.vector = message_in.wrench.force;
//       vec_in.header = message_in.header;
//       tf2::doTransform(vec_in, vec_out, transform);
//       message_out.wrench.force = vec_out.vector;
//
//       vec_in.vector = message_in.wrench.torque;
//       tf2::doTransform(vec_in, vec_out, transform);
//       message_out.wrench.torque = vec_out.vector;
//     } else {
//       message_out = message_in;
//     }
//   }
};

}  // namespace admittance_controller

#endif  // ADMITTANCE_CONTROLLER__ADMITTANCE_RULE_HPP_
