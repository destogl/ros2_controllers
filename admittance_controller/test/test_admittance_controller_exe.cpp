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

#include "test_admittance_controller.hpp"

#include "gtest/gtest.h"

#include <limits>
#include <memory>
#include <utility>
#include <vector>

TEST_F(AdmittanceControllerTest, publish_status_success)
{
  // TODO: Write also a test when Cartesian commands are used.
  SetUpController(true, true);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  broadcast_tfs();
  ASSERT_EQ(controller_->update(), controller_interface::return_type::OK);

  ControllerStateMsg msg;
  subscribe_and_get_messages(msg);

  // Check that force command are all zero since not used
  ASSERT_EQ(msg.input_force_command.header.frame_id, control_frame_);
  ASSERT_EQ(msg.input_force_command.wrench.force.x, 0.0);
  ASSERT_EQ(msg.input_force_command.wrench.force.y, 0.0);
  ASSERT_EQ(msg.input_force_command.wrench.force.z, 0.0);
  ASSERT_EQ(msg.input_force_command.wrench.torque.x, 0.0);
  ASSERT_EQ(msg.input_force_command.wrench.torque.y, 0.0);
  ASSERT_EQ(msg.input_force_command.wrench.torque.z, 0.0);

  // Check Cartesian command message
  ASSERT_EQ(msg.input_pose_command.header.frame_id, control_frame_);
  ASSERT_FALSE(std::isnan(msg.input_pose_command.pose.position.x));
  ASSERT_FALSE(std::isnan(msg.input_pose_command.pose.position.y));
  ASSERT_FALSE(std::isnan(msg.input_pose_command.pose.position.z));
  ASSERT_FALSE(std::isnan(msg.input_pose_command.pose.orientation.x));
  ASSERT_FALSE(std::isnan(msg.input_pose_command.pose.orientation.y));
  ASSERT_FALSE(std::isnan(msg.input_pose_command.pose.orientation.z));
  ASSERT_FALSE(std::isnan(msg.input_pose_command.pose.orientation.w));

  // Check joint command message
  ASSERT_TRUE(std::equal(
    msg.input_joint_command.joint_names.begin(), msg.input_joint_command.joint_names.end(),
                         joint_names_.begin(), joint_names_.end()));
  ASSERT_EQ(msg.input_joint_command.points.size(), 1u);
  ASSERT_TRUE(std::equal(msg.input_joint_command.points[0].positions.begin(), msg.input_joint_command.points[0].positions.end(), joint_state_values_.begin(), joint_state_values_.end()));

  ASSERT_TRUE(std::find_if_not(msg.input_joint_command.points[0].velocities.begin(), msg.input_joint_command.points[0].velocities.end(),
    [](const auto & value){ return value == 0.0;}) == msg.input_joint_command.points[0].velocities.end());

  // Check messages filled from AdmittanceRule.cpp
  ASSERT_EQ(msg.measured_force.header.frame_id, sensor_frame_);
  ASSERT_EQ(msg.measured_force.wrench.force.x, fts_state_values_[0]);
  ASSERT_EQ(msg.measured_force.wrench.force.y, fts_state_values_[1]);
  ASSERT_EQ(msg.measured_force.wrench.force.z, fts_state_values_[2]);
  ASSERT_EQ(msg.measured_force.wrench.torque.x, fts_state_values_[3]);
  ASSERT_EQ(msg.measured_force.wrench.torque.y, fts_state_values_[4]);
  ASSERT_EQ(msg.measured_force.wrench.torque.z, fts_state_values_[5]);

  ASSERT_EQ(msg.measured_force_control_frame.header.frame_id, control_frame_);
  ASSERT_EQ(msg.measured_force_control_frame.wrench.force.x, 0.0);
  ASSERT_EQ(msg.measured_force_control_frame.wrench.force.y, 0.0);
  ASSERT_EQ(msg.measured_force_control_frame.wrench.force.z, 0.0);
  ASSERT_EQ(msg.measured_force_control_frame.wrench.torque.x, 0.0);
  ASSERT_EQ(msg.measured_force_control_frame.wrench.torque.y, 0.0);
  ASSERT_EQ(msg.measured_force_control_frame.wrench.torque.z, 0.0);

  ASSERT_EQ(msg.measured_force_endeffector_frame.header.frame_id, endeffector_frame_);
  ASSERT_EQ(msg.measured_force_endeffector_frame.wrench.force.x, 0.0);
  ASSERT_EQ(msg.measured_force_endeffector_frame.wrench.force.y, 0.0);
  ASSERT_EQ(msg.measured_force_endeffector_frame.wrench.force.z, 0.0);
  ASSERT_EQ(msg.measured_force_endeffector_frame.wrench.torque.x, 0.0);
  ASSERT_EQ(msg.measured_force_endeffector_frame.wrench.torque.y, 0.0);
  ASSERT_EQ(msg.measured_force_endeffector_frame.wrench.torque.z, 0.0);

  ASSERT_EQ(msg.desired_pose.header.frame_id, control_frame_);
  ASSERT_FALSE(std::isnan(msg.desired_pose.pose.position.x));
  ASSERT_FALSE(std::isnan(msg.desired_pose.pose.position.y));
  ASSERT_FALSE(std::isnan(msg.desired_pose.pose.position.z));
  ASSERT_FALSE(std::isnan(msg.desired_pose.pose.orientation.x));
  ASSERT_FALSE(std::isnan(msg.desired_pose.pose.orientation.y));
  ASSERT_FALSE(std::isnan(msg.desired_pose.pose.orientation.z));
  ASSERT_FALSE(std::isnan(msg.desired_pose.pose.orientation.w));

  ASSERT_EQ(msg.relative_desired_pose.header.frame_id, control_frame_);
  ASSERT_FALSE(std::isnan(msg.relative_desired_pose.transform.translation.x));
  ASSERT_FALSE(std::isnan(msg.relative_desired_pose.transform.translation.y));
  ASSERT_FALSE(std::isnan(msg.relative_desired_pose.transform.translation.z));
  ASSERT_FALSE(std::isnan(msg.relative_desired_pose.transform.rotation.x));
  ASSERT_FALSE(std::isnan(msg.relative_desired_pose.transform.rotation.y));
  ASSERT_FALSE(std::isnan(msg.relative_desired_pose.transform.rotation.z));
  ASSERT_FALSE(std::isnan(msg.relative_desired_pose.transform.rotation.w));

  // Check joint related messages
  ASSERT_TRUE(std::equal(
    msg.joint_names.begin(), msg.joint_names.end(), joint_names_.begin(), joint_names_.end()));

  ASSERT_TRUE(std::equal(
    msg.actual_joint_states.positions.begin(), msg.actual_joint_states.positions.end(),
    joint_state_values_.begin(), joint_state_values_.end()));

  ASSERT_TRUE(std::equal(
    msg.actual_joint_states.positions.begin(), msg.actual_joint_states.positions.end(),
    joint_state_values_.begin(), joint_state_values_.end()));

  ASSERT_TRUE(std::equal(
    msg.desired_joint_states.positions.begin(), msg.desired_joint_states.positions.end(),
                         joint_state_values_.begin(), joint_state_values_.end()));

  ASSERT_TRUE(std::find_if_not(msg.error_joint_state.positions.begin(), msg.error_joint_state.positions.end(),
    [](const auto & value){ return value == 0.0;}) == msg.error_joint_state.positions.end());
}

TEST_F(AdmittanceControllerTest, receive_message_and_publish_updated_status)
{
  SetUpController(true, true);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  broadcast_tfs();
  ASSERT_EQ(controller_->update(), controller_interface::return_type::OK);

  // After first update state values should be written to command values
  ASSERT_TRUE(std::equal(joint_state_values_.begin(), joint_state_values_.end(), joint_command_values_.begin(), joint_command_values_.end()));

  ControllerStateMsg msg;
  subscribe_and_get_messages(msg);
  ASSERT_EQ(msg.input_force_command.header.frame_id, control_frame_);
  ASSERT_EQ(msg.input_pose_command.header.frame_id, control_frame_);

  publish_commands();
  ASSERT_TRUE(controller_->wait_for_commands(executor));

  broadcast_tfs();
  ASSERT_EQ(controller_->update(), controller_interface::return_type::OK);

  EXPECT_NEAR(joint_command_values_[0], 0.0, COMMON_THRESHOLD);

  subscribe_and_get_messages(msg);
  ASSERT_EQ(msg.input_force_command.header.frame_id, control_frame_);
  ASSERT_EQ(msg.input_pose_command.header.frame_id, endeffector_frame_);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  int res = RUN_ALL_TESTS();

  return res;
}
