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
/// \author: Andy Zelenak

#include "admittance_controller/incremental_kinematics.hpp"

#include "Eigen/VectorXf"
#include "tf2_eigen/tf2_eigen.h"

namespace admittance_controller
{
IncrementalKinematics::IncrementalKinematics(const std::shared_ptr<rclcpp::Node> & node, const std::string & group_name) : node_(node)
{
  // TODO(andyz): Parameterize robot description and joint group
  std::unique_ptr<robot_model_loader::RobotModelLoader> model_loader_ptr =
      std::unique_ptr<robot_model_loader::RobotModelLoader>(new robot_model_loader::RobotModelLoader(node_, "/robot_description", false /* do not load kinematics plugins */));
  const moveit::core::RobotModelPtr& kinematic_model = model_loader_ptr->getModel();
  // TODO(andyz): joint_model_group_ is a raw pointer. Is it thread safe?
  joint_model_group_ = kinematic_model->getJointModelGroup(group_name);
  kinematic_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model);

  // By default, the MoveIt Jacobian frame is the last link
}

bool IncrementalKinematics::convertCartesianDeltasToJointDeltas(const std::vector<double> & delta_x_vec, const geometry_msgs::msg::TransformStamped & ik_base_to_tip_trafo, std::vector<double> & delta_theta_vec)
{
  // see here for this conversion: https://stackoverflow.com/questions/26094379/typecasting-eigenvectorxd-to-stdvector
  Eigen::VectorXf delta_x = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(&delta_x_vec[0], delta_x_vec.size());

  // Transform delta_x to the moveit_jacobian_frame
  try
  {
    // 4x4 transformation matrix
    const Eigen::Isometry3d affine_transform = tf2::transformToEigen(ik_base_to_tip_trafo);

    // Build the 6x6 transformation matrix
    Eigen::MatrixXd twist_transform(6,6);
    // upper left 3x3 block is the rotation part
    twist_transform.block(0,0,3,3) = affine_transform.rotation();
    // upper right 3x3 block is all zeros
    twist_transform.block(0,4,3,3) = Eigen::MatrixXd::Zero(3,3);
    // lower left 3x3 block is tricky. See https://core.ac.uk/download/pdf/154240607.pdf
    Eigen::MatrixXd pos_vector_3x3(3,3);
    pos_vector_3x3(0,0) = 0;  pos_vector_3x3(0,1) = -affine_transform.translation().z();  pos_vector_3x3(0,2) = affine_transform.translation().y();
    pos_vector_3x3(1, 0) = affine_transform.translation().z();  pos_vector_3x3(1,1) = 0;  pos_vector_3x3(1,2) = -affine_transform.translation().x();
    pos_vector_3x3(2, 0) = -affine_transform.translation().y();  pos_vector_3x3(2,1) = affine_transform.translation().x();  pos_vector_3x3(1,2) = 0;
    twist_transform.block(3,0,3,3) = pos_vector_3x3 * affine_transform.rotation();
    // lower right 3x3 block is the rotation part
    twist_transform.block(3,3,3,3) = affine_transform.rotation();

    delta_x = twist_transform * delta_x;
  }
  catch (tf2::TransformException & ex)
  {
    RCLCPP_ERROR(node_->get_logger(), "Transformation of wrench failed.");
    return false;
  }

  // Multiply with the pseudoinverse to get delta_theta
  jacobian_ = kinematic_state_->getJacobian(joint_model_group_);
  svd_ = Eigen::JacobiSVD<Eigen::MatrixXd>(jacobian_, Eigen::ComputeThinU | Eigen::ComputeThinV);
  matrix_s_ = svd_.singularValues().asDiagonal();
  pseudo_inverse_ = svd_.matrixV() * matrix_s_.inverse() * svd_.matrixU().transpose();
  Eigen::VectorXf  delta_theta = pseudo_inverse_ * delta_x;

  std::vector<double> delta_theta_v(&delta_theta[0], delta_theta.data() + delta_theta.cols() * delta_theta.rows());
  delta_theta_vec = delta_theta_v;

  return true;
}

bool IncrementalKinematics::convertJointDeltasToCartesianDeltas(const std::vector<double> &  delta_theta_vec, const geometry_msgs::msg::TransformStamped & ik_base_to_tip_trafo, std::vector<double> & delta_x_vec)
{
  // TODO(andyz): Please add here FK implementation
}

}  // namespace admittance_controller
