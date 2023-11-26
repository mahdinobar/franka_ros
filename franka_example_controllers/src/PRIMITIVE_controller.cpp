// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/PRIMITIVE_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka_example_controllers/pseudo_inversion.h>
#include <iostream>
#include <vector>

#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <string>
#include "/usr/local/MATLAB/R2023b/extern/include/mat.h"

namespace franka_example_controllers {

bool PRIMITIVEController::init(hardware_interface::RobotHW* robot_hardware,
                               ros::NodeHandle& node_handle) {
  position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface_ == nullptr) {
    ROS_ERROR("PRIMITIVEController: Error getting position joint interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("PRIMITIVEController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("PRIMITIVEController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  position_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM("PRIMITIVEController: Exception getting joint handles: " << e.what());
      return false;
    }
  }

  for (size_t i = 0; i < q_start.size(); i++) {
    if (std::abs(position_joint_handles_[i].getPosition() - q_start[i]) > 0.1) {
      ROS_ERROR_STREAM(
          "PRIMITIVEController: Robot is not in the expected starting position for "
          "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
          "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
      return false;
    }
  }
  auto* model_interface = robot_hardware->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ =
        std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle("panda_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "PRIMITIVEController: Exception getting model handle from interface: " << ex.what());
    return false;
  }
  auto* state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  try {
    state_handle_ =
        std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle("panda_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "PRIMITIVEController: Exception getting state handle from interface: " << ex.what());
    return false;
  }
  PRIMITIVE_publisher_.init(node_handle, "PRIMITIVE_messages", 1);
  return true;
}

void PRIMITIVEController::starting(const ros::Time& /* time */) {
  for (size_t i = 0; i < 7; ++i) {
    initial_pose_[i] = position_joint_handles_[i].getPosition();
  }
//  std::ifstream inputfile_q_star("/home/mahdi/ETHZ/codes/rl_reach/code/logs/q_log_b.txt");
//  if (!inputfile_q_star.is_open()) {
//    std::cout << "Error reading q_log file" << std::endl;
//  }
//  for (int row = 0; row < Target_Traj_ROWS; ++row) {
//    std::string row_text_q;
//    std::getline(inputfile_q_star, row_text_q);
//    std::istringstream row_stream_q(row_text_q);
//    for (int column = 0; column < 9; ++column) {
//      double number_q;
//      char delimiter;
//      row_stream_q >> number_q >> delimiter;
//      q_star[row][column] = number_q;
//      std::cout << q_star[row][column] << " ";
//      std::cout << std::endl;
//    }
//  }
  initial_O_T_EE_ = model_handle_->getPose(franka::Frame::kEndEffector);
  elapsed_time_ = ros::Duration(0.0);
}

void PRIMITIVEController::update(const ros::Time& /*time*/, const ros::Duration& period) {
//  int mp = 5000;
  for (size_t i = 0; i < 7; ++i) {
    joints_pose_[i] = position_joint_handles_[i].getPosition();
  }
  elapsed_time_ += period;
//  std::cout << "period=" << period << " \n";
//  std::cout << std::endl;
//  std::cout << "elapsed_time_=" << elapsed_time_ << " \n";
//  std::cout << std::endl;
//  std::cout << "idx_out=" << idx_out << " \n";
//  std::cout << std::endl;
  const double t_B=5;
  if (elapsed_time_.toSec()==t_B) {
//    std::cout << "!!!changed_q_c_setpoint idx_out=" << idx_out << " \n";
//    std::cout << std::endl;
    idx_command += 1;
    q_command[0] = q_start[0] + 0.1 * (3.14 / 180);
  }
  if (rate_trigger_() && PRIMITIVE_publisher_.trylock()) {
    for (size_t i = 0; i < 7; ++i) {
      PRIMITIVE_publisher_.msg_.q_c[i] = q_command[i];
    }
    PRIMITIVE_publisher_.unlockAndPublish();
  }
  franka::RobotState robot_state = state_handle_->getRobotState();
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d EEposition(transform.translation());
  if (rate_trigger_() && PRIMITIVE_publisher_.trylock()) {
    for (size_t i = 0; i < 3; ++i) {
      PRIMITIVE_publisher_.msg_.EEposition[i] = EEposition(i);
    }
    PRIMITIVE_publisher_.unlockAndPublish();
  }
  for (size_t i = 0; i < 7; ++i) {
    position_joint_handles_[i].setCommand(q_command[i]);
  }
  idx_out += 1;
  if (debug) {
    std::cout << "+++++++++++++++++++++++++++++++++++idx_command=" << idx_command << " \n";
    std::cout << std::endl;
  }
}

void PRIMITIVEController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::PRIMITIVEController,
                       controller_interface::ControllerBase)
