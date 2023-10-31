// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/MB_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_example_controllers {

bool MBController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface_ == nullptr) {
    ROS_ERROR(
        "MBController: Error getting position joint interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("MBController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("MBController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  position_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "MBController: Exception getting joint handles: " << e.what());
      return false;
    }
  }

  std::array<double, 7> q_start{{-0.76543793,-0.08999656,-0.19902707,-2.04154379,-0.12972969,2.73708789,2.73708976}};
  for (size_t i = 0; i < q_start.size(); i++) {
    if (std::abs(position_joint_handles_[i].getPosition() - q_start[i]) > 0.1) {
      ROS_ERROR_STREAM(
          "MBController: Robot is not in the expected starting position for "
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
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle("panda_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM("MBController: Exception getting model handle from interface: " << ex.what());
    return false;
  }
  return true;
}

void MBController::starting(const ros::Time& /* time */) {
  for (size_t i = 0; i < 7; ++i) {
    initial_pose_[i] = position_joint_handles_[i].getPosition();
  }
  initial_O_T_EE_ = model_handle_->getPose(franka::Frame::kEndEffector);
  elapsed_time_ = ros::Duration(0.0);
}

void MBController::update(const ros::Time& /*time*/,
                                            const ros::Duration& period) {
  elapsed_time_ += period;
  std::cout << "period=" << period << " ***\n";

  double delta_angle = M_PI / 16 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec())) * 0.2;
  std::cout << "delta_angle=" << delta_angle << " ***\n";
  for (size_t i = 0; i < 7; ++i) {
    joints_pose_[i]=position_joint_handles_[i].getPosition();
    if (i == 4) {
      position_joint_handles_[i].setCommand(initial_pose_[i] - delta_angle);
    } else {
      position_joint_handles_[i].setCommand(initial_pose_[i] + delta_angle);
    }
  }

  std::cout << "joints_pose_=";
  for (int j=0 ; j<7 ; j++ )
  {
    std::cout << joints_pose_[j]<<"\t";
  }
  std::cout << "***\n";

  // get jacobian
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  std::cout << ">>>>>>>>>>>>>>>jacobian_array=\n";
    for (int i = 0; i < 6; i++)
    {
      for (int j = 0; j < 7; j++)
      {
        std::cout << jacobian_array[i,j] << " ";
      }

      // Newline for new row
      std::cout << std::endl;
    }
    // Gets the 4x4 pose matrix for the given frame in base frame, calculated from the current robot state.
    std::array<double, 16> O_T_EE_ = model_handle_->getPose(franka::Frame::kEndEffector);
    std::cout << "@@@@@@@@@@@@@@@@@@O_T_EE_=\n";
    for (int i = 0; i < 4; i++)
    {
      for (int j = 0; j < 4; j++)
      {
        std::cout << O_T_EE_[i,j] << " ";
      }

      // Newline for new row
      std::cout << std::endl;
    }
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::MBController,
                       controller_interface::ControllerBase)
