// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_cartesian_command_interface.h>

namespace franka_example_controllers {

class MBController : public controller_interface::MultiInterfaceController<
                         franka_hw::FrankaModelInterface,
                         hardware_interface::PositionJointInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  hardware_interface::PositionJointInterface* position_joint_interface_;
  std::vector<hardware_interface::JointHandle> position_joint_handles_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  ros::Duration elapsed_time_;
  std::array<double, 7> initial_pose_{};
  std::array<double, 7> joints_pose_{};
  std::array<double, 16> initial_O_T_EE_{};

};

}  // namespace franka_example_controllers
