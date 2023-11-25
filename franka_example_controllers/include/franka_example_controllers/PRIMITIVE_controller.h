// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <franka_example_controllers/PRIMITIVEmessages.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/trigger_rate.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>

namespace franka_example_controllers {

class PRIMITIVEController : public controller_interface::MultiInterfaceController<
                                franka_hw::FrankaModelInterface,
                                hardware_interface::PositionJointInterface,
                                franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void stopping(const ros::Time&) override;
  //  void stopRequest(const ros::Time&  time);

 private:
  hardware_interface::PositionJointInterface* position_joint_interface_;
  std::vector<hardware_interface::JointHandle> position_joint_handles_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  ros::Duration elapsed_time_;
  std::array<double, 7> initial_pose_{};
  std::array<double, 7> joints_pose_{};
  std::array<double, 16> initial_O_T_EE_{};
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;  // seems to be franka_states
  int idx_command = 0;
  int idx_out = 1;
  Eigen::Matrix<double, 7, 1> vq = {0, 0, 0, 0, 0, 0, 0};
  Eigen::Vector3d EEposition;
  std::array<double, 3> I_e = {0, 0, 0};
  franka_hw::TriggerRate rate_trigger_{1.0};
  realtime_tools::RealtimePublisher<PRIMITIVEmessages> PRIMITIVE_publisher_;
  const bool debug = false;
  static const int Target_Traj_ROWS = 5175;
  static const int Target_Traj_COLUMNS = 3;
  float r_star[Target_Traj_ROWS][Target_Traj_COLUMNS];
  float v_star[Target_Traj_ROWS][Target_Traj_COLUMNS];
  float q_star[Target_Traj_ROWS][9];
  std::array<double, 7> q_start{
      {-0.76543793, -0.08999656, -0.19902707, -2.04154379, -0.12972969, 2.73708789, 2.73708976}};
  std::array<double, 7> q_command{{-0.76543793, -0.08999656, -0.19902707, -2.04154379, -0.12972969, 2.73708789, 2.73708976}};
};

}  // namespace franka_example_controllers
