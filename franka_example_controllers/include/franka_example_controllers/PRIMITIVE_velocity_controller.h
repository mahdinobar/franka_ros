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
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float64MultiArray.h"

#include "franka_example_controllers/KalmanFilter.h"
#include "/home/mahdi/catkin_ws/src/franka_ros/franka_example_controllers/src/KalmanFilter.cpp"


namespace franka_example_controllers {

class PRIMITIVEVelocityController : public controller_interface::MultiInterfaceController<
                                        franka_hw::FrankaModelInterface,
                                        hardware_interface::VelocityJointInterface,
                                        franka_hw::FrankaStateInterface> {
 public:
  PRIMITIVEVelocityController();

  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;

  void starting(const ros::Time&) override;

  void update(const ros::Time&, const ros::Duration& period) override;

  void stopping(const ros::Time&) override;

  void stopRequest(const ros::Time& time);


 private:
  hardware_interface::VelocityJointInterface* velocity_joint_interface_;
  std::vector<hardware_interface::JointHandle> velocity_joint_handles_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  ros::Duration elapsed_time_;
  double t_0;
  std::array<double, 7> initial_pose_{};
  std::array<double, 7> joints_pose_{};
  std::array<double, 7> joints_vel_{};
  std::array<double, 16> initial_O_T_EE_{};
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;  // seems to be franka_states
  int idx_1 = 0;
//  TODO check
  int k = 0;
  int idx_i3 = 0;

  std::array<double, 3> I_e = {0, 0, 0};
  franka_hw::TriggerRate rate_trigger_{1000.0};
  realtime_tools::RealtimePublisher<PRIMITIVEmessages> PRIMITIVE_publisher_;
  realtime_tools::RealtimePublisher<geometry_msgs::Vector3Stamped> STEPPERMOTOR_publisher_;
  struct Commands {
    double x;
    double y;
    double z;
    ros::Time stamp;
    Commands() : x(55.0), y(66.0), z(77.0), stamp(0.0) {}
  };
  realtime_tools::RealtimeBuffer<Commands> command_;
  Commands command_struct_;
  ros::Subscriber sub_command_;

  struct Commands2 {
    std::vector<double> data;
    ros::Time stamp2;
    Commands2() : data({0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}), stamp2(0.0) {}
  };
  realtime_tools::RealtimeBuffer<Commands2> command_2_;
  Commands2 command_struct_2_;
  ros::Subscriber sub_command_2_;

  bool allow_multiple_cmd_vel_publishers_;
  const bool debug = false;
  static const int Target_Traj_ROWS = 6381;
  static const int Target_Traj_COLUMNS = 3;
  float r_star[Target_Traj_ROWS][Target_Traj_COLUMNS];
//  float v_star[Target_Traj_ROWS][Target_Traj_COLUMNS];
  float q_star[Target_Traj_ROWS][9];
//  std::array<double, 7> q_start{
//      {-0.00155713, -0.7739, -0.00012461, -2.38384, -0.00188551, 1.56271, 0.806219}};
//  std::array<double, 7> q_start{
//      {0, -0.785398163397, 0, -2.35619449019, 0, 1.57079632679, 0.785398163397}};
//  std::array<double, 3> r_star_0 = {0.30587, -0.00013972, 0.46695};
  std::array<double, 3> r_star_0 = {0.514, -0.320,   0.101};
  Eigen::Matrix<double, 7, 1> dq_command = {0, 0, 0, 0, 0, 0, 0};
//  std::array<double, 3> r_star_0 = {0.5145, -0.2698,  0.1541};

//  std::array<double, 3> r_star_tf = {+0.6250, -0.5250, +0.0250};
//  std::array<double, 3> r_star_tf = {0.5145, 0.16734781111,  0.1541};
//  std::array<double, 3> r_star_tf = {0.425,-0.025,0.025};
  std::array<double, 3> r_star_tf = {0.514, -0.270,   0.101};
  double v_star_dir[3];
  double v_star[3];
  std::array<double, 3> r_star_2 = r_star_0;
  std::array<double, 3> e_t = {0, 0, 0};
  std::array<double, 3> e_EE_target = {0, 0, 0};
  std::array<double, 7> dq_max = {0.006981317008, 0.003490658504, 0.003490658504, 0.005235987756,
                                  0.006981317008, 0.00872664626,  0.00872664626};  // dq_c [rad/1ms]
  //  Eigen::Matrix<double, 16, 1> T_F_ftc2_raw = {0.707, -0.707, 0, 0, 0.707, 0.707, 0, 0, 0, 0, 1,
  //  0, 0, 0, 0.1124, 1};
//  Eigen::Matrix<double, 4, 4> T_F_ftc2{{0.707, 0.707, 0.0, 0.0},
//                                       {-0.707, 0.707, 0.0, 0.0},
//                                       {0.0, 0.0, 1.0, 0.1124},
//                                       {0.0, 0.0, 0.0, 1.0}};
//  Eigen::Matrix<double, 4, 4> T_ftc2_ftc{{1.0, 0.0, 0.0, 0.0},
//                                       {0, 1, 0, 0},
//                                       {0, 0, 1, -0.009},
//                                       {0, 0, 0, 1}};
  Eigen::Matrix<double, 4, 4> T_ftc2_ftc{{1.0, 0.0, 0.0, 0.0},
                                       {0, 1, 0, 0},
                                       {0, 0, 1, 0},
                                       {0, 0, 0, 1}};
  void cmdVelCallback(const geometry_msgs::Vector3& data);
  void cmdVelCallback2(const std_msgs::Float64MultiArray& command);
//  Eigen::Matrix<double, 4, 4> T_ftc_ca;
//  Eigen::Vector<double, 3> p_obj_o = {0, 0, 0};
//  Eigen::Vector<double, 3> drift = {0.057, 0.016, 0.018};
  Eigen::Vector<double, 3> drift = {0,0,0};
//  Eigen::Vector<double, 3> p_obj_ca = {0, 0, 0};
  Eigen::Vector<double, 3> p_star_w_measured {0, 0, 0};
  //  Eigen::Vector<double, 3> p_Ftoftc2_F = {0, 0, +0.1124};
  //  Eigen::Vector<double, 3> p_ftc2_o = {0, 0, 0};
  Eigen::MatrixXd x_star;
  Eigen::MatrixXd y_star;
  Eigen::MatrixXd z_star;
  Eigen::MatrixXd t_star;
  double norm_v_star_dir;
  bool warm_up = true;
  int k_KF;

  // TODO bring into starting?
  Eigen::Matrix<double, 3, 3> A{{1, 0, 0},
                                {0, 1, 0},
                                {0, 0, 1}};
  Eigen::Matrix<double, 3, 1> B{{0},
                                {1},
                                {0}};
  Eigen::Matrix<double, 3, 3> C{{1, 0, 0},
                                {0, 1, 0},
                                {0, 0, 1}};
  // covariance matrix of the state estimation error P0- abbreviated as "state covariance matrix"
  Eigen::Matrix<double, 3, 3> P0{{1, 0, 0},
                                 {0, 4, 0},
                                 {0, 0, 1}};

  // covariance matrix of the measurement noise
  Eigen::Matrix<double, 3, 3> R{{4, 0, 0},
                                {0, 25, 0},
                                {0, 0, 4}};
  // covariance matrix of the state disturbance
  Eigen::Matrix<double, 3, 3> Q{{1, 0, 0},
                                {0, 4, 0},
                                {0, 0, 1}};
  // guess of the initial state estimate
  Eigen::Matrix<double, 3, 1> x0{{514},
                                 {-270},
                                 {101}};
  Eigen::Matrix<double, 1, 1> u{{0.0341}};
  //  TODO
  unsigned int maxDataSamples_KF = 2;
  double u_KF = 0.0341;
  bool received_measurement=false;
  double dt;


  Eigen::Matrix<double, 3, 1> X_prediction_ahead =x0;
  Eigen::Matrix<double, 3, 1> estimatesAposteriori =x0;
  Eigen::Matrix<double, 3, 1> estimatesApriori;
  Eigen::Matrix<double, 3, 3> covarianceAposteriori =P0;
  Eigen::Matrix<double, 3, 3> covarianceApriori;
  Eigen::Matrix<double, 3, 3> gainMatrices;




};
}  // namespace franka_example_controllers
