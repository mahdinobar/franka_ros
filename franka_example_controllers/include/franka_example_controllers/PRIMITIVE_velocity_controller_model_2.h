// Copyright (c) 2024 Mahdi Nobar
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
#include "geometry_msgs/Vector3Stamped.h"
#include "std_msgs/Float64MultiArray.h"

#include "/home/mahdi/catkin_ws/src/franka_ros/franka_example_controllers/src/KalmanFilter.cpp"
#include "franka_example_controllers/KalmanFilter.h"

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
  //  TODO check
  int k = 0;
  int k_c = 0;
  int idx_i3 = 0;
  std::array<double, 3> I_e = {0, 0, 0};
  franka_hw::TriggerRate rate_trigger_{1000.0};
  realtime_tools::RealtimePublisher<PRIMITIVEmessages> PRIMITIVE_publisher_;
  realtime_tools::RealtimePublisher<geometry_msgs::Vector3Stamped> STEPPERMOTOR_publisher_;
  struct Commands {
    double x;
    double y;
    double z;
    double t_stamp_camera_measurement;
    Commands() : x(55.0), y(66.0), z(77.0), t_stamp_camera_measurement(0.0) {}
  };
  realtime_tools::RealtimeBuffer<Commands> command_;
  Commands command_struct_;
  ros::Subscriber sub_command_;
  bool allow_multiple_cmd_vel_publishers_;
  const bool debug = false;
  static const int Target_Traj_ROWS = 6381;
  static const int Target_Traj_COLUMNS = 3;
  float q_star[Target_Traj_ROWS][9];
  Eigen::Matrix<double, 3, 1> r_star_0 = {0.534, -0.2965, 0.1542};
  Eigen::Matrix<double, 3, 1> r_star = r_star_0;
  Eigen::Matrix<double, 7, 1> dq_command = {0, 0, 0, 0, 0, 0, 0};
  Eigen::Matrix<double, 3, 1> r_star_tf_warm_up = {0.534, -0.2465, 0.1542};
  Eigen::Matrix<double, 3, 1> r_star_tf = {0.534, +0.2285, 0.1542};
  double v_star_dir[3];
  double v_star[3];
  std::array<double, 3> e_t = {0, 0, 0};
  std::array<double, 3> e_EE_target = {0, 0, 0};
  std::array<double, 7> dq_max = {0.006981317008, 0.003490658504, 0.003490658504, 0.005235987756,
                                  0.006981317008, 0.00872664626,  0.00872664626};  // dq_c [rad/1ms]
  Eigen::Matrix<double, 4, 4> T_ftc2_ftc{{1.0, 0.0, 0.0, 0.0},
                                         {0, 1, 0, 0},
                                         {0, 0, 1, 0},
                                         {0, 0, 0, 1}};
  void cmdVelCallback(const geometry_msgs::Vector3Stamped& data);
  void cmdVelCallback2(const std_msgs::Float64MultiArray& command);
  Eigen::Vector<double, 3> drift = {0, 0, 0};
  Eigen::Vector<double, 3> p_hat_w{0, 0, 0};
  Eigen::MatrixXd x_star;
  Eigen::MatrixXd y_star;
  Eigen::MatrixXd z_star;
  Eigen::MatrixXd t_star;
  double norm_v_star_dir;
  bool warm_up = true;
  int k_KF = 0;
  // TODO bring into starting?
  Eigen::Matrix<double, 3, 3> A{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  Eigen::Matrix<double, 3, 1> B{{0}, {1}, {0}};
  Eigen::Matrix<double, 3, 3> C{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  // covariance matrix of the state estimation error P0- abbreviated as "state covariance matrix"
  Eigen::Matrix<double, 3, 3> P0{{1, 0, 0}, {0, 4, 0}, {0, 0, 1}};

  // covariance matrix of the measurement noise
  Eigen::Matrix<double, 3, 3> R{{4, 0, 0}, {0, 25, 0}, {0, 0, 4}};
  // covariance matrix of the state disturbance
  Eigen::Matrix<double, 3, 3> Q{{1, 0, 0}, {0, 4, 0}, {0, 0, 1}};
  // guess of the initial state estimate
  Eigen::Matrix<double, 3, 1> x0 = r_star_tf_warm_up;
  // ATTENTION to dimension
  Eigen::Matrix<double, 1, 1> u;  //[m/ms]
  double u_mean = 0.0341e-3;      //[m/ms]
  double u_std = 0.000050776e-3;  //[m/ms]
  //  TODO
  unsigned int maxDataSamples_KF = 2;
  bool received_measurement = false;
  double dt = 0;
  Eigen::Matrix<double, 3, 1> X_prediction_ahead = x0;
  Eigen::Matrix<double, 3, 1> estimatesAposteriori = x0;
  Eigen::Matrix<double, 3, 1> estimatesApriori;
  Eigen::Matrix<double, 3, 3> covarianceAposteriori = P0;
  Eigen::Matrix<double, 3, 3> covarianceApriori;
  Eigen::Matrix<double, 3, 3> gainMatrices;
  int artificial_wait_idx = 0;

  double K_p = 5;
  double K_i = 0.5;
};
}  // namespace franka_example_controllers
