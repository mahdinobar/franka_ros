// Copyright (c) 2024 Mahdi Nobar
#pragma once

#include <array>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
//#include <franka_example_controllers/PRIMITIVEmessages.h>
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
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float64MultiArray.h"

#include "std_msgs/Bool.h"

#include "/home/mahdi/catkin_ws/src/franka_ros/franka_example_controllers/src/KalmanFilter.cpp"
#include "franka_example_controllers/KalmanFilter.h"

#include <torch/script.h>
#include <torch/torch.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

#include <actionlib/client/simple_action_client.h>
#include <franka_gripper/MoveAction.h>

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
  double t_0_;
  double t_0_2_;
  double t_0_EE;
  std::array<double, 7> initial_pose_{};
  std::array<double, 7> joints_pose_{};
  std::array<double, 7> joints_vel_{};
  std::array<double, 16> initial_O_T_EE_{};
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;  // seems to be franka_states
  //  TODO check
  int k = 0;
  double k_timer = 0;

  int delay_SAC = 0;
  int k_SAC = 0;
  int k_PID = 0;
  int k_startup_speed_profile = 0;
  int idx_i3 = 0;
  std::array<double, 3> I_e = {0, 0, 0};
//  franka_hw::TriggerRate rate_trigger_{1000.0};
//  realtime_tools::RealtimePublisher<PRIMITIVEmessages> PRIMITIVE_publisher_;
  realtime_tools::RealtimePublisher<geometry_msgs::Vector3Stamped> STEPPERMOTOR_publisher_;
  realtime_tools::RealtimePublisher<geometry_msgs::Vector3Stamped> publisher_r_star_;
  realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped> publisher_dq_SAC_;
  realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped> publisher_dq_PID_;
  realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped> publisher_filtered_dq_;

  struct Commands {
    double x;
    double y;
    double z;
    double t_stamp_camera_measurement;
    Commands() : x(55.0), y(66.0), z(77.0), t_stamp_camera_measurement(0.0) {}
  };
  realtime_tools::RealtimeBuffer<Commands> command_;
  realtime_tools::RealtimeBuffer<Commands> command_EE_;
  Commands command_struct_;
//  Commands command_struct_EE_;
  ros::Subscriber sub_command_;
//  ros::Subscriber sub_command_EE_;
  bool allow_multiple_cmd_vel_publishers_;
  const bool debug = false;
  static const int Target_Traj_ROWS = 6381;
  static const int Target_Traj_COLUMNS = 3;
  float q_star[Target_Traj_ROWS][9];
  //  Eigen::Matrix<double, 3, 1> r_star_0 = {0.5341719324165605, -0.2758445190875657,
  //  0.14369105360211876};
//  Eigen::Matrix<double, 3, 1> r_star_0 = {0.53106, -0.261387, 0.13763};
//  Eigen::Matrix<double, 3, 1> r_star_0 = {0.52782, -0.26734, 0.13539};
  Eigen::Matrix<double, 3, 1> r_star_0 = {0.5345, -0.2741, 0.1441};
  Eigen::Matrix<double, 3, 1> r_star = r_star_0;
  Eigen::Matrix<double, 7, 1> dq_command_PID = {0, 0, 0, 0, 0, 0, 0};
  Eigen::Matrix<double, 7, 1> dq_command = {0, 0, 0, 0, 0, 0, 0};
  //  Eigen::Matrix<double, 3, 1> r_star_tf_start_up = {0.534121626277439, -0.2453536243445049,
  //  0.15352824044213864}; Eigen::Matrix<double, 3, 1> r_star_tf_start_up = {0.5341719324165605,
  //  -0.2458445190875657, 0.14369105360211876};
  Eigen::Matrix<double, 3, 1> r_star_tf_start_up = {0.5345, -0.2455, 0.1392}; //from geometric manual measurement 4 cm above center of upper surface of cube
  //  Eigen::Matrix<double, 3, 1> r_star_tf = {0.534121626277439, +0.229646376,
  //  0.15352824044213864}; Eigen::Matrix<double, 3, 1> r_star_tf = {0.5341719324165605,
  //  0.229155481, 0.14369105360211876}; Eigen::Matrix<double, 3, 1> r_star_tf = {0.5345, -0.2465,
  //  0.1442};
  Eigen::Matrix<double, 3, 1> r_star_tf = {0.5345, 0.2285, 0.1392};  // 475 mm forwarded
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
  void cmdVelCallback_EE(const geometry_msgs::Vector3Stamped& data);
  void openGripper();
  actionlib::SimpleActionClient<franka_gripper::MoveAction> gripper_client_;
  void cmdVelCallback2(const std_msgs::Float64MultiArray& command);
  Eigen::Vector<double, 3> drift = {0, 0, 0};
  Eigen::Vector<double, 3> p_hat_w{0, 0, 0};
  Eigen::Vector<double, 3> p_hat_EE_w{0, 0, 0};
  Eigen::MatrixXd x_star;
  Eigen::MatrixXd y_star;
  Eigen::MatrixXd z_star;
  Eigen::MatrixXd t_star;
  double norm_v_star_dir;
  bool start_up = true;
  // TODO bring into starting?
  Eigen::Matrix<double, 3, 3> A{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  Eigen::Matrix<double, 3, 1> B{{0}, {1}, {0}};
  Eigen::Matrix<double, 3, 3> C{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  // covariance matrix of the state estimation error P0- abbreviated as "state covariance matrix"
//  Eigen::Matrix<double, 3, 3> P0{{1, 0, 0}, {0, 4, 0}, {0, 0, 1}};
//  Eigen::Matrix<double, 3, 3> P0{{0.04e-6, 0, 0}, {0, 0.09e-6, 0}, {0, 0, 0.04e-6}}; // [m^2]
  Eigen::Matrix<double, 3, 3> P0{{0.01e-6, 0, 0}, {0, 0.04e-6, 0}, {0, 0, 0.01e-6}}; // [m^2]

  // covariance matrix of the measurement noise
//  Eigen::Matrix<double, 3, 3> R{{4, 0, 0}, {0, 25, 0}, {0, 0, 4}};
//  Eigen::Matrix<double, 3, 3> R{{0.0625e-6, 0, 0}, {0, 0.0625e-6, 0}, {0, 0, 0.0625e-6}}; // [m^2]
  Eigen::Matrix<double, 3, 3> R{{0.25-6, 0, 0}, {0, 36e-6, 0}, {0, 0, 0.01e-6}}; // [m^2]
  // covariance matrix of the state disturbance
//  Eigen::Matrix<double, 3, 3> Q{{1, 0, 0}, {0, 4, 0}, {0, 0, 1}};
//  Eigen::Matrix<double, 3, 3> Q{{0.01e-6, 0, 0}, {0, 0.04e-6, 0}, {0, 0, 0.02e-6}}; // [m^2]
  Eigen::Matrix<double, 3, 3> Q{{0.01e-6, 0, 0}, {0, 0.04e-6, 0}, {0, 0, 0.04e-6}}; // [m^2]
  // guess of the initial state estimate
  //  Eigen::Matrix<double, 3, 1> x0 = r_star_tf_start_up;
  // ATTENTION to dimension
  Eigen::Matrix<double, 1, 1> u;  //[m/ms]
  double u_mean = 0.0349028e-3;      //[m/ms]
  double u_std = 0.000050776e-3;  //[m/ms]
  //  TODO
  unsigned int maxDataSamples_KF = 2;
  bool received_measurement = false;
  double dt = 0;
  double dt_EE = 0;
  //  Eigen::Matrix<double, 3, 1> X_prediction_ahead = x0;
  //  Eigen::Matrix<double, 3, 1> estimatesAposteriori = x0;
  Eigen::Matrix<double, 3, 1> X_prediction_ahead;
  Eigen::Matrix<double, 3, 1> estimatesAposteriori;
  Eigen::Matrix<double, 3, 1> estimatesApriori;
  Eigen::Matrix<double, 3, 3> covarianceAposteriori = P0;
  Eigen::Matrix<double, 3, 3> covarianceApriori;
  Eigen::Matrix<double, 3, 3> gainMatrices;
  int artificial_wait_idx = 0;

  torch::jit::script::Module actor;
  Eigen::Matrix<double, 1, 6> dq_SAC{0, 0, 0, 0, 0, 0};
  // Pre-allocate the tensor and vector outside the real-time loop
  torch::Tensor obs = torch::empty({1, 27}, torch::kDouble);  // ATTENTION to SAC state space dimension
  std::vector<torch::jit::IValue> observations = {obs};       // Pre-allocate and wrap the tensor


  Eigen::Vector3d EEposition_kinematics{0, 0, 0};
  Eigen::Vector3d EEposition{0, 0, 0};
//  Eigen::Vector3d EEposition_ob2_test{0, 0, 0};
  bool received_measurement_EE = false;
  Eigen::Vector3d e_mismatch_1{0, 0, 0};
  Eigen::Vector3d e_mismatch_2{0, 0, 0};
  double K_mismatch_1 = 0.2;
  double K_mismatch_2 = 0.2;

  // [Hz] this is for SAC and PI controller commands
  int freq_PID = 10;
  int freq_SAC = 10;
  // [Hz] this is for KF+target stars at startup +PI controller feedback inputs at startup phase only
  int freq_fast = 1000;

  pinocchio::Model model_pino_biased;
  Eigen::Vector3d delta_EEposition_kinematics;


  double alpha_LPF = 1.0; // Smoothing factor of low-pass filter
  Eigen::Matrix<double, 7, 1> filtered_dq= {0, 0, 0, 0, 0, 0, 0};


  double K_p = 1.0;
  double K_i = 0.1;

  bool gripper_opened = false;
  ros::Publisher gripper_command_publisher_;

};

}  // namespace franka_example_controllers
