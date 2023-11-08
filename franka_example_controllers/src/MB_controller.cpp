// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/MB_controller.h>

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

#include <fstream>
#include <iostream>
#include <string>

namespace franka_example_controllers {

bool MBController::init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) {
  position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface_ == nullptr) {
    ROS_ERROR("MBController: Error getting position joint interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("MBController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("MBController: Wrong number of joint names, got " << joint_names.size()
                                                                       << " instead of 7 names!");
    return false;
  }
  position_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM("MBController: Exception getting joint handles: " << e.what());
      return false;
    }
  }

  std::array<double, 7> q_start{
      {-0.76543793, -0.08999656, -0.19902707, -2.04154379, -0.12972969, 2.73708789, 2.73708976}};
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
    model_handle_ =
        std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle("panda_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM("MBController: Exception getting model handle from interface: " << ex.what());
    return false;
  }
  auto* state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  try {
    state_handle_ =
        std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle("panda_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM("MBController: Exception getting state handle from interface: " << ex.what());
    return false;
  }
  return true;
}

bool saveArray(const double* pdata, size_t length, const std::string& file_path) {
  std::ofstream os(file_path.c_str(), std::ios::binary | std::ios::out);
  if (!os.is_open())
    return false;
  os.write(reinterpret_cast<const char*>(pdata), std::streamsize(length * sizeof(double)));
  os.close();
  return true;
}

bool loadArray(double* pdata, size_t length, const std::string& file_path) {
  std::ifstream is(file_path.c_str(), std::ios::binary | std::ios::in);
  if (!is.is_open())
    return false;
  is.read(reinterpret_cast<char*>(pdata), std::streamsize(length * sizeof(double)));
  is.close();
  return true;
}

void MBController::starting(const ros::Time& /* time */) {
  for (size_t i = 0; i < 7; ++i) {
    initial_pose_[i] = position_joint_handles_[i].getPosition();
  }
  initial_O_T_EE_ = model_handle_->getPose(franka::Frame::kEndEffector);
  elapsed_time_ = ros::Duration(0.0);
}

// define global vars
static const int Target_Traj_ROWS = 5175;
static const int Target_Traj_COLUMNS = 3;
double r_star[Target_Traj_ROWS][Target_Traj_COLUMNS];
double v_star[Target_Traj_ROWS][Target_Traj_COLUMNS];
// load data into the global vars
void load_target_trajectory() {
  std::ifstream inputfile_r_star(
      "/home/mahdi/ETHZ/codes/rl_reach/code/logs/currentPosition_log.txt");
  if (!inputfile_r_star.is_open()) {
    std::cout << "Error reading desired position" << std::endl;
  }
  std::ifstream inputfile_v_star("/home/mahdi/ETHZ/codes/rl_reach/code/logs/currentVel_log.txt");
  if (!inputfile_v_star.is_open()) {
    std::cout << "Error reading desired velocity" << std::endl;
  }
  for (int row = 0; row < Target_Traj_ROWS; ++row) {
    std::string row_text_r;
    std::getline(inputfile_r_star, row_text_r);
    std::istringstream row_stream_r(row_text_r);
    std::string row_text_v;
    std::getline(inputfile_v_star, row_text_v);
    std::istringstream row_stream_v(row_text_v);
    for (int column = 0; column < Target_Traj_COLUMNS; ++column) {
      double number_r;
      double number_v;
      char delimiter;
      row_stream_r >> number_r >> delimiter;
      r_star[row][column] = number_r;
      row_stream_v >> number_v >> delimiter;
      v_star[row][column] = number_v;
    }
  }
}

void MBController::update(const ros::Time& /*time*/, const ros::Duration& period) {
  elapsed_time_ += period;
  idx += 1;
  double delta_angle = M_PI / 16 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec())) * 0.2;

  for (size_t i = 0; i < 7; ++i) {
    //    joints_pose_[i] = position_joint_handles_[i].getPosition();
    if (i == 4) {
      position_joint_handles_[i].setCommand(initial_pose_[i] - delta_angle);
    } else {
      position_joint_handles_[i].setCommand(initial_pose_[i] + delta_angle);
    }
  }
  if (idx % 3000 == 0) {
    std::cout << "idx=" << idx << " \n";
    std::cout << "period=" << period << " \n";
    std::cout << "delta_angle=" << delta_angle << " \n";
    // get jacobian
    std::array<double, 42> jacobian_array =
        model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

    franka::RobotState robot_state = state_handle_->getRobotState();
    //    std::array<double, 16> O_T_EE = robot_state.O_T_EE.data();
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    Eigen::Vector3d EEposition(transform.translation());
    Eigen::Quaterniond orientation(transform.rotation());
    std::cout << "transform=\n";
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        std::cout << transform(i, j) << " ";
      }
      // Newline for new row
      std::cout << std::endl;
    }
    std::cout << "*******1-EEposition=\n";
    for (int i = 0; i < 3; i++) {
      std::cout << EEposition(i) << " ";
      std::cout << std::endl;
    }
    std::cout << "orientation_scalar=" << orientation.w();
    std::cout << std::endl;
    std::cout << "orientation_vec=" << orientation.vec();
    std::cout << std::endl;

    Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
    std::cout << "*******3-q=\n" << q;
    std::cout << std::endl;

    Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.q.data());
    std::cout << "dq=" << dq;
    std::cout << std::endl;

    double K_p = 50;
    double e_t[3];
    e_t[0] = (r_star[idx][0] - EEposition(0));
    e_t[1] = (r_star[idx][1] - EEposition(1));
    e_t[2] = (r_star[idx][2] - EEposition(2));
    std::cout << "e_t[0]=" << e_t[0];
    std::cout << std::endl;
    std::cout << "e_t[1]=" << e_t[1];
    std::cout << std::endl;
    std::cout << "e_t[2]=" << e_t[2];
    std::cout << std::endl;
    //    double K_i=50;
    //    double K_d=1;
    Eigen::Vector<double, 3> vc;
    vc(0) = v_star[idx][0] + K_p * e_t[0];  //+ K_i * np.sum(e[:,1:],1)*ts + K_d*(v_ref-v_e)
    vc(1) = v_star[idx][1] + K_p * e_t[1];
    vc(2) = v_star[idx][2] + K_p * e_t[2];

//    Eigen::Map<Eigen::MatrixXd>(vc, 6, 7);

    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    //    Eigen::Map<const Eigen::Matrix<double, 7, 1>> drdtheta(jacobian*dq);
    std::cout << "*******2-jacobian*dq=\n";
    std::cout << jacobian * dq;
    std::cout << std::endl;
    std::cout << "jacobian=\n" << jacobian;
    std::cout << std::endl;

    std::vector<int> ind_translational_jacobian{0, 1, 2, 3};
    std::vector<int> ind_dof{0, 1, 2, 3, 4, 5, 6};
    Eigen::Matrix<double, 3, 7> J_translation = jacobian(ind_translational_jacobian, ind_dof);
    std::cout << "*******4-J_translation=\n" << J_translation;//(ind_translational_jacobian, ind_dof);
    std::cout << std::endl;
    Eigen::MatrixXd J_translation_pinv;
    pseudoInverse(J_translation, J_translation_pinv);
    std::cout << "J_translation_pinv=\n" << J_translation_pinv;
    std::cout << std::endl;
    std::cout << "vc=\n" << vc;
    std::cout << std::endl;
    Eigen::Matrix<double, 7, 1> vq;
    vq = J_translation_pinv * vc;
    std::cout << "=======vq=\n" << vq;
    std::cout << std::endl;
  }
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::MBController,
                       controller_interface::ControllerBase)
