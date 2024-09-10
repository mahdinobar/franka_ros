// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/PRIMITIVE_velocity_controller.h>

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

#include <ros/callback_queue.h>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <string>
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float64MultiArray.h"

namespace franka_example_controllers {
Eigen::MatrixXd CSVopen(std::string fileToOpen) {
  // the inspiration for creating this function was drawn from here (I did NOT copy and paste the
  // code)
  // https://stackoverflow.com/questions/34247057/how-to-read-csv-file-and-assign-to-eigen-matrix
  // NOTE THAT THIS FUNCTION IS CALLED BY THE FUNCTION: SimulateSystem::openFromFile(std::string
  // Afile, std::string Bfile, std::string Cfile, std::string x0File, std::string inputSequenceFile)
  // the input is the file: "fileToOpen.csv":
  // a,b,c
  // d,e,f
  // This function converts input file data into the Eigen matrix format
  // the matrix entries are stored in this variable row-wise. For example if we have the matrix:
  // M=[a b c
  //	  d e f]
  // the entries are stored as matrixEntries=[a,b,c,d,e,f], that is the variable "matrixEntries" is
  // a row vector later on, this vector is mapped into the Eigen matrix format
  std::vector<double> matrixEntries;
  // in this object we store the data from the matrix
  std::ifstream matrixDataFile(fileToOpen);
  // this variable is used to store the row of the matrix that contains commas
  std::string matrixRowString;
  // this variable is used to store the matrix entry;
  std::string matrixEntry;
  // this variable is used to track the number of rows
  int matrixRowNumber = 0;
  while (getline(matrixDataFile,
                 matrixRowString))  // here we read a row by row of matrixDataFile and store every
                                    // line into the string variable matrixRowString
  {
    std::stringstream matrixRowStringStream(
        matrixRowString);  // convert matrixRowString that is a string to a stream variable.

    while (getline(matrixRowStringStream, matrixEntry,
                   ','))  // here we read pieces of the stream matrixRowStringStream until every
                          // comma, and store the resulting character into the matrixEntry
    {
      matrixEntries.push_back(
          stod(matrixEntry));  // here we convert the string to double and fill in the row vector
                               // storing all the matrix entries
    }
    matrixRowNumber++;  // update the column numbers
  }
  // here we convert the vector variable into the matrix and return the resulting object,
  // note that matrixEntries.data() is the pointer to the first memory location at which the entries
  // of the vector matrixEntries are stored;
  return Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      matrixEntries.data(), matrixRowNumber, matrixEntries.size() / matrixRowNumber);
}

void CameraChatterCallback(geometry_msgs::Vector3 msg) {
  ROS_INFO("I heard??????????");
  std::cout << "+++++++++++++++++++++I heard p_obj_ca+++++++++++++++++++++" << "\n";
  std::cout << "msg=" << msg << "\n";
}

PRIMITIVEVelocityController::PRIMITIVEVelocityController()
    : command_struct_(), command_struct_2_() {}

void PRIMITIVEVelocityController::cmdVelCallback(const geometry_msgs::Vector3& data) {
  command_struct_.x = data.x;
  command_struct_.y = data.y;
  command_struct_.z = data.z;
  command_struct_.stamp = ros::Time::now();
  command_.writeFromNonRT(command_struct_);
}
void PRIMITIVEVelocityController::cmdVelCallback2(const std_msgs::Float64MultiArray& command) {
  command_struct_2_.data = command.data;
  command_struct_2_.stamp2 = ros::Time::now();
  command_2_.writeFromNonRT(command_struct_2_);
}
bool PRIMITIVEVelocityController::init(hardware_interface::RobotHW* robot_hardware,
                                       ros::NodeHandle& node_handle) {
  velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
  if (velocity_joint_interface_ == nullptr) {
    ROS_ERROR("PRIMITIVEVelocityController: Error getting position joint interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("PRIMITIVEVelocityController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("PRIMITIVEVelocityController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  velocity_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "PRIMITIVEVelocityController: Exception getting joint handles: " << e.what());
      return false;
    }
  }

  //  for (size_t i = 0; i < q_start.size(); i++) {
  //    if (std::abs(velocity_joint_handles_[i].getPosition() - q_start[i]) > 0.1) {
  //      ROS_ERROR_STREAM(
  //          "PRIMITIVEVelocityController: Robot is not in the expected starting position for "
  //          "running this example. Run `roslaunch franka_example_controllers move_to_start.launch
  //          " "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
  //      return false;
  //    }
  //  }
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
    ROS_ERROR_STREAM("PRIMITIVEVelocityController: Exception getting model handle from interface: "
                     << ex.what());
    return false;
  }
  auto* state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  try {
    state_handle_ =
        std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle("panda_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM("PRIMITIVEVelocityController: Exception getting state handle from interface: "
                     << ex.what());
    return false;
  }
  PRIMITIVE_publisher_.init(node_handle, "PRIMITIVE_messages", 1e6, false);

  sub_command_ =
      node_handle.subscribe("/p_obj_ca", 100, &PRIMITIVEVelocityController::cmdVelCallback, this);
  sub_command_2_ =
      node_handle.subscribe("/T_ftc_ca", 100, &PRIMITIVEVelocityController::cmdVelCallback2, this);
  ros::spinOnce();

  //  franka::RobotState robot_state = state_handle_->getRobotState();
  //  Eigen::Map<Eigen::Matrix<double, 4, 4>> T_o_F(robot_state.O_T_EE.data());
  //  //    Eigen::Map<Eigen::Matrix<double, 4, 4>> T_ftc_ca2(curr_cmd2.data);
  //  Eigen::Matrix4d T_F_o = T_o_F.inverse();
  //  Eigen::Transform<double, 3, Eigen::Affine> transform(T_F_o);
  ////  Eigen::Affine3d transform_T_F_o(T_F_o);
  //  p_ftc2_o = transform * p_Ftoftc2_F;
  //  r_star_tf[0] = p_ftc2_o(0);
  //  r_star_tf[1] = p_ftc2_o(1);
  //  r_star_tf[2] = p_ftc2_o(2);
  //  std::cout << "T_F_o=" << T_F_o << std::endl;
  //  std::cout << "p_Ftoftc2_F=" << p_Ftoftc2_F << std::endl;
  //  std::cout << "p_ftc2_o=" << p_ftc2_o << std::endl;

  return true;
}

void PRIMITIVEVelocityController::starting(const ros::Time& /* time */) {
  for (size_t i = 0; i < 7; ++i) {
    initial_pose_[i] = velocity_joint_handles_[i].getPosition();
  }
  std::ifstream inputfile_q_star("/home/mahdi/ETHZ/codes/RLCFEP/code/logs/q_log_b_interp.txt");
  if (!inputfile_q_star.is_open()) {
    std::cout << "Error reading q_log_interp file" << std::endl;
  }
  for (int row = 0; row < Target_Traj_ROWS; ++row) {
    std::string row_text_q;
    std::getline(inputfile_q_star, row_text_q);
    std::istringstream row_stream_q(row_text_q);
    for (int column = 0; column < 9; ++column) {
      double number_q;
      char delimiter;
      row_stream_q >> number_q >> delimiter;
      q_star[row][column] = number_q;
    }
  }
  initial_O_T_EE_ = model_handle_->getPose(franka::Frame::kEndEffector);
  //  elapsed_time_ = ros::Duration();
  t_init = ros::Time::now();
  std::cout << "ROS system clock t_init=" << t_init << " \n";
  std::cout << std::endl;
  std::cout << "ros::Time::isSimTime()=" << ros::Time::isSimTime() << " \n";
  std::cout << std::endl;
  std::cout << "ros::Time::isSystemTime()=" << ros::Time::isSystemTime() << " \n";
  std::cout << std::endl;
  //  std::ifstream inputfile_r_star(
  //      "/home/mahdi/ETHZ/codes/RLCFEP/code/logs/currentPosition_log_b.txt");
  //  if (!inputfile_r_star.is_open()) {
  //    std::cout << "Error reading r_log_interp file" << std::endl;
  //  }
  //  std::ifstream
  //  inputfile_v_star("/home/mahdi/ETHZ/codes/RLCFEP/code/logs/currentVel_log_b.txt"); if
  //  (!inputfile_v_star.is_open()) {
  //    std::cout << "Error reading v_log_interp file" << std::endl;
  //  }
  //  for (int row = 0; row < Target_Traj_ROWS; ++row) {
  //    std::string row_text_r;
  //    std::string row_text_v;
  //    std::getline(inputfile_r_star, row_text_r);
  //    std::getline(inputfile_v_star, row_text_v);
  //    std::istringstream row_stream_r(row_text_r);
  //    std::istringstream row_stream_v(row_text_v);
  //    for (int column = 0; column < 3; ++column) {
  //      double number_r;
  //      double number_v;
  //      char delimiter;
  //      row_stream_r >> number_r >> delimiter;
  //      row_stream_v >> number_v >> delimiter;
  //      r_star[row][column] = number_r;
  //      v_star[row][column] = number_v;
  //    }
  //  }
  std::string x_fileToOpen = "/home/mahdi/Documents/kalman/myCode/logs/measurements/x_star.csv";
  std::string y_fileToOpen = "/home/mahdi/Documents/kalman/myCode/logs/measurements/y_star.csv";
  std::string z_fileToOpen = "/home/mahdi/Documents/kalman/myCode/logs/measurements/z_star.csv";
  std::string t_fileToOpen = "/home/mahdi/Documents/kalman/myCode/logs/measurements/t.csv";
  x_star = CSVopen("/home/mahdi/Documents/kalman/myCode/logs/measurements/x_star.csv");
  y_star = CSVopen(y_fileToOpen);
  z_star = CSVopen(z_fileToOpen);
  t_star = CSVopen(t_fileToOpen);

  // l2-norm
  double accum = 0.;
  for (int i = 0; i < 3; ++i) {
    v_star_dir[i] = (r_star_tf[i] - r_star_0[i]);
    accum += v_star_dir[i] * v_star_dir[i];
  }
  norm_v_star_dir = sqrt(accum);
}

void PRIMITIVEVelocityController::update(const ros::Time& rosTime, const ros::Duration& period) {
  //  TODO improve how smoothen initially by mp
  int mp = 1;
  if (idx_1ms > 100) {
    mp = 1;
  }
  double dti1 = 0.001 * mp;
  //  //    TODO check joints_pose_ updates and i.c. is correct
  //  for (size_t i = 0; i < 7; ++i) {
  //    joints_pose_[i] = velocity_joint_handles_[i].getPosition();
  //  }
  franka::RobotState robot_state = state_handle_->getRobotState();
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d EEposition(transform.translation());

  Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.q.data());
  //  UNCOMMENT for ROS camera camera (non) real-time communications
  //  if (idx_1ms % (mp * 100) == 0) {
  //    try {
  //      Commands curr_cmd = *(command_.readFromRT());
  //      //      std::cout << "+curr_cmd.x=" << curr_cmd.x << "\n";
  //      p_obj_ca(0) = curr_cmd.x;
  //      p_obj_ca(1) = curr_cmd.y;
  //      p_obj_ca(2) = curr_cmd.z;
  //    } catch (int N) {
  //      std::cout << "CANNOT hear p_obj_ca!" << "\n";
  //    }
  //    try {
  //      Commands2 curr_cmd2 = *(command_2_.readFromRT());
  //      //      std::cout << "+curr_cmd2.data[0]=" << curr_cmd2.data[0] << "\n";
  //      Eigen::Map<Eigen::Matrix<double, 4, 4>> T_o_ftc(robot_state.O_T_EE.data());
  //      //      Eigen::Matrix4d T_o_ftc2 = T_o_F * T_F_ftc2;
  //      for (int i = 0; i < 4; ++i) {
  //        for (int j = 0; j < 4; ++j) {
  //          T_ftc_ca(i, j) = curr_cmd2.data[i * 4 + j];
  //        }
  //      }
  //      //            Eigen::Matrix4d T_o_ca = T_o_ftc.inverse()*T_ftc2_ftc*T_ftc_ca2;
  //      Eigen::Matrix4d T_o_ca = T_o_ftc * T_ftc_ca;
  //      //            Eigen::Matrix4d T_o_ca = T_ftc_ca2*T_ftc2_ftc*T_o_ftc.inverse();
  //      Eigen::Affine3d transform_T_ca_o(T_o_ca);
  //      Eigen::Vector3d t(transform_T_ca_o.translation());
  //      //      Eigen::Matrix3d R(transform_T_ca_o.rotation());
  //      Eigen::Matrix3d R = T_o_ca({0, 1, 2}, {0, 1, 2});
  //      p_obj_o = R * p_obj_ca + t + drift;
  //      if (debug) {
  //        std::cout << "+T_o_ftc=" << T_o_ftc << "\n";
  //        //            std::cout << "+T_o_ftc.inverse()=" << T_o_ftc.inverse() << "\n";
  //        //            std::cout << "+T_ftc2_ftc=" << T_ftc2_ftc << "\n";
  //        std::cout << "+T_ftc_ca=" << T_ftc_ca << "\n";
  //        std::cout << "+T_o_ca=" << T_o_ca << "\n";
  //        std::cout << "+R=" << R << "\n";
  //        std::cout << "+t=" << t << "\n";
  //        std::cout << "+p_obj_ca=" << p_obj_ca << "\n";
  //        std::cout << "+p_obj_o=" << p_obj_o << "\n";
  //      }
  //    } catch (int N) {
  //      std::cout << "-CANNOT hear T_ftc_ca-" << "\n";
  //    }
  //  }
  if (idx_1ms % mp == 0) {
    //    std::cout << "t_star[idx_1]=" << t_star(idx_1) << "\n";
    //    elapsed_time_ += period;
    //    double v_star_dir_length =
    //        0.02011276 + (-0.0003050539 - 0.02011276) /
    //                         std::pow(1 + std::pow(idx_1 / 16.15651, 1.732438), 4.038572);
    //    double v_star_dir_length =
    //        (34.69208 +
    //         (0.000003675498 - 34.69208) / std::pow(1 + std::pow(idx_1 / 115439.1, 4.234534),
    //         15413090)) /
    //        1000;

    //    plot 34.1/(1+exp(-0.003*(x-2000))) in [0,4000]
    double v_star_dir_length = 34.1 / (1 + std::exp(-0.003 * (idx_1 - 2000))) / 1000 - 34.1 / (1 + std::exp(-0.003 * (0 - 2000))) / 1000;

    for (int i = 0; i < 3; ++i) {
      v_star[i] = v_star_dir[i] / norm_v_star_dir * v_star_dir_length;
    }
    r_star_2[0] = dti1 * v_star[0] + r_star_2[0];
    r_star_2[1] = dti1 * v_star[1] + r_star_2[1];
    r_star_2[2] = dti1 * v_star[2] + r_star_2[2];
    std::array<double, 42> jacobian_array =
        model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    std::vector<int> ind_translational_jacobian{0, 1, 2};
    std::vector<int> ind_dof{0, 1, 2, 3, 4, 5, 6};
    Eigen::Matrix<double, 3, 7> J_translation = jacobian(ind_translational_jacobian, ind_dof);
    Eigen::MatrixXd J_translation_pinv;
    //    franka::RobotState robot_state = state_handle_->getRobotState();
    //    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    //    Eigen::Vector3d EEposition(transform.translation());
    //    Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
    //    Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.q.data());
    double K_p = 1;
    double K_i = 1;

    e_t[0] = (r_star_2[0] - EEposition(0));
    e_t[1] = (r_star_2[1] - EEposition(1));
    e_t[2] = (r_star_2[2] - EEposition(2));
    Eigen::Vector<double, 3> vc;
    for (int i = 0; i < 3; ++i) {
      I_e[i] += e_t[i] * dti1;
      vc(i) = v_star[i] + K_p * e_t[i] +
              K_i * I_e[i];  //+ K_i * np.sum(e[:,1:],1)*dti1 + K_d*(v_ref-v_e)
    }
    pseudoInverse(J_translation, J_translation_pinv);
    dq_command = J_translation_pinv * vc;
    //  TODO should idx_1 be updated here or end of call?
    idx_1 += 1;

    if (false) {
      std::cout << "**************************************************idx_1=" << idx_1 << " \n";
      std::cout << "period=" << period << " \n";
      std::cout << "transform=\n";
      for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
          std::cout << transform(i, j) << " ";
        }
        std::cout << std::endl;
      }
      std::cout << "*******1-EEposition=\n";
      for (int i = 0; i < 3; i++) {
        std::cout << EEposition(i) << " ";
        std::cout << std::endl;
      }
      std::cout << "*******3-q=\n" << q;
      std::cout << std::endl;
      std::cout << "dq=" << dq;
      std::cout << std::endl;
      std::cout << "@@@@@@@@@r_star[idx_1][0]=" << r_star[idx_1][0];
      std::cout << std::endl;
      std::cout << "e_t[0]=" << e_t[0];
      std::cout << std::endl;
      std::cout << "e_t[1]=" << e_t[1];
      std::cout << std::endl;
      std::cout << "e_t[2]=" << e_t[2];
      std::cout << std::endl;
      std::cout << "*******2-jacobian*dq=\n";
      std::cout << jacobian * dq;
      std::cout << std::endl;
      std::cout << "jacobian=\n" << jacobian;
      std::cout << std::endl;
      std::cout << "*******4-J_translation=\n" << J_translation;
      std::cout << std::endl;
      std::cout << "J_translation_pinv=\n" << J_translation_pinv;
      std::cout << std::endl;
      std::cout << "vc=\n" << vc;
      std::cout << std::endl;
      std::cout << "dq_command=\n";
      for (int i = 0; i < 7; i++) {
        std::cout << dq_command(i) << std::endl;
      }
      std::cout << std::endl;
    }
    if (false) {
      if (rate_trigger_() && PRIMITIVE_publisher_.trylock()) {
        for (size_t i = 0; i < 42; ++i) {
          PRIMITIVE_publisher_.msg_.jacobian_array[i] = jacobian_array[i];
        }
        PRIMITIVE_publisher_.unlockAndPublish();
      }
      if (rate_trigger_() && PRIMITIVE_publisher_.trylock()) {
        for (size_t i = 0; i < 6; ++i) {
          for (size_t j = 0; i < 7; ++j) {
            PRIMITIVE_publisher_.msg_.jacobian[i] = jacobian(i, j);
          }
        }
        PRIMITIVE_publisher_.unlockAndPublish();
      }
      if (rate_trigger_() && PRIMITIVE_publisher_.trylock()) {
        for (size_t i = 0; i < 3; ++i) {
          for (size_t j = 0; j < 7; ++j) {
            PRIMITIVE_publisher_.msg_.J_translation[i * 3 + j] = J_translation(i, j);
            PRIMITIVE_publisher_.msg_.J_translation_pinv[i * 3 + j] = J_translation_pinv(i, j);
          }
        }
        PRIMITIVE_publisher_.unlockAndPublish();
      }
    }
  }
  if (false) {
    if (rate_trigger_() && PRIMITIVE_publisher_.trylock()) {
      for (size_t i = 0; i < 3; ++i) {
        PRIMITIVE_publisher_.msg_.r_star[i] = r_star[idx_1][i];
        //      PRIMITIVE_publisher_.msg_.v_star[i] = v_star[idx_1][i];
        PRIMITIVE_publisher_.msg_.EEposition[i] = EEposition(i);
      }
      PRIMITIVE_publisher_.unlockAndPublish();
    }
  }
  e_EE_target[0] = (r_star_tf[0] - EEposition(0));
  e_EE_target[1] = (r_star_tf[1] - EEposition(1));
  e_EE_target[2] = (r_star_tf[2] - EEposition(2));
  // l2-norm
  double accum = 0.;
  for (int i = 0; i < 3; ++i) {
    accum += e_EE_target[i] * e_EE_target[i];
  }
  double norm_e_EE_t = sqrt(accum);
  //  //  TODO // allows to wait a bit more reaching the target
  //  if (norm_e_EE_t < 0.001 && idx_1ms > 1000) {
  //    idx_i3 += 10;
  //  }
  //  //  TODO
  //  if (norm_e_EE_t < 0.001 && idx_i3 > 10) {
  if (norm_e_EE_t < 0.001) {
    std::cout << "STOPPING!!!!!!!!!!!!!!!!" << " \n";
    std::cout << "norm_e_EE_t=" << norm_e_EE_t << " \n";
    std::cout << "*******1-EEposition=\n";
    for (int i = 0; i < 3; i++) {
      std::cout << EEposition(i) << " ";
      std::cout << std::endl;
    }
    std::cout << "idx_1ms=" << idx_1ms << " \n";
    PRIMITIVEVelocityController::stopRequest(ros::Time::now());
  } else {
    for (size_t i = 0; i < 7; ++i) {
      if (std::abs(dq_command(i)/1000) > dq_max[i]) {
        std::cout << "-------i=" << i << "\n";
        std::cout << "-------NOOOOOOOOOOOO=" << dq_command(i) << "\n";
        std::cout << "-------norm_e_EE_t=" << norm_e_EE_t << "\n";
        if (std::signbit(dq_command(i)))
        {
          dq_command(i) = -dq_max[i];
        }else
        {
          dq_command(i) = +dq_max[i];
        }
      }
      velocity_joint_handles_[i].setCommand(dq_command(i));
    }
  }
  idx_1ms += 1;
  if (rate_trigger_() && PRIMITIVE_publisher_.trylock()) {
    for (size_t i = 0; i < 7; ++i) {
      // inner loop 1: ds_i1=1ms (1000 Hz)
      PRIMITIVE_publisher_.msg_.dq_c[i] = dq_command(i);
    }
    PRIMITIVE_publisher_.unlockAndPublish();
  }
}
void PRIMITIVEVelocityController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}
}  // namespace franka_example_controllers
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::PRIMITIVEVelocityController,
                       controller_interface::ControllerBase)
