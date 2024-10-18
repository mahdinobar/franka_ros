// Copyright (c) 2024 Mahdi Nobar
#define MODEL_0 0  // or 0 if you want Kalman Filter MODEL_0 to be false
#define MODEL_1 0  // or 0 if you want KF MODEL_1 to be false
#define MODEL_2 1  // or 0 if you want KF MODEL_2 to be false
#if MODEL_0
#include </home/mahdi/catkin_ws/src/franka_ros/franka_example_controllers/include/franka_example_controllers/PRIMITIVE_velocity_controller.h>
#elif MODEL_1
#include </home/mahdi/catkin_ws/src/franka_ros/franka_example_controllers/include/franka_example_controllers/PRIMITIVE_velocity_controller_model_1.h>
#elif MODEL_2
#include </home/mahdi/catkin_ws/src/franka_ros/franka_example_controllers/include/franka_example_controllers/PRIMITIVE_velocity_controller_model_2.h>
#endif

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
#include <random>
#include <string>
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "std_msgs/Float64MultiArray.h"

#include <torch/script.h>
#include <torch/torch.h>

namespace franka_example_controllers {
Eigen::MatrixXd CSVopen(std::string fileToOpen) {
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
  return Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      matrixEntries.data(), matrixRowNumber, matrixEntries.size() / matrixRowNumber);
}

PRIMITIVEVelocityController::PRIMITIVEVelocityController() : command_struct_() {}
void PRIMITIVEVelocityController::cmdVelCallback(const geometry_msgs::Vector3Stamped& data) {
  command_struct_.x = data.vector.x;
  command_struct_.y = data.vector.y;
  //  TODO pay attention
  command_struct_.z = data.vector.z;
  //  TODO correct time stamp must be immediately after capturing data?e.g.,timestamp of the depth
  //  TODO map?
  //  command_struct_.stamp = ros::Time::now();
  command_struct_.t_stamp_camera_measurement = data.header.stamp.toSec();
  command_.writeFromNonRT(command_struct_);
  received_measurement = true;
  cout << "Camera measurement received!" << endl;
  cout << "data.x" << data.vector.x << endl;
  cout << "data.y" << data.vector.y << endl;
  cout << "data.z" << data.vector.z << endl;
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
  //  TODO
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
  STEPPERMOTOR_publisher_.init(node_handle, "STEPPERMOTOR_messages", 1e6, false);
  sub_command_ =
      node_handle.subscribe("/p_hat_w", 100, &PRIMITIVEVelocityController::cmdVelCallback, this);
  ros::spinOnce();

  //  position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
  //  if (position_joint_interface_ == nullptr) {
  //    ROS_ERROR(
  //        "JointPositionExampleController: Error getting position joint interface from
  //        hardware!");
  //    return false;
  //  }
  //  position_joint_handles_.resize(7);
  //  for (size_t i = 0; i < 7; ++i) {
  //    try {
  //      position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
  //    } catch (const hardware_interface::HardwareInterfaceException& e) {
  //      ROS_ERROR_STREAM(
  //          "JointPositionExampleController: Exception getting joint handles: " << e.what());
  //      return false;
  //    }
  // Load your serialized model --- SAC Actor Neural Network
  actor = torch::jit::load(
      "/home/mahdi/catkin_ws/src/franka_ros/franka_example_controllers/config/"
      "traced_model_Cpp.pt");
  std::cout << "+++++Actor model loaded successfully.+++++" << std::endl;
  //  torch::Tensor input_tensor = torch::ones({1, 27});  // Example random tensor
  //  // Wrap inputs in a vector of torch::jit::IValue
  //  std::vector<torch::jit::IValue> inputs;
  //  inputs.push_back(input_tensor);
  //  // Run the model's forward pass
  //  torch::jit::IValue output = actor.forward(inputs);
  //  // Assuming the model returns two outputs as a tuple
  //  auto outputs = output.toTuple();
  //  // Extract the individual outputs from the tuple
  //  torch::Tensor output_1 = outputs->elements()[0].toTensor();
  //  // Print the outputs (or use them as needed)
  //  std::cout << "Output 1: " << output_1 << std::endl;
  //  auto sizes = output_1.sizes();
  //  std::cout << "Output 1 values: " << std::endl;
  //  if (output_1.dim() == 2) {
  //    // For a 1D tensor
  //    for (int i = 0; i < output_1.size(1); ++i) {
  //      std::cout << output_1[0][i].item<float>() << " ";  // Assuming it's a float tensor
  //    }
  //  }

  return true;
}

void PRIMITIVEVelocityController::starting(const ros::Time& /* time */) {
  t_0 = ros::Time::now().toSec();
  //  ros::Time t_check = ros::Time::now();
  //  cout << "t_0=" << t_0 << endl;
  //  cout << "t_check=" << t_check << endl;
  std::cout << "ros::Time::isSimTime()=" << ros::Time::isSimTime() << " \n";
  std::cout << std::endl;
  std::cout << "ros::Time::isSystemTime()=" << ros::Time::isSystemTime() << " \n";
  std::cout << std::endl;
  for (size_t i = 0; i < 7; ++i) {
    initial_pose_[i] = velocity_joint_handles_[i].getPosition();
  }
  initial_O_T_EE_ = model_handle_->getPose(franka::Frame::kEndEffector);
  ////  TODO do you still need offline data?
  ////  load offline measurement data just for development
  //  std::string x_fileToOpen =
  //      "/home/mahdi/Documents/kalman/myCode/logs/measurements/x_star_model_0.csv";
  //  std::string y_fileToOpen =
  //      "/home/mahdi/Documents/kalman/myCode/logs/measurements/y_star_model_0.csv";
  //  std::string z_fileToOpen =
  //      "/home/mahdi/Documents/kalman/myCode/logs/measurements/z_star_model_0.csv";
  //  std::string t_fileToOpen =
  //  "/home/mahdi/Documents/kalman/myCode/logs/measurements/t_model_1.csv"; x_star =
  //  CSVopen(x_fileToOpen) / 1000; y_star = CSVopen(y_fileToOpen) / 1000; z_star =
  //  CSVopen(z_fileToOpen) / 1000; t_star = CSVopen(t_fileToOpen); std::cout << "x_star=" <<
  //  x_star
  //  <<"\n"; std::cout << "y_star(0)=" << y_star(0) << "\n"; std::cout << "y_star(last)=" <<
  //  y_star(Eigen::last) << "\n"; std::cout << "y_star(1000)=" << y_star(1000) <<"\n"; std::cout
  //  << "z_star=" << z_star <<"\n"; std::cout << "t_star=" << t_star <<"\n";
  // l2-norm
  double accum = 0.;
  for (int i = 0; i < 3; ++i) {
    v_star_dir[i] = (r_star_tf_warm_up(i) - r_star_0(i));
    accum += v_star_dir[i] * v_star_dir[i];
  }
  norm_v_star_dir = sqrt(accum);
}

void PRIMITIVEVelocityController::update(const ros::Time& rosTime, const ros::Duration& period) {
  //  TODO change command frequency by ms
  int ms = 1;
  //  TODO Attention on subscription rate
  if (k % (ms * 1) == 0) {
    try {
      Commands curr_cmd = *(command_.readFromRT());
      //      TODO Pay attention
      p_hat_w(0) = (curr_cmd.x + 21) / 1000;
      p_hat_w(1) = (curr_cmd.y + 25) / 1000;
      p_hat_w(2) = (curr_cmd.z + 54) / 1000;
      // TODO
      double t_measurement = curr_cmd.t_stamp_camera_measurement;
      dt = (t_measurement - t_0) * 1000;  //[ms]
      t_0 = t_measurement;
    } catch (int N) {
      std::cout << "ERROR: CANNOT hear p_hat_w!" << "\n";
    }
  }
  double dti1 = 0.001 * ms;
  //  //    TODO check joints_pose_ updates and i.c. is correct
  //  for (size_t i = 0; i < 7; ++i) {
  //    joints_pose_[i] = velocity_joint_handles_[i].getPosition();
  //  }
  franka::RobotState robot_state = state_handle_->getRobotState();
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d EEposition(transform.translation());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.q.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_J(robot_state.tau_J.data());
  //  UNCOMMENT for ROS camera camera (non) real-time communications
  //  if (k % (ms * 100) == 0) {
  //    try {
  //      Commands curr_cmd = *(command_.readFromRT());
  //      //      std::cout << "+curr_cmd.x=" << curr_cmd.x << "\n";
  //      p_hat_w(0) = curr_cmd.x;
  //      p_hat_w(1) = curr_cmd.y;
  //      p_hat_w(2) = curr_cmd.z;
  //    } catch (int N) {
  //      std::cout << "CANNOT hear p_hat_w!" << "\n";
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
  //      p_obj_o = R * p_hat_w + t + drift;
  //      if (debug) {
  //        std::cout << "+T_o_ftc=" << T_o_ftc << "\n";
  //        //            std::cout << "+T_o_ftc.inverse()=" << T_o_ftc.inverse() << "\n";
  //        //            std::cout << "+T_ftc2_ftc=" << T_ftc2_ftc << "\n";
  //        std::cout << "+T_ftc_ca=" << T_ftc_ca << "\n";
  //        std::cout << "+T_o_ca=" << T_o_ca << "\n";
  //        std::cout << "+R=" << R << "\n";
  //        std::cout << "+t=" << t << "\n";
  //        std::cout << "+p_hat_w=" << p_hat_w << "\n";
  //        std::cout << "+p_obj_o=" << p_obj_o << "\n";
  //      }
  //    } catch (int N) {
  //      std::cout << "-CANNOT hear T_ftc_ca-" << "\n";
  //    }
  //  }
  if (k % ms == 0) {
    //     TODO smooth warm_up speed profile
    double v_star_dir_length = 34.9028 / (1 + std::exp(-0.04 * (k_c - 250))) / 1000 -
                               34.9028 / (1 + std::exp(-0.04 * (0 - 250))) / 1000;
    if (warm_up == true) {
      for (int i = 0; i < 3; ++i) {
        v_star[i] = v_star_dir[i] / norm_v_star_dir * v_star_dir_length;
        r_star(i) = dti1 * v_star[i] + r_star(i);
      }
    } else if (warm_up == false) {
      //  TODO how can you make KF conditions especially initially more efficient?
      if (received_measurement == true and dt > 0) {
        if (MODEL_0) {
          B(1) = dt;  //[ms]
          estimatesApriori = A * estimatesAposteriori + B * u;
          covarianceApriori = A * covarianceAposteriori * (A.transpose()) + Q;
          Eigen::Matrix<double, 3, 3> Sk;
          Sk = R + C * covarianceApriori * (C.transpose());
          Sk = Sk.inverse();
          gainMatrices = covarianceApriori * (C.transpose()) * Sk;
          estimatesAposteriori = estimatesApriori + gainMatrices * (p_hat_w - C * estimatesApriori);
          cout << "&&&&&&&&&&&&&&&&&&&&&&&&" << estimatesApriori << endl;
          cout << "estimatesApriori=" << estimatesApriori << endl;
          cout << "gainMatrices=" << gainMatrices << endl;
          cout << "p_hat_w=" << p_hat_w << endl;
          cout << "C=" << C << endl;
          cout << "estimatesApriori=" << estimatesApriori << endl;
          cout << "estimatesAposteriori=" << estimatesAposteriori << endl;
          Eigen::MatrixXd In;
          In = Eigen::MatrixXd::Identity(3, 3);
          Eigen::MatrixXd IminusKC;
          IminusKC.resize(3, 3);
          IminusKC = In - gainMatrices * C;  // I-KC
          covarianceAposteriori = IminusKC * covarianceApriori * (IminusKC.transpose()) +
                                  gainMatrices * R * (gainMatrices.transpose());
          X_prediction_ahead = estimatesAposteriori;
          received_measurement = false;
          k_KF += 1;
        }
        if (MODEL_1) {
          A(0, 3) = dt;  //[ms]
          A(1, 4) = dt;  //[ms]
          A(2, 5) = dt;  //[ms]
          estimatesApriori = A * estimatesAposteriori + B * u;
          covarianceApriori = A * covarianceAposteriori * (A.transpose()) + Q;
          Eigen::Matrix<double, 3, 3> Sk;
          Sk = R + C * covarianceApriori * (C.transpose());
          Sk = Sk.inverse();
          gainMatrices = covarianceApriori * (C.transpose()) * Sk;
          estimatesAposteriori = estimatesApriori + gainMatrices * (p_hat_w - C * estimatesApriori);
          Eigen::MatrixXd In;
          In = Eigen::MatrixXd::Identity(6, 6);
          Eigen::MatrixXd IminusKC;
          IminusKC.resize(6, 6);
          IminusKC = In - gainMatrices * C;  // I-KC
          covarianceAposteriori = IminusKC * covarianceApriori * (IminusKC.transpose()) +
                                  gainMatrices * R * (gainMatrices.transpose());
          X_prediction_ahead = estimatesAposteriori;
          received_measurement = false;
          k_KF += 1;
        }
        if (MODEL_2) {
          //          std::random_device rd{};
          //          std::mt19937 gen{rd()};
          //          std::normal_distribution<double> d{0.0349, 0.000050776};
          std::random_device rd{};
          std::mt19937 gen{rd()};
          std::normal_distribution<double> gauss_dist{u_mean, u_std};
          u(0, 0) = gauss_dist(gen);
          //          cout << "u(0, 0)=" << u(0, 0) << endl;
          B(1) = dt;  //[ms]
          estimatesApriori = A * estimatesAposteriori + B * u;
          covarianceApriori = A * covarianceAposteriori * (A.transpose()) + Q;
          Eigen::Matrix<double, 3, 3> Sk;
          Sk = R + C * covarianceApriori * (C.transpose());
          Sk = Sk.inverse();
          gainMatrices = covarianceApriori * (C.transpose()) * Sk;
          estimatesAposteriori = estimatesApriori + gainMatrices * (p_hat_w - C * estimatesApriori);
          Eigen::MatrixXd In;
          In = Eigen::MatrixXd::Identity(3, 3);
          Eigen::MatrixXd IminusKC;
          IminusKC.resize(3, 3);
          IminusKC = In - gainMatrices * C;  // I-KC
          covarianceAposteriori = IminusKC * covarianceApriori * (IminusKC.transpose()) +
                                  gainMatrices * R * (gainMatrices.transpose());
          X_prediction_ahead = estimatesAposteriori;
          received_measurement = false;
          k_KF += 1;
        }
      } else {
        if (MODEL_0) {
          B(1) = 1;  //[ms]
          X_prediction_ahead = A * X_prediction_ahead + B * u;
        }
        if (MODEL_1) {
          A(0, 3) = 1;  //[ms]
          A(1, 4) = 1;  //[ms]
          A(2, 5) = 1;  //[ms]
          X_prediction_ahead = A * X_prediction_ahead + B * u;
        }
        if (MODEL_2) {
          std::random_device rd{};
          std::mt19937 gen{rd()};
          std::normal_distribution<double> gauss_dist{u_mean, u_std};
          u(0, 0) = gauss_dist(gen);
          //          cout << "u(0, 0)=" << u(0, 0) << endl;
          B(1) = 1;  //[ms]
          X_prediction_ahead = A * X_prediction_ahead + B * u;
        }
      }
      if (MODEL_0) {
        v_star[0] = 0;
        // ATTENTION to dimension
        v_star[1] = 0.0341;  //[m/s]
        v_star[2] = 0;
        //      offline just demo KF
        //      r_star[0] = x_star(k_KF);
        //      r_star[1] = y_star(k_KF);
        //      r_star[2] = z_star(k_KF);
        //      k_KF += 1;
        r_star(0) = X_prediction_ahead(0);
        r_star(1) = X_prediction_ahead(1);
        r_star(2) = X_prediction_ahead(2);
      }
      if (MODEL_1) {
        r_star(0) = X_prediction_ahead(0);
        r_star(1) = X_prediction_ahead(1);
        r_star(2) = X_prediction_ahead(2);
        v_star[0] = X_prediction_ahead(3) * 1000;
        v_star[1] = X_prediction_ahead(4) * 1000;
        v_star[2] = X_prediction_ahead(5) * 1000;
      }
      if (MODEL_2) {
        r_star(0) = X_prediction_ahead(0);
        r_star(1) = X_prediction_ahead(1);
        r_star(2) = X_prediction_ahead(2);
        v_star[0] = 0;
        v_star[1] = u(0, 0) * 1000;  //[m/s]
        v_star[2] = 0;
      }
    }
    std::array<double, 42> jacobian_array =
        model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    std::vector<int> ind_translational_jacobian{0, 1, 2};
    std::vector<int> ind_dof{0, 1, 2, 3, 4, 5, 6};
    Eigen::Matrix<double, 3, 7> J_translation = jacobian(ind_translational_jacobian, ind_dof);
    //    if (k % 2000 == 0) {
    //      cout << "+++J_translation=" << J_translation << "\n";
    //    }
    Eigen::MatrixXd J_translation_pinv;
    e_t[0] = (-r_star(0) + EEposition(0));
    e_t[1] = (-r_star(1) + EEposition(1));
    e_t[2] = (-r_star(2) + EEposition(2));
    Eigen::Vector<double, 3> vc;
    for (int i = 0; i < 3; ++i) {
      // ATTENTION to dimenstion
      I_e[i] += -e_t[i] * dti1;  // in [m/s] because jacobian is in m to rad and dq are in rad/sec
      vc(i) = v_star[i] + K_p * (-e_t[i]) +
              K_i * I_e[i];  //+ K_i * np.sum(e[:,1:],1)*dti1 + K_d*(v_ref-v_e)
    }
    pseudoInverse(J_translation, J_translation_pinv);
    dq_command_PID = J_translation_pinv * vc;

    if (k % 1000 == 0) {
      torch::Tensor obs = torch::tensor({static_cast<float>(e_t.at(0)),
                                         static_cast<float>(e_t.at(1)),
                                         static_cast<float>(e_t.at(2)),
                                         static_cast<float>(q(0)),
                                         static_cast<float>(q(1)),
                                         static_cast<float>(q(2)),
                                         static_cast<float>(q(3)),
                                         static_cast<float>(q(4)),
                                         static_cast<float>(q(5)),
                                         static_cast<float>(dq(0)),
                                         static_cast<float>(dq(1)),
                                         static_cast<float>(dq(2)),
                                         static_cast<float>(dq(3)),
                                         static_cast<float>(dq(4)),
                                         static_cast<float>(dq(5)),
                                         static_cast<float>(tau_J(0)),
                                         static_cast<float>(tau_J(1)),
                                         static_cast<float>(tau_J(2)),
                                         static_cast<float>(tau_J(3)),
                                         static_cast<float>(tau_J(4)),
                                         static_cast<float>(tau_J(5)),
                                         static_cast<float>(dq_command_PID(0)),
                                         static_cast<float>(dq_command_PID(1)),
                                         static_cast<float>(dq_command_PID(2)),
                                         static_cast<float>(dq_command_PID(3)),
                                         static_cast<float>(dq_command_PID(4)),
                                         static_cast<float>(dq_command_PID(5))},
                                        torch::kFloat32)
                              .reshape({1, 27});
      // Wrap inputs in a vector of torch::jit::IValue
      std::vector<torch::jit::IValue> observations;
      observations.push_back(obs);
      // Run the model's forward pass
      torch::jit::IValue output = actor.forward(observations);
      // Extract tensors from the output tuple
      auto output_tuple = output.toTuple();
      torch::Tensor output_tensor = output_tuple->elements()[0].toTensor();
      // Convert Tensor to Eigen matrix
      Eigen::MatrixXd dq_SAC(output_tensor.size(0), output_tensor.size(1));
      std::cout << "output_tensor.size(0)= " << output_tensor.size(0) << std::endl;
      std::cout << "output_tensor.size(1)= " << output_tensor.size(1) << std::endl;
      std::memcpy(dq_SAC.data(), output_tensor.data_ptr<float>(),
                  output_tensor.numel() * sizeof(float));
      std::cout << "!!!!!!!!!!!dq_SAC=";
      for (int i = 0; i < 6; i++) {
        std::cout << dq_SAC(i) << " ";
      }
      std::cout << "\n";
    }
    //    dq_command = dq_command_PID + dq_SAC;

    dq_command = dq_command_PID;
    //  TODO should k_c be updated here or end of call?
    k_c += 1;
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
    //      if (rate_trigger_() && PRIMITIVE_publisher_.trylock()) {
    //        for (size_t i = 0; i < 3; ++i) {
    //          PRIMITIVE_publisher_.msg_.r_star[i] = r_star[k_c][i];
    //          //      PRIMITIVE_publisher_.msg_.v_star[i] = v_star[k_c][i];
    //          PRIMITIVE_publisher_.msg_.EEposition[i] = EEposition(i);
    //        }
    //        PRIMITIVE_publisher_.unlockAndPublish();
    //      }
  }
  if (warm_up == true) {
    e_EE_target[0] = (r_star_tf_warm_up(0) - EEposition(0));
    e_EE_target[1] = (r_star_tf_warm_up(1) - EEposition(1));
    e_EE_target[2] = (r_star_tf_warm_up(2) - EEposition(2));
  } else if (warm_up == false) {
    e_EE_target[0] = (r_star_tf(0) - EEposition(0));
    e_EE_target[1] = (r_star_tf(1) - EEposition(1));
    e_EE_target[2] = (r_star_tf(2) - EEposition(2));
  }
  // l2-norm
  double accum = 0.;
  for (int i = 0; i < 3; ++i) {
    accum += e_EE_target[i] * e_EE_target[i];
  }
  double norm_e_EE_t = sqrt(accum);
  //  TODO check
  if (norm_e_EE_t < 0.001 and warm_up == true) {
    if (false) {
      std::cout << "==========Warm-up ended==========" << " \n";
      std::cout << "norm_e_EE_t=" << norm_e_EE_t << " \n";
      std::cout << "EEposition=\n";
    }
    for (int i = 0; i < 3; i++) {
      std::cout << EEposition(i) << " ";
      std::cout << std::endl;
    }
    if (false) {
      std::cout << "k=" << k << " \n";
      std::cout << "k_c=" << k_c << " \n";
    }
    //    TODO this is not necessarily is going to lock
    //    publish message to switch on the conveyor belt
    if (rate_trigger_() && STEPPERMOTOR_publisher_.trylock()) {
      STEPPERMOTOR_publisher_.msg_.vector.x = 1;  // send command to trigger stepper motor
      STEPPERMOTOR_publisher_.msg_.header.stamp = ros::Time::now();
      STEPPERMOTOR_publisher_.unlockAndPublish();
    }
    //    TODO artificially wait to be sure the command published for the stepper motor trigger
    //    TODO implement more efficient solution
    artificial_wait_idx += 1;
    if (artificial_wait_idx > 10) {  // 3 ms artificial delay
      std::cout << "waiting!, artificial_wait_idx=" << artificial_wait_idx << " \n";
      warm_up = false;
      //    TODO
      //  // TODO uncomment for offline demo
      //      r_star_tf_warm_up[0] = 511 / 1000;
      //      r_star_tf_warm_up[1] = 150 / 1000;
      //      r_star_tf_warm_up[2] = 101 / 1000;
      //      r_star_tf_warm_up[0] = p_hat_w(0)/1000;
      //      r_star_tf_warm_up[1] = p_hat_w(1)/1000;
      //      r_star_tf_warm_up[2] = p_hat_w(2)/1000;
      //      r_star_tf_warm_up[0] = x_star(Eigen::last);
      //      r_star_tf_warm_up[1] = y_star(Eigen::last);
      //      r_star_tf_warm_up[2] = z_star(Eigen::last);
    }
    //      TODO improve conditions
  } else if ((norm_e_EE_t < 0.001 and warm_up == false)) {
    if (false) {
      std::cout << "++++++++++++++++TARGET REACHED, STOPPING+++++++++++++++" << " \n";
      std::cout << "k_KF=" << k_KF << " \n";
      std::cout << "k_c=" << k_c << " \n";
      std::cout << "k=" << k << " \n";
      std::cout << "norm_e_EE_t=" << norm_e_EE_t << " \n";
      std::cout << "EEposition=\n";
    }
    for (int i = 0; i < 3; i++) {
      std::cout << EEposition(i) << " ";
      std::cout << std::endl;
    }
    PRIMITIVEVelocityController::stopRequest(ros::Time::now());
  } else {
    if (false) {
      std::cout << "==================================" << " \n";
      std::cout << "norm_e_EE_t=" << norm_e_EE_t << " \n";
      std::cout << "p_hat_w(0)=" << p_hat_w(0) << " \n";
      std::cout << "p_hat_w(1)=" << p_hat_w(1) << " \n";
      std::cout << "p_hat_w(2)=" << p_hat_w(2) << " \n";
      std::cout << "r_star(0)=" << r_star(0) << " \n";
      std::cout << "r_star(1)=" << r_star(1) << " \n";
      std::cout << "r_star(2)=" << r_star(2) << " \n";
      std::cout << "EEposition(0)=" << EEposition(0) << " \n";
      std::cout << "EEposition(1)=" << EEposition(1) << " \n";
      std::cout << "EEposition(2)=" << EEposition(2) << " \n";
      std::cout << "X_prediction_ahead(0)=" << X_prediction_ahead(0) << " \n";
      std::cout << "X_prediction_ahead(1)=" << X_prediction_ahead(1) << " \n";
      std::cout << "X_prediction_ahead(2)=" << X_prediction_ahead(2) << " \n";
      std::cout << "x0(0)=" << x0(0) << " \n";
      std::cout << "x0(1)=" << x0(1) << " \n";
      std::cout << "x0(2)=" << x0(2) << " \n";
      std::cout << "dt=" << dt << " \n";
      std::cout << "k=" << k << " \n";
      std::cout << "k_KF=" << k_KF << " \n";
    }

    //    //    TODO is it efficient to poublish always like this?!
    //    for (size_t i = 0; i < 10; ++i) {
    //      //    publish message to switch on the conveyor belt
    //      if (rate_trigger_() && STEPPERMOTOR_publisher_.trylock()) {
    //        STEPPERMOTOR_publisher_.msg_.vector.x = 1;
    //        STEPPERMOTOR_publisher_.msg_.vector.y = 2025;
    //        STEPPERMOTOR_publisher_.msg_.header.stamp = ros::Time::now();
    //        STEPPERMOTOR_publisher_.unlockAndPublish();
    //      }
    //    }
    //    //    TODO this is not necessarily is going to lock so I put here continuously to try to
    //    send switch on command
    //    //    publish message to switch on the conveyor belt
    //    if (rate_trigger_() && STEPPERMOTOR_publisher_.trylock()) {
    //      STEPPERMOTOR_publisher_.msg_.vector.x = 1;
    //      STEPPERMOTOR_publisher_.msg_.header.stamp=ros::Time::now();
    //      STEPPERMOTOR_publisher_.unlockAndPublish();
    //    }
    //  enforce joint constraints
    for (size_t i = 0; i < 7; ++i) {
      if (std::abs(dq_command(i) / 1000) > dq_max[i]) {
        if (true) {
          std::cout << "------------At joint i=" << i << "\n";
          std::cout << "JOINT LIMIT HIT!" << endl;
          std::cout << "dq_command(i)" << dq_command(i) << "\n";
          std::cout << "norm_e_EE_t=" << norm_e_EE_t << "\n";
          std::cout << "k=" << k << "\n";
        }
        if (std::signbit(dq_command(i))) {
          dq_command(i) = -dq_max[i];
        } else {
          dq_command(i) = +dq_max[i];
        }
      }
      //              send control command
      velocity_joint_handles_[i].setCommand(dq_command(i));
    }
  }
  k += 1;
  //  TODO can this publish be moved just after command?
  if (rate_trigger_() && PRIMITIVE_publisher_.trylock()) {
    for (size_t i = 0; i < 7; ++i) {
      // inner loop 1: k=1ms (1000 Hz)
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
