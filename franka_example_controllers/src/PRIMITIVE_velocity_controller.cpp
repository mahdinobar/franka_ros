// Copyright (c) 2024 Mahdi Nobar

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

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
#include <chrono>
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
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Bool.h"

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

PRIMITIVEVelocityController::PRIMITIVEVelocityController()
    : command_struct_(), command_struct_EE_(), gripper_client_("franka_gripper/move", true) {}
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
  if (false) {
    cout << "Camera measurement received!\n" << endl;
    cout << "data.x=" << data.vector.x << endl;
    cout << "data.y=" << data.vector.y << endl;
    cout << "data.z=" << data.vector.z << endl;
  }
}
void PRIMITIVEVelocityController::cmdVelCallback_EE(const geometry_msgs::Vector3Stamped& data) {
  command_struct_EE_.x = data.vector.x;
  command_struct_EE_.y = data.vector.y;
  //  TODO pay attention
  command_struct_EE_.z = data.vector.z;
  //  TODO correct time stamp must be immediately after capturing data?e.g.,timestamp of the depth
  //  TODO map?
  //  command_struct_.stamp = ros::Time::now();
  command_struct_EE_.t_stamp_camera_measurement = data.header.stamp.toSec();
  command_EE_.writeFromNonRT(command_struct_EE_);
  received_measurement_EE = true;
  cout << "Camera EE measurement received!!\n" << endl;
  cout << "data_EE.x=" << data.vector.x << endl;
  cout << "data_EE.y=" << data.vector.y << endl;
  cout << "data_EE.z=" << data.vector.z << endl;
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
  //  PRIMITIVE_publisher_.init(node_handle, "PRIMITIVE_messages", 1);
  STEPPERMOTOR_publisher_.init(node_handle, "STEPPERMOTOR_messages", 1);
  publisher_r_star_.init(node_handle, "r_star_messages", 1);
  publisher_dq_SAC_.init(node_handle, "dq_SAC_messages", 1);
  publisher_dq_PID_.init(node_handle, "dq_PID_messages", 1);
  publisher_filtered_dq_.init(node_handle, "filtered_dq_messages", 1);

  sub_command_ =
      node_handle.subscribe("/p_hat_w", 100, &PRIMITIVEVelocityController::cmdVelCallback, this);
//  sub_command_EE_ = node_handle.subscribe("/p_hat_EE_w", 100,
//                                          &PRIMITIVEVelocityController::cmdVelCallback_EE, this);
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
      "traced_model_Cpp_Fep_HW_274_double.pt");
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

  // You should change here to set up your own URDF file or just pass it as an argument of this
  // example.
  const std::string urdf_filename = std::string(
      "/home/mahdi/catkin_ws/src/franka_ros/franka_description/robots/panda/"
      "panda_corrected_Nosc.urdf");
  // Load the urdf model
  pinocchio::Model model;
  pinocchio::urdf::buildModel(urdf_filename, model);
  std::cout << "model name: " << model.name << std::endl;
  // Create data required by the algorithms
  pinocchio::Data data(model);

  //  uncomment for two kinematics based experiments
  const std::string urdf_filename_biased = std::string(
      "/home/mahdi/catkin_ws/src/franka_ros/franka_description/robots/panda/"
      "panda_corrected_Nosc_biased_1.urdf");
  pinocchio::urdf::buildModel(urdf_filename_biased, model_pino_biased);

  // Sample a random configuration
  //  Eigen::VectorXd qtest = randomConfiguration(model);
  //  Eigen::VectorXd qtest = {                           -0.24160292308450512,
  //                           0.4132453289912173,
  //                           -0.24160502188821328,
  //                           -2.012426832981177,
  //                           -0.1389382115601044,
  //                           2.4397471245148825,
  //                           0.8195687449770361,
  //                           0,
  //                           0};
  //  Eigen::Vector<double, 9> qtest = {-0.24160292308450512,
  //                                    0.4132453289912173,
  //                                    -0.24160502188821328,
  //                                    -2.012426832981177,
  //                                    -0.1389382115601044,
  //                                    2.4397471245148825,
  //                                    0.8195687449770361,
  //                                    0,
  //                                    0};
  //  Eigen::Vector<double, 9> qtest = {-1.3059549114453173,
  //                                    0.7192076477077032,
  //                                    0.875579792128569,
  //                                    -1.9994787244113494,
  //                                    -0.8830029684405695,
  //                                    2.3681265268060896,
  //                                    2.4830138708386156,
  //                                    0,
  //                                    0};
  //  Eigen::Vector<double, 9> qtest = {-1.234047570759358,
  //                                    0.7457857909705199,
  //                                    0.773629118388543,
  //                                    -1.9915847135262992,
  //                                    -0.8438119640223302,
  //                                    2.4205093688146047,
  //                                    2.4531920247405683,
  //                                    0,
  //                                    0};
  // //  config_1
  //  Eigen::Vector<double, 9> qtest = {-0.9301793111064306,
  //                                    0.49450755029781285,
  //                                    0.45471354257384894,
  //                                    -2.0204698669967036,
  //                                    -0.23666417265518308,
  //                                    2.4288299029403184,
  //                                    2.0510621763268397,
  //                                    0,
  //                                    0};
  //  config_2
  //  Eigen::Vector<double, 9> qtest = {-0.9881747826233245,
  //                                    0.4708130590649354,
  //                                    0.5668641826347794,
  //                                    -2.1084923157428275,
  //                                    -0.43437336797782583,
  //                                    2.4932114290686305,
  //                                    2.2504281153844463,
  //                                    0,
  //                                    0};
  //  config_3
  //  Eigen::Vector<double, 9> qtest = {-1.0296787462318309,
  //                                    0.48096827299833256,
  //                                    0.875553047883381,
  //                                    -2.244043147040059,
  //                                    -0.6480849138365852,
  //                                    2.4866819327407415,
  //                                    2.687620257659091,
  //                                    0,
  //                                    0};
  //  config_4
  Eigen::Vector<double, 9> qtest = {-0.22683544711236076,
                                    0.4152892646837951,
                                    -0.2240776697835826,
                                    -2.029656763049754,
                                    -0.1323494169192162,
                                    2.433754967707292,
                                    1.939142517407308,
                                    0,
                                    0};
  std::cout << "qtest: " << qtest.transpose() << std::endl;
  // Perform the forward kinematics over the kinematic tree
  pinocchio::forwardKinematics(model, data, qtest);
  pinocchio::updateFramePlacements(model, data);
  // Print link names and indexes
  for (size_t i = 0; i < model.frames.size(); ++i) {
    std::cout << "Index: " << i << ", Link name: " << model.frames[i].name << std::endl;
  }

  // Print out the placement of each joint of the kinematic tree
  std::cout << "oMi\n";
  for (pinocchio::JointIndex joint_id = 0; joint_id < (pinocchio::JointIndex)model.njoints;
       ++joint_id) {
    std::cout << std::setw(24) << std::left << model.names[joint_id] << ": " << std::fixed
              << std::setprecision(6) << data.oMi[joint_id].translation().transpose() << std::endl;
  }
  // Print joint transformations including fixed links
  for (size_t frame_id = 0; frame_id < model.frames.size(); ++frame_id) {
    const auto& frame = model.frames[frame_id];
    std::cout << std::setw(24) << std::left << frame.name << ": " << std::fixed
              << std::setprecision(6) << data.oMf[frame_id].translation().transpose() << std::endl;
  }

  std::cout << "oMf\n";
  for (pinocchio::JointIndex joint_id = 0; joint_id < (pinocchio::JointIndex)model.njoints;
       ++joint_id) {
    std::cout << std::setw(24) << std::left << model.names[joint_id] << ": " << std::fixed
              << std::setprecision(6) << data.oMf[joint_id].translation().transpose() << std::endl;
  }
  std::cout << "liMi\n";
  for (pinocchio::JointIndex joint_id = 0; joint_id < (pinocchio::JointIndex)model.njoints;
       ++joint_id) {
    std::cout << std::setw(24) << std::left << model.names[joint_id] << ": " << std::fixed
              << std::setprecision(6) << data.liMi[joint_id].translation().transpose() << std::endl;
  }
  std::cout << "iMf\n";
  for (pinocchio::JointIndex joint_id = 0; joint_id < (pinocchio::JointIndex)model.njoints;
       ++joint_id) {
    std::cout << std::setw(24) << std::left << model.names[joint_id] << ": " << std::fixed
              << std::setprecision(6) << data.iMf[joint_id].translation().transpose() << std::endl;
  }

  int JOINT_ID = 5;
  pinocchio::Data::Matrix6x J(6, model.nv);
  J.setZero();
  std::cout << "after set zero J:\n" << J << std::endl;
  pinocchio::computeJointJacobian(model, data, qtest, JOINT_ID, J);
  std::cout << "URDF5 Pinnocchio Jacobian matrix J:\n" << J << std::endl;

  JOINT_ID = 6;
  J.setZero();
  pinocchio::computeJointJacobian(model, data, qtest, JOINT_ID, J);
  std::cout << "URDF6 Pinnocchio Jacobian matrix J:\n" << J << std::endl;

  JOINT_ID = 7;
  J.setZero();
  pinocchio::computeJointJacobian(model, data, qtest, JOINT_ID, J);
  std::cout << "URDF7 Pinnocchio Jacobian matrix J:\n" << J << std::endl;

  JOINT_ID = 8;
  J.setZero();
  pinocchio::computeJointJacobian(model, data, qtest, JOINT_ID, J);
  std::cout << "URDF8 Pinnocchio Jacobian matrix J:\n" << J << std::endl;

  // Get the frame ID of the last frame
  pinocchio::FrameIndex frame_id = model.frames.size() - 1;
  cout << "model.frames.size()=" << model.frames.size() << endl;
  // Resize the Jacobian matrix to fit a 6xN Jacobian (spatial Jacobian, for 6 DoF in SE(3) space)
  Eigen::MatrixXd JJ(6, model.nv);
  // Calculate the Jacobian for the frame with respect to the joint configuration qtest
  pinocchio::computeFrameJacobian(model, data, qtest, frame_id, pinocchio::ReferenceFrame::LOCAL,
                                  JJ);
  // Print the Jacobian for the frame
  std::cout << "Jacobian for frame " << model.frames[frame_id].name << ":\n" << JJ << std::endl;

  cout << "===============================================" << endl;
  for (size_t frame_id = 0; frame_id < model.frames.size(); ++frame_id) {
    const auto& frame = model.frames[frame_id];
    std::cout << "Frame Name: " << frame.name << ", Frame ID: " << frame_id << std::endl;

    // Compute Jacobian for the frame
    Eigen::MatrixXd jacobian(6, model.nv);
    pinocchio::computeFrameJacobian(model, data, qtest, frame_id, pinocchio::ReferenceFrame::WORLD,
                                    jacobian);

    // Extract the translational part
    //    Eigen::MatrixXd translational_jacobian = jacobian.topRows(3);
    std::cout << "Jacobian for " << frame.name << ":\n" << jacobian << std::endl;
  }
  cout << "===============================================" << endl;

  cout << "////////////////////////////////////////////////" << endl;
  for (size_t frame_id = 0; frame_id < model.frames.size(); ++frame_id) {
    const auto& frame = model.frames[frame_id];
    std::cout << "Frame Name: " << frame.name << ", Frame ID: " << frame_id << std::endl;

    // Compute Jacobian for the frame
    Eigen::MatrixXd jacobian(6, model.nv);
    pinocchio::computeFrameJacobian(model, data, qtest, frame_id, pinocchio::ReferenceFrame::LOCAL,
                                    jacobian);

    // Extract the translational part
    //    Eigen::MatrixXd translational_jacobian = jacobian.topRows(3);
    std::cout << "Jacobian for " << frame.name << ":\n" << jacobian << std::endl;
  }
  cout << "////////////////////////////////////////////////" << endl;

  for (size_t frame_id = 0; frame_id < model.frames.size(); ++frame_id) {
    const auto& frame = model.frames[frame_id];
    std::cout << "Frame Name: " << frame.name << ", Frame ID: " << frame_id << std::endl;

    // Compute Jacobian for the frame
    Eigen::MatrixXd jacobian(6, model.nv);
    pinocchio::computeFrameJacobian(model, data, qtest, frame_id,
                                    pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian);

    // Extract the translational part
    //    Eigen::MatrixXd translational_jacobian = jacobian.topRows(3);
    std::cout << "Jacobian for " << frame.name << ":\n" << jacobian << std::endl;
  }

  cout << "************************************************" << endl;
  const auto& frame = model.frames[26];
  // Compute Jacobian for the frame
  Eigen::MatrixXd jacobian(6, model.nv);
  pinocchio::computeFrameJacobian(model, data, qtest, 26,
                                  pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian);
  //  Eigen::MatrixXd translational_jacobian = jacobian.topRows(3);
  Eigen::MatrixXd translational_jacobian = jacobian.block(0, 0, 3, 7);
  std::cout << "URDF Pinocchio translational_jacobian (3x7)=\n"
            << translational_jacobian << std::endl;
  cout << "************************************************" << endl;

  gripper_command_publisher_ = node_handle.advertise<std_msgs::Bool>("/gripper_command", 1);
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
    v_star_dir[i] = (r_star_tf_start_up(i) - r_star_0(i));
    accum += v_star_dir[i] * v_star_dir[i];
  }
  norm_v_star_dir = sqrt(accum);

  // Perform warm-up passes
  for (int i = 0; i < 10; ++i) {
    torch::jit::IValue warmup_output = actor.forward(observations);
    warmup_output.toTuple()->elements()[0].toTensor();
  }
}

void PRIMITIVEVelocityController::openGripper() {
  franka_gripper::MoveGoal goal;
  goal.width = 0.08;  // open width in meters (max opening)
  goal.speed = 0.1;   // opening speed

  ROS_INFO("Sending open command to gripper...");
  gripper_client_.sendGoal(goal);
//  bool finished_before_timeout = gripper_client_.waitForResult(ros::Duration(5.0));
//  if (finished_before_timeout) {
//    ROS_INFO("Gripper opened successfully.");
//  } else {
//    ROS_WARN("Gripper open command timed out.");
//  }
}

void PRIMITIVEVelocityController::update(const ros::Time& rosTime, const ros::Duration& period) {
  //    camera target measurement subscription
  try {
    Commands curr_cmd = *(command_.readFromRT());
    //      TODO Pay attention: here we correct the camere raw measurements offsets
    // ATTENTION: based on primitive 50 camera estimation of upper edge corner of April tag:
    // offset is {-0.06, +2.99, -0.12};
    p_hat_w(0) = (curr_cmd.x + 20.5 - 0.06) / 1000;
    p_hat_w(1) = (curr_cmd.y + 25 + 2.99) / 1000;
    p_hat_w(2) = (curr_cmd.z + 39 - 0.12) / 1000;
    // TODO
    double t_measurement = curr_cmd.t_stamp_camera_measurement;
    dt = (t_measurement - t_0) * 1000;  //[ms]
    t_0 = t_measurement;
  } catch (int N) {
    std::cout << "ERROR: CANNOT hear p_hat_w!" << "\n";
  }
//  //    camera end effector measurement subscription
//  try {
//    Commands curr_cmd_EE = *(command_EE_.readFromRT());
//    //      TODO Pay attention: here we correct the camere raw measurements offsets
//    // ATTENTION: based on primitive 50 camera estimation of upper edge corner of April tag:
//    // offset is {-0.06, +2.99, -0.12};
//    p_hat_EE_w(0) = (curr_cmd_EE.x - 0.06) / 1000;
//    p_hat_EE_w(1) = (curr_cmd_EE.y + 2.99) / 1000;
//    p_hat_EE_w(2) = (curr_cmd_EE.z - 0.12) / 1000;
//    // TODO
//    double t_measurement_EE = curr_cmd_EE.t_stamp_camera_measurement;
//    dt_EE = (t_measurement_EE - t_0_EE) * 1000;  //[ms]
//    t_0_EE = t_measurement_EE;
//  } catch (int N) {
//    std::cout << "ERROR: CANNOT hear p_hat_EE_w!" << "\n";
//  }

  double dt_fast = 0.001 * (1000 / freq_fast);  // [s]
  //  //    TODO check joints_pose_ updates and i.c. is correct
  //  for (size_t i = 0; i < 7; ++i) {
  //    joints_pose_[i] = velocity_joint_handles_[i].getPosition();
  //  }
  franka::RobotState robot_state = state_handle_->getRobotState();
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_J(robot_state.tau_J.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));

  //  comment when you use observer
  Eigen::Vector3d EEposition(transform.translation());

  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  std::vector<int> ind_translational_jacobian{0, 1, 2};
  std::vector<int> ind_dof{0, 1, 2, 3, 4, 5, 6};
  Eigen::Matrix<double, 3, 7> J_translation = jacobian(ind_translational_jacobian, ind_dof);

  //  //  uncomment for observer 1 of true EE position based on sparse camera measurements
  //  EEposition_kinematics = transform.translation();
  //  if (received_measurement_EE == true and dt_EE > 0) {
  //    e_mismatch_1 = e_mismatch_1 + K_mismatch_1 * (p_hat_EE_w - EEposition);
  //    received_measurement_EE = false;
  //  }
  //  EEposition = EEposition_kinematics + e_mismatch_1;

  //  //  uncomment for observer 2 of true EE position based on sparse camera measurements
  //  delta_EEposition_kinematics = J_translation * dq * dt_fast;
  //  if (k == 0) {
  //    EEposition_kinematics = transform.translation();
  //    EEposition = EEposition_kinematics;
  //  }
  //  if (received_measurement_EE == true and dt_EE > 0) {
  //    e_mismatch_2 = K_mismatch_2 * (p_hat_EE_w - EEposition);
  //    received_measurement_EE = false;
  //  } else {
  //    e_mismatch_2 = {0, 0, 0};
  //  }
  //  EEposition = EEposition + e_mismatch_2 + delta_EEposition_kinematics;

  if (start_up == true) {
    //     TODO smooth start_up speed profile
    double v_star_dir_length =
        34.9028 / (1 + std::exp(-0.04 * (k_startup_speed_profile - 250))) / 1000 -
        34.9028 / (1 + std::exp(-0.04 * (0 - 250))) / 1000;
    for (int i = 0; i < 3; ++i) {
      v_star[i] = v_star_dir[i] / norm_v_star_dir * v_star_dir_length;
      r_star(i) = dt_fast * v_star[i] + r_star(i);
    }
  } else if (start_up == false) {
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
        if (false) {
          cout << "&&&&&&&&&&&&&&&&&&&&&&&&" << estimatesApriori << endl;
          cout << "estimatesApriori=" << estimatesApriori << endl;
          cout << "gainMatrices=" << gainMatrices << endl;
          cout << "p_hat_w=" << p_hat_w << endl;
          cout << "C=" << C << endl;
          cout << "estimatesApriori=" << estimatesApriori << endl;
          cout << "estimatesAposteriori=" << estimatesAposteriori << endl;
        }
        Eigen::MatrixXd In;
        In = Eigen::MatrixXd::Identity(3, 3);
        Eigen::MatrixXd IminusKC;
        IminusKC.resize(3, 3);
        IminusKC = In - gainMatrices * C;  // I-KC
        covarianceAposteriori = IminusKC * covarianceApriori * (IminusKC.transpose()) +
                                gainMatrices * R * (gainMatrices.transpose());
        X_prediction_ahead = estimatesAposteriori;
        received_measurement = false;
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
      }
    } else {
      if (MODEL_0) {
        B(1) = 1 * (1000 / freq_fast);  //[ms]
        X_prediction_ahead = A * X_prediction_ahead + B * u;
      }
      if (MODEL_1) {
        A(0, 3) = 1 * (1000 / freq_fast);  //[ms]
        A(1, 4) = 1 * (1000 / freq_fast);  //[ms]
        A(2, 5) = 1 * (1000 / freq_fast);  //[ms]
        X_prediction_ahead = A * X_prediction_ahead + B * u;
      }
      if (MODEL_2) {
        std::random_device rd{};
        std::mt19937 gen{rd()};
        std::normal_distribution<double> gauss_dist{u_mean, u_std};
        u(0, 0) = gauss_dist(gen);
        //          cout << "u(0, 0)=" << u(0, 0) << endl;
        B(1) = 1 * (1000 / freq_fast);  //[ms] //TODO ATTENTION
        X_prediction_ahead = A * X_prediction_ahead + B * u;
      }
    }

    if (MODEL_0) {
      v_star[0] = 0;
      // ATTENTION to dimension
      v_star[1] = 0.0341;  //[m/s]
      v_star[2] = 0;
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

  if (false) {
    if (k % 100 == 0) {
      cout << "************************************************" << endl;
      std::cout << "model_pino_biased name: " << model_pino_biased.name << std::endl;
      // Create data required by the algorithms
      pinocchio::Data data_pino(model_pino_biased);
      const auto& frame = model_pino_biased.frames[26];
      // Compute Jacobian for the frame
      Eigen::MatrixXd jacobian_tmp(6, model_pino_biased.nv);
      Eigen::Matrix<double, 9, 1> q_extended;
      // Copy the original 7 elements
      q_extended.head<7>() = q;
      // Add two zero rows at the end
      q_extended.tail<2>().setZero();
      pinocchio::computeFrameJacobian(model_pino_biased, data_pino, q_extended, 26,
                                      pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian_tmp);
      //  Eigen::MatrixXd J_translation_biased = jacobian.topRows(3);
      Eigen::MatrixXd J_translation_biased = jacobian_tmp.block(0, 0, 3, 7);
      std::cout << "URDF Pinocchio J_translation_biased (3x7)=\n"
                << J_translation_biased << std::endl;
      cout << "************************************************" << endl;
    }
  }

  if (start_up == true) {
    e_t[0] = (-r_star(0) + EEposition(0));
    e_t[1] = (-r_star(1) + EEposition(1));
    e_t[2] = (-r_star(2) + EEposition(2));
    Eigen::Vector<double, 3> vc;
    for (int i = 0; i < 3; ++i) {
      // ATTENTION to dimenstion
      I_e[i] +=
          -e_t[i] * dt_fast;  // in [m/s] because jacobian is in m to rad and dq are in rad/sec
      vc(i) = v_star[i] + K_p * (-e_t[i]) +
              K_i * I_e[i];  //+ K_i * np.sum(e[:,1:],1)*dt_fast + K_d*(v_ref-v_e)
    }
    Eigen::MatrixXd J_translation_pinv;
    pseudoInverse(J_translation, J_translation_pinv);

    dq_command_PID = J_translation_pinv * vc;
  } else if (start_up == false) {
    if (k_PID % (1000 / freq_PID) == 0) {
      e_t[0] = (-r_star(0) + EEposition(0));
      e_t[1] = (-r_star(1) + EEposition(1));
      e_t[2] = (-r_star(2) + EEposition(2));
      Eigen::Vector<double, 3> vc;
      for (int i = 0; i < 3; ++i) {
        // ATTENTION to dimenstion
        I_e[i] +=
            -e_t[i] / freq_PID;  // in [m/s] because jacobian is in m to rad and dq are in rad/sec
        vc(i) = v_star[i] + K_p * (-e_t[i]) +
                K_i * I_e[i];  //+ K_i * np.sum(e[:,1:],1)/ freq_PID + K_d*(v_ref-v_e)
      }

      //      //      comment when two kinematics based experiments
      //      Eigen::MatrixXd J_translation_pinv;
      //      pseudoInverse(J_translation, J_translation_pinv);
      //      dq_command_PID = J_translation_pinv * vc;

      //  uncomment for fematics based experiments and when not use observer
      Eigen::MatrixXd J_translation_pinv_biased;
      pinocchio::Data data_pino(model_pino_biased);
      const auto& frame = model_pino_biased.frames[26];
      // Compute Jacobian for the frame
      Eigen::MatrixXd jacobian_tmp(6, model_pino_biased.nv);
      Eigen::Matrix<double, 9, 1> q_extended;
      // Copy the original 7 elements
      q_extended.head<7>() = q;
      // Add two zero rows at the end
      q_extended.tail<2>().setZero();
      pinocchio::computeFrameJacobian(model_pino_biased, data_pino, q_extended, 26,
                                      pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian_tmp);
      Eigen::MatrixXd J_translation_biased = jacobian_tmp.block(0, 0, 3, 7);
      pseudoInverse(J_translation_biased, J_translation_pinv_biased);
      dq_command_PID = J_translation_pinv_biased * vc;

      //      if (false) {
      //        if (rate_trigger_() && PRIMITIVE_publisher_.trylock()) {
      //          for (size_t i = 0; i < 42; ++i) {
      //            PRIMITIVE_publisher_.msg_.jacobian_array[i] = jacobian_array[i];
      //          }
      //          PRIMITIVE_publisher_.unlockAndPublish();
      //        }
      //        if (rate_trigger_() && PRIMITIVE_publisher_.trylock()) {
      //          for (size_t i = 0; i < 6; ++i) {
      //            for (size_t j = 0; i < 7; ++j) {
      //              PRIMITIVE_publisher_.msg_.jacobian[i] = jacobian(i, j);
      //            }
      //          }
      //          PRIMITIVE_publisher_.unlockAndPublish();
      //        }
      //        //        if (rate_trigger_() && PRIMITIVE_publisher_.trylock()) {
      //        //          for (size_t i = 0; i < 3; ++i) {
      //        //            for (size_t j = 0; j < 7; ++j) {
      //        //              PRIMITIVE_publisher_.msg_.J_translation[i * 3 + j] =
      //        J_translation(i, j);
      //        //              PRIMITIVE_publisher_.msg_.J_translation_pinv[i * 3 + j] =
      //        //                  J_translation_pinv_biased(i, j);
      //        //            }
      //        //          }
      //        //          PRIMITIVE_publisher_.unlockAndPublish();
      //        //        }
      //      }
    }
    k_PID += 1;
  }

  if (k > 2000 && k < 2500) {
    openGripper();  // Replace this with your actual gripper-opening function
    gripper_opened = true;  // To avoid repeating the command
  }
  //  TODO should k_startup_speed_profile be updated here or end of call?
  k_startup_speed_profile += 1;  // k_startup_speed_profile for the start_up phase speed profile

  if (start_up == true) {
    e_EE_target[0] = (r_star_tf_start_up(0) - EEposition(0));
    e_EE_target[1] = (r_star_tf_start_up(1) - EEposition(1));
    e_EE_target[2] = (r_star_tf_start_up(2) - EEposition(2));
  } else if (start_up == false) {
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

  // TODO manual motor trigger delay compensation: more robust solution required
  //    if (std::abs(e_EE_target[1]) < 0.020801 and start_up == true) {
  //  TODO improve temporary solution: due to delay manually approximated corrosponding startup
  //  phase, trigger motor after k~730[ms]
  if (k > 600 and start_up == true) {
    //    TODO this is not necessarily is going to lock
    //    publish message to switch on the conveyor belt
    //    if (rate_trigger_() && STEPPERMOTOR_publisher_.trylock()) {
    if (STEPPERMOTOR_publisher_.trylock()) {
      STEPPERMOTOR_publisher_.msg_.vector.x = 1;  // send command to trigger stepper motor
      //      STEPPERMOTOR_publisher_.msg_.header.stamp = ros::Time::now();
      STEPPERMOTOR_publisher_.unlockAndPublish();
    }
    if (k = 600) {
      std::cout << "Triggered stepper motor sooner!" << endl;
    }
  }

  //  end startup phase if you reach below 1 mm distance to initial condition
  if (norm_e_EE_t < 0.001 and start_up == true) {
    if (false) {
      std::cout << "==========Start-up ended==========" << " \n";
      std::cout << "norm_e_EE_t=" << norm_e_EE_t << " \n";
      std::cout << "EEposition=\n";
      for (int i = 0; i < 3; i++) {
        std::cout << EEposition(i) << " ";
        std::cout << std::endl;
        std::cout << "k=" << k << " \n";
        std::cout << "k_startup_speed_profile=" << k_startup_speed_profile << " \n";
      }
    }
    start_up = false;
    std::cout << "Reached end of start-up phase!" << endl;
    // TODO ATTENTION: initialize KF at initial position
    X_prediction_ahead = EEposition;
    estimatesAposteriori = EEposition;

    //    //    TODO this is not necessarily is going to lock
    //    //    publish message to switch on the conveyor belt
    //    if (rate_trigger_() && STEPPERMOTOR_publisher_.trylock()) {
    //      STEPPERMOTOR_publisher_.msg_.vector.x = 1;  // send command to trigger stepper motor
    //      STEPPERMOTOR_publisher_.msg_.header.stamp = ros::Time::now();
    //      STEPPERMOTOR_publisher_.unlockAndPublish();
    //    }
    //    //    TODO artificially wait to be sure the command published for the stepper motor
    //    trigger
    //    //    TODO implement more efficient solution
    //    artificial_wait_idx += 1;
    //    if (artificial_wait_idx > 3) {  // 3 ms artificial delay
    //      start_up = false;
    //      std::cout << "Reached end of start-up phase!" << endl;
    //      // TODO ATTENTION: initialize KF at initial position
    //      X_prediction_ahead = EEposition;
    //      estimatesAposteriori = EEposition;
    //      //    TODO
    //      //  // TODO uncomment for offline demo
    //      //      r_star_tf_start_up[0] = 511 / 1000;
    //      //      r_star_tf_start_up[1] = 150 / 1000;
    //      //      r_star_tf_start_up[2] = 101 / 1000;
    //      //      r_star_tf_start_up[0] = p_hat_w(0)/1000;
    //      //      r_star_tf_start_up[1] = p_hat_w(1)/1000;
    //      //      r_star_tf_start_up[2] = p_hat_w(2)/1000;
    //      //      r_star_tf_start_up[0] = x_star(Eigen::last);
    //      //      r_star_tf_start_up[1] = y_star(Eigen::last);
    //      //      r_star_tf_start_up[2] = z_star(Eigen::last);
    //    } else {
    //      std::cout << "waiting!, artificial_wait_idx=" << artificial_wait_idx << " \n";
    //    }
    //  stop condition at end of tracking

  } else if ((norm_e_EE_t < 0.005 and start_up == false)) {
    if (false) {
      std::cout << "++++++++++++++++TARGET REACHED, STOPPING+++++++++++++++" << " \n";
      std::cout << "k_startup_speed_profile=" << k_startup_speed_profile << " \n";
      std::cout << "k=" << k << " \n";
      std::cout << "norm_e_EE_t=" << norm_e_EE_t << " \n";
      std::cout << "EEposition=\n";
    }
    std::cout << "Reached near the final position: stopping!" << endl;
    std::cout << "EEposition=" << endl;
    for (int i = 0; i < 3; i++) {
      std::cout << EEposition(i) << " ";
      std::cout << std::endl;
    }
    PRIMITIVEVelocityController::stopRequest(ros::Time::now());
  } else {
    if (k_SAC % (1000 / freq_SAC) == 0 and start_up == false) {
      // Directly access obs data pointer to modify values without reallocation
      double* obs_data = obs.data_ptr<double>();
      obs_data[0] = e_t.at(0);
      obs_data[1] = e_t.at(1);
      obs_data[2] = e_t.at(2);
      obs_data[3] = q(0);
      obs_data[4] = q(1);
      obs_data[5] = q(2);
      obs_data[6] = q(3);
      obs_data[7] = q(4);
      obs_data[8] = q(5);
      //      obs_data[9] = dq(0);
      //      obs_data[10] = dq(1);
      //      obs_data[11] = dq(2);
      //      obs_data[12] = dq(3);
      //      obs_data[13] = dq(4);
      //      obs_data[14] = dq(5);
      filtered_dq = alpha_LPF * dq + (1.0 - alpha_LPF) * filtered_dq;
      obs_data[9] = filtered_dq(0);
      obs_data[10] = filtered_dq(1);
      obs_data[11] = filtered_dq(2);
      obs_data[12] = filtered_dq(3);
      obs_data[13] = filtered_dq(4);
      obs_data[14] = filtered_dq(5);
      obs_data[15] = dq_command_PID(0);
      obs_data[16] = dq_command_PID(1);
      obs_data[17] = dq_command_PID(2);
      obs_data[18] = dq_command_PID(3);
      obs_data[19] = dq_command_PID(4);
      obs_data[20] = dq_command_PID(5);
      //      obs_data[15] = tau_J(0);
      //      obs_data[16] = tau_J(1);
      //      obs_data[17] = tau_J(2);
      //      obs_data[18] = tau_J(3);
      //      obs_data[19] = tau_J(4);
      //      obs_data[20] = tau_J(5);
      //      obs_data[21] = dq_command_PID(0);
      //      obs_data[22] = dq_command_PID(1);
      //      obs_data[23] = dq_command_PID(2);
      //      obs_data[24] = dq_command_PID(3);
      //      obs_data[25] = dq_command_PID(4);
      //      obs_data[26] = dq_command_PID(5);

      // Run the model's forward pass without re-pushing to observations
      torch::jit::IValue output = actor.forward(observations);

      // Extract tensor output, check properties outside loop if possible
      auto output_tuple = output.toTuple();
      torch::Tensor output_tensor = output_tuple->elements()[0].toTensor();
      assert(output_tensor.sizes() == torch::IntArrayRef({1, 6}) &&
             output_tensor.dtype() == torch::kDouble);
      assert(output_tensor.is_contiguous());

      //      // Map output tensor data directly to dq_SAC without copying
      //      Eigen::Map<Eigen::Matrix<double, 1, 6>>
      //      dq_SAC_map(output_tensor.data_ptr<double>()); dq_SAC = dq_SAC_map;  // Copy mapped
      //      data to dq_SAC
      Eigen::Map<Eigen::Matrix<double, 1, 6>>(output_tensor.data_ptr<double>()).swap(dq_SAC);
      if (false) {
        std::cout << "++++++++++++++++++++++\n";
        std::cout << "dq_SAC updated!!!\n";
        std::cout << "k=" << k << "\n";
        std::cout << "k_SAC=" << k_SAC << "\n";
        std::cout << "dq_SAC=" << dq_SAC << "\n";
        std::cout << "++++++++++++++++++++++\n";
      }
      //      }
      if (false) {
        std::cout << "!!!!!!!!!!!NEW dq_SAC=";
        for (int i = 0; i < 6; i++) {
          std::cout << dq_SAC(i) << " ";
        }
        std::cout << "\n";
      }
      //      auto end = std::chrono::high_resolution_clock::now();
      //      auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end -
      //      start).count(); std::cout << "Time spent: " << duration << " microseconds" <<
      //      std::endl; if (k > 2000) {
      //        PRIMITIVEVelocityController::stopRequest(ros::Time::now());
    }
    if (start_up == false) {
      k_SAC += 1;  // TODO ATTENTION: should be after if condition of SAC after startup phase (check
      // concept)
    }
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
      //      std::cout << "x0(0)=" << x0(0) << " \n";
      //      std::cout << "x0(1)=" << x0(1) << " \n";
      //      std::cout << "x0(2)=" << x0(2) << " \n";
      std::cout << "dt=" << dt << " \n";
      std::cout << "k=" << k << " \n";
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
      dq_command(i) = dq_command_PID(i) + dq_SAC(i);
      //      dq_command(i) = dq_command_PID(i);
      // TODO ATTENTION:  Check SAFETY LIMITS per 1 [ms]
      if (std::abs(dq_command(i) / 1000) > dq_max[i]) {
        if (true) {
          std::cout << "------------At joint i=" << i << "\n";
          std::cout << "JOINT LIMIT HIT!" << endl;
          std::cout << "dq_command(i)" << dq_command(i) << "\n";
          std::cout << "dq_SAC(i)" << dq_SAC(i) << "\n";
          std::cout << "dq_command_PID(i)" << dq_command_PID(i) << "\n";
          std::cout << "norm_e_EE_t=" << norm_e_EE_t << "\n";
          std::cout << "k_SAC=" << k_SAC << "\n";
          std::cout << "k_PID=" << k_PID << "\n";
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
  //  TODO can this publish be moved just after command? or more efficiently publish?
  //  if (rate_trigger_() && PRIMITIVE_publisher_.trylock()) {
  //  if (rate_trigger_() && PRIMITIVE_publisher_.trylock()) {
  //  if (PRIMITIVE_publisher_.trylock()) {
  //    PRIMITIVE_publisher_.msg_.header.stamp = rosTime; //ros::Time::now();
  //    //    dq_command_float = dq_command.cast<float>();
  //    for (size_t i = 0; i < 7; ++i) {
  //      // inner loop 1: k=1ms (1000 Hz)
  //      //      PRIMITIVE_publisher_.msg_.dq_command[i] = dq_command(i);
  //      //      PRIMITIVE_publisher_.msg_.EEposition[i] = EEposition(i);
  ////      PRIMITIVE_publisher_.msg_.dq_command_PID[i] = dq_command_PID(i);
  //      PRIMITIVE_publisher_.msg_.filtered_dq[i] = filtered_dq(i);
  //      if (i < 6) {
  ////        PRIMITIVE_publisher_.msg_.dq_SAC[i] = dq_SAC(i);
  //      }
  //      if (i < 3) {
  ////        PRIMITIVE_publisher_.msg_.r_star[i] = r_star(i);
  ////        PRIMITIVE_publisher_.msg_.EEposition[i] = EEposition(i);
  //        //        PRIMITIVE_publisher_.msg_.EEposition_ob2_test[i] = EEposition_ob2_test(i);
  ////        PRIMITIVE_publisher_.msg_.delta_EEposition_kinematics[i] =
  ///delta_EEposition_kinematics(i); /        PRIMITIVE_publisher_.msg_.e_mismatch_1[i] =
  ///e_mismatch_1(i); /        PRIMITIVE_publisher_.msg_.e_mismatch_2[i] = e_mismatch_2(i);
  //      }
  //    }
  //    PRIMITIVE_publisher_.unlockAndPublish();
  //  }
  if (publisher_r_star_.trylock()) {
    publisher_r_star_.msg_.vector.x = r_star(0);
    publisher_r_star_.msg_.vector.y = r_star(1);
    publisher_r_star_.msg_.vector.z = r_star(2);
    publisher_r_star_.unlockAndPublish();
  }
  if (publisher_dq_SAC_.trylock()) {
    publisher_dq_SAC_.msg_.pose.position.x = dq_SAC(0);
    publisher_dq_SAC_.msg_.pose.position.y = dq_SAC(1);
    publisher_dq_SAC_.msg_.pose.position.z = dq_SAC(2);
    publisher_dq_SAC_.msg_.pose.orientation.x = dq_SAC(3);
    publisher_dq_SAC_.msg_.pose.orientation.y = dq_SAC(4);
    publisher_dq_SAC_.msg_.pose.orientation.z = dq_SAC(5);
    publisher_dq_SAC_.unlockAndPublish();
  }
  if (publisher_dq_PID_.trylock()) {
    publisher_dq_PID_.msg_.pose.position.x = dq_command_PID(0);
    publisher_dq_PID_.msg_.pose.position.y = dq_command_PID(1);
    publisher_dq_PID_.msg_.pose.position.z = dq_command_PID(2);
    publisher_dq_PID_.msg_.pose.orientation.x = dq_command_PID(3);
    publisher_dq_PID_.msg_.pose.orientation.y = dq_command_PID(4);
    publisher_dq_PID_.msg_.pose.orientation.z = dq_command_PID(5);
    publisher_dq_PID_.unlockAndPublish();
  }

  if (publisher_filtered_dq_.trylock()) {
    publisher_filtered_dq_.msg_.pose.position.x = filtered_dq(0);
    publisher_filtered_dq_.msg_.pose.position.y = filtered_dq(1);
    publisher_filtered_dq_.msg_.pose.position.z = filtered_dq(2);
    publisher_filtered_dq_.msg_.pose.orientation.x = filtered_dq(3);
    publisher_filtered_dq_.msg_.pose.orientation.y = filtered_dq(4);
    publisher_filtered_dq_.msg_.pose.orientation.z = filtered_dq(5);
    publisher_filtered_dq_.msg_.pose.orientation.w = filtered_dq(6);
    publisher_filtered_dq_.unlockAndPublish();
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
