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

#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <string>
#include "/usr/local/MATLAB/R2023b/extern/include/mat.h"

namespace franka_example_controllers {

    bool PRIMITIVEVelocityController::init(hardware_interface::RobotHW *robot_hardware,
                                           ros::NodeHandle &node_handle) {
        velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
        if (velocity_joint_interface_ == nullptr) {
            ROS_ERROR("PRIMITIVEVelocityController: Error getting position joint interface from hardware!");
            return false;
        }
        std::vector <std::string> joint_names;
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
            } catch (const hardware_interface::HardwareInterfaceException &e) {
                ROS_ERROR_STREAM(
                        "PRIMITIVEVelocityController: Exception getting joint handles: " << e.what());
                return false;
            }
        }

        for (size_t i = 0; i < q_start.size(); i++) {
            if (std::abs(velocity_joint_handles_[i].getPosition() - q_start[i]) > 0.1) {
                ROS_ERROR_STREAM(
                        "PRIMITIVEVelocityController: Robot is not in the expected starting position for "
                        "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
                        "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
                return false;
            }
        }
        auto *model_interface = robot_hardware->get<franka_hw::FrankaModelInterface>();
        if (model_interface == nullptr) {
            ROS_ERROR_STREAM(
                    "CartesianImpedanceExampleController: Error getting model interface from hardware");
            return false;
        }
        try {
            model_handle_ =
                    std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle("panda_model"));
        } catch (hardware_interface::HardwareInterfaceException &ex) {
            ROS_ERROR_STREAM("PRIMITIVEVelocityController: Exception getting model handle from interface: "
                                     << ex.what());
            return false;
        }
        auto *state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
        try {
            state_handle_ =
                    std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle("panda_robot"));
        } catch (hardware_interface::HardwareInterfaceException &ex) {
            ROS_ERROR_STREAM("PRIMITIVEVelocityController: Exception getting state handle from interface: "
                                     << ex.what());
            return false;
        }
        PRIMITIVE_publisher_.init(node_handle, "PRIMITIVE_messages", 1);
        return true;
    }

    void PRIMITIVEVelocityController::starting(const ros::Time & /* time */) {
        for (size_t i = 0; i < 7; ++i) {
            initial_pose_[i] = velocity_joint_handles_[i].getPosition();
        }
        std::ifstream inputfile_q_star("/home/mahdi/ETHZ/codes/rl_reach/code/logs/q_log_b_interp.txt");
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
        elapsed_time_ = ros::Duration();
        t_init = ros::Time::now();
        std::cout << "ROS system clock t_init=" << t_init << " \n";
        std::cout << std::endl;
        step_k = 0;
        std::cout << "ros::Time::isSimTime()=" << ros::Time::isSimTime() << " \n";
        std::cout << std::endl;
        std::cout << "ros::Time::isSystemTime()=" << ros::Time::isSystemTime() << " \n";
        std::cout << std::endl;

        std::ifstream inputfile_r_star(
                "/home/mahdi/ETHZ/codes/rl_reach/code/logs/currentPosition_log_b.txt");
        if (!inputfile_r_star.is_open()) {
            std::cout << "Error reading r_log_interp file" << std::endl;
        }
        std::ifstream inputfile_v_star("/home/mahdi/ETHZ/codes/rl_reach/code/logs/currentVel_log_b.txt");
        if (!inputfile_v_star.is_open()) {
            std::cout << "Error reading v_log_interp file" << std::endl;
        }
        std::cout << "r_star and v_star" << " ";
        std::cout << std::endl;

        for (int row = 0; row < Target_Traj_ROWS; ++row) {
            std::string row_text_r;
            std::string row_text_v;
            std::getline(inputfile_r_star, row_text_r);
            std::getline(inputfile_v_star, row_text_v);
            std::istringstream row_stream_r(row_text_r);
            std::istringstream row_stream_v(row_text_v);
            for (int column = 0; column < 3; ++column) {
                double number_r;
                double number_v;
                char delimiter;
                row_stream_r >> number_r >> delimiter;
                row_stream_v >> number_v >> delimiter;
                r_star[row][column] = number_r;
                v_star[row][column] = number_v;
                if (debug) {
                    std::cout << r_star[row][column] << " r";
                    std::cout << std::endl;
                    std::cout << v_star[row][column] << " v";
                    std::cout << std::endl;
                }
            }
        }
    }

    void PRIMITIVEVelocityController::update(const ros::Time &rosTime, const ros::Duration &period) {
        //////  uncomment for PRIMITIVE control
        //  //  int mp = 5000;
        //  for (size_t i = 0; i < 7; ++i) {
        //    joints_vel_[i] = velocity_joint_handles_[i].getVelocity();
        //  }
        //  elapsed_time_ += period;
        //  //  std::cout << "step_k=" << step_k << " \n";
        //  //  std::cout << std::endl;
        //  //  std::cout << "q_star[step_k][0]=" << q_star[step_k][0] << " \n";
        //  //  std::cout << std::endl;
        //  //  for (int i = 0; i < 7; ++i) {
        //  //    dq_command[i] = q_star[step_k][i];
        //  //  }
        //  const double t_B = 5.000;
        //  const double K_p = 1;
        //  const double dq_star = 5 * (3.14159265359 / 180);
        //  if (elapsed_time_.toSec() == t_B) {
        //    //    dq_command[0] = K_p*(dq_star-joints_vel_[0]);
        //    dq_command[0] = dq_star;
        //  }
        //  if (rate_trigger_() && PRIMITIVE_publisher_.trylock()) {
        //    for (size_t i = 0; i < 7; ++i) {
        //      PRIMITIVE_publisher_.msg_.dq_c[i] = dq_command[i];
        //    }
        //    PRIMITIVE_publisher_.unlockAndPublish();
        //  }
        //  franka::RobotState robot_state = state_handle_->getRobotState();
        //  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
        //  Eigen::Vector3d EEposition(transform.translation());
        //  if (rate_trigger_() && PRIMITIVE_publisher_.trylock()) {
        //    for (size_t i = 0; i < 3; ++i) {
        //      PRIMITIVE_publisher_.msg_.EEposition[i] = EEposition(i);
        //    }
        //    PRIMITIVE_publisher_.unlockAndPublish();
        //  }
        //  for (size_t i = 0; i < 7; ++i) {
        //    velocity_joint_handles_[i].setCommand(dq_command[i]);
        //  }
        //  if (debug) {
        //    std::cout << "idx_command=" << idx_command << " \n";
        //    std::cout << std::endl;
        //  }
        //  step_k += 1;
        //  //  if (step_k > Target_Traj_ROWS - 50) {
        //  //    PRIMITIVEVelocityController::stopRequest(ros::Time::now());
        //  //  }

        int mp = 1;
        double ts = 0.001 * mp;
        //    TODO check joints_pose_ updates and i.c. is correct
        for (size_t i = 0; i < 7; ++i) {
            joints_pose_[i] = velocity_joint_handles_[i].getPosition();
        }
        franka::RobotState robot_state = state_handle_->getRobotState();
        Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
        Eigen::Vector3d EEposition(transform.translation());
        if (idx_out % mp == 0) {
            elapsed_time_ += period;
            std::array<double, 42> jacobian_array =
                    model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
            Eigen::Map <Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
            std::vector<int> ind_translational_jacobian{0, 1, 2};
            std::vector<int> ind_dof{0, 1, 2, 3, 4, 5, 6};
            Eigen::Matrix<double, 3, 7> J_translation = jacobian(ind_translational_jacobian, ind_dof);
            Eigen::MatrixXd J_translation_pinv;
            franka::RobotState robot_state = state_handle_->getRobotState();
            Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
            Eigen::Vector3d EEposition(transform.translation());
            Eigen::Quaterniond orientation(transform.rotation());
            Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
            Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.q.data());
            double K_p = 40;
            double K_i = 40;
            double e_t[3];
            e_t[0] = (r_star[idx_command][0] - EEposition(0));
            e_t[1] = (r_star[idx_command][1] - EEposition(1));
            e_t[2] = (r_star[idx_command][2] - EEposition(2));
            Eigen::Vector<double, 3> vc;
            //    double K_d=1;
            for (int i = 0; i < 3; ++i) {
                I_e[i] += e_t[i] * ts;
                vc(i) = v_star[idx_command][i] + K_p * e_t[i] +
                        K_i * I_e[i];  //+ K_i * np.sum(e[:,1:],1)*ts + K_d*(v_ref-v_e)
            }
            pseudoInverse(J_translation, J_translation_pinv);
            dq_command = J_translation_pinv * vc;
            //  TODO should idx_command be updated here or end of call?
            idx_command += 1;
            if (debug) {
                std::cout << "**************************************************idx_command=" << idx_command
                          << " \n";
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
                std::cout << "@@@@@@@@@r_star[idx_command][0]=" << r_star[idx_command][0];
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
        if (rate_trigger_() && PRIMITIVE_publisher_.trylock()) {
            for (size_t i = 0; i < 3; ++i) {
                PRIMITIVE_publisher_.msg_.r_star[i] = r_star[idx_command][i];
                PRIMITIVE_publisher_.msg_.v_star[i] = v_star[idx_command][i];
                PRIMITIVE_publisher_.msg_.EEposition[i] = EEposition(i);
            }
            PRIMITIVE_publisher_.unlockAndPublish();
        }
        if (idx_command >= Target_Traj_ROWS) {
            PRIMITIVEVelocityController::stopRequest(ros::Time::now());
        } else {
            for (size_t i = 0; i < 7; ++i) {
                velocity_joint_handles_[i].setCommand(dq_command(i));
            }
        }

        idx_out += 1;
        if (rate_trigger_() && PRIMITIVE_publisher_.trylock()) {
            for (size_t i = 0; i < 7; ++i) {
                PRIMITIVE_publisher_.msg_.dq_c[i] = dq_command(i);
            }
            PRIMITIVE_publisher_.unlockAndPublish();
        }
    }

    void PRIMITIVEVelocityController::stopping(const ros::Time & /*time*/) {
        // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
        // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
        // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
    }

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::PRIMITIVEVelocityController,
        controller_interface::ControllerBase
)
