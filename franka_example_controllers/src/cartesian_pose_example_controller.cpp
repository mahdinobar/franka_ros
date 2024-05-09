// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/cartesian_pose_example_controller.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <controller_interface/controller_base.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <vector>

namespace franka_example_controllers {

    bool CartesianPoseExampleController::init(hardware_interface::RobotHW *robot_hardware,
                                              ros::NodeHandle &node_handle) {
        cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();
        if (cartesian_pose_interface_ == nullptr) {
            ROS_ERROR(
                    "CartesianPoseExampleController: Could not get Cartesian Pose "
                    "interface from hardware");
            return false;
        }

        std::string arm_id;
        if (!node_handle.getParam("arm_id", arm_id)) {
            ROS_ERROR("CartesianPoseExampleController: Could not get parameter arm_id");
            return false;
        }

        try {
            cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
                    cartesian_pose_interface_->getHandle(arm_id + "_robot"));
        } catch (const hardware_interface::HardwareInterfaceException &e) {
            ROS_ERROR_STREAM(
                    "CartesianPoseExampleController: Exception getting Cartesian handle: " << e.what());
            return false;
        }

        auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
        if (state_interface == nullptr) {
            ROS_ERROR("CartesianPoseExampleController: Could not get state interface from hardware");
            return false;
        }

        try {
            auto state_handle = state_interface->getHandle(arm_id + "_robot");

//            std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
            std::array<double, 7> q_start{
                    {-0.42529795, 0.11298615, 0.20446317, -2.52843438, -0.15231932, 2.63230466, M_PI_4}};
            for (size_t i = 0; i < q_start.size(); i++) {
                if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 1) {
                    ROS_ERROR_STREAM(
                            "CartesianPoseExampleController: Robot is not in the expected starting position for "
                            "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
                            "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
                    return false;
                }
            }
        } catch (const hardware_interface::HardwareInterfaceException &e) {
            ROS_ERROR_STREAM(
                    "CartesianPoseExampleController: Exception getting state handle: " << e.what());
            return false;
        }

        return true;
    }

    void CartesianPoseExampleController::starting(const ros::Time & /* time */) {
        initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
        std::cout << "+++++++++++initial_pose_[12]=" << initial_pose_[12] << " \n";
        interp_x = linspace(initial_pose_[12], p_ee_target[0], 50000);
//        interp_y = linspace(initial_pose_[13], p_ee_target[1], 10000);
//        interp_z = linspace(initial_pose_[14], p_ee_target[2], 10000);
        elapsed_time_ = ros::Duration(0.0);
    }

    void CartesianPoseExampleController::update(const ros::Time & /* time */,
                                                const ros::Duration &period) {
        elapsed_time_ += period;

        double radius = 0.15;
        double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec()));
        double delta_x = radius * std::sin(angle);
        double delta_z = radius * (std::cos(angle) - 1);
        std::array<double, 16> new_pose = initial_pose_;
        int dk = 100;
        if (step_k % dk == 0) {
            std::cout << "---interp_x[step_k]=" << interp_x[step_k] << " \n";
        }
        new_pose[12] = interp_x[step_k];
//        new_pose[12] -= delta_x;
//        new_pose[14] -= delta_z;
//        new_pose[12] = 0.45039319;
//        int dk = 100;
//        if (step_k%dk==0 && step_k/dk<1000) {
//            std::cout << "§§§delta_x=" << delta_x << " \n";
//            std::cout << "§§§step_k=" << step_k << " \n";
//            new_pose[12] = interp_x[step_k/dk];
////            new_pose[13] = interp_y[step_k/dk];
////            new_pose[14] = interp_z[step_k/dk];
//        } else {
//            new_pose[12] = 0.45039319;
//            new_pose[13] = -0.09860044;
//            new_pose[14] = 0.17521834;
//        }
        cartesian_pose_handle_->setCommand(new_pose);
        step_k += 1;
    }

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianPoseExampleController,
        controller_interface::ControllerBase
)
