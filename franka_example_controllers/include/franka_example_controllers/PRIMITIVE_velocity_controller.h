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
#include <realtime_tools/realtime_buffer.h>
#include "geometry_msgs/Vector3.h"


namespace franka_example_controllers {

    class PRIMITIVEVelocityController : public controller_interface::MultiInterfaceController<
            franka_hw::FrankaModelInterface,
            hardware_interface::VelocityJointInterface,
            franka_hw::FrankaStateInterface> {
    public:
        PRIMITIVEVelocityController();

        bool init(hardware_interface::RobotHW *robot_hardware, ros::NodeHandle &node_handle) override;

        void starting(const ros::Time &) override;

        void update(const ros::Time &, const ros::Duration &period) override;

        void stopping(const ros::Time &) override;

        void stopRequest(const ros::Time &time);

    private:
        hardware_interface::VelocityJointInterface *velocity_joint_interface_;
        std::vector <hardware_interface::JointHandle> velocity_joint_handles_;
        std::unique_ptr <franka_hw::FrankaModelHandle> model_handle_;
        ros::Duration elapsed_time_;
        ros::Time t_init;
        std::array<double, 7> initial_pose_{};
        std::array<double, 7> joints_pose_{};
        std::array<double, 7> joints_vel_{};
        std::array<double, 16> initial_O_T_EE_{};
        std::unique_ptr <franka_hw::FrankaStateHandle> state_handle_;  // seems to be franka_states
        int idx_i1 = 0;
        int idx_i2 = 1;
        std::array<double, 3> I_e = {0, 0, 0};
        franka_hw::TriggerRate rate_trigger_{1000.0};
        realtime_tools::RealtimePublisher <PRIMITIVEmessages> PRIMITIVE_publisher_;

        struct Commands
        {
            double x;
            double y;
            double z;
            ros::Time stamp;
            Commands() : x(55.0),y(66.0),z(77.0), stamp(0.0) {}
        };
        realtime_tools::RealtimeBuffer<Commands> command_;
        Commands command_struct_;
        ros::Subscriber sub_command_;
        bool allow_multiple_cmd_vel_publishers_;

        const bool debug = false;
        static const int Target_Traj_ROWS = 6381;
        static const int Target_Traj_COLUMNS = 3;
        float r_star[Target_Traj_ROWS][Target_Traj_COLUMNS];
        float v_star[Target_Traj_ROWS][Target_Traj_COLUMNS];
        float q_star[Target_Traj_ROWS][9];
//        std::array<double, 7> q_start{
//                {-0.76543793, -0.08999656, -0.19902707, -2.04154379, -0.12972969, 2.73708789, 2.73708976}};
        std::array<double, 7> q_start{
                {-0.00155713, -0.7739, -0.00012461, -2.38384, -0.00188551, 1.56271, 0.806219}};
//  std::array<double, 7> dq_command{{0,0,0,0,0,0,0}};
        Eigen::Matrix<double, 7, 1> dq_command = {0, 0, 0, 0, 0, 0, 0};
        std::array<double, 3> r_star_0 = {0.307926, -0.000730912, 0.573038};
//        std::array<double, 3> r_star_tf = {0.476529, -0.253709, 0.193911};
        std::array<double, 3> r_star_tf = {0.465203,        -0.237464,        0.168568};

        double v_star_2[3];
        std::array<double, 3> r_star_2 = r_star_0;
        std::array<double, 3> e_t = {0, 0, 0};
        std::array<double, 3> e_EE_target = {0, 0, 0};
        std::array<double, 7> dq_max = {0.006981317008, 0.003490658504, 0.003490658504, 0.005235987756, 0.006981317008, 0.00872664626, 0.00872664626}; //dq_c [rad/1ms]

        void cmdVelCallback(const geometry_msgs::Vector3& command);

    };

}  // namespace franka_example_controllers
