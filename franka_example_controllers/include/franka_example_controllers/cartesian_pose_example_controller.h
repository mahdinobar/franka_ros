// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <memory>
#include <string>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <franka_hw/franka_cartesian_command_interface.h>
#include <vector>


namespace franka_example_controllers {

    class CartesianPoseExampleController
            : public controller_interface::MultiInterfaceController<franka_hw::FrankaPoseCartesianInterface,
                    franka_hw::FrankaStateInterface> {
    public:
        bool init(hardware_interface::RobotHW *robot_hardware, ros::NodeHandle &node_handle) override;

        void starting(const ros::Time &) override;

        void update(const ros::Time &, const ros::Duration &period) override;

    private:
        franka_hw::FrankaPoseCartesianInterface *cartesian_pose_interface_;
        std::unique_ptr <franka_hw::FrankaCartesianPoseHandle> cartesian_pose_handle_;
        ros::Duration elapsed_time_;
        std::array<double, 16> initial_pose_{};
        std::array<double, 3> p_ee_target{{0.45039319, -0.09860044, 0.17521834}};
        int step_k = 0;
        std::vector<double> interp_x;
        std::vector<double> interp_y;
        std::vector<double> interp_z;
        std::vector<double> linspace(double min, double max, int n) {
            std::vector<double> result;
            int iterator = 0;
            for (int i = 0; i <= n - 2; i++) {
                double temp = min + i * (max - min) / (floor((double) n) - 1);
                result.insert(result.begin() + iterator, temp);
                iterator += 1;
            }
            result.insert(result.begin() + iterator, max);
            return result;
        }
    };

}  // namespace franka_example_controllers
