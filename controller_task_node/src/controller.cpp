//  Copyright 2023 LeoDrive A.Ş. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include "controller_task_node/controller.hpp"

#include <algorithm>

namespace controller
{


  Controller::Controller(const rclcpp::NodeOptions &options)
      : Node("controller", options)
  {
    pub_cmd_ = create_publisher<AckermannControlCommand>("/control/command/control_cmd", 1);
    pub_trajectory_ = create_publisher<Trajectory>("/control/trajectory", 1);;
    pub_lateral_deviation_ = create_publisher<std_msgs::msg::Float64>("/control/lateral_deviation", 1);;
    pub_longitudinal_velocity_error_ = create_publisher<std_msgs::msg::Float64>("/control/longitudinal_velocity_error", 1);;
    pub_gear_cmd_ = create_publisher<autoware_auto_vehicle_msgs::msg::GearCommand>("/control/command/gear_cmd", 1);;
    sub_kinematics_ = create_subscription<Odometry>(
        "/localization/kinematic_state", 1, [this](const Odometry::SharedPtr msg)
        { odometry_ = msg; });

    using namespace std::literals::chrono_literals;
    timer_ = rclcpp::create_timer(
        this, get_clock(), 30ms, std::bind(&Controller::onTimer, this));
  }

  void Controller::onTimer()
  {
    // To make vehicle able to drive, we need to publish the gear command to drive

    autoware_auto_vehicle_msgs::msg::GearCommand gear_cmd;
    gear_cmd.command = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE;
    gear_cmd.stamp = this->now();
    pub_gear_cmd_->publish(gear_cmd);

    // Calculate the control command and error values.
  }

  double Controller::calcSteerCmd()
  {
    double steer = 0.0;
    // Calculate the steering angle here.
    return steer;
  }

  double Controller::calcAccCmd()
  {
    double acc = 0.0;
    // Calculate the acceleration here.
    return acc;
  }
  double calcLateralDeviation()
  {
    double lateral_deviation = 0.0;
    // Calculate the lateral deviation here.
    return lateral_deviation;
  }

} // namespace controller

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(controller::Controller)
