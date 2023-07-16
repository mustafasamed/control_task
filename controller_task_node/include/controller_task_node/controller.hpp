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

#ifndef CONTROLLER_TASK_NODE__CONTROLLER_HPP_
#define CONTROLLER_TASK_NODE__CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>

#include <memory>

namespace controller
{
  using autoware_auto_control_msgs::msg::AckermannControlCommand;
  using autoware_auto_planning_msgs::msg::Trajectory;
  using autoware_auto_planning_msgs::msg::TrajectoryPoint;
  using geometry_msgs::msg::Pose;
  using geometry_msgs::msg::Twist;
  using nav_msgs::msg::Odometry;

  class Controller : public rclcpp::Node // this line defined "Controller" class as a subclass of rclcpp::Node class.
  {
  public:
    explicit Controller(const rclcpp::NodeOptions &options); // this is a constructor that takes parameter of type "rclcpp::NodeOptions" and 
    //!! initializes a "Node" object with the name "controller" using the provided options. !! (This can be seen from the cpp file.)
    // "explicit" means there is no implicit conversion in the class.(in "void controller_function(10)", 10 won't be used in any int parameter of any function inside of the Controller class)
    ~Controller() = default; // This is a destructor. A destructor is a member function that is invoked automatically when the object goes out of scope or is explicitly destroyed by a call to delete.
    // In our case, it is automatically called.(at program termination, out of scope, dynamic memory deallocation, container destruction)
    

  private: // these msgs are not accessible from outside but can be accessed only with member functions or friend functions.
    rclcpp::Subscription<Odometry>::SharedPtr sub_kinematics_;
    rclcpp::Publisher<Trajectory>::SharedPtr pub_trajectory_;
    rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_cmd_;
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr pub_gear_cmd_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_lateral_deviation_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_longitudinal_velocity_error_;
    // change begin 

    // This part is to add TrajectoryPoints message in the controller class. 
    std::vector<TrajectoryPoint> trajectoryPoints_; // This defined message stores the waypoint values.
    
    //visualization rate
    int visualization_rate_;           // this can be private since onTimer() will use this only (member function)

    // these two variables will be stored after findClosestTrajectoryPoint() function so that a function contains findClosestTrajectoryPoint() will have these
    double min_distance_;
    int start_index_;

    // For Lateral Control
    double lateral_deviation_;         // Member variable to store lateral deviation. This is stored from calcLateralDeviation() to be used in calcSteerCmd()
    double previous_lateral_error_;    // prev. lateral error and integral lateral error were added into controller class variable just to...
    double integral_of_lateral_error_; // be calculated in calculateSteeringAngle() function.
    
    // For Longitudinal Control
    double previous_velocity_error_;
    double integral_of_velocity_error_;
    double velocity_;                   // This one is also used in lateral control to divide PID into segments with respect to velocity.
    // change end //

    rclcpp::TimerBase::SharedPtr timer_;
    Odometry::SharedPtr odometry_;

    void onTimer();
    double calcSteerCmd();
    double calcAccCmd();
    double calcLateralDeviation();
    void findClosestTrajectoryPoint();
  };

 }// namespace controller

#endif // CONTROLLER_TASK_NODE__CONTROLLER_HPP_
