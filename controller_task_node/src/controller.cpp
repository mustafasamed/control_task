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

// To calculate lateral deviation
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

 #include "controller_task_node/calculateDistance.hpp"

#include "controller_task_node/controller.hpp"

#include <algorithm>

// change begin //
#include <fstream> // this is to include waypoints.txt file into controller.cpp. This makes a txt file is read or written by the controller.cpp.
// change end //

namespace controller
{

  Controller::Controller(const rclcpp::NodeOptions &options) 
      : Node("controller", options),
      visualization_rate_(50), // Initialize visualization_rate_ to 50
      previous_lateral_error_(0), // Initialize previous_lateral_error_ to 0
      integral_of_lateral_error_(0), // Initialize integral_of_lateral_error_ to 0
      previous_velocity_error_(0), // Initialize previous_lateral_error_ to 0
      integral_of_velocity_error_(0) // Initialize integral_of_lateral_error_ to 0
  {
        std::cout << "Controller constructor called!" << std::endl; // Debug output
    // change begin //
    // to define calculateDistance function and use it at OnTime()

    // Read trajectory points from the waypoints.txt file
    std::ifstream file("/home/mustafasamed/leo_ws/src/control_task/controller_task_node/config/waypoints.txt");
    if (file.is_open())
    {
        double x, y, z, qx, qy, qz, qw, velocity;
        while (file >> x >> y >> z >> qx >> qy >> qz >> qw >> velocity)
        {
            // Create a TrajectoryPoint message and add it to the trajectoryPoints_ vector
            TrajectoryPoint point;  // after "point.", pose and longitudinal_velocity_mps were added to match with the mesage structure.
            point.pose.position.x = x;
            point.pose.position.y = y;
            point.pose.position.z = z;
            point.pose.orientation.x = qx;
            point.pose.orientation.y = qy;
            point.pose.orientation.z = qz;
            point.pose.orientation.w = qw;
            point.longitudinal_velocity_mps = velocity; // this is longitudinal velocity(forward velocity).
            trajectoryPoints_.push_back(point); // this line makes trajectoryPoints_ vector to store all the waypoint values which are stored in point variable of TrajectoryPoint type.
        }
        file.close();
    }
    else
    {
        std::cout<<"waypoints.txt file cannot be read.";
    }
// change end //

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

    void Controller::findClosestTrajectoryPoint() //This function finds the closest trajectory point to the vehicle
  {
      int num_points = trajectoryPoints_.size();
      min_distance_ = std::numeric_limits<double>::max();
      start_index_ = 0;
      if (odometry_ && !trajectoryPoints_.empty())
      {
          //const auto& current_position = odometry_->pose.pose.position;
          for (int i = 0; i < num_points; ++i)
          {
              const auto& point = trajectoryPoints_[i].pose.position;
              double distance = calculateDistance(odometry_->pose.pose.position.x, odometry_->pose.pose.position.y, point.x, point.y);
              if (distance < min_distance_)
              {
                  min_distance_ = distance;
                  start_index_ = i;
              }
          }
      }
  }


  void Controller::onTimer()
  {
    // To make vehicle able to drive, we need to publish the gear command to drive

    autoware_auto_vehicle_msgs::msg::GearCommand gear_cmd;
    gear_cmd.command = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE;
    gear_cmd.stamp = this->now();
    pub_gear_cmd_->publish(gear_cmd);

    // Calculate the control command
    double steer = calcSteerCmd();
    double acc = calcAccCmd();
   // Create and populate the AckermannControlCommand message
    AckermannControlCommand control_cmd;
    control_cmd.stamp = this->now();
    control_cmd.lateral.steering_tire_angle = steer;
    control_cmd.longitudinal.acceleration = acc; 

    // Publish the control command
    pub_cmd_->publish(control_cmd);

    // Calculate lateral deviation
    findClosestTrajectoryPoint();
    lateral_deviation_ = calcLateralDeviation(); // this will be used in calculateSteeringAngle() as an input. 

     //velocity for being used in lateral control
    velocity_=trajectoryPoints_[start_index_].longitudinal_velocity_mps;

    // Publish lateral deviation error
    std_msgs::msg::Float64 lateral_error_msg;
    lateral_error_msg.data = lateral_deviation_;
    pub_lateral_deviation_->publish(lateral_error_msg);

    // Calculate longitudinal velocity error
    double velocity_error = trajectoryPoints_[start_index_].longitudinal_velocity_mps - odometry_->twist.twist.linear.x; 

    // Publish longitudinal velocity error
    std_msgs::msg::Float64 velocity_error_msg;
    velocity_error_msg.data = velocity_error;
    pub_longitudinal_velocity_error_->publish(velocity_error_msg);

    // Publish trajectory points for visualization in RViz
    Trajectory trajectory; // this is a variable of Trajectory type.
    trajectory.header.stamp = this->now(); 
    trajectory.header.frame_id = "map";

    // Calculate the start index based on the car's position
    int num_points = trajectoryPoints_.size(); // to compare with an integer "i", we also need an integer.

    // Determine the end index based on the start index and visualization rate
    int end_index = std::min(start_index_ + visualization_rate_, num_points);

    // Add the selected trajectory points to the Trajectory message
    for (int i = start_index_; i < end_index; ++i)
    {
        trajectory.points.push_back(trajectoryPoints_[i]);
    }
    pub_trajectory_->publish(trajectory);

    // Calculate the control command and error values.
  }

  double Controller::calcSteerCmd()
  {
    double steer = 0.0;
    // Calculate the steering angle here.
    // Define PID gains for lateral control(the default values were defined by Ziegler–Nichols method(no overshoot case))
    // Default values
    double Kp = 0.04;   // Proportional gain 
    double Ki = 0.0;    // Integral gain
    double Kd = 0.0132; // Derivative gain
    double dt = 0.03;   // 30ms

    if(velocity_ > 6)    //This if statement is mostly for the beginning and the end segment of the trajectory.
    {
      Kd = 4*0.0132;    // This decreases the oscillation.
    }
    else if(velocity_ > 4)// This elseif statement is mostly for the transition segment between U turn and the straight-similar trajectory.
    {
      Kd = 15*0.0132;    // This decreases oscillation 
    } 
    else{                // This else statement is for U turn.
      Kd = 32*0.0132;    // This decreases oscillation.
      Kp = 10*Kp;        // This decreases steady-state error. Otherwise, the vehicle takes U turn too wide.
    } 
    // Calculate PID terms
    double proportional_term = Kp * lateral_deviation_;                                // Propotional term
    double integral_term = Ki * integral_of_lateral_error_ * dt;                      // Accumulate integral error over time
    double derivative_term = Kd * (lateral_deviation_ - previous_lateral_error_) / dt; // Calculate rate of change of error

    // Calculate total steering command
    steer = proportional_term + integral_term + derivative_term;

    //  update integral and previous lateral error for next iteration
    integral_of_lateral_error_ += lateral_deviation_;
    previous_lateral_error_ = lateral_deviation_;

    return steer;
  }

  double Controller::calcAccCmd()
  {
    double acc = 0.0;
    // PID parameter ratio was determined by Ziegler–Nichols method(no overshoot case)
    double kp = 4; // PID parameters are constant for longitudinal control. That's why they are defined here but not in controller class.
    double ki = 0;
    double kd = 1.32; 
    // Calculate the acceleration here.
   
    double desired_velocity = trajectoryPoints_[start_index_].longitudinal_velocity_mps; // These values are from waypoints.txt file.
    double current_velocity = odometry_->twist.twist.linear.x;                           // These values are from odometry.
    double velocity_error = desired_velocity - current_velocity;                         // The velocity error.

    double dt = 0.03; // Control loop time step (30ms)

    // Calculate the proportional term
    double proportional_term = kp * velocity_error;

    // Calculate the integral term
    integral_of_velocity_error_ += velocity_error * dt;
    double integral_term = ki * integral_of_velocity_error_;

    // Calculate the derivative term
    double derivative_term = kd * (velocity_error - previous_velocity_error_) / dt;

    // Calculate the acceleration command using the PID terms
    acc = proportional_term + integral_term + derivative_term;

    // Update the previous velocity error for the next iteration
    previous_velocity_error_ = velocity_error;

  return acc;
  }
  double Controller::calcLateralDeviation() // I added this function into controller class since it is added in controller.hpp private section and I can use declared variables of controller class in this function.
  {
    double lateral_deviation = 0.0;
    // Calculate the lateral deviation here.
  // Check if the trajectoryPoints_ vector is not empty
  if (!trajectoryPoints_.empty())
  {
    // Vector from closest trajectory point to the vehicle
    const auto& vehiclePosition = odometry_->pose.pose.position;
    const auto& trajectoryPosition = trajectoryPoints_[start_index_].pose.position;

    // Calculate the vector from the trajectory point to the vehicle's position
    double vectorX = vehiclePosition.x - trajectoryPosition.x;
    double vectorY = vehiclePosition.y - trajectoryPosition.y;

    // Assuming the closest trajectory point is stored in the variable 'closestPoint'
    const auto& trajectoryOrientation = trajectoryPoints_[start_index_].pose.orientation;

    // Convert the orientation to a vector representation
    tf2::Quaternion q(
        trajectoryOrientation.x,
        trajectoryOrientation.y,
        trajectoryOrientation.z,
        trajectoryOrientation.w);
    tf2::Matrix3x3 m(q);
    tf2Scalar roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

     // Create a vector representing the direction of the trajectory
    double trajectoryDirectionX = cos(yaw);
    double trajectoryDirectionY = sin(yaw);

    // Calculate the cross product between the vector and the trajectory direction vector
    double crossProduct = vectorX * trajectoryDirectionY - vectorY * trajectoryDirectionX;

    // Calculate the magnitude of the trajectory direction vector
    double trajectoryDirectionMagnitude = std::sqrt(trajectoryDirectionX * trajectoryDirectionX + trajectoryDirectionY * trajectoryDirectionY);

    // Calculate the lateral deviation
    lateral_deviation = crossProduct / trajectoryDirectionMagnitude;
  }
  return lateral_deviation;
  }

} // namespace controller

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(controller::Controller)
