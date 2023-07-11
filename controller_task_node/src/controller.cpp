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

 #include "controller_task_node/calculateDistance.hpp"
 #include "controller_task_node/calculateSteeringAngle.hpp"

#include "controller_task_node/controller.hpp"

#include <algorithm>

// change begin //
#include <fstream> // this is to include waypoints.txt file into controller.cpp. This makes a txt file is read or written by the controller.cpp.
// change end //


namespace controller
{

  Controller::Controller(const rclcpp::NodeOptions &options) 
      : Node("controller", options),
      visualization_rate_(50) // Initialize visualization_rate_ to 50
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

    void Controller::findClosestTrajectoryPoint() // this function is to prevent having dublicate codes in both OnTime() & calcSteerCmd()
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

    // change begin //
    // Publish trajectory points for visualization in RViz
    Trajectory trajectory; // this is a variable of Trajectory type.
    trajectory.header.stamp = this->now(); 
    trajectory.header.frame_id = "map";

    // Calculate the start index based on the car's position
    int num_points = trajectoryPoints_.size(); // to compare with an integer "i", we also need an integer.
    
    // Call the helper function to find the closest trajectory point
    findClosestTrajectoryPoint();

    // Determine the end index based on the start index and visualization rate
    int end_index = std::min(start_index_ + visualization_rate_, num_points);

    // Add the selected trajectory points to the Trajectory message
    for (int i = start_index_; i < end_index; ++i)
    {
        trajectory.points.push_back(trajectoryPoints_[i]);
    }
    pub_trajectory_->publish(trajectory);
  }
    // change end //


    // Calculate the control command and error values.
  

  double Controller::calcSteerCmd()
  {
    //double steer = 0.0;
    // Calculate the steering angle here.
    
    // change begin //
    
    // Call the helper function to find the closest trajectory point
    findClosestTrajectoryPoint(); // might be unnecessary

    // Calculate the steering angle here.
    double previous_lateral_error = 0/* obtain previous lateral error */;
    double integral_of_lateral_error = 0/* obtain integral of lateral error */;


    double steer = calculateSteeringAngle(lateral_deviation_, integral_of_lateral_error, previous_lateral_error);
    // change end //

    return steer;
  }

  double Controller::calcAccCmd()
  {
    double acc = 0.0;
    // Calculate the acceleration here.
    return acc;
  }
  double Controller::calcLateralDeviation() // I added this function into controller class since it is added in controller.hpp private section and I can use declared variables of controller class in this function.
  {
    double lateral_deviation = 0.0;
    // Calculate the lateral deviation here.

      // change begin //
  // Check if the trajectoryPoints_ vector is not empty
  if (!trajectoryPoints_.empty()) // this-> is added since calcLateralDeviation() is not a member func of controller clas
  {
    // Get the desired lateral position based on the start_index
    double desired_lateral_position = trajectoryPoints_[start_index_].pose.position.y;

    // Check if the odometry_ message is valid
    if (odometry_)
    {
      // Get the current lateral position from the odometry message
      double current_lateral_position = odometry_->pose.pose.position.y;

      // Calculate the lateral deviation as the difference between current and desired lateral positions
      lateral_deviation = current_lateral_position - desired_lateral_position;
    }
  }

    lateral_deviation_ = lateral_deviation;
    // change end //
    std::cout<<"lateral_deviation:"<<lateral_deviation<<std::endl;
    return lateral_deviation;
  }

} // namespace controller

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(controller::Controller)
