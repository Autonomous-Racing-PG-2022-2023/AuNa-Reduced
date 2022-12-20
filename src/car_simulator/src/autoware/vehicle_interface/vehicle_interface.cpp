// Copyright 2017-2019 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "vehicle_interface.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>

VehicleInterface::VehicleInterface()
: Node("vehicle_interface"),
  vehicle_info_(vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo())
{
	/* setup parameters */
	namespace_ = declare_parameter("namespace", "");
	base_frame_id_ = declare_parameter("base_frame_id", "base_link");
	
	RCLCPP_INFO_THROTTLE(
		get_logger(),
		*get_clock(),
		std::chrono::milliseconds(1000).count(),
		"namespace: %s, base_frame_id: %s, loop_rate: %lf",
		namespace_.c_str(),
		base_frame_id_.c_str(),
		loop_rate_
	);

	/* parameters for vehicle specifications */
	tire_radius_ = vehicle_info_.wheel_radius_m;
	wheel_base_ = vehicle_info_.wheel_base_m;

	/* subscribers */
	using std::placeholders::_1;
	using std::placeholders::_2;

	// From autoware
	control_cmd_sub_ = create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
		"/control/command/control_cmd",
		rclcpp::QoS{1},
		std::bind(&VehicleInterface::callbackControlCmd, this, _1)
	);

	/* publisher */
	
	// To vehicle
	cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
		(namespace_ + std::string("/cmd_vel")).c_str(),
		rclcpp::QoS{1}
	);

	// To Autoware
	vehicle_twist_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>(
		"/vehicle/status/velocity_status",
		rclcpp::QoS{1}
	);
	steering_status_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>(
		"/vehicle/status/steering_status",
		rclcpp::QoS{1}
	);
	
}

void VehicleInterface::callbackControlCmd(
  const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg)
{
	autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr control_cmd_ptr_ = msg;

	//To Vehicle
	
	/* publish cmd_vel */
	{
		
		geometry_msgs::msg::Twist cmd_vel;
		
		geometry_msgs::msg::Vector3 linear;
		linear.x = control_cmd_ptr_->longitudinal.speed;
		cmd_vel.linear = linear;
		
		geometry_msgs::msg::Vector3 angular;
		angular.z = control_cmd_ptr_->lateral.steering_tire_angle;
		cmd_vel.angular = angular;
		
		cmd_vel_pub_->publish(cmd_vel);
	}

	//TODO: Use measued calues
	double current_velocity = control_cmd_ptr_->longitudinal.speed;
	double current_steer = control_cmd_ptr_->lateral.steering_tire_angle;

	//To Autoware

	std_msgs::msg::Header header;
	header.frame_id = base_frame_id_;
	header.stamp =  this->now();

	/* publish vehicle status twist */
	{
		autoware_auto_vehicle_msgs::msg::VelocityReport twist;
		twist.header = header;
		twist.longitudinal_velocity = current_velocity;
		twist.lateral_velocity = 0.0;
		twist.heading_rate = 0.0;
		vehicle_twist_pub_->publish(twist);
	}

	/* publish current status steering */
	{
		autoware_auto_vehicle_msgs::msg::SteeringReport steer_msg;
		steer_msg.stamp = header.stamp;
		steer_msg.steering_tire_angle = current_steer;//Desired steering angle
		steering_status_pub_->publish(steer_msg);
	}
}

