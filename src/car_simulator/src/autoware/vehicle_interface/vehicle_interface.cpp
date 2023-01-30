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

//odom => Speed with covariance; Also pose if necessary

VehicleInterface::VehicleInterface()
: Node("vehicle_interface"),
  vehicle_info_(vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo())
{
	/* setup parameters */
	base_frame_id_ = declare_parameter("base_frame_id", "base_link");
	steering_frame_id_ = declare_parameter("steering_frame_id", "left_steering");
	
	RCLCPP_INFO_THROTTLE(
		get_logger(),
		*get_clock(),
		std::chrono::milliseconds(1000).count(),
		"base_frame_id: %s, loop_rate: %lf",
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
		"~/control/command/control_cmd",
		rclcpp::QoS{1},
		std::bind(&VehicleInterface::callbackControlCmd, this, _1)
	);
	
	//From vehicle
	odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
		"~/odom",
		rclcpp::QoS{1},
		std::bind(&VehicleInterface::callbackOdom, this, _1)
	);
	
	tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
		"tf",
		rclcpp::QoS{1},
		std::bind(&VehicleInterface::callbackTf, this, _1)
	);
	//TODO:Maybe use filter

	/* publisher */
	
	// To vehicle
	cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
		"~/cmd_vel",
		rclcpp::QoS{1}
	);

	// To Autoware
	vehicle_twist_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>(
		"~/vehicle/status/velocity_status",
		rclcpp::QoS{1}
	);
	steering_status_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>(
		"~/vehicle/status/steering_status",
		rclcpp::QoS{1}
	);
	
}

void VehicleInterface::callbackControlCmd(
	const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg
)
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
}

void VehicleInterface::callbackOdom(
	const nav_msgs::msg::Odometry::ConstSharedPtr msg
)
{
	nav_msgs::msg::Odometry::ConstSharedPtr odom_ptr_ = msg;
	
	//Only use messages of base_frame for velocity
	if(odom_ptr_->child_frame_id == base_frame_id_){
	
		//To Autoware

		std_msgs::msg::Header header = odom_ptr_->header;

		/* publish vehicle status twist */
		{
			//TODO: Use different message with covariance and with angular and linear velocity
			autoware_auto_vehicle_msgs::msg::VelocityReport twist;
			twist.header = header;
			twist.longitudinal_velocity = odom_ptr_->twist.twist.linear.x;
			twist.lateral_velocity = odom_ptr_->twist.twist.linear.z;
			twist.heading_rate = odom_ptr_->twist.twist.linear.y;
			vehicle_twist_pub_->publish(twist);
		}
	}
	
}

void VehicleInterface::callbackTf(
	const tf2_msgs::msg::TFMessage::ConstSharedPtr msg
)
{
	tf2_msgs::msg::TFMessage::ConstSharedPtr tf_ptr_ = msg;
	
	for(const geometry_msgs::msg::TransformStamped& current_transform : tf_ptr_->transforms){
		
		//Only use messages of steering_frame for steering
		if(current_transform.child_frame_id == steering_frame_id_){

			/* publish current status steering */
			{
				autoware_auto_vehicle_msgs::msg::SteeringReport steer_msg;
				steer_msg.stamp = current_transform.header.stamp;
				steer_msg.steering_tire_angle = current_transform.transform.rotation.z * 100.0;//Desired steering angle
				steering_status_pub_->publish(steer_msg);
			}
			break;
		}
	}
	
	
	
}

