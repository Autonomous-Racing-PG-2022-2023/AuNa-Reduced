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

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"

#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/control_mode_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/engage.hpp"
#include "autoware_auto_vehicle_msgs/msg/gear_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/gear_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"
#include "autoware_auto_vehicle_msgs/srv/control_mode_command.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"

#include "nav_msgs/msg/odometry.hpp"

#include "sensor_msgs/msg/imu.hpp"

#include "tf2/exceptions.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <optional>
#include <string>

class VehicleInterface : public rclcpp::Node
{
public:
	VehicleInterface();

private:
	/* subscribers */
	rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr control_cmd_sub_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
	rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;

	/* publishers */

	// To vehicle
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

	// To Autoware
	rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr vehicle_twist_pub_;
	rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_status_pub_;
	rclcpp::Publisher<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr acceleration_status_pub_;

	rclcpp::TimerBase::SharedPtr timer_;
	
	std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

	/* ros param */
	std::string base_frame_id_;
	std::string steering_frame_id_;
	
	double steering_translation_ratio_;

	vehicle_info_util::VehicleInfo vehicle_info_;

	/* callbacks */
	void callbackControlCmd(const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg);
	void callbackOdom( const nav_msgs::msg::Odometry::ConstSharedPtr msg);
	void callbackImu( const sensor_msgs::msg::Imu::ConstSharedPtr msg);
	void callbackTf(const tf2_msgs::msg::TFMessage::ConstSharedPtr msg);

	/*  functions */
	void publishCommands();
};