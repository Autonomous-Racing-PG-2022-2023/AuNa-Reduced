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

//TODO: Maybe use frames for tranmslation?
VehicleInterface::VehicleInterface()
: Node("vehicle_interface"),
  vehicle_info_(vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo())
{
	/* setup parameters */
	base_frame_id_ = declare_parameter("base_frame_id", "base_link");
	steering_frame_id_ = declare_parameter("steering_frame_id", "left_steering");
	
	steering_translation_ratio_ = declare_parameter<double>("steering_translation_ratio", 1.0);
	
	RCLCPP_INFO_THROTTLE(
		get_logger(),
		*get_clock(),
		std::chrono::milliseconds(1000).count(),
		"base_frame_id: %s, steering_frame_id: %s, steering_translation_ratio: %lf",
		base_frame_id_.c_str(),
		steering_frame_id_.c_str(),
		steering_translation_ratio_
	);

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
	
	imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
		"~/imu",
		rclcpp::QoS{1},
		std::bind(&VehicleInterface::callbackImu, this, _1)
	);
	
	tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
		"tf",
		rclcpp::QoS{1},
		std::bind(&VehicleInterface::callbackTf, this, _1)
	);
	//TODO:Maybe use filter
	
	tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
	std::shared_ptr<tf2_ros::CreateTimerROS> timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
		this->get_node_base_interface(),
		this->get_node_timers_interface()
	);
	tf_buffer_->setCreateTimerInterface(timer_interface);
	
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

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
	acceleration_status_pub_ = create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>(
		"~/vehicle/status/acceleration",
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
		
		cmd_vel.angular.z = this->steering_translation_ratio_ * control_cmd_ptr_->lateral.steering_tire_angle;
		
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
			twist.lateral_velocity = odom_ptr_->twist.twist.linear.y;
			twist.heading_rate = odom_ptr_->twist.twist.linear.z;
			vehicle_twist_pub_->publish(twist);
		}
	}
	
}

void VehicleInterface::callbackImu(
	const sensor_msgs::msg::Imu::ConstSharedPtr msg
)
{
	sensor_msgs::msg::Imu::ConstSharedPtr imu_ptr_ = msg;
	
	//Transform into base_frame
	geometry_msgs::msg::TransformStamped transform_msg;
	try{
		transform_msg = tf_buffer_->lookupTransform(this->base_frame_id_, imu_ptr_->header.frame_id, tf2::TimePointZero);//FIXME: Check for matching timestamp tf2_ros::fromMsg(t_link.header.stamp));
	} catch (std::exception& e) {
		RCLCPP_ERROR(this->get_logger(), "%s", e.what());
		return;
	}
	
	tf2::Transform tf_transform;
	tf2::fromMsg(transform_msg.transform, tf_transform);
	
	tf2::Vector3 tf_linear_acceleration(imu_ptr_->linear_acceleration.x, imu_ptr_->linear_acceleration.y, imu_ptr_->linear_acceleration.z);
	tf2::Vector3 tf_angular_velocity(imu_ptr_->angular_velocity.x, imu_ptr_->angular_velocity.y, imu_ptr_->angular_velocity.z);
	
	const tf2::Vector3 tf_transformed_linear_acceleration = tf_transform * tf_linear_acceleration;
	const tf2::Vector3 tf_transformed_angular_velocity = tf_transform * tf_angular_velocity;

	std_msgs::msg::Header header = transform_msg.header;

	/* publish acceleration status */
	{
		geometry_msgs::msg::AccelWithCovarianceStamped accel;
		accel.header = header;
		accel.accel.accel.linear.x = tf_transformed_linear_acceleration.x();
		accel.accel.accel.linear.y = tf_transformed_linear_acceleration.y();
		accel.accel.accel.linear.y = tf_transformed_linear_acceleration.y();
		//TODO: Do we need angular velocity or angular acceleration?
		accel.accel.accel.angular.x = tf_transformed_angular_velocity.x();
		accel.accel.accel.angular.y = tf_transformed_angular_velocity.y();
		accel.accel.accel.angular.y = tf_transformed_angular_velocity.y();
		//TODO: Fill covariance!
		acceleration_status_pub_->publish(accel);
	}
	
}

//TODO: Fetch steeting status directly from motor and use steering_translation_ratio_ accordingly here too.
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
				const geometry_msgs::msg::Quaternion rotation = current_transform.transform.rotation;
				const double yaw = std::atan2(2.0 * (rotation.x * rotation.y + rotation.w * rotation.z), rotation.w * rotation.w + rotation.x * rotation.x - rotation.y * rotation.y - rotation.z * rotation.z);
				
				autoware_auto_vehicle_msgs::msg::SteeringReport steer_msg;
				steer_msg.stamp = current_transform.header.stamp;
				steer_msg.steering_tire_angle = yaw;//Counter-clockwise is positive for ros and autoware
				steering_status_pub_->publish(steer_msg);
			}
			break;
		}
	}
	
	
	
}

