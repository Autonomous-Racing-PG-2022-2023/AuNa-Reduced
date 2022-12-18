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
	base_frame_id_ = declare_parameter("base_frame_id", "base_link");
	command_timeout_ms_ = declare_parameter("command_timeout_ms", 1000);
	loop_rate_ = declare_parameter("loop_rate", 30.0);

	/* parameters for vehicle specifications */
	tire_radius_ = vehicle_info_.wheel_radius_m;
	wheel_base_ = vehicle_info_.wheel_base_m;

	steering_offset_ = declare_parameter("steering_offset", 0.0);
	enable_steering_rate_control_ = declare_parameter("enable_steering_rate_control", false);

	/* parameters for emergency stop */
	emergency_brake_ = declare_parameter("emergency_brake", 0.7);

	/* vehicle parameters */
	vgr_coef_a_ = declare_parameter("vgr_coef_a", 15.713);
	vgr_coef_b_ = declare_parameter("vgr_coef_b", 0.053);
	vgr_coef_c_ = declare_parameter("vgr_coef_c", 0.042);
	accel_pedal_offset_ = declare_parameter("accel_pedal_offset", 0.0);
	brake_pedal_offset_ = declare_parameter("brake_pedal_offset", 0.0);
	speed_scale_factor_ = declare_parameter("speed_scale_factor", 1.0);

	/* parameters for limitter */
	max_throttle_ = declare_parameter("max_throttle", 0.2);
	max_brake_ = declare_parameter("max_brake", 0.8);
	max_steering_wheel_ = declare_parameter("max_steering_wheel", 2.7 * M_PI);
	max_steering_wheel_rate_ = declare_parameter("max_steering_wheel_rate", 6.6);
	min_steering_wheel_rate_ = declare_parameter("min_steering_wheel_rate", 0.5);
	steering_wheel_rate_low_vel_ = declare_parameter("steering_wheel_rate_low_vel", 5.0);
	steering_wheel_rate_stopped_ = declare_parameter("steering_wheel_rate_stopped", 5.0);
	low_vel_thresh_ = declare_parameter("low_vel_thresh", 1.389);  // 5.0kmh

	/* parameters for turn signal recovery */
	hazard_thresh_time_ = declare_parameter("hazard_thresh_time", 0.20);  // s

	/* subscribers */
	using std::placeholders::_1;
	using std::placeholders::_2;

	// From autoware
	control_cmd_sub_ = create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
		"/control/command/control_cmd",
		1,
		std::bind(&VehicleInterface::callbackControlCmd, this, _1)
	);
	gear_cmd_sub_ = create_subscription<autoware_auto_vehicle_msgs::msg::GearCommand>(
		"/control/command/gear_cmd",
		1,
		std::bind(&VehicleInterface::callbackGearCmd, this, _1)
	);
	turn_indicators_cmd_sub_ = create_subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>(
		"/control/command/turn_indicators_cmd",
		rclcpp::QoS{1},
		std::bind(&VehicleInterface::callbackTurnIndicatorsCommand, this, _1)
	);
	hazard_lights_cmd_sub_ = create_subscription<autoware_auto_vehicle_msgs::msg::HazardLightsCommand>(
		"/control/command/hazard_lights_cmd",
		rclcpp::QoS{1},
		std::bind(&VehicleInterface::callbackHazardLightsCommand, this, _1)
	);

	actuation_cmd_sub_ = create_subscription<ActuationCommandStamped>(
		"/control/command/actuation_cmd",
		1,
		std::bind(&VehicleInterface::callbackActuationCmd, this, _1)
	);
	emergency_sub_ = create_subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>(
		"/control/command/emergency_cmd",
		1,
		std::bind(&VehicleInterface::callbackEmergencyCmd, this, _1)
	);

	/* publisher */
	
	// To vehicle
	cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
		"/robot/cmd_vel",
		rclcpp::QoS{1}
	);

	// To Autoware
	control_mode_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>(
		"/vehicle/status/control_mode",
		rclcpp::QoS{1}
	);
	vehicle_twist_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>(
		"/vehicle/status/velocity_status",
		rclcpp::QoS{1}
	);
	steering_status_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>(
		"/vehicle/status/steering_status",
		rclcpp::QoS{1}
	);
	gear_status_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::GearReport>(
		"/vehicle/status/gear_status",
		rclcpp::QoS{1}
	);
	turn_indicators_status_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>(
		"/vehicle/status/turn_indicators_status",
		rclcpp::QoS{1}
	);
	hazard_lights_status_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::HazardLightsReport>(
		"/vehicle/status/hazard_lights_status",
		rclcpp::QoS{1}
	);
	actuation_status_pub_ = create_publisher<ActuationStatusStamped>(
		"/vehicle/status/actuation_status",
		1
	);
	steering_wheel_status_pub_ = create_publisher<SteeringWheelStatusStamped>(
		"/vehicle/status/steering_wheel_status",
		1
	);
	door_status_pub_ = create_publisher<tier4_api_msgs::msg::DoorStatus>(
		"/vehicle/status/door_status",
		1
	);

	// Timer
	const auto period_ns = rclcpp::Rate(loop_rate_).period();
	timer_ = rclcpp::create_timer(
		this,
		get_clock(),
		period_ns,
		std::bind(&VehicleInterface::publishCommands, this)
	);
}

void VehicleInterface::callbackActuationCmd(const ActuationCommandStamped::ConstSharedPtr msg)
{
  actuation_command_received_time_ = this->now();
  actuation_cmd_ptr_ = msg;
}

void VehicleInterface::callbackEmergencyCmd(
  const tier4_vehicle_msgs::msg::VehicleEmergencyStamped::ConstSharedPtr msg)
{
  is_emergency_ = msg->emergency;
}

void VehicleInterface::callbackControlCmd(
  const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg)
{
  control_command_received_time_ = this->now();
  control_cmd_ptr_ = msg;
}

void VehicleInterface::callbackGearCmd(
  const autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg)
{
  gear_cmd_ptr_ = msg;
}

void VehicleInterface::callbackTurnIndicatorsCommand(
  const autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr msg)
{
  turn_indicators_cmd_ptr_ = msg;
}

void VehicleInterface::callbackHazardLightsCommand(
  const autoware_auto_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr msg)
{
  hazard_lights_cmd_ptr_ = msg;
}

void VehicleInterface::publishCommands()
{
	/* guard */
	if (!control_cmd_ptr_) {
		return;
	}

	const rclcpp::Time current_time = get_clock()->now();
	
	/* publish cmd_vel */
	{
		
		geometry_msgs::msg::Twist cmd_vel;
		
		geometry_msgs::msg::Vector3 linear;
		linear.x = control_cmd_ptr_->longitudinal.speed;
		cmd_vel.linear = linear;
		
		geometry_msgs::msg::Vector3 angular;
		angular.z = control_cmd_ptr_->lateral.steering_tire_angle;
		cmd_vel.angular = angular;
		
		cmd_vel_pub_->publish(cmd_vel);//TODO: Mayvbe use stamped?
	}
}

