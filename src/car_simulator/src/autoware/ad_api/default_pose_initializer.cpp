// Copyright 2022 TIER IV, Inc.
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

#include "default_pose_initializer.hpp"

#include <memory>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

DefaultPoseInitializer::DefaultPoseInitializer() : Node("default_pose_initializer")
{
	frame_id_ = this->declare_parameter("frame_id", "base_link");
	x_ = this->declare_parameter<double>("position.x", 0.0);
	y_ = this->declare_parameter<double>("position.y", 0.0);
	z_ = this->declare_parameter<double>("position.z", 0.0);
	roll_ = this->declare_parameter<double>("orientation.roll", 0.0);
	pitch_ = this->declare_parameter<double>("orientation.pitch", 0.0);
	yaw_ = this->declare_parameter<double>("orientation.yaw", 0.0);
	
	const auto adaptor = component_interface_utils::NodeAdaptor(this);
	group_cli_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	adaptor.init_cli(cli_initialize_, group_cli_);
	adaptor.init_sub(sub_state_, [this](const State::Message::ConstSharedPtr msg) { state_ = *msg; });

	const auto period = rclcpp::Rate(1.0).period();
	timer_ = rclcpp::create_timer(this, get_clock(), period, [this]() { on_timer(); });

	state_.stamp = now();
	state_.state = State::Message::UNKNOWN;
}

void DefaultPoseInitializer::on_timer()
{
	timer_->cancel();
	if (state_.state == State::Message::UNINITIALIZED) {
		try {
			RCLCPP_INFO_THROTTLE(
				get_logger(),
				*get_clock(),
				std::chrono::milliseconds(1000).count(),
				"Sending initial pose. frame_id: %s, x: %f, y: %f, z: %f, roll: %f, pitch: %f, yaw: %f,",
				frame_id_.c_str(),
				x_,
				y_,
				z_,
				roll_,
				pitch_,
				yaw_
			);
			
			std::array<double, 36> covariance;
			covariance.fill(0.0);
			tf2::Quaternion q;
			q.setRPY(this->roll_, this->pitch_, this->yaw_);
			
			geometry_msgs::msg::PoseWithCovarianceStamped pose;
			pose.header.stamp = this->now();
			pose.header.frame_id = this->frame_id_;
			
			pose.pose.pose.position.x = this->x_;
			pose.pose.pose.position.y = this->y_;
			pose.pose.pose.position.z = this->z_;
			pose.pose.pose.orientation = tf2::toMsg(q);
			pose.pose.covariance = covariance;

			const auto req = std::make_shared<Initialize::Service::Request>();
			req->pose.push_back(pose);
			Initialize::Service::Response::SharedPtr response = cli_initialize_->call(req);
			
			if(!response->status.success){
				RCLCPP_INFO(get_logger(), "Initialization failed: Code: %d, Message: %s", response->status.code, response->status.message.c_str());
			}
		} catch (const component_interface_utils::ServiceException & error) {
			if(!error.status().success){
				RCLCPP_INFO(get_logger(), "Initialization failed: Code: %d, Message: %s", error.status().code, error.status().message.c_str());
			}
		}
	}
	timer_->reset();
}


int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::executors::MultiThreadedExecutor executor;
	auto node = std::make_shared<DefaultPoseInitializer>();
	executor.add_node(node);
	executor.spin();
	executor.remove_node(node);
	rclcpp::shutdown();
}