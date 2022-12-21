#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_msgs/msg/tf_message.hpp"

class FramePublisher : public rclcpp::Node
{
private:
	rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_;
	std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
	std::string src_;
	std::string dst_;
public:
	FramePublisher()
	: Node("frame_remap")
	{
		src_ = this->declare_parameter("src", "");
		dst_ = this->declare_parameter("dst", "");

		// Initialize the transform broadcaster
		tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

		RCLCPP_INFO_THROTTLE(
			get_logger(),
			*get_clock(),
			std::chrono::milliseconds(1000).count(),
			"Mapping frames from %s to %s",
			src_.c_str(),
			dst_.c_str()
		);

		subscription_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
			"tf",
			10,
			std::bind(&FramePublisher::handle_pose, this, std::placeholders::_1)
		);
	}

private:
	void handle_pose(const std::shared_ptr<tf2_msgs::msg::TFMessage> msg)
	{
		for(const geometry_msgs::msg::TransformStamped& current_transform : msg->transforms){
			std::string frame_id_str(current_transform.header.frame_id);
			if(frame_id_str.find(src_) != std::string::npos){

				geometry_msgs::msg::TransformStamped t = current_transform;

				// Read message content and assign it to
				// corresponding tf variables
				t.header.stamp = this->get_clock()->now();
				t.header.frame_id = (dst_ + frame_id_str.substr(src_.length())).c_str();
				t.child_frame_id = (dst_ + std::string(current_transform.child_frame_id).substr(src_.length())).c_str();

				// Send the transformation
				tf_broadcaster_->sendTransform(t);
				break;
			}
		}
	}
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<FramePublisher>());
	rclcpp::shutdown();
	return 0;
}