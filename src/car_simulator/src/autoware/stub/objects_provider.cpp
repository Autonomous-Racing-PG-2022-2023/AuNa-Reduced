#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "autoware_auto_perception_msgs/msg/predicted_objects.hpp"

class ObjectsProvider : public rclcpp::Node
{
private:
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<autoware_auto_perception_msgs::msg::PredictedObjects>::SharedPtr publisher_;

	std::string frame_id_;
	int publish_period_;
	std::string dst_topic_;
public:
	ObjectsProvider()
	: Node("objects_provider")
	{
		frame_id_ = this->declare_parameter("frame_id", "base_link");
		publish_period_ = this->declare_parameter<int>("publish_period", 100);
		dst_topic_ = this->declare_parameter("dst_topic", "~/output/objects");

		RCLCPP_INFO_THROTTLE(
			get_logger(),
			*get_clock(),
			std::chrono::milliseconds(1000).count(),
			"Publishing empty objects in topic %s for frame %s all %d miliseconds",
			dst_topic_.c_str(),
			frame_id_.c_str(),
			publish_period_
		);
		
		timer_ = rclcpp::create_timer(
			this,
			this->get_clock(),
			std::chrono::milliseconds(this->publish_period_),
			std::bind(&ObjectsProvider::timeCallback, this)
		);
		
		publisher_ = create_publisher<autoware_auto_perception_msgs::msg::PredictedObjects>(
			dst_topic_,
			rclcpp::QoS{1}
		);
	}

private:
	void timeCallback()
	{
		autoware_auto_perception_msgs::msg::PredictedObjects objects;
		objects.header.stamp = this->now();
		objects.header.frame_id = this->frame_id_;
		
		publisher_->publish(objects);
	}
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ObjectsProvider>());
	rclcpp::shutdown();
	return 0;
}