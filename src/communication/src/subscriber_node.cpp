#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "autoware_auto_perception_msgs/msg/predicted_object.hpp"

void callback(const autoware_auto_perception_msgs::msg::PredictedObject::SharedPtr message, const std::string& robotName)
{
	  RCLCPP_INFO(rclcpp::get_logger("subscriber_node"), "'%s' received: '%d'",robotName.c_str(), message->object_id.uuid[0]);
}



int main(int argc, char** argv)
{
	std::string robotName;
	if (argc > 1) {
		robotName = argv[1];
	} else {
		RCLCPP_ERROR(rclcpp::get_logger("subscriber_node"), "Missing robot name argument");
		return 1;
	}

	//creating subscription on global communication topic		
	rclcpp::init(argc, argv); 
	auto node = rclcpp::Node::make_shared("subscriber_node");
	auto subscription = node->create_subscription<autoware_auto_perception_msgs::msg::PredictedObject>(
			        "/topic", 10,
				[robotName](const autoware_auto_perception_msgs::msg::PredictedObject::SharedPtr message) {
				callback(message, robotName);
				});

	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
