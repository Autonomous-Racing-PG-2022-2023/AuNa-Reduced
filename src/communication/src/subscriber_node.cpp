#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

void callback(const std_msgs::msg::String::SharedPtr message, const std::string& robotName)
{
	  RCLCPP_INFO(rclcpp::get_logger("subscriber_node"), "'%s' received: '%s'",robotName.c_str(), message->data.c_str());
}

int main(int argc, char** argv)
{
	std::string robotName;
	// Shouldn't this be >= 1?
	if (argc > 1) {
		robotName = argv[1];
	} else {
		RCLCPP_ERROR(rclcpp::get_logger("subscriber_node"), "Missing robot name argument");
		return 1;
	}

	// Problem: Also gets messages from itself. Idea: Set ignore_local_publications	= true
	rclcpp::init(argc, argv); 
	auto node = rclcpp::Node::make_shared("subscriber_node");
	auto subscription = node->create_subscription<std_msgs::msg::String>(
			        "/topic", 10,
				[robotName](const std_msgs::msg::String::SharedPtr message) {
				callback(message, robotName);
				});

	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
