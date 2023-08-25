#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

void callback(const std_msgs::msg::String::SharedPtr message, const std::string& robotName)
{
	// Check if the message contains the robot's name (ignore self messages) 
    	if (message->data.find(robotName) != std::string::npos) {
        	//RCLCPP_INFO(rclcpp::get_logger("subscriber_node"), "Ignoring message with robot name: '%s'", robotName.c_str());
        	return;
    	}  
	RCLCPP_INFO(rclcpp::get_logger("subscriber_node"), "'%s' received: '%s'",robotName.c_str(), message->data.c_str());


	//future works:
	//store the messages in a queue and process them to improve the performance 
	//Having velocity, direction and postion of other cars can contribute to a better estimation of the robot's own position  
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
	auto subscription = node->create_subscription<std_msgs::msg::String>(
			        "/comm_topic", 10,
				[robotName](const std_msgs::msg::String::SharedPtr message) {
				callback(message, robotName);
				});

	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
