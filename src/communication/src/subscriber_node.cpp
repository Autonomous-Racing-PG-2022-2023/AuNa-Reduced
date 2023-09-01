#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "autoware_auto_perception_msgs/msg/predicted_object.hpp"

void callback(const autoware_auto_perception_msgs::msg::PredictedObject::SharedPtr message, const std::string& robotName)
{
	// Check if the message contains the robot's name (ignore self messages) 
    	//if (message->data.find(robotName) != std::string::npos) {
        	//RCLCPP_INFO(rclcpp::get_logger("subscriber_node"), "Ignoring message with robot name: '%s'", robotName.c_str());
        //	return;
    	//}  

	std::string robotUUID = robotName.substr(5);
	int current = 0;
	char char1, char2;
	uint8_t uuid;

	bool ownMessage = true;

	for(std::string::iterator it = robotUUID.begin(); it != robotUUID.end() && current < 16 && ownMessage == true; it++) {
			char1 = *it;

			it++;
			if(it == robotUUID.end()) {
					char2 = 0;
			} else {
					char2 = *it;
			}

			uuid = (char1 - '0') << 4;
			uuid += (char2 - '0') << 0;

			if(uuid != message->object_id.uuid[current]) ownMessage = false;

			current++;
	}

	if(ownMessage) {
		//RCLCPP_INFO(rclcpp::get_logger("subscriber_node"), "Ignoring message with robot name: '%s'", robotName.c_str());
	} else {
		//RCLCPP_INFO(rclcpp::get_logger("subscriber_node"), "'%s' received: '%s'",robotName.c_str(), message->object_id.uuid[0]);
	}
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
