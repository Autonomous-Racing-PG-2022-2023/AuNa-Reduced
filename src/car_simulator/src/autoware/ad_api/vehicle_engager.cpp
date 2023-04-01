#include <chrono>
#include <cstdlib>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "tier4_external_api_msgs/srv/engage.hpp"

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	
	std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("vehicle_engager");
	rclcpp::Client<tier4_external_api_msgs::srv::Engage>::SharedPtr client = node->create_client<tier4_external_api_msgs::srv::Engage>("api/autoware/set/engage");

	auto request = std::make_shared<tier4_external_api_msgs::srv::Engage::Request>();
	request->engage = true;

	while (!client->wait_for_service(std::chrono::seconds(5))) {
		if (!rclcpp::ok()) {
			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
			return 0;
		}
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
	}

	auto result = client->async_send_request(request);
	// Wait for the result.
	if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
	{
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Successfully engaged vehicle");
	} else {
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to engage vehicle");
	}

	rclcpp::shutdown();
	return 0;
}