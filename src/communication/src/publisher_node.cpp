#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("publisher_node");
	auto publisher = node->create_publisher<std_msgs::msg::String>("topic", 10);
	rclcpp::WallRate loop_rate(2);
	int count = 0;
	while (rclcpp::ok()) {
		auto message = std_msgs::msg::String();
		message.data = "What do we need to publish? " + std::to_string(count);
		RCLCPP_INFO(node->get_logger(), "Test: published : '%s'", message.data.c_str());
		publisher->publish(message);
		rclcpp::spin_some(node);
		loop_rate.sleep();
		++count;
		}

	rclcpp::shutdown();
	return 0;
}
