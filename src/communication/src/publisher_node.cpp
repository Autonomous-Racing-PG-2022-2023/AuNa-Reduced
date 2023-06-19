#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"

rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
std::vector<std_msgs::msg::String> messages;


void velocityReportCallback(const autoware_auto_vehicle_msgs::msg::VelocityReport::SharedPtr msg, const std::string& robotName)
{
	double longitudinalVelocity = msg->longitudinal_velocity;
	double lateralVelocity = msg->lateral_velocity;
	//RCLCPP_INFO(rclcpp::get_logger("subscriber_node"), "Velocity Report: Longitudinal=%.2f, Lateral=%.2f",
	//					                        longitudinalVelocity, lateralVelocity);
	//auto message = std_msgs::msg::String();
	std_msgs::msg::String message;
	message.data = "Robot "+ robotName +"'s velocity report: , Long = " + std::to_string(longitudinalVelocity)+
		", Lateral = " + std::to_string(lateralVelocity);
	messages.push_back(message);
	//RCLCPP_INFO(rclcpp::get_logger("subscription_node"), "published '%s'", message.data.c_str());
	//publisher -> publish(message);

}


int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	
	std::string robotNumber;
	    if (argc > 1) {
		    robotNumber = argv[1];
	    } else {
		    RCLCPP_ERROR(rclcpp::get_logger("publisher_node"), "Missing robot number argument");
		    return 1;
	    }


	auto node = rclcpp::Node::make_shared("publisher_node");
	publisher = node->create_publisher<std_msgs::msg::String>("topic", 10);
	
	std::vector<rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr> subscriptions;
	
	int numRobots = std::stoi(robotNumber);
	messages.resize(numRobots);

	for(int i = 0; i < numRobots; ++i) {
		std::string robotName = "robot" + std::to_string(i);
		std::string robotTopic = "/"+robotName+ "/vehicle/status/velocity_status";
		auto subscription = node -> create_subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>(
				robotTopic, 10,
				[robotName] (const autoware_auto_vehicle_msgs::msg::VelocityReport::SharedPtr msg) {
				velocityReportCallback(msg,robotName);
				});
		subscriptions.push_back(subscription);
		RCLCPP_INFO(node -> get_logger(), "Subscription created on %s", robotTopic.c_str());
	}
	//auto subscription = node->create_subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>(
	//					"/robot0/vehicle/status/velocity_status",10,velocityReportCallback);
	//RCLCPP_INFO(node->get_logger(), "Subscription created on vehicle/status/velocity_status");

	rclcpp::WallRate loop_rate(0.2); // 0.2 Hz (every 5 seconds)
	//int count = 0;
	while (rclcpp::ok()) {
		for (const auto& message : messages) {
			if(!message.data.empty()){
				RCLCPP_INFO(node->get_logger(), "Publishing '%s'", message.data.c_str());
				publisher -> publish(message);
			}
		}

		messages.clear();
		messages.resize(numRobots);

		rclcpp::spin_some(node);
		loop_rate.sleep();
	}
	

	rclcpp::shutdown();
	return 0;
}







