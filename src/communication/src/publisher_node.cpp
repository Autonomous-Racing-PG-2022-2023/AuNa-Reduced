#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"

rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
std::vector<std_msgs::msg::String> messages;

// Adds VelocityReport {longitudinal_velocity, lateral_velocity} to messages
void velocityReportCallback(const autoware_auto_vehicle_msgs::msg::VelocityReport::SharedPtr msg, const std::string& robotName)
{
        double longitudinalVelocity = msg->longitudinal_velocity;
        double lateralVelocity = msg->lateral_velocity;

        std_msgs::msg::String message;
        message.data = "Robot "+ robotName +"'s velocity report: , Long = " + std::to_string(longitudinalVelocity)+
                ", Lateral = " + std::to_string(lateralVelocity);
        messages.push_back(message);
}


int main(int argc, char** argv)
{
        rclcpp::init(argc, argv);

        std::string robotNumber;
        std::string robotName;
            if (argc > 2) {
                    robotNumber = argv[1];
                    robotName = argv[2];
            } else {
                    RCLCPP_ERROR(rclcpp::get_logger("publisher_node"), "Missing robot number and robot name arguments");
                    return 1;
            }


        auto node = rclcpp::Node::make_shared("publisher_node");
        // QoS is set to 10, history keeps last 10 messages
        // TODO: Consider renaming this topic
        publisher = node->create_publisher<std_msgs::msg::String>("/topic", 10);

        // TODO: Probably remove this
        std::vector<rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr> subscriptions;

        int numRobots = std::stoi(robotNumber);

        // Subscribe to velocity_status publisher
        std::string robotTopic = "/"+robotName+ "/vehicle/status/velocity_status";
        auto subscription = node -> create_subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>(
                              robotTopic, 10,
                              [robotName] (const autoware_auto_vehicle_msgs::msg::VelocityReport::SharedPtr msg) {
                              velocityReportCallback(msg,robotName);
                              });

        // Publish each message in messages
        rclcpp::WallRate loop_rate(0.2); // 0.2 Hz (every 5 seconds)
        while (rclcpp::ok()) {
                for (const auto& message : messages) {
                        if(!message.data.empty()){
                                RCLCPP_INFO(node->get_logger(), "Publishing '%s'", message.data.c_str());
                                publisher -> publish(message);
                        }
                        else {
                                RCLCPP_INFO(node->get_logger(), "Message data was empty");
                        }
                }

                messages.clear();

                // Complete all available queued work without blocking
                rclcpp::spin_some(node);
                loop_rate.sleep();
        }

        rclcpp::shutdown();
        return 0;
}






