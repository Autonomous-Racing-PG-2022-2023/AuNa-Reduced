#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"

#include "autoware_auto_perception_msgs/msg/predicted_objects.hpp"
#include "autoware_auto_perception_msgs/msg/predicted_object.hpp"
#include "autoware_auto_perception_msgs/msg/object_classification.hpp"
#include "autoware_auto_perception_msgs/msg/predicted_object_kinematics.hpp"

rclcpp::Publisher<autoware_auto_perception_msgs::msg::PredictedObject>::SharedPtr publisher;
std::vector<autoware_auto_perception_msgs::msg::PredictedObject> messages;
geometry_msgs::msg::PoseArray poses;
geometry_msgs::msg::Vector3 linearVector;
unique_identifier_msgs::msg::UUID uuid;


// Storing Velocity Report in a Queue (messages). We do not want to pushlish every message directly, but have use scheduled publishing 
void velocityReportCallback(const autoware_auto_vehicle_msgs::msg::VelocityReport::SharedPtr msg, const std::string& robotName)
{
        linearVector.x = msg->longitudinal_velocity;
        linearVector.y = msg->lateral_velocity;
}


void pathCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        poses = *msg;

        autoware_auto_perception_msgs::msg::PredictedPath predicted_path;
        predicted_path.path.push_back(poses.poses[0]);
        predicted_path.path.push_back(poses.poses[1]);

        geometry_msgs::msg::PoseWithCovariance initial_pose_with_covariance;
        initial_pose_with_covariance.pose = poses.poses[0];

        geometry_msgs::msg::Vector3 emptyVector;
        emptyVector.x = 0.0;
        emptyVector.y = 0.0;
        emptyVector.z = 0.0;

        builtin_interfaces::msg::Duration time_step;
        time_step.sec = 1;
        time_step.nanosec = 0;

      	geometry_msgs::msg::TwistWithCovariance initial_twist_with_covariance;
        initial_twist_with_covariance.twist.linear = linearVector;
        initial_twist_with_covariance.twist.angular = emptyVector;

      	geometry_msgs::msg::AccelWithCovariance initial_acceleration_with_covariance;
        initial_acceleration_with_covariance.accel.linear = emptyVector;
        initial_acceleration_with_covariance.accel.angular = emptyVector;

        predicted_path.time_step = time_step;

        autoware_auto_perception_msgs::msg::ObjectClassification classification;
        classification.label = 1;	// 1 == CAR
        classification.probability = 1.0;

        autoware_auto_perception_msgs::msg::PredictedObjectKinematics kinematics;
        kinematics.initial_pose_with_covariance = initial_pose_with_covariance;
        kinematics.initial_twist_with_covariance = initial_twist_with_covariance;
        kinematics.initial_acceleration_with_covariance = initial_acceleration_with_covariance;
        kinematics.predicted_paths.push_back(predicted_path);

        autoware_auto_perception_msgs::msg::Shape shape;
        shape.type = 1;		// BOUNDING_BOX=0, CYLINDER=1, POLYGON=2
        geometry_msgs::msg::Vector3 dimensions;
        dimensions.x = 1.0;
        dimensions.y = 0.0;
        dimensions.z = 1.0;
        shape.dimensions = dimensions;

        autoware_auto_perception_msgs::msg::PredictedObject predictedObject;
        predictedObject.object_id = uuid;
        predictedObject.existence_probability = 1.0;
        predictedObject.classification.push_back(classification);
        predictedObject.kinematics = kinematics;
        predictedObject.shape = shape;

        messages.push_back(predictedObject);
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

	//creating Publisher Node 
        auto node = rclcpp::Node::make_shared("publisher_node");
        publisher = node->create_publisher<autoware_auto_perception_msgs::msg::PredictedObject>("/topic", 10);

        std::vector<rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr> subscriptions;

        int numRobots = std::stoi(robotNumber);
        messages.resize(numRobots);

        //for(int i = 0; i < numRobots; ++i) {
        //      std::string robotName = "robot" + std::to_string(i);
        //      std::string robotTopic = "/"+robotName+ "/vehicle/status/velocity_status";
        //      auto subscription = node -> create_subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>(
        //                      robotTopic, 10,
        //                      [robotName] (const autoware_auto_vehicle_msgs::msg::VelocityReport::SharedPtr msg) {
        //                      velocityReportCallback(msg,robotName);
        //                      });
        //      subscriptions.push_back(subscription);
        //      RCLCPP_INFO(node -> get_logger(), "Subscription created on %s", robotTopic.c_str());
        //}
        //
        //
        std::string robotTopic = "/"+robotName+ "/vehicle/status/velocity_status";
        //auto subscription = node->create_subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>(
        //                                      "/robot0/vehicle/status/velocity_status",10,velocityReportCallback);

        std::string robotUUID = robotName.substr(5);
        int current = 0;
        char char1, char2;

        for(std::string::iterator it = robotUUID.begin(); it != robotUUID.end() && current < 16; it++) {
                char1 = *it;

                it++;
                if(it == robotUUID.end()) {
                        char2 = 0;
                } else {
                        char2 = *it;
                }

                uuid.uuid[current] = (char1 - '0') << 4;
                uuid.uuid[current] += (char2 - '0') << 0;

                current++;
        }

        linearVector.x = 0.0;
        linearVector.y = 0.0;
        linearVector.z = 0.0;

        auto velSub = node -> create_subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>(
                              robotTopic, 10,
                              [robotName] (const autoware_auto_vehicle_msgs::msg::VelocityReport::SharedPtr msg) {
                              velocityReportCallback(msg,robotName);
                              });
        //RCLCPP_INFO(node->get_logger(), "Subscription created on vehicle/status/velocity_status");

        auto pathSub = node -> create_subscription<geometry_msgs::msg::PoseArray>(
                              "/"+robotName+ "/wallfollowing/debug/path/poses", 
                              10,
                              [] (const geometry_msgs::msg::PoseArray::SharedPtr msg) {
                              pathCallback(msg);
                              });

        rclcpp::WallRate loop_rate(20); // 2 Hz (every 0.5 seconds)
        //int count = 0;
        while (rclcpp::ok()) {
                for (const auto& message : messages) {
                        if(message.object_id.uuid[0] != 0){
                                publisher -> publish(message);
                        }
                }

                messages.clear();
                rclcpp::spin_some(node);
                loop_rate.sleep();
        }


        rclcpp::shutdown();
        return 0;
}






