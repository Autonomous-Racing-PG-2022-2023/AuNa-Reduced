#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <map>

#include "rclcpp/rclcpp.hpp"

#include "autoware_auto_perception_msgs/msg/predicted_objects.hpp"
#include "autoware_auto_perception_msgs/msg/predicted_object.hpp"
#include "autoware_auto_perception_msgs/msg/object_classification.hpp"
#include "autoware_auto_perception_msgs/msg/predicted_object_kinematics.hpp"

std::string robotName_;

class ObjectsProvider : public rclcpp::Node
{
private:
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<autoware_auto_perception_msgs::msg::PredictedObjects>::SharedPtr publisher_;
	rclcpp::Subscription<autoware_auto_perception_msgs::msg::PredictedObject>::SharedPtr subscription_;

	std::map<int, autoware_auto_perception_msgs::msg::PredictedObject> dynamic_objects;

	std::string frame_id_;
	int publish_period_;
	std::string dst_topic_;
public:
	ObjectsProvider()
	: Node("objects_provider")
	{
		frame_id_ = this->declare_parameter("frame_id", "map");
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

		subscription_ = this->create_subscription<autoware_auto_perception_msgs::msg::PredictedObject>(
			"/topic", 
			rclcpp::QoS{10}.best_effort(),
			std::bind(&ObjectsProvider::callback, this, std::placeholders::_1)
		);
	}

private:
	void timeCallback()
	{
		autoware_auto_perception_msgs::msg::PredictedObjects objects;
		objects.header.stamp = this->now();
		objects.header.frame_id = this->frame_id_;

		geometry_msgs::msg::Pose path1;
		path1.position.x = 2.0;
		path1.position.y = -0.0;
		path1.position.z = 0.0;
		path1.orientation.w = 1.0;
		path1.orientation.x = 0.0;
		path1.orientation.y = 0.0;
		path1.orientation.z = 0.0;

		geometry_msgs::msg::Pose path2;
		path2.position.x = 2.0;
		path2.position.y = -0.0;
		path2.position.z = 0.0;
		path2.orientation.w = 1.0;
		path2.orientation.x = 0.0;
		path2.orientation.y = 0.0;
		path2.orientation.z = 0.0;

		geometry_msgs::msg::Vector3 vector;
		vector.x = 0.0;
		vector.y = 0.0;
		vector.z = 0.0;

		builtin_interfaces::msg::Duration time_step;
		time_step.sec = 1;
		time_step.nanosec = 0;

		geometry_msgs::msg::PoseWithCovariance initial_pose_with_covariance;
		initial_pose_with_covariance.pose.position.x = 2.0;
		initial_pose_with_covariance.pose.position.y = -0.0;
		initial_pose_with_covariance.pose.position.z = 0.0;
		initial_pose_with_covariance.pose.orientation.w = 1.0;
		initial_pose_with_covariance.pose.orientation.x = 0.0;
		initial_pose_with_covariance.pose.orientation.y = 0.0;
		initial_pose_with_covariance.pose.orientation.z = 0.0;

      	geometry_msgs::msg::TwistWithCovariance initial_twist_with_covariance;
		initial_twist_with_covariance.twist.linear = vector;
		initial_twist_with_covariance.twist.angular = vector;

      	geometry_msgs::msg::AccelWithCovariance initial_acceleration_with_covariance;
		initial_acceleration_with_covariance.accel.linear = vector;
		initial_acceleration_with_covariance.accel.angular = vector;

      	autoware_auto_perception_msgs::msg::PredictedPath predicted_path;
		predicted_path.path.push_back(path1);
		predicted_path.path.push_back(path2);
		predicted_path.time_step = time_step;

		unique_identifier_msgs::msg::UUID object_id;
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

		autoware_auto_perception_msgs::msg::PredictedObject dummyObject;
		dummyObject.object_id = object_id;
		dummyObject.existence_probability = 1.0;
		dummyObject.classification.push_back(classification);
		dummyObject.kinematics = kinematics;
		dummyObject.shape = shape;

		//objects.objects.push_back(dummyObject);
		for(auto it = dynamic_objects.begin(); it != dynamic_objects.end(); ++it) {
			if(it->first != 0) {
				objects.objects.push_back(it->second);
			}
		}
		
		publisher_->publish(objects);
	}

	void callback(const autoware_auto_perception_msgs::msg::PredictedObject::SharedPtr message)
	{
		std::string robotUUID = robotName_.substr(5);
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
			//RCLCPP_INFO(rclcpp::get_logger("subscriber_node"), "Ignoring message with robot name: '%s'", robotName_.c_str());
		} else {
			//RCLCPP_INFO(rclcpp::get_logger("subscriber_node"), "New Message: %d %d %s", uuid, message->object_id.uuid[--current], robotName_.c_str());
			dynamic_objects[message->object_id.uuid[0]] = *message;
		}
	}

};

int main(int argc, char * argv[])
{
	if (argc > 1) {
		robotName_ = argv[1];
	} else {
		RCLCPP_ERROR(rclcpp::get_logger("subscriber_node"), "Missing robot name argument");
		return 1;
	}

	rclcpp::init(argc, argv);
	auto node = std::make_shared<ObjectsProvider>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}