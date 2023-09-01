#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_msgs/msg/tf_message.hpp"

class StaticFramePublisher : public rclcpp::Node
{
private:

std::string src_;
std::string dst_;

std::string dst_tf_static_;

double translation_x_;
double translation_y_;
double translation_z_;
double yaw_;
double pitch_;
double roll_;

rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr static_tf_publisher_;


bool frame_id_is_identity;


public:
	explicit StaticFramePublisher(char * transformation[])
	: Node("tf_publish_static_map")
	{

		src_ = this->declare_parameter("src_", "");
		dst_ = this->declare_parameter("dst_", "");

		dst_tf_static_ = this->declare_parameter("dst_tf_static", "tf_static");

		translation_x_ = this->declare_parameter("translation_x_", 0.0);
		translation_y_ = this->declare_parameter("translation_y_", 0.0);
		translation_z_ = this->declare_parameter("translation_z_", 0.0);
		yaw_ = this->declare_parameter("yaw_", 0.0);
		pitch_ = this->declare_parameter("pitch_", 0.0);
		translation_x_ = this->declare_parameter("roll_", 0.0);

		frame_id_is_identity = (src_ == dst_);

		if(frame_id_is_identity){
			RCLCPP_INFO_THROTTLE(
				get_logger(),
				*get_clock(),
				std::chrono::milliseconds(1000).count(),
				"Mapping is a identity mapping. Shuting down node..."
			);
			rclcpp::shutdown();
		}else{
			// Initialize the transform broadcaster
			static_tf_publisher_ = create_publisher<tf2_msgs::msg::TFMessage>(
				dst_tf_static_.c_str(),
				rclcpp::QoS(1)
					.keep_all()
					.transient_local()// Ensure nodes that subscribe late still receive
										// static transformation
			);

			RCLCPP_INFO_THROTTLE(
				get_logger(),
				*get_clock(),
				std::chrono::milliseconds(1000).count(),
				"Mapping frames to %s. From frames %s to %s.",
				dst_tf_static_.c_str(),
				src_.c_str(),
				dst_.c_str()
			);

			//Send transform
			tf2_msgs::msg::TFMessage message;
			geometry_msgs::msg::TransformStamped t;
			t.header.frame_id = frame_id_;
			t.header.stamp = this->get_clock()->now();
			t.child_frame_id = child_frame_id_;
			
			t.transform.translation.x = translation_x_;
			t.transform.translation.y = translation_y_;
			t.transform.translation.z = translation_z_;
			t.transform.rotation = tf2::Quaternion(yaw_, pitch_, roll_);
			message.transforms.push_back(t);

			sendStaticTransform(message);
			rclcpp::shutdown();

		}
	}

private:
  void sendStaticTransform(const tf2_msgs::msg::TFMessage& msg) {
		for(const geometry_msgs::msg::TransformStamped& current_transform: msg.transforms) {
			auto predicate = [&current_transform](const geometry_msgs::msg::TransformStamped existing) {
				return (
					(current_transform.header.frame_id == existing.header.frame_id)
					&&(current_transform.child_frame_id == existing.child_frame_id)
				);
			};
			auto existing = std::find_if(net_message_.transforms.begin(), net_message_.transforms.end(), predicate);

			if(existing != net_message_.transforms.end()) {
				*existing = current_transform;
			} else {
				net_message_.transforms.push_back(current_transform);

				RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(), "Statically mapping from %s to %s", current_transform.header.frame_id.c_str(), current_transform.child_frame_id.c_str());
			}
		}

		this->static_tf_publisher_->publish(net_message_);
	}

};

int main(int argc, char * argv[])
{
  auto logger = rclcpp::get_logger("logger");

     // "child_frame_name x y z roll pitch yaw topic_name"


  // Pass parameters and initialize node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticFramePublisher>(argv));
  rclcpp::shutdown();
  return 0;
}