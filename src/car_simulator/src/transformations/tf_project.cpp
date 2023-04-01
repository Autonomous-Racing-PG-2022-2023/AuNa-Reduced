#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "tf2/exceptions.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class TFProjector : public rclcpp::Node {
   private:
	rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_;
	rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr
		static_subscription_;

	rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_publisher_;
	rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr static_tf_publisher_;
	
	std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

	tf2_msgs::msg::TFMessage net_message_;

	std::string src_frame_id_;
	std::string src_child_frame_id_;
	
	std::string dst_frame_id_;
	std::string dst_child_frame_id_;

	std::string src_tf_;
	std::string dst_tf_;

	std::string src_tf_static_;
	std::string dst_tf_static_;
	
	bool frame_id_is_identity;
	bool tf_is_identity;
	bool tf_static_is_identity;

   public:
	TFProjector()
		: Node("frame_project") {
		src_frame_id_ = this->declare_parameter("src_frame_id", "");
		src_child_frame_id_ = this->declare_parameter("src_child_frame_id", "");
		
		dst_frame_id_ = this->declare_parameter("dst_frame_id", "");
		dst_child_frame_id_ = this->declare_parameter("dst_child_frame_id", "");

		src_tf_ = this->declare_parameter("src_tf", "tf");
		dst_tf_ = this->declare_parameter("dst_tf", "tf");

		src_tf_static_ = this->declare_parameter("src_tf_static", "tf_static");
		dst_tf_static_ = this->declare_parameter("dst_tf_static", "tf_static");
		
		frame_id_is_identity = ((src_frame_id_ == dst_frame_id_) && (src_child_frame_id_ == dst_child_frame_id_));
		tf_is_identity = (src_tf_ == dst_tf_);
		tf_static_is_identity = (src_tf_static_ == dst_tf_static_);
		
		if(frame_id_is_identity && tf_is_identity && tf_static_is_identity){
			RCLCPP_INFO_THROTTLE(
				get_logger(),
				*get_clock(),
				std::chrono::milliseconds(1000).count(),
				"Projecting frame on itself is a no-op. Shuting down node..."
			);
			rclcpp::shutdown();
		}else{
			// Initialize the transform broadcaster
			tf_publisher_		 = create_publisher<tf2_msgs::msg::TFMessage>(dst_tf_.c_str(), 100);
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
				"Projectiong frames %s -> %s to %s -> %s resolving %s -> %s in %s, %s to %s, %s.",
				src_frame_id_.c_str(),
				src_child_frame_id_.c_str(),
				dst_frame_id_.c_str(),
				dst_child_frame_id_.c_str(),
				src_frame_id_.c_str(),
				dst_frame_id_.c_str(),
				src_tf_.c_str(),
				src_tf_static_.c_str(),
				dst_tf_.c_str(),
				dst_tf_static_.c_str()
			);

			subscription_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
				src_tf_.c_str(),
				10,
				std::bind(&TFProjector::handle_pose<false>, this, std::placeholders::_1)
			);

			static_subscription_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
				src_tf_static_.c_str(),
				10,
				std::bind(&TFProjector::handle_pose<true>, this, std::placeholders::_1)
			);
			// TODO:Maybe use filter
			
			//TF2 message buffer
			tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
			std::shared_ptr<tf2_ros::CreateTimerROS> timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
				this->get_node_base_interface(),
				this->get_node_timers_interface()
			);
			tf_buffer_->setCreateTimerInterface(timer_interface);
			
			tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);//FIXME: Use listener for dst_topics
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

	template<bool is_static>
	void handle_pose(const std::shared_ptr<tf2_msgs::msg::TFMessage> msg) {
		for(const geometry_msgs::msg::TransformStamped& current_transform:
			msg->transforms) {
			// Only map messages with frames from our namespace
			// We resend the original message together with a mapping to our new
			// namespace
			if(current_transform.header.frame_id == src_frame_id_ && current_transform.child_frame_id == src_child_frame_id_) {
				tf2_msgs::msg::TFMessage send_message;

				geometry_msgs::msg::TransformStamped t_link = current_transform;
				t_link.header.frame_id = src_frame_id_;
				t_link.child_frame_id = dst_frame_id_;
				
				//src_frame_id_ -> dst_frame_id_ *  dst_frame_id_ -> dst_child_frame_id_ = src_frame_id_ -> src_child_frame_id_
				//=> src_frame_id_ -> dst_frame_id_ = src_frame_id_ -> src_child_frame_id_ * (dst_frame_id_ -> dst_child_frame_id_)^-1
				
				//NOTE: We are fetching the transformation dst_child_frame_id_ -> dst_frame_id_ which is equal to (dst_frame_id_ -> dst_child_frame_id_)^-1
				geometry_msgs::msg::TransformStamped dst_transform_msg;
				try{
					dst_transform_msg = tf_buffer_->lookupTransform(dst_frame_id_, dst_child_frame_id_, tf2::TimePointZero);//FIXME: Check for matching timestamp tf2_ros::fromMsg(t_link.header.stamp));
				} catch (std::exception& e) {
					RCLCPP_ERROR(this->get_logger(), "%s", e.what());
					return;
				}
				tf2::Transform old_transform;
				tf2::Transform dst_transform;
				tf2::fromMsg(t_link.transform, old_transform);
				tf2::fromMsg(dst_transform_msg.transform, dst_transform);
				
				tf2::Transform new_transform = old_transform * dst_transform;
				
				t_link.transform = tf2::toMsg(new_transform);

				send_message.transforms.push_back(t_link);

				// Send the transformation
				if(is_static) {
					this->sendStaticTransform(send_message);
				} else {
					this->tf_publisher_->publish(send_message);
				}
			}
		}
	}
};

int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TFProjector>());
	rclcpp::shutdown();
	return 0;
}