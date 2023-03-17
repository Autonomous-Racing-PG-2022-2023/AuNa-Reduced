#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

class TFRemapper : public rclcpp::Node {
   private:
	rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_;
	rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr
		static_subscription_;

	rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_publisher_;
	rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr static_tf_publisher_;

	tf2_msgs::msg::TFMessage net_message_;

	std::string src_;
	std::string dst_;

	std::string src_tf_;
	std::string dst_tf_;

	std::string src_tf_static_;
	std::string dst_tf_static_;
	
	bool frame_id_is_identity;
	bool tf_is_identity;
	bool tf_static_is_identity;

   public:
	TFRemapper()
		: Node("frame_remap") {
		src_ = this->declare_parameter("src", "");
		dst_ = this->declare_parameter("dst", "");

		src_tf_ = this->declare_parameter("src_tf", "tf");
		dst_tf_ = this->declare_parameter("dst_tf", "tf");

		src_tf_static_ = this->declare_parameter("src_tf_static", "tf_static");
		dst_tf_static_ = this->declare_parameter("dst_tf_static", "tf_static");
		
		frame_id_is_identity = (src_ == dst_);
		tf_is_identity = (src_tf_ == dst_tf_);
		tf_static_is_identity = (src_tf_static_ == dst_tf_static_);
		
		if(frame_id_is_identity && tf_is_identity && tf_static_is_identity){
			RCLCPP_INFO_THROTTLE(
				get_logger(),
				*get_clock(),
				std::chrono::milliseconds(1000).count(),
				"Mapping is a identity mapping. Shuting down node..."
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
				"Mapping frames in %s, %s to %s, %s, from frames %s to %s.",
				src_tf_.c_str(),
				src_tf_static_.c_str(),
				dst_tf_.c_str(),
				dst_tf_static_.c_str(),
				src_.c_str(),
				dst_.c_str()
			);

			subscription_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
				src_tf_.c_str(),
				10,
				std::bind(&TFRemapper::handle_pose<false>, this, std::placeholders::_1)
			);

			static_subscription_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
				src_tf_static_.c_str(),
				10,
				std::bind(&TFRemapper::handle_pose<true>, this, std::placeholders::_1)
			);
			// TODO:Maybe use filter
		}
	}

   private:
	void sendStaticTransform(const tf2_msgs::msg::TFMessage& msg) {
		for(const geometry_msgs::msg::TransformStamped& current_transform: msg.transforms) {
			auto predicate = [&current_transform](const geometry_msgs::msg::TransformStamped existing) {
				return current_transform.child_frame_id == existing.child_frame_id;
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

	/*
   * TODO: If required maybe map src/a->src/b to src/a->dst/a->src/b->dst/b
   */
	template<bool is_static>
	void handle_pose(const std::shared_ptr<tf2_msgs::msg::TFMessage> msg) {
		for(const geometry_msgs::msg::TransformStamped& current_transform:
			msg->transforms) {
			// Only map messages with frames from our namespace
			// We resend the original message together with a mapping to our new
			// namespace
			if(current_transform.header.frame_id.find(src_) != std::string::npos || current_transform.child_frame_id.find(src_) != std::string::npos) {
				tf2_msgs::msg::TFMessage send_message_static;

				if(current_transform.header.frame_id.find(src_) != std::string::npos) {
					geometry_msgs::msg::TransformStamped t_link;
					t_link.header.stamp	   = current_transform.header.stamp;
					t_link.header.frame_id = current_transform.header.frame_id;
					t_link.child_frame_id = (dst_ + current_transform.header.frame_id.substr(src_.length()));

					send_message_static.transforms.push_back(t_link);
				}

				if(current_transform.child_frame_id.find(src_) != std::string::npos) {
					geometry_msgs::msg::TransformStamped t_link;
					t_link.header.stamp	   = current_transform.header.stamp;
					t_link.header.frame_id = current_transform.child_frame_id;
					t_link.child_frame_id = (dst_ + current_transform.child_frame_id.substr(src_.length()));

					send_message_static.transforms.push_back(t_link);
				}

				// Send the transformation
				if(is_static) {
					//Only publish original transform in another topic to avoid publishing it twice in the same topic
					if(!tf_static_is_identity){
						send_message_static.transforms.push_back(current_transform);
					}
				} else {
					//Only publish original transform in another topic to avoid publishing it twice in the same topic
					if(!tf_is_identity){
						tf2_msgs::msg::TFMessage send_message;
						send_message.transforms.push_back(current_transform);
						this->tf_publisher_->publish(send_message);
					}
				}
				this->sendStaticTransform(
					send_message_static
				);	// TODO:Maybe first sort out already seen
					// frames
			}
		}
	}
};

int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TFRemapper>());
	rclcpp::shutdown();
	return 0;
}