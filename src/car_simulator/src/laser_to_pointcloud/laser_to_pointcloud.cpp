#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "laser_geometry/laser_geometry.hpp"

class LaserToPointcloud : public rclcpp::Node
{
private:
	laser_geometry::LaserProjection projection_;
	std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
	
	std::string target_frame_;
	std::string src_topic_;
	std::string dst_topic_;
	double range_cutoff_;
	laser_geometry::channel_option::ChannelOption chanel_option_;
public:
	LaserToPointcloud()
	: Node("laser_to_pointcloud")
	{
		target_frame_ = this->declare_parameter("target_frame", "base_link");
		src_topic_ = this->declare_parameter("src_topic", "/laser/out");
		dst_topic_ = this->declare_parameter("dst_topic", "/pointcloud_raw");
		range_cutoff_ = this->declare_parameter<double>("range_cutoff", 1.0);
		chanel_option_ = static_cast<laser_geometry::channel_option::ChannelOption>(this->declare_parameter<int>("channel_option", laser_geometry::channel_option::ChannelOption::Default));

		RCLCPP_INFO_THROTTLE(
			get_logger(),
			*get_clock(),
			std::chrono::milliseconds(1000).count(),
			"Transforming Laser %s to Pointcloud %s",
			src_topic_.c_str(),
			dst_topic_.c_str()
		);
		
		//TF2 message buffer
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
		std::shared_ptr<tf2_ros::CreateTimerROS> timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
            this->get_node_base_interface(),
            this->get_node_timers_interface()
        );
        tf_buffer_->setCreateTimerInterface(timer_interface);
		
		//TODO: Maybe add filter to only call callback when new transformation is available

		subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
			src_topic_,
			10,
			std::bind(&LaserToPointcloud::scanCallback, this, std::placeholders::_1)
		);
		
		publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
			dst_topic_,
			rclcpp::QoS{1}
		);
	}

private:
	void scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_in)
	{
		sensor_msgs::msg::PointCloud2 cloud_out;
		projection_.transformLaserScanToPointCloud(
			target_frame_,
			*scan_in,
			cloud_out,
			*tf_buffer_,
			range_cutoff_,
			chanel_option_
		);
		publisher_->publish(cloud_out);
	}
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<LaserToPointcloud>());
	rclcpp::shutdown();
	return 0;
}