#pragma once

#include "rclcpp/rclcpp.hpp"

#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "nav_msgs/msg/path.hpp"

#include "tf2/exceptions.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "car_simulator_msgs/msg/track.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_pointcloud.h>

#include "circle.hpp"
#include "geometric_math.hpp"

class Wallfollowing : public rclcpp::Node
{
public:
	Wallfollowing();
	
	enum TargetMethod{
		CIRCLE_TANGENTS,
		TRACK_CENTER,
		CENTER_PATH,
	};

private:
	/* subscribers */
	rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_report_sub_;
	rclcpp::Subscription<car_simulator_msgs::msg::Track>::SharedPtr track_sub_;
	
	std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

	/* publishers */
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

	/* ros param */
	std::string frame_id_;
	double min_scan_time_offset_;//Minimum time between two scans
	int max_path_points_;
	Wallfollowing::TargetMethod target_method_;
	double safety_wall_distance_;
	double max_curve_circle_radius_;
	double max_view_dist_min_;
	double max_view_dist_max_;
	double max_view_dist_radius_multiplier_;
	double prediction_min_distance_;
	double prediction_max_distance_;
	double prediction_time_;
	double prediction_average_weight_;
	double target_min_distance_;
	double target_collision_precision_;
	
	/* data */
	rclcpp::Time last_scan_time;
	nav_msgs::msg::Path path;
	autoware_auto_vehicle_msgs::msg::VelocityReport velocityReport;
	double predicted_position_average;

	/* callbacks */
	void callbackVelocityReport(const autoware_auto_vehicle_msgs::msg::VelocityReport::ConstSharedPtr velocityReport);
	void callbackTrack(const car_simulator_msgs::msg::Track::ConstSharedPtr track_in);

	/*  functions */
	void addPathPoint(const geometry_msgs::msg::PoseStamped& new_pose);
	geometry_msgs::msg::Point determinePredictedCarPosition(const std::string& target_frame);
	geometry_msgs::msg::Point determineTrackCenter(const Circle<geometry_msgs::msg::Point>& left_circle, const Circle<geometry_msgs::msg::Point>& right_circle, const geometry_msgs::msg::Point& predicted_position);
	bool lineTooCloseToPointcloud(const pcl::PointXYZ& car_position, const Line<pcl::PointXYZ>& line, const pcl::PointCloud<pcl::PointXYZ>& cloud);
	std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point> determineTargetPathPoint(const pcl::PointXYZ& car_position, const Circle<geometry_msgs::msg::Point>& left_circle, const Circle<geometry_msgs::msg::Point>& right_circle, const pcl::PointCloud<pcl::PointXYZ>& left_cloud, const pcl::PointCloud<pcl::PointXYZ>& right_cloud, const pcl::PointCloud<pcl::PointXYZ>& upper_cloud, double min_distance, double max_distance, double epsilon);
	geometry_msgs::msg::Point determineTargetCarPosition(const car_simulator_msgs::msg::Track::ConstSharedPtr track, const Circle<geometry_msgs::msg::Point>& left_circle, const Circle<geometry_msgs::msg::Point>& right_circle, const Circle<geometry_msgs::msg::Point>& upper_circle, const geometry_msgs::msg::Point& predicted_position);
	void followWalls(const car_simulator_msgs::msg::Track::ConstSharedPtr track);
	void publishPath();
};