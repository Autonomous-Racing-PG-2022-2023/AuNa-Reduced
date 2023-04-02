#pragma once

#include "rclcpp/rclcpp.hpp"

#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"
#include "autoware_auto_planning_msgs/msg/path.hpp"

#include "vehicle_info_util/vehicle_info_util.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

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
#include <pcl/octree/octree_search.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/ring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include "circle.hpp"
#include "geometric_math.hpp"
#include "convex_hull.hpp"

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
	static constexpr double octree_voxel_size = 0.15;

	/* subscribers */
	rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_report_sub_;
	rclcpp::Subscription<car_simulator_msgs::msg::Track>::SharedPtr track_sub_;
	
	std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

	/* publishers */
	rclcpp::Publisher<autoware_auto_planning_msgs::msg::Path>::SharedPtr path_pub_;
	
	rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr poses_pub_;
	
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr left_bound_pub_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr right_bound_pub_;
	
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr predicted_position_pub_;

	/* ros param */
	vehicle_info_util::VehicleInfo vehicle_info_;
	
	bool enable_debug_topics_;
	
	std::string frame_id_;
	double min_scan_time_offset_;//Minimum time between two scans
	int max_path_points_;
	Wallfollowing::TargetMethod target_method_;
	
	double prediction_min_distance_;
	double prediction_max_distance_;
	double prediction_time_;
	double prediction_average_weight_;
	double target_min_distance_;
	double target_collision_precision_;
	double max_yaw_deviation_;
	double stop_detection_speed_threshold_;
	double max_longitudinal_velocity_mps_;
	double max_lateral_velocity_mps_;
	double max_heading_rate_rps_;
	
	/* data */
	rclcpp::Time last_scan_time;
	autoware_auto_planning_msgs::msg::Path path;
	autoware_auto_vehicle_msgs::msg::VelocityReport velocityReport;
	double predicted_position_average;
	tf2::Transform last_transform_to_map;

	/* callbacks */
	void callbackVelocityReport(const autoware_auto_vehicle_msgs::msg::VelocityReport::ConstSharedPtr velocityReport);
	void callbackTrack(const car_simulator_msgs::msg::Track::ConstSharedPtr track_in);

	/*  functions */
	tier4_autoware_utils::LinearRing2d calculateFootprint(const geometry_msgs::msg::Pose& pose);
	bool doesOverlap(const tier4_autoware_utils::LinearRing2d& polygon, const pcl::PointCloud<pcl::PointXYZ>& cloud);
	bool lineTooCloseToPointcloud(const Line<pcl::PointXYZ>& line, const pcl::PointCloud<pcl::PointXYZ>& cloud, const double radius);
	bool isPoseinFront(const geometry_msgs::msg::Pose& reference_pose, const geometry_msgs::msg::Pose& current_pose);
	std::vector<convex_hull::point> getConvexHull2D(const pcl::PointCloud<pcl::PointXYZ>& cloud);
	std::vector<convex_hull::point> getConvexHull2D(const std::vector<convex_hull::point>& points);
	pcl::PointXYZ determineTrackCenter(const pcl::PointXYZ& nearest_left, const pcl::PointXYZ& nearest_right, const pcl::PointXYZ& previous_point, double max_yaw = std::numeric_limits<double>::max());
	pcl::PointXYZ determineTrackDirection(const pcl::PointXYZ& nearest_left, const pcl::PointXYZ& nearest_right, const pcl::PointXYZ& previous_point, double max_yaw = std::numeric_limits<double>::max());
	std::pair<std::size_t, std::size_t> getNearestPoints(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>& left_octree, pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>& right_octree, const pcl::PointXYZ& predicted_position);

	
	void addPathPoint(const geometry_msgs::msg::PoseStamped& new_pose, const geometry_msgs::msg::TransformStamped& transform_msg);
	void setPathBounds(pcl::PointCloud<pcl::PointXYZ>& left_cloud, pcl::PointCloud<pcl::PointXYZ>& right_cloud, const std::vector<geometry_msgs::msg::Point>& untransformed_path_point_positions, const geometry_msgs::msg::TransformStamped& transform_msg);
	
	pcl::PointXYZ determinePredictedCarPosition();
	pcl::PointXYZ determineTargetPathPoint(const pcl::PointCloud<pcl::PointXYZ>& left_cloud, const pcl::PointCloud<pcl::PointXYZ>& right_cloud, const pcl::PointCloud<pcl::PointXYZ>& upper_cloud, pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>& left_octree, pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>& right_octree, const pcl::PointXYZ& initial_predicted_position, double epsilon);
	
	void followWalls(const car_simulator_msgs::msg::Track::ConstSharedPtr track);
	void publishPath();
};