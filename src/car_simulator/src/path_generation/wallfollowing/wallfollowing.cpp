#include "wallfollowing.hpp"

#include <math.h>
#include <algorithm>
#include <execution>

#include "pcl_conversions/pcl_conversions.h"

#include "tf2/utils.h"

#include "circle.hpp"
#include "geometric_math.hpp"

Wallfollowing::Wallfollowing()
: Node("wallfollowing"), vehicle_info_(vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo()), last_scan_time(this->now())
{
	/* setup parameters */
	enable_debug_topics_ = declare_parameter<bool>("enable_debug_topics", false);
	
	frame_id_ = declare_parameter("frame_id", "map");
	min_scan_time_offset_ = declare_parameter<double>("min_scan_time_offset", 0.0001);
	max_path_points_ = declare_parameter<int>("max_path_points", 10);
	const std::string target_method_string = this->declare_parameter("target_method", "CENTER_PATH");
	if(target_method_string == "CIRCLE_TANGENTS"){
		target_method_ = Wallfollowing::TargetMethod::CIRCLE_TANGENTS;
	}else if(target_method_string == "TRACK_CENTER"){
		target_method_ = Wallfollowing::TargetMethod::TRACK_CENTER;
	}else if(target_method_string == "CENTER_PATH"){
		target_method_ = Wallfollowing::TargetMethod::CENTER_PATH;
	}else{
		RCLCPP_ERROR(this->get_logger(), "Invalid value for parameter target_method: %s", target_method_string.c_str());
	}
	
	prediction_min_distance_ = declare_parameter<double>("prediction_min_distance", 0.1);
	prediction_max_distance_ = declare_parameter<double>("prediction_max_distance", 5.0);
	prediction_time_ = declare_parameter<double>("prediction_time", 0.01);
	prediction_average_weight_ = declare_parameter<double>("prediction_average_weight", 5.0);
	target_min_distance_ = declare_parameter<double>("target_min_distance", 0.3);
	target_collision_precision_ = declare_parameter<double>("target_collision_precision", 0.01);
	max_yaw_deviation_ = declare_parameter<double>("max_yaw_deviation", 0.7);
	
	stop_detection_speed_threshold_ = declare_parameter<double>("stop_detection_speed_threshold", 0.1);
	
	max_longitudinal_velocity_mps_ = this->declare_parameter<double>("max_longitudinal_velocity_mps", 100.0);
	max_lateral_velocity_mps_ = this->declare_parameter<double>("max_lateral_velocity_mps", 100.0);
	max_heading_rate_rps_ = this->declare_parameter<double>("max_heading_rate_rps", 100.0);
	
	RCLCPP_INFO_THROTTLE(
		get_logger(),
		*get_clock(),
		std::chrono::milliseconds(1000).count(),
		"enable_debug_topics: %s, frame_id: %s, min_scan_time_offset: %lf, max_path_points: %d, target_method: %d, "
		"prediction_min_distance: %lf, "
		"prediction_max_distance: %lf, prediction_time: %lf, prediction_average_weight: %lf, target_min_distance: %lf, "
		"target_collision_precision: %lf, max_yaw_deviation: %lf, stop_detection_speed_threshold: %lf, max_longitudinal_velocity_mps: %lf, max_lateral_velocity_mps: %lf, max_heading_rate_rps: %lf",
		(enable_debug_topics_ ? "true" : "false"),
		frame_id_.c_str(),
		min_scan_time_offset_,
		max_path_points_,
		target_method_,
		prediction_min_distance_,
		prediction_max_distance_,
		prediction_time_,
		prediction_average_weight_,
		target_min_distance_,
		target_collision_precision_,
		max_yaw_deviation_,
		stop_detection_speed_threshold_,
		max_longitudinal_velocity_mps_,
		max_lateral_velocity_mps_,
		max_heading_rate_rps_
	);

	/* subscribers */
	using std::placeholders::_1;

	velocity_report_sub_ = create_subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>(
		"~/input/velocity_report",
		rclcpp::QoS{1},
		std::bind(&Wallfollowing::callbackVelocityReport, this, _1)
	);
	
	track_sub_ = create_subscription<car_simulator_msgs::msg::Track>(
		"~/input/track",
		rclcpp::QoS{1},
		std::bind(&Wallfollowing::callbackTrack, this, _1)
	);
	
	tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
	std::shared_ptr<tf2_ros::CreateTimerROS> timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
		this->get_node_base_interface(),
		this->get_node_timers_interface()
	);
	tf_buffer_->setCreateTimerInterface(timer_interface);
	
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	/* publisher */

	path_pub_ = create_publisher<autoware_auto_planning_msgs::msg::Path>(
		"~/output/path",
		rclcpp::QoS{1}
	);
	
	poses_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
		"~/debug/path/poses",
		rclcpp::QoS{1}
	);
	
	left_bound_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
		"~/debug/bounds/left",
		rclcpp::QoS{1}
	);
	
	right_bound_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
		"~/debug/bounds/right",
		rclcpp::QoS{1}
	);
	
	predicted_position_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
		"~/debug/predicted_position",
		rclcpp::QoS{1}
	);
	
	
	
	//Init path
	this->path.header.frame_id = frame_id_;
	this->path.points.emplace_back();//First pose is start pose (origin)
	path.points[0].longitudinal_velocity_mps = max_longitudinal_velocity_mps_;
	path.points[0].lateral_velocity_mps = max_lateral_velocity_mps_;
	path.points[0].heading_rate_rps = max_heading_rate_rps_;
}

//NOTE: Mostly untested
tier4_autoware_utils::LinearRing2d Wallfollowing::calculateFootprint(const geometry_msgs::msg::Pose& pose){
	//Calculate footprint polygon
	tier4_autoware_utils::LinearRing2d footprint_polygon = this->vehicle_info_.createFootprint();
	
	//Rotate in correct direction
	tf2::Transform transform;
	tf2::fromMsg(pose, transform);
	
	const tf2::Matrix3x3 transform_matrix = transform.getBasis();
	const tf2::Vector3 transform_origin = transform.getOrigin();
	
	boost::geometry::strategy::transform::matrix_transformer<double, 2, 2> boost_transform(
		transform_matrix[0][0], transform_matrix[0][1], transform_origin[0],
		transform_matrix[1][0], transform_matrix[1][1], transform_origin[1],
		0.0, 0.0, 1.0
	);
	
	tier4_autoware_utils::LinearRing2d transformed_footprint_polygon;
	boost::geometry::transform(footprint_polygon, transformed_footprint_polygon, boost_transform);

	return transformed_footprint_polygon;
}

//TODO: Maybe accelerate with PCL structures or by using path bounds
bool Wallfollowing::doesOverlap(const tier4_autoware_utils::LinearRing2d& polygon, const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
	return std::any_of(std::execution::par_unseq, cloud.begin(), cloud.end(), [&polygon](const pcl::PointXYZ& point){
		const tier4_autoware_utils::Point2d boost_point(point.x, point.y);
		return boost::geometry::within(boost_point, polygon);
	});
}

//TODO: Maybe accelerate with PCL structures or by using path bounds
bool Wallfollowing::lineTooCloseToPointcloud(const Line<pcl::PointXYZ>& line, const pcl::PointCloud<pcl::PointXYZ>& cloud, const double radius)
{
	return std::any_of(std::execution::par_unseq, cloud.begin(), cloud.end(), [&line, &radius](const pcl::PointXYZ& point){
		return GeometricFunctions::calcShortestDistanceToLine(point, line) < radius;
	});
}

//Basically copied from Autoware
bool Wallfollowing::isPoseinFront(const geometry_msgs::msg::Pose& reference_pose, const geometry_msgs::msg::Pose& current_pose){
	const double src_yaw = tf2::getYaw(reference_pose.orientation);
	const double dx = current_pose.position.x - reference_pose.position.x;
	const double dy = current_pose.position.y - reference_pose.position.y;
	const double pose_direction_yaw = std::atan2(dy, dx);

	double normalized_rad;
	const double value = std::fmod(src_yaw - pose_direction_yaw, 2 * M_PI);
	if (-M_PI <= value && value < M_PI) {
		normalized_rad = value;
	}else{
		normalized_rad = (value - std::copysign(2 * M_PI, value));
	}
	
	return (std::fabs(normalized_rad) < M_PI / 2.0);
}

//NOTE: Using this, cause pcl::ConcaveHull is buggy (crashes) because QHull is shit
//TODO: Different algorithm (pcl::ConvexHull, when it works. Or a parallel one)
std::vector<convex_hull::point> Wallfollowing::getConvexHull2D(const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
	std::vector<convex_hull::point> points(cloud.size());
	std::transform(std::execution::par_unseq, cloud.begin(), cloud.end(), points.begin(), [](const pcl::PointXYZ& point){
		return convex_hull::point(point.x, point.y);
	});
	
	return getConvexHull2D(points);
}

std::vector<convex_hull::point> Wallfollowing::getConvexHull2D(const std::vector<convex_hull::point>& points)
{
	return convex_hull::quickHull(points);
}

pcl::PointXYZ Wallfollowing::determineTrackCenter(const pcl::PointXYZ& nearest_left, const pcl::PointXYZ& nearest_right, const pcl::PointXYZ& previous_point, double max_yaw)
{
	pcl::PointXYZ ret;
	ret.x = (nearest_left.x + nearest_right.x) / 2;
	ret.y = (nearest_left.y + nearest_right.y) / 2;
	
	//Correct yaw
	//TODO:Maybe move on line between nearest points instead on y-axis
	const double actual_yaw = std::atan2(ret.y - previous_point.y, ret.x - previous_point.x);
	const double yaw = std::copysign(std::min(actual_yaw, max_yaw), actual_yaw);
	
	ret.y = std::sin(yaw) * ret.x;
	
	return ret;
}

pcl::PointXYZ Wallfollowing::determineTrackDirection(const pcl::PointXYZ& nearest_left, const pcl::PointXYZ& nearest_right, const pcl::PointXYZ& previous_point, double max_yaw)
{
	//Calc difference
	pcl::PointXYZ difference;
	difference.x = nearest_right.x - nearest_left.x;
	difference.y = nearest_right.y - nearest_left.y;
	
	//Get orthogonal vector pointing in positive x direction
	pcl::PointXYZ rotated;
	rotated.x = std::abs(difference.y);
	rotated.y = -std::copysign(difference.x, difference.y);//If y is positive we have (y, -x), otherwise we habe (-y, x)
	
	//Correct yaw
	const double actual_yaw = std::atan2(rotated.y - previous_point.y, rotated.x - previous_point.x);
	const double yaw = std::copysign(std::min(actual_yaw, max_yaw), actual_yaw);
	
	pcl::PointXYZ ret;
	ret.x = std::cos(yaw);
	ret.y = std::sin(yaw);
	
	return ret;
}
	
std::pair<std::size_t, std::size_t> Wallfollowing::getNearestPoints(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>& left_octree, pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>& right_octree, const pcl::PointXYZ& predicted_position){
	pcl::Indices left_indices;
	pcl::Indices right_indices;
	std::vector<float> squared_distances;
	
	left_octree.nearestKSearch(predicted_position, 1, left_indices, squared_distances);
	right_octree.nearestKSearch(predicted_position, 1, right_indices, squared_distances);
	
	if(left_indices.empty() || right_indices.empty()){
		RCLCPP_ERROR(this->get_logger(), "Failed to find nearest point");
		return std::make_pair(0, 0);
	}
	
	return std::make_pair(left_indices[0], right_indices[0]);
}

void Wallfollowing::addPathPoint(const geometry_msgs::msg::PoseStamped& new_pose, const geometry_msgs::msg::TransformStamped& transform_msg)
{
	//Transform pose from cloud_frame to base_frame
	geometry_msgs::msg::PoseStamped new_pose_transformed;
	try{
		tf2::doTransform(new_pose, new_pose_transformed, transform_msg);
	} catch (std::exception& e) {
		RCLCPP_ERROR(this->get_logger(), "%s", e.what());
		return;
	}
	
	//Delete all poses, that are not in front of their successor
	for(size_t i = 1; i < path.points.size(); ++i){
		if(!isPoseinFront(path.points[i - 1].pose, path.points[i].pose)){
			path.points.erase(path.points.begin() + i);
			i--;//Reduce i, so that the i stays the same after loop increment
		}
	}
	
	//Check if path is behind current vehicle position. If so, discard it
	if(isPoseinFront(path.points[0].pose, new_pose_transformed.pose)){
		//Delete all poses, that are not behind the new pose
		for(size_t i = 1; i < path.points.size(); ++i){
			if(!isPoseinFront(path.points[i].pose, new_pose_transformed.pose)){
				path.points.erase(path.points.begin() + i);
				i--;//Reduce i, so that the i stays the same after loop increment
			}
		}
		
		//If we reached the upper bound, remove element
		if(path.points.size() > 1 && path.points.size() >= static_cast<size_t>(max_path_points_ - 1)){
			path.points.erase(path.points.begin() + 1);
		}
		
		autoware_auto_planning_msgs::msg::PathPoint new_point;
		new_point.pose = new_pose_transformed.pose;
		new_point.longitudinal_velocity_mps = max_longitudinal_velocity_mps_;
		new_point.lateral_velocity_mps = max_lateral_velocity_mps_;
		new_point.heading_rate_rps = max_heading_rate_rps_;
		
		path.points.push_back(new_point);
	}
}

//Copied from https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
static void intersections(const boost::geometry::model::linestring<boost::geometry::model::d2::point_xy<float, boost::geometry::cs::cartesian>>& a, const boost::geometry::model::linestring<boost::geometry::model::d2::point_xy<float, boost::geometry::cs::cartesian>>& b, std::vector<boost::geometry::model::d2::point_xy<float, boost::geometry::cs::cartesian>>& intersection_points){
	constexpr double EPSILON = 1e-9;
	
	for(size_t i = 1; i < a.size(); ++i){
		const boost::geometry::model::d2::point_xy<float, boost::geometry::cs::cartesian>& a_start = a[i - 1];
		const boost::geometry::model::d2::point_xy<float, boost::geometry::cs::cartesian>& a_end = a[i];
		
		const float s1_x = a_end.x() - a_start.x();
		const float s1_y = a_end.y() - a_start.y();
		
		for(size_t j = 1; j < b.size(); ++j){
			const boost::geometry::model::d2::point_xy<float, boost::geometry::cs::cartesian>& b_start = b[j - 1];
			const boost::geometry::model::d2::point_xy<float, boost::geometry::cs::cartesian>& b_end = b[j];
			
			const float s2_x = b_end.x() - b_start.x();
			const float s2_y = b_end.y() - b_start.y();
			
			const float determinant = (-s2_x * s1_y + s1_x * s2_y);
			
			if(std::abs(determinant) >= EPSILON){
				const float s = (-s1_y * (a_start.x() - b_start.x()) + s1_x * (a_start.y() - b_start.y())) / determinant;
				const float t = ( s2_x * (a_start.y() - b_start.y()) - s2_y * (a_start.x() - b_start.x())) / determinant;

				if (s >= 0 && s <= 1 && t >= 0 && t <= 1){
					intersection_points.emplace_back(a_start.x() + (t * s1_x), a_start.y() + (t * s1_y));
				}
			}
		}
	}
}

//TODO: Maybe other way to calc bounds (like concave hull or just polyline or something)
void Wallfollowing::setPathBounds(pcl::PointCloud<pcl::PointXYZ>& left_cloud, pcl::PointCloud<pcl::PointXYZ>& right_cloud, const std::vector<geometry_msgs::msg::Point>& untransformed_path_point_positions, const geometry_msgs::msg::TransformStamped& transform_msg){
	tf2::Transform transform;
	tf2::fromMsg(transform_msg.transform, transform);
	
	std::vector<convex_hull::point> left_hull = getConvexHull2D(left_cloud);
	std::vector<convex_hull::point> right_hull = getConvexHull2D(right_cloud);
	
	//If path point is outside of path bounds, add additional bound points
	
	//First create hull around all drivable area
	std::vector<convex_hull::point> points(left_hull.size() + right_hull.size());
	std::copy(std::execution::par_unseq, left_hull.begin(), left_hull.end(), points.begin());
	std::copy(std::execution::par_unseq, right_hull.begin(), right_hull.end(), points.begin());
	std::vector<convex_hull::point> hull = getConvexHull2D(points);


	//create linestring path
	boost::geometry::model::linestring<boost::geometry::model::d2::point_xy<float, boost::geometry::cs::cartesian>> pathAsLinestring;
	
	
	//Then test if points + radius lie within. If not add bound points
	const double car_radius = std::max(
		this->vehicle_info_.front_overhang_m + this->vehicle_info_.wheel_base_m,
		std::max(
			this->vehicle_info_.rear_overhang_m,
			std::max(
				this->vehicle_info_.wheel_tread_m / 2.0 + this->vehicle_info_.left_overhang_m,
				this->vehicle_info_.wheel_tread_m / 2.0 + this->vehicle_info_.right_overhang_m
			)
		)
	);
	
	for(size_t i = 0; i < untransformed_path_point_positions.size(); ++i){
		const convex_hull::point point(untransformed_path_point_positions[i].x, untransformed_path_point_positions[i].y);
		pathAsLinestring.push_back(boost::geometry::model::d2::point_xy<float, boost::geometry::cs::cartesian>(untransformed_path_point_positions[i].x, untransformed_path_point_positions[i].y));
		for(size_t j = 0; j < hull.size(); ++j){
			const Line<convex_hull::point> line(hull[j], hull[(j + 1) % hull.size()]);
			
			if(GeometricFunctions::calcShortestDistanceToLine(point, line) < car_radius){
				//Calc line vector
				convex_hull::point difference;
				difference.x = hull[(j + 1) % hull.size()].x - hull[j].x;
				difference.y = hull[(j + 1) % hull.size()].y - hull[j].y;
				
				//Normalize
				difference.x /= line.length();
				difference.y /= line.length();
				
				//Get orthogonal vector in outer direction (assuming convex hull is counter-clock-wise ordered)
				convex_hull::point rotated;
				rotated.x = -difference.y;
				rotated.y = difference.x;
				
				//Calculate new point (definitly outside of radius)
				convex_hull::point new_point;
				new_point.x = point.x + rotated.x * (2.0 * car_radius);
				new_point.y = point.y + rotated.y * (2.0 * car_radius);
				
				hull.insert(hull.begin() + i, new_point);
				
				//Insert on correct side based on angle
				const double angle = std::atan2(new_point.y, new_point.x);
				if(angle >= 0.0){
					left_hull.push_back(new_point);
				}else{
					right_hull.push_back(new_point);
				}
				break;//Continue with next point
			}
		}
	}

	//Sort by angle around center (cw for left, ccw for right)
	//std::sort(std::execution::par_unseq, left_hull.begin(), left_hull.end(), [](const convex_hull::point& a, const convex_hull::point& b){
	//	const double angle_a = std::atan2(a.y, a.x);
	//	const double angle_b = std::atan2(b.y, b.x);
		
	//	return angle_a < angle_b;
	//});
	
	//std::sort(std::execution::par_unseq, right_hull.begin(), right_hull.end(), [](const convex_hull::point& a, const convex_hull::point& b){
	//	const double angle_a = -std::atan2(a.y, a.x);
	//	const double angle_b = -std::atan2(b.y, b.x);
		
	//	return angle_a < angle_b;
	//});

	left_hull = projectPointOnPath(left_hull, pathAsLinestring);
	right_hull = projectPointOnPath(right_hull, pathAsLinestring);
	
	//Only consider line on the inner side of the track

	/*
	boost::geometry::model::d2::point_xy<float, boost::geometry::cs::cartesian> car_position(0.0, 0.0);
	
	boost::geometry::model::linestring<boost::geometry::model::d2::point_xy<float, boost::geometry::cs::cartesian>> left_bound_poly;
	left_bound_poly.resize(left_hull.size());
	std::transform(std::execution::par_unseq, left_hull.begin(), left_hull.end(), left_bound_poly.begin(), [](const convex_hull::point& point){
		return boost::geometry::model::d2::point_xy<float, boost::geometry::cs::cartesian>(point.x, point.y);
	});
	boost::geometry::correct(left_bound_poly);
	
	boost::geometry::model::linestring<boost::geometry::model::d2::point_xy<float, boost::geometry::cs::cartesian>> right_bound_poly;
	right_bound_poly.resize(right_hull.size());
	std::transform(std::execution::par_unseq, right_hull.begin(), right_hull.end(), right_bound_poly.begin(), [](const convex_hull::point& point){
		return boost::geometry::model::d2::point_xy<float, boost::geometry::cs::cartesian>(point.x, point.y);
	});
	boost::geometry::correct(right_bound_poly);
	
	//Draw line from point to car. If it is intersected by the hull, remove the point
	left_hull.erase(std::remove_if(std::execution::seq, left_hull.begin(), left_hull.end(), [&car_position, &left_bound_poly](const convex_hull::point& point){
		const boost::geometry::model::d2::point_xy<float, boost::geometry::cs::cartesian> boost_point(point.x, point.y);
		
		boost::geometry::model::linestring<boost::geometry::model::d2::point_xy<float, boost::geometry::cs::cartesian>> line;
		line.push_back(car_position);
		line.push_back(boost_point);
		
		std::vector<boost::geometry::model::d2::point_xy<float, boost::geometry::cs::cartesian>> intersection_points;
		intersections(line, left_bound_poly, intersection_points);//FIXME: Boost returned wrong results (intersections that don't exist); Use boost::geometry::intersects if it works in future versions boost::geometry::intersection(line, left_bound_poly, intersection_points);
		
		//Cannot just check for size of intersection_points cause boost seems to return some points twice
		for(const boost::geometry::model::d2::point_xy<float, boost::geometry::cs::cartesian>& intersection_point : intersection_points){
			if(intersection_point.x() != boost_point.x() || intersection_point.y() != boost_point.y()){
				return true;
			}
		}
		return false;
	}), left_hull.end());
	
	right_hull.erase(std::remove_if(std::execution::par_unseq, right_hull.begin(), right_hull.end(), [&car_position, &right_bound_poly](const convex_hull::point& point){
		const boost::geometry::model::d2::point_xy<float, boost::geometry::cs::cartesian> boost_point(point.x, point.y);
		
		boost::geometry::model::linestring<boost::geometry::model::d2::point_xy<float, boost::geometry::cs::cartesian>> line;
		line.push_back(car_position);
		line.push_back(boost_point);
		
		std::vector<boost::geometry::model::d2::point_xy<float, boost::geometry::cs::cartesian>> intersection_points;
		intersections(line, right_bound_poly, intersection_points);//FIXME: Boost returned wrong results (intersections that don't exist); Use boost::geometry::intersects if it works in future versions boost::geometry::intersection(line, right_bound_poly, intersection_points);
		
		//Cannot just check for size of intersection_points cause boost seems to return some points twice
		for(const boost::geometry::model::d2::point_xy<float, boost::geometry::cs::cartesian>& intersection_point : intersection_points){
			if(intersection_point.x() != boost_point.x() || intersection_point.y() != boost_point.y()){
				return true;
			}
		}
		return false;
	}), right_hull.end());
	*/
	//Store into path
	path.left_bound.resize(left_hull.size());
	std::transform(std::execution::par_unseq, left_hull.begin(), left_hull.end(), path.left_bound.begin(), [&transform](const convex_hull::point& point){
		const tf2::Vector3 transformed_point = transform * tf2::Vector3(point.x, point.y, 0.0);
		
		geometry_msgs::msg::Point geometry_point;
		geometry_point.x = transformed_point.x();
		geometry_point.y = transformed_point.y();
		
		return geometry_point;
	});
	
	path.right_bound.resize(right_hull.size());
	std::transform(std::execution::par_unseq, right_hull.begin(), right_hull.end(), path.right_bound.begin(), [&transform](const convex_hull::point& point){
		const tf2::Vector3 transformed_point = transform * tf2::Vector3(point.x, point.y, 0.0);
		
		geometry_msgs::msg::Point geometry_point;
		geometry_point.x = transformed_point.x();
		geometry_point.y = transformed_point.y();
		
		return geometry_point;
	});
}

std::vector<convex_hull::point> Wallfollowing::projectPointOnPath(const std::vector<convex_hull::point>& hull, const boost::geometry::model::linestring<boost::geometry::model::d2::point_xy<float, boost::geometry::cs::cartesian>>& line)
{
	
	std::vector<convex_hull::point> final_points;

	const boost::geometry::model::d2::point_xy<float, boost::geometry::cs::cartesian>& first_segment_start = line[0];
	const boost::geometry::model::d2::point_xy<float, boost::geometry::cs::cartesian>& first_segment_end = line[1];
	boost::geometry::model::d2::point_xy<float, boost::geometry::cs::cartesian> start_end_vector = line[0];

	const boost::geometry::model::d2::point_xy<float, boost::geometry::cs::cartesian>& last_segment_start = line[line.size()-2];
	const boost::geometry::model::d2::point_xy<float, boost::geometry::cs::cartesian>& last_segment_end = line[line.size()-1];
	boost::geometry::model::d2::point_xy<float, boost::geometry::cs::cartesian> last_segment_start_end_vector = line[line.size()-2];

	std::vector<float> list_of_t_values_first_segment;
	std::vector<float> list_of_t_values_last_segment;
	boost::geometry::model::d2::point_xy<float, boost::geometry::cs::cartesian> tmp_hull_point;



	for(size_t j = 0; j < hull.size(); ++j){

		boost::geometry::set<0>(tmp_hull_point, hull[j].x);
		boost::geometry::set<1>(tmp_hull_point, hull[j].y);



		// paramaterized position d(t) = a + t * (b - a)
		boost::geometry::subtract_point(start_end_vector, first_segment_end);
		boost::geometry::subtract_point(last_segment_start_end_vector, last_segment_end);
		boost::geometry::subtract_point(tmp_hull_point, first_segment_start);
		boost::geometry::subtract_point(tmp_hull_point, last_segment_start);

		const float t_first_segment = boost::geometry::dot_product(tmp_hull_point, start_end_vector) / boost::geometry::dot_product(start_end_vector, start_end_vector);
		const float t_last_segment = boost::geometry::dot_product(tmp_hull_point, last_segment_start_end_vector) / boost::geometry::dot_product(last_segment_start_end_vector, last_segment_start_end_vector);
	
		list_of_t_values_first_segment.push_back(t_first_segment);
		list_of_t_values_last_segment.push_back(t_last_segment);

	}


	//Smallest element on first segment
	std::vector<float>::iterator min_iter = std::max_element(std::execution::par_unseq, list_of_t_values_first_segment.begin(), list_of_t_values_first_segment.end(), [](const float a, const float b){
		return a > b;
	});

	std::vector<float>::iterator max_iter = std::min_element(std::execution::par_unseq, list_of_t_values_last_segment.begin(), list_of_t_values_last_segment.end(), [](const float a, const float b){
		
		return a > b;
	});

	if(std::distance(min_iter, max_iter) < 0){
		//TODO: Copy min_iter to hull.end()
		std::copy(hull.begin() + std::distance(list_of_t_values_first_segment.begin(), min_iter), hull.end(), std::back_inserter(final_points));

		//TODO: Copy hull.begin() to max_iter
		std::copy(hull.begin(), hull.begin() + std::distance(list_of_t_values_last_segment.begin(), max_iter), std::back_inserter(final_points));

	}else{
		//TODO: Copy min_iter to max_iter
		std::copy(hull.begin() + std::distance(list_of_t_values_first_segment.begin(), min_iter), hull.begin() + std::distance(list_of_t_values_last_segment.begin(), max_iter), std::back_inserter(final_points));

	} 

	return final_points;

}


pcl::PointXYZ Wallfollowing::determinePredictedCarPosition()
{
	const autoware_auto_vehicle_msgs::msg::VelocityReport currentVelocityReport = this->velocityReport;//FIXME: Fetch atomically to ensure consistent access
	const double speed = std::sqrt(currentVelocityReport.lateral_velocity * currentVelocityReport.lateral_velocity + currentVelocityReport.longitudinal_velocity * currentVelocityReport.longitudinal_velocity);
	
	//If speed is 0.0 use min distance
	if(speed > stop_detection_speed_threshold_){
		//Calculate prediction distance and position
		rclcpp::Duration delta_time = (this->now() - currentVelocityReport.header.stamp) + rclcpp::Duration::from_seconds(prediction_time_);
		double prediction_distance = std::min(prediction_min_distance_ + speed * delta_time.seconds(), prediction_max_distance_);
		
		pcl::PointXYZ ret;
		ret.x = prediction_distance * std::max(0.0f, currentVelocityReport.longitudinal_velocity) / speed;
		ret.y = prediction_distance * currentVelocityReport.lateral_velocity / speed;
		
		return ret;
	}else{
		return pcl::PointXYZ(prediction_min_distance_, 0.0, 0.0);
	}
}

pcl::PointXYZ Wallfollowing::determineTargetPathPoint(
	const pcl::PointCloud<pcl::PointXYZ>& left_cloud, 
	const pcl::PointCloud<pcl::PointXYZ>& right_cloud, 
	const pcl::PointCloud<pcl::PointXYZ>& upper_cloud, 
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>& left_octree, 
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>& right_octree, 
	const pcl::PointXYZ& initial_predicted_position, 
	double epsilon
)
{
	//Get new position by bisection
	const double car_radius = std::max(
		this->vehicle_info_.front_overhang_m + this->vehicle_info_.wheel_base_m,
		std::max(
			this->vehicle_info_.rear_overhang_m,
			std::max(
				this->vehicle_info_.wheel_tread_m / 2.0 + this->vehicle_info_.left_overhang_m,
				this->vehicle_info_.wheel_tread_m / 2.0 + this->vehicle_info_.right_overhang_m
			)
		)
	);
	
    double distance = GeometricFunctions::distance(pcl::PointXYZ(), initial_predicted_position);
    double t = 0.5;
	double scale = 0.5;
	//TODO: We can exactly calculate the amount of iterations here. Maybe do that.
    while (distance * scale > epsilon)
    {
		const pcl::PointXYZ predicted_position(initial_predicted_position.x * t, initial_predicted_position.y * t, initial_predicted_position.z * t);
		const std::pair<std::size_t, std::size_t> nearest_points = getNearestPoints(left_octree, right_octree, predicted_position);
		const pcl::PointXYZ center_point = determineTrackCenter(left_cloud.at(nearest_points.first), right_cloud.at(nearest_points.second), pcl::PointXYZ(), max_yaw_deviation_);
		Line<pcl::PointXYZ> line(pcl::PointXYZ(), center_point);

        const bool too_close = (
			   lineTooCloseToPointcloud(line, right_cloud, car_radius)
			|| lineTooCloseToPointcloud(line, left_cloud, car_radius)
			|| lineTooCloseToPointcloud(line, upper_cloud, car_radius)//TODO: Needed? upper_cloud is always subset of another cloud
		);
		
		scale *= 0.5;
		t += (too_close ? -scale : scale);
    }

    return pcl::PointXYZ(initial_predicted_position.x * t, initial_predicted_position.y * t, initial_predicted_position.z * t);
}

//TODO:Currently we assume, that the given data is in a coordinate system where x+ is longitudinal_velocity if dorection of driving. We'll need to transform data accordingly to ensure this for different track->header.frame_id (the target frame is the frame of the velocity report currently)
void Wallfollowing::followWalls(const car_simulator_msgs::msg::Track::ConstSharedPtr track)
{
	//Convert message
	pcl::PointCloud<pcl::PointXYZ>::Ptr left_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	pcl::PointCloud<pcl::PointXYZ>::Ptr right_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	pcl::PointCloud<pcl::PointXYZ>::Ptr upper_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	
	pcl::fromROSMsg(track->left_cloud, *left_cloud);
	pcl::fromROSMsg(track->right_cloud, *right_cloud);
	pcl::fromROSMsg(track->upper_cloud, *upper_cloud);
	
	//TODO: Maybe reuse octree from wall detection somehow?
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> left_octree(octree_voxel_size);
	left_octree.setInputCloud(left_cloud);
	left_octree.addPointsFromInputCloud();
	
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> right_octree(octree_voxel_size);
	right_octree.setInputCloud(right_cloud);
	right_octree.addPointsFromInputCloud();
	
	//Determine predicted car position (based on speed)
	pcl::PointXYZ predicted_position = determinePredictedCarPosition();
	
	//Determine target position by bisection
	predicted_position = determineTargetPathPoint(*left_cloud, *right_cloud, *upper_cloud, left_octree, right_octree, predicted_position, target_collision_precision_);
	
	//Ensure point if far enough away
	//NOTE: Must happen before averaging to get the correct direction to move the point to (averaging changes direction cause it changes the y value only)
	const double predicted_position_distance = std::sqrt(predicted_position.x * predicted_position.x + predicted_position.y * predicted_position.y);
	const double predicted_position_new_distance = (std::max(predicted_position_distance, target_min_distance_) / predicted_position_distance);
	predicted_position.x *= predicted_position_new_distance;
	predicted_position.y *= predicted_position_new_distance;
	predicted_position.z *= predicted_position_new_distance;
	
	//Average over time
	//TODO: Maybe smarter averaging (also taking into account time and distance of average points)
	predicted_position_average = (predicted_position.y + predicted_position_average * prediction_average_weight_) / (prediction_average_weight_ + 1.0);
	const pcl::PointXYZ predicted_position_after_averaging(predicted_position.x, predicted_position_average, predicted_position.z);
	const double predicted_position_after_averaging_distance = std::sqrt(predicted_position_after_averaging.x * predicted_position_after_averaging.x + predicted_position_after_averaging.y * predicted_position_after_averaging.y);
	//Only apply averaging if this doesn't move the point too close to the vehicle
	if(predicted_position_after_averaging_distance >= target_min_distance_){
		predicted_position.y = predicted_position_average;
	}
	
	//Put point in center of track
	const std::pair<std::size_t, std::size_t> nearest_points = getNearestPoints(left_octree, right_octree, predicted_position);
	pcl::PointXYZ target_position = determineTrackCenter(left_cloud->at(nearest_points.first), right_cloud->at(nearest_points.second), pcl::PointXYZ(), max_yaw_deviation_);
	
	//If target_position is negative use predicted pos. If target_position is too near, move it farer away
	const double target_position_distance = std::sqrt(target_position.x * target_position.x + target_position.y * target_position.y);
	if(target_position.x < 0.0){
		target_position = predicted_position;
	}else if(target_position_distance < target_min_distance_){
		const double target_position_new_distance = (std::max(target_position_distance, target_min_distance_) / target_position_distance);
		target_position.x *= target_position_new_distance;
		target_position.y *= target_position_new_distance;
		target_position.z *= target_position_new_distance;
	}
	
	//Calculate orientation
	const pcl::PointXYZ direction = determineTrackDirection(left_cloud->at(nearest_points.first), right_cloud->at(nearest_points.second), pcl::PointXYZ(), max_yaw_deviation_);
	
	//Calculate transform into correct frame
	geometry_msgs::msg::TransformStamped transform_from_map_msg, transform_to_map_msg, transform_from_track_msg;
	try{
		transform_from_track_msg = tf_buffer_->lookupTransform(this->frame_id_, track->header.frame_id, tf2::TimePointZero);//FIXME: Check for matching timestamp tf2_ros::fromMsg(t_link.header.stamp));
		transform_from_map_msg = tf_buffer_->lookupTransform(track->header.frame_id, "map", tf2::TimePointZero);//FIXME: Check for matching timestamp tf2_ros::fromMsg(t_link.header.stamp));
		transform_to_map_msg = tf_buffer_->lookupTransform("map", this->frame_id_, tf2::TimePointZero);//FIXME: Check for matching timestamp tf2_ros::fromMsg(t_link.header.stamp));
	} catch (std::exception& e) {
		RCLCPP_ERROR(this->get_logger(), "%s", e.what());
		return;
	}
	
	tf2::Transform transform_from_map;
	tf2::fromMsg(transform_from_map_msg.transform, transform_from_map);
	
	//Create pose
	const tf2::Vector3 tf_direction(direction.x, direction.y, direction.z);
	tf2::Quaternion orientation_tf = shortestArcQuat(tf2::Vector3(1.0, 0.0, 0.0), tf_direction);
	
	geometry_msgs::msg::PoseStamped pose;
	pose.header = track->header;
	pose.pose.position.x = target_position.x;
	pose.pose.position.y = target_position.y;
	pose.pose.position.z = target_position.z;
	pose.pose.orientation = tf2::toMsg(orientation_tf);
	
	//Reset origin pose, so that all poses are in the same coordinate space
	path.points[0].pose = geometry_msgs::msg::Pose();
	
	
	//Recalculate old point poses
	//TODO: Actually those old points don't have much benefit. Maybe remove them or only use them for pose averaging
	for(size_t i = 1; i < path.points.size(); ++i){
		//Transform old position into track frame
		const tf2::Vector3 tf_transformed_last_pos = transform_from_map * last_transform_to_map * tf2::Vector3(path.points[i].pose.position.x, path.points[i].pose.position.y, path.points[i].pose.position.z);
		const pcl::PointXYZ transformed_last_pos(tf_transformed_last_pos.x(), tf_transformed_last_pos.y(), tf_transformed_last_pos.z());
		
		const pcl::PointXYZ previous_position(path.points[i - 1].pose.position.x, path.points[i - 1].pose.position.y, path.points[i - 1].pose.position.z);
		
		//Calculate new nearest center of track
		const std::pair<std::size_t, std::size_t> current_nearest_points = getNearestPoints(left_octree, right_octree, transformed_last_pos);
		const pcl::PointXYZ current_target_position = determineTrackCenter(left_cloud->at(current_nearest_points.first), right_cloud->at(current_nearest_points.second), previous_position, max_yaw_deviation_);
		
		//Calculate new orientation
		const pcl::PointXYZ current_direction = determineTrackDirection(left_cloud->at(current_nearest_points.first), right_cloud->at(current_nearest_points.second), previous_position, max_yaw_deviation_);
		
		//Update pose
		const tf2::Vector3 current_tf_direction(current_direction.x, current_direction.y, current_direction.z);
		tf2::Quaternion current_orientation_tf = shortestArcQuat(tf2::Vector3(1.0, 0.0, 0.0), current_tf_direction);
		
		path.points[i].pose.position.x = current_target_position.x;
		path.points[i].pose.position.y = current_target_position.y;
		path.points[i].pose.position.z = current_target_position.z;
		path.points[i].pose.orientation = tf2::toMsg(current_orientation_tf);
	}
	
	//Transform poses (including origin)
	std::vector<geometry_msgs::msg::Point> untransformed_path_point_positions(path.points.size());
	for(size_t i = 0; i < path.points.size(); ++i){
		untransformed_path_point_positions[i] = path.points[i].pose.position;
		
		geometry_msgs::msg::PoseStamped pose_transformed;
		pose_transformed.header = track->header;
		pose_transformed.pose = path.points[i].pose;
		try{
			tf2::doTransform(pose_transformed, pose_transformed, transform_from_track_msg);
		} catch (std::exception& e) {
			RCLCPP_ERROR(this->get_logger(), "%s", e.what());
			return;
		}
		path.points[i].pose = pose_transformed.pose;
	}
	
	//Store last_transform_to_map
	tf2::fromMsg(transform_to_map_msg.transform, last_transform_to_map);

	//Update and publish path
	this->addPathPoint(pose, transform_from_track_msg);
	this->setPathBounds(*left_cloud, *right_cloud, untransformed_path_point_positions, transform_from_track_msg);
	
	this->path.header.stamp = track->header.stamp;
	this->publishPath();
	
	if(enable_debug_topics_){
		geometry_msgs::msg::PoseArray poses;
		poses.header = this->path.header;
		poses.poses.resize(path.points.size());
		
		std::transform(std::execution::par_unseq, path.points.begin(), path.points.end(), poses.poses.begin(), [](const autoware_auto_planning_msgs::msg::PathPoint& point){
			return point.pose;
		});
		
		poses_pub_->publish(poses);
		
		
		pcl::PointCloud<pcl::PointXYZ> left_bound_cloud;
		left_bound_cloud.resize(path.left_bound.size());
		std::transform(std::execution::par_unseq, path.left_bound.begin(), path.left_bound.end(), left_bound_cloud.begin(), [](const geometry_msgs::msg::Point& point){
			return pcl::PointXYZ(point.x, point.y, point.z);
		});
		
		sensor_msgs::msg::PointCloud2 left_bound_cloud_msg;
		pcl::toROSMsg(left_bound_cloud, left_bound_cloud_msg);
		
		left_bound_cloud_msg.header = this->path.header;
		
		left_bound_pub_->publish(left_bound_cloud_msg);
		
		pcl::PointCloud<pcl::PointXYZ> right_bound_cloud;
		right_bound_cloud.resize(path.right_bound.size());
		std::transform(std::execution::par_unseq, path.right_bound.begin(), path.right_bound.end(), right_bound_cloud.begin(), [](const geometry_msgs::msg::Point& point){
			return pcl::PointXYZ(point.x, point.y, point.z);
		});
		
		sensor_msgs::msg::PointCloud2 right_bound_cloud_msg;
		pcl::toROSMsg(right_bound_cloud, right_bound_cloud_msg);
		
		right_bound_cloud_msg.header = this->path.header;
		
		right_bound_pub_->publish(right_bound_cloud_msg);
		
		geometry_msgs::msg::PoseStamped predicted_position_pose;
		predicted_position_pose.header = track->header;
		predicted_position_pose.pose.position.x = predicted_position.x;
		predicted_position_pose.pose.position.y = predicted_position.y;
		predicted_position_pose.pose.position.z = predicted_position.z;
		predicted_position_pose.pose.orientation = tf2::toMsg(orientation_tf);
		
		geometry_msgs::msg::PoseStamped predicted_position_pose_transformed;
		try{
			tf2::doTransform(predicted_position_pose, predicted_position_pose_transformed, transform_from_track_msg);
		} catch (std::exception& e) {
			RCLCPP_ERROR(this->get_logger(), "%s", e.what());
			return;
		}
		
		predicted_position_pub_->publish(predicted_position_pose_transformed);
	}
}

void Wallfollowing::publishPath()
{
	path_pub_->publish(this->path);
}

void Wallfollowing::callbackVelocityReport(const autoware_auto_vehicle_msgs::msg::VelocityReport::ConstSharedPtr velocityReport)
{
	this->velocityReport = *velocityReport;
} 

void Wallfollowing::callbackTrack(const car_simulator_msgs::msg::Track::ConstSharedPtr track_in)
{
	rclcpp::Time scan_time = track_in->header.stamp;
	if (scan_time - last_scan_time > rclcpp::Duration::from_seconds(min_scan_time_offset_))
	{
		followWalls(track_in);
	}
	last_scan_time = scan_time;
}

