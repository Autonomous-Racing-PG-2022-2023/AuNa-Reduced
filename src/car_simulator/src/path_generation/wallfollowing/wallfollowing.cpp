#include "wallfollowing.hpp"

#include "pcl_conversions/pcl_conversions.h"

#include "circle.hpp"
#include "geometric_math.hpp"

//FIXME: Remove magic numbers. Make them parameters
Wallfollowing::Wallfollowing()
: Node("wallfollowing"), last_scan_time(this->now())
{
	/* setup parameters */
	frame_id_ = declare_parameter("frame_id", "base_link");
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
	
	safety_wall_distance_ = declare_parameter<double>("safety_wall_distance", 0.1);
	max_curve_circle_radius_ = declare_parameter<double>("max_curve_circle_radius", 1000.0);
	max_view_dist_min_ = declare_parameter<double>("max_view_dist_min", 1.0);
	max_view_dist_max_ = declare_parameter<double>("max_view_dist_max", 5.0);
	max_view_dist_radius_multiplier_ = declare_parameter<double>("max_view_dist_radius_multiplier", 0.3);
	prediction_min_distance_ = declare_parameter<double>("prediction_min_distance", 0.1);
	prediction_max_distance_ = declare_parameter<double>("prediction_max_distance", 5.0);
	prediction_time_ = declare_parameter<double>("prediction_time", 0.01);
	prediction_average_weight_ = declare_parameter<double>("prediction_average_weight", 5.0);
	target_min_distance_ = declare_parameter<double>("target_min_distance", 0.3);
	target_collision_precision_ = declare_parameter<double>("target_collision_precision", 0.01);
	
	RCLCPP_INFO_THROTTLE(
		get_logger(),
		*get_clock(),
		std::chrono::milliseconds(1000).count(),
		"frame_id: %s, min_scan_time_offset: %lf, max_path_points: %d, target_method: %d, safety_wall_distance: %lf, max_curve_circle_radius: %lf, "
		"max_view_dist_min: %lf, max_view_dist_max: %lf, max_view_dist_radius_multiplier: %lf, prediction_min_distance: %lf, "
		"prediction_max_distance: %lf, prediction_time: %lf, prediction_average_weight: %lf, target_min_distance: %lf, "
		"target_collision_precision: %lf",
		frame_id_.c_str(),
		min_scan_time_offset_,
		max_path_points_,
		target_method_,
		safety_wall_distance_,
		max_curve_circle_radius_,
		max_view_dist_min_,
		max_view_dist_max_,
		max_view_dist_radius_multiplier_,
		prediction_min_distance_,
		prediction_max_distance_,
		prediction_time_,
		prediction_average_weight_,
		target_min_distance_,
		target_collision_precision_
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

	path_pub_ = create_publisher<nav_msgs::msg::Path>(
		"~/output/path",
		rclcpp::QoS{1}
	);
	
	//Init path
	this->path.header.frame_id = frame_id_;
}

void Wallfollowing::addPathPoint(const geometry_msgs::msg::PoseStamped& new_pose)
{
	//Transform pose from cloud_frame to base_frame
	geometry_msgs::msg::PoseStamped new_pose_transformed;
	try{
		geometry_msgs::msg::TransformStamped velocity_transform_msg;
		velocity_transform_msg = tf_buffer_->lookupTransform(new_pose.header.frame_id, this->frame_id_, tf2::TimePointZero);//FIXME: Check for matching timestamp tf2_ros::fromMsg(t_link.header.stamp));
		tf2::doTransform(new_pose, new_pose_transformed, velocity_transform_msg);
	} catch (std::exception& e) {
		RCLCPP_ERROR(this->get_logger(), "%s", e.what());
		return;
	}
	
	//TODO:Check if path is behind current vehicle position. If so, discard it
	
	//If we reached the upper bound, remove element
	if(path.poses.size() >= static_cast<size_t>(max_path_points_)){
		path.poses.erase(path.poses.begin());
	}
	path.poses.push_back(new_pose_transformed);
}

geometry_msgs::msg::Point Wallfollowing::determinePredictedCarPosition(const std::string& target_frame)
{
	autoware_auto_vehicle_msgs::msg::VelocityReport currentVelocityReport = this->velocityReport;//Fetch to ensure consistent access
	
	geometry_msgs::msg::TransformStamped velocity_transform_msg;
	try{
		velocity_transform_msg = tf_buffer_->lookupTransform(currentVelocityReport.header.frame_id, target_frame, tf2::TimePointZero);//FIXME: Check for matching timestamp tf2_ros::fromMsg(t_link.header.stamp));
	} catch (std::exception& e) {
		RCLCPP_ERROR(this->get_logger(), "%s", e.what());
	}
	tf2::Transform velocity_tranmsform;
	tf2::fromMsg(velocity_transform_msg.transform, velocity_tranmsform);
	
	rclcpp::Duration delta_time = (this->now() - currentVelocityReport.header.stamp) + rclcpp::Duration::from_seconds(prediction_time_);
    double prediction_distance = std::min(prediction_min_distance_ + (currentVelocityReport.lateral_velocity + currentVelocityReport.longitudinal_velocity) * delta_time.seconds(), prediction_max_distance_);
	
	tf2::Vector3 predictedCarPositionTransfromed = velocity_tranmsform * (prediction_distance * tf2::Vector3(currentVelocityReport.lateral_velocity, currentVelocityReport.longitudinal_velocity, 0.0).normalize());
	
	geometry_msgs::msg::Point ret;
	ret.x = predictedCarPositionTransfromed.x();
	ret.y = predictedCarPositionTransfromed.y();
	
    return ret;
}

geometry_msgs::msg::Point Wallfollowing::determineTrackCenter(const Circle<geometry_msgs::msg::Point>& left_circle, const Circle<geometry_msgs::msg::Point>& right_circle, const geometry_msgs::msg::Point& predicted_position)
{
    geometry_msgs::msg::Point left_point = left_circle.getClosestPoint(predicted_position);
    geometry_msgs::msg::Point right_point = right_circle.getClosestPoint(predicted_position);
	
	geometry_msgs::msg::Point ret;
	ret.x = (left_point.x + right_point.x) / 2;
	ret.y = (left_point.y + right_point.y) / 2;
	
    return ret;
}

//TODO: Maybe accelerate with PCL structures
bool Wallfollowing::lineTooCloseToPointcloud(const pcl::PointXYZ& car_position, const Line<pcl::PointXYZ>& line, const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    double line_length = line.length();
    for (const pcl::PointXYZ& point : cloud)
    {
        if (
			   GeometricFunctions::distance(car_position, point) < line_length // +SAFETY_DISTANCE
            && GeometricFunctions::calcShortestDistanceToLine(point, line) < safety_wall_distance_
		){
            return true;
        }
    }
    return false;
}

std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point> Wallfollowing::determineTargetPathPoint(const pcl::PointXYZ& car_position, const Circle<geometry_msgs::msg::Point>& left_circle, const Circle<geometry_msgs::msg::Point>& right_circle, const pcl::PointCloud<pcl::PointXYZ>& left_cloud, const pcl::PointCloud<pcl::PointXYZ>& right_cloud, const pcl::PointCloud<pcl::PointXYZ>& upper_cloud, double min_distance, double max_distance, double epsilon)
{
    geometry_msgs::msg::Point predicted_position;
    geometry_msgs::msg::Point center_point;
    bool too_close = false;
    double distance = max_distance - min_distance;
    double start = min_distance;
    double end = max_distance;
    while (distance > epsilon)
    {
        predicted_position.y = start + distance / 2;
        center_point = determineTrackCenter(left_circle, right_circle, predicted_position);
        Line<pcl::PointXYZ> line(car_position, pcl::PointXYZ(center_point.x, center_point.y, center_point.z));

        too_close = (
			   lineTooCloseToPointcloud(car_position, line, right_cloud)
			|| lineTooCloseToPointcloud(car_position, line, left_cloud)
			|| lineTooCloseToPointcloud(car_position, line, upper_cloud)
		);

        distance = end - start;
        if (too_close)
        {
            end = predicted_position.y;
        }
        else
        {
            start = predicted_position.y;
        }
    }
    return std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(predicted_position, center_point);
}

geometry_msgs::msg::Point Wallfollowing::determineTargetCarPosition(const car_simulator_msgs::msg::Track::ConstSharedPtr track, const Circle<geometry_msgs::msg::Point>& left_circle, const Circle<geometry_msgs::msg::Point>& right_circle, const Circle<geometry_msgs::msg::Point>& upper_circle, const geometry_msgs::msg::Point& predicted_position)
{
    geometry_msgs::msg::Point left_point = left_circle.getClosestPoint(predicted_position);
    geometry_msgs::msg::Point right_point = right_circle.getClosestPoint(predicted_position);
    geometry_msgs::msg::Point central_point;
	central_point.x = (left_point.x + right_point.x) / 2;
	central_point.y = (left_point.y + right_point.y) / 2;
    double track_width = std::abs(left_point.x - right_point.x);
    //geometry_msgs::msg::Point left_car_position;
	//left_car_position.x = track->car_position.x;
	//left_car_position.y = track->car_position.y - track_width / 2;
	//geometry_msgs::msg::Point right_car_position;
	//right_car_position.x = track->car_position.x;
	//right_car_position.y = track->car_position.y + track_width / 2;

    geometry_msgs::msg::Point target_position = central_point;

    geometry_msgs::msg::Point upper_point;
    if (track->curve_type.data == car_simulator_msgs::msg::TrackCurveType::CURVE_TYPE_LEFT)
    {
        if (predicted_position.y > track->curve_entry.y)
        {
            upper_point = upper_circle.getClosestPoint(predicted_position);
            target_position.x = upper_point.x - track_width / 2;
			target_position.y = (left_point.y + right_point.y) / 2;
        }
    }
    else if (track->curve_type.data == car_simulator_msgs::msg::TrackCurveType::CURVE_TYPE_RIGHT)
    {
        if (predicted_position.y > track->curve_entry.y)
        {
            upper_point = upper_circle.getClosestPoint(predicted_position);
			target_position.x = upper_point.x + track_width / 2;
			target_position.y = (left_point.y + right_point.y) / 2;
        }
    }

    if (target_method_ == Wallfollowing::CIRCLE_TANGENTS)
    {
        std::vector<geometry_msgs::msg::Point> left_tangent_points;
        std::vector<geometry_msgs::msg::Point> right_tangent_points;
        // left curve
        if (left_circle.getCenter().x < 0 && right_circle.getCenter().x < 0 &&
            (left_circle.getRadius() < max_curve_circle_radius_ || right_circle.getRadius() < max_curve_circle_radius_))
        {
			Circle<geometry_msgs::msg::Point> safety_circle(
				left_circle.getCenter(),
				left_circle.getRadius() + safety_wall_distance_
			);
            left_tangent_points = safety_circle.calcTangents(track->car_position);
            if (left_tangent_points.size() > 0)
            {
                target_position = left_tangent_points.front();
            }
        }
        // right curve
        if (left_circle.getCenter().x > 0 && right_circle.getCenter().x > 0 &&
            (left_circle.getRadius() < max_curve_circle_radius_ || right_circle.getRadius() < max_curve_circle_radius_))
        {
			Circle<geometry_msgs::msg::Point> safety_circle(
				right_circle.getCenter(),
				right_circle.getRadius() + safety_wall_distance_
			);
            right_tangent_points = safety_circle.calcTangents(track->car_position);
            if (right_tangent_points.size() > 0)
            {
                target_position = right_tangent_points.front();
            }
        }
    }

    return target_position;
}

void Wallfollowing::followWalls(const car_simulator_msgs::msg::Track::ConstSharedPtr track)
{
	//Convert message
	const pcl::PointXYZ car_position(track->car_position.x, track->car_position.y, track->car_position.z);
	
	Circle<geometry_msgs::msg::Point> left_circle;
	Circle<geometry_msgs::msg::Point> right_circle;
	Circle<geometry_msgs::msg::Point> upper_circle;
	
	pcl::PointCloud<pcl::PointXYZ> left_cloud;
	pcl::PointCloud<pcl::PointXYZ> right_cloud;
	pcl::PointCloud<pcl::PointXYZ> upper_cloud;
	
	Circle<geometry_msgs::msg::Point>::fromROSMsg(track->left_circle, left_circle);
	Circle<geometry_msgs::msg::Point>::fromROSMsg(track->right_circle, right_circle);
	Circle<geometry_msgs::msg::Point>::fromROSMsg(track->upper_circle, upper_circle);
	
	pcl::fromROSMsg(track->left_cloud, left_cloud);
	pcl::fromROSMsg(track->right_cloud, right_cloud);
	pcl::fromROSMsg(track->upper_cloud, upper_cloud);
	
    geometry_msgs::msg::Point predicted_position;
    geometry_msgs::msg::Point target_position;
    if (target_method_ == Wallfollowing::CIRCLE_TANGENTS || target_method_ == Wallfollowing::TRACK_CENTER)
    {
        predicted_position = determinePredictedCarPosition(track->header.frame_id);
        target_position = determineTargetCarPosition(track, left_circle, right_circle, upper_circle, predicted_position);
    }
    else if (target_method_ == Wallfollowing::CENTER_PATH)
    {
        double radius = std::min(right_circle.getRadius(), left_circle.getRadius());
        double max_view_dist = std::max(max_view_dist_min_, radius * max_view_dist_radius_multiplier_);
        max_view_dist = std::min(max_view_dist, max_view_dist_max_);
        std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point> predicted_target_position = determineTargetPathPoint(car_position, left_circle, right_circle, left_cloud, right_cloud, upper_cloud, target_min_distance_, max_view_dist, target_collision_precision_);
        predicted_position = predicted_target_position.first;
        target_position = predicted_target_position.second;
        predicted_position_average = (predicted_position.y + predicted_position_average * prediction_average_weight_) / (prediction_average_weight_ + 1.0);
        predicted_position.y = predicted_position_average;
        target_position = determineTrackCenter(left_circle, right_circle, predicted_position);
    }
	
	geometry_msgs::msg::PoseStamped pose;
	pose.header = track->header;
	pose.pose.position = target_position;
	//TODO:pose.pose.orientation = ;

	this->addPathPoint(pose);
	this->publishPath();
}

void Wallfollowing::publishPath()
{
	this->path.header.stamp = this->path.poses[this->path.poses.size()].header.stamp;
	path_pub_->publish(this->path);
}

void Wallfollowing::callbackVelocityReport(const autoware_auto_vehicle_msgs::msg::VelocityReport::ConstSharedPtr velocityReport)
{
	this->velocityReport = *velocityReport;
} 

void Wallfollowing::callbackTrack(const car_simulator_msgs::msg::Track::ConstSharedPtr track_in)
{
	rclcpp::Time scan_time = track_in->header.stamp;
	if (scan_time - last_scan_time > rclcpp::Duration::from_seconds(min_scan_time_offset_) && scan_time > last_scan_time)
	{
		followWalls(track_in);
	}
	last_scan_time = scan_time;
}

