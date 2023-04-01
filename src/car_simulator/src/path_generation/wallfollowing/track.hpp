#pragma once

#include "rclcpp/rclcpp.hpp"

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_pointcloud.h>

#include "car_simulator_msgs/msg/track.hpp"

#include "circle.hpp"

class TrackGenerator
{	
private:
    static pcl::IndicesConstPtr cropPointcloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, float minimum_x);
    static pcl::PointXYZ calcNearestPointToPoint(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, const pcl::PointXYZ& point);
    static bool isCurveEntryInFront(const pcl::PointXYZ& curve_entry_point, const pcl::PointXYZ& lowest_point, double threshold);
    static pcl::PointXYZ getCurveEntry(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);

public:
	TrackGenerator(){}

    static car_simulator_msgs::msg::Track generateTrack(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, pcl::IndicesConstPtr left_wall, pcl::IndicesConstPtr right_wall, double radius_propotions, double in_front_threshold, double upper_wall_offset, double sac_distance_to_model_threshold, int sac_max_iterations);
};