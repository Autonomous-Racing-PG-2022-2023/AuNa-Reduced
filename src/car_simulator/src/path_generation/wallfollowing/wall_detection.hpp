#pragma once

#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "car_simulator_msgs/msg/track.hpp"

#include "tf2/exceptions.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>

#include "circle.hpp"

static const uint WALL_DETECTION_WALL_ID_LEFT = 0;
static const uint WALL_DETECTION_WALL_ID_RIGHT = 1;

class WallDetection : public rclcpp::Node
{
public:
	WallDetection();
private:
	/* subscribers */
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
	
	std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

	/* publishers */
	rclcpp::Publisher<car_simulator_msgs::msg::Track>::SharedPtr track_pub_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacles_pub_;
	
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxels_pub_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clusters_pub_;

	/* ros param */
	bool enable_debug_topics_;
	
	double voxel_size_;
	int min_points_per_voxel_;
	bool filter_by_min_score_enabled_;
	double filter_by_min_score_;
	bool sor_enabled_;
	double sor_mean_k_;
	double sor_stddev_mul_thresh_;
	
	double preserved_voxels_radius_;
	
	int db_search_minimum_points_;
	double db_search_radius_;
	
	double wall_radius_;
	bool use_prediction_for_walls_;
	bool use_prediction_for_obstacles_;
	double distance_threshold_;
	int score_threshold_;
	double minimum_confidence_;
	
	int sac_max_iterations_;
	double sac_distance_to_model_threshold_;
	
	double track_radius_propotions_;
	double track_in_front_threshold_;
	double track_upper_wall_offset_;
	
	/* data */
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr preserved_voxels;
	tf2::Transform last_transform_to_map;

	/* callbacks */
	void callbackPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr point_cloud_in);
	
	/*  functions */
	bool voxelSearch(pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBL>& octree, const pcl::PointXYZRGBL& voxel, pcl::Indices& points);
	
	std::pair<pcl::Indices, pcl::Indices> wallextensionByRadius(const pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr cloud,  const std::unordered_map<uint32_t, pcl::IndicesPtr>& mapClusters, const std::unordered_set<uint32_t>& inputIgnoreIDs,int margin);
	pcl::PointXYZRGBL getTrackDirectionByRadius(const pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr cloud, const pcl::PointXYZRGBL cutoff, int margin);
	pcl::PointXYZRGBL getTrackDirection(const pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr cloud);
	int64_t findLargestCluster(const std::unordered_map<uint32_t, pcl::IndicesPtr>& clusters, uint32_t ignoreID);
	std::pair<int64_t, int64_t> determineWallIDs(const pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr cloud, const std::unordered_map<uint32_t, pcl::IndicesPtr>& mapToCheck, const pcl::PointXYZRGBL& track_direction, double radius);
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBL> boxing(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZRGBL>::Ptr& cloud_out);
	void classifyVoxels(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZRGBL>::Ptr& cloud_out);
	void preserveVoxel(const pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr& cloud);
	void publishObstacles(const pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr& input_cloud, const pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr& cluster_cloud, pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBL>& octree, const std::unordered_map<uint32_t, pcl::IndicesPtr>& mapClusters, const std::unordered_set<uint32_t>& ignoreIDs);
	void publishTrack(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr& input_cloud, const pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr& cluster_cloud, pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBL>& octree, const pcl::IndicesConstPtr wallLeft, const pcl::IndicesConstPtr wallRight);
	void publishPointCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher, const pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr& input_cloud);
	
	/*
    static uint64_t get_voxel_id(float x, float y, float z)
    {
        uint64_t voxel_id = 0;
        union {
            float floaty;
            uint32_t inty;
        } float_uint;

        float_uint.floaty = x;
        voxel_id |= static_cast<uint64_t>(float_uint.inty) << 32;
        float_uint.floaty = y;
        voxel_id |= static_cast<uint64_t>(float_uint.inty) << 0;
        // float_uint.floaty = z;
        // voxel_id |= float_uint.inty;
        return voxel_id;
    }
*/
};
