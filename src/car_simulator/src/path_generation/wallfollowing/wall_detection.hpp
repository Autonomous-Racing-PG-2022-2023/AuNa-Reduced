#pragma once

#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "car_simulator_msgs/msg/track.hpp"

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

	/* publishers */
	rclcpp::Publisher<car_simulator_msgs::msg::Track>::SharedPtr track_pub_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacles_pub_;

	/* ros param */
	double voxel_size_;
	int min_points_per_voxel_;
	bool filter_by_min_score_enabled_;
	double filter_by_min_score_;
	bool sor_enabled_;
	double sor_mean_k_;
	double sor_stddev_mul_thresh_;
	
	int minimum_points_;
	double epsilon_;
	
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

	/* callbacks */
	void callbackPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr point_cloud_in);
	
	/*  functions */
	std::pair<pcl::Indices, pcl::Indices> addClustersOnRegression(const pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr cloud,  const std::unordered_map<uint32_t, pcl::IndicesPtr>& mapClusters, const std::unordered_set<uint32_t>& inputIgnoreIDs, const pcl::IndicesConstPtr leftWall, const pcl::IndicesConstPtr rightWall);
	int64_t findLargestCluster(const std::unordered_map<uint32_t, pcl::IndicesPtr>& clusters, uint32_t ignoreID);
	std::pair<int64_t, int64_t> determineWallIDs(const pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr cloud, const std::unordered_map<uint32_t, pcl::IndicesPtr>& mapToCheck, double radius);
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBL> boxing(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZRGBL>::Ptr& cloud_out);
	void classifyVoxels(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr& cloud_in, const pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBL>& octree, pcl::PointCloud<pcl::PointXYZRGBL>::Ptr& cloud_out);
	void publishObstacles(const pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr& input_cloud, const pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr& cluster_cloud, pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBL>& octree, const std::unordered_map<uint32_t, pcl::IndicesPtr>& mapClusters, const std::unordered_set<uint32_t>& ignoreIDs);
	void publishTrack(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr& input_cloud, const pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr& cluster_cloud, pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBL>& octree, const pcl::IndicesConstPtr wallLeft, const pcl::IndicesConstPtr wallRight);
	
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
