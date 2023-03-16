#include "wall_detection.hpp"

#include <algorithm>
#include <execution>

#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "pcl_conversions/pcl_conversions.h"

#include "dbscan.hpp"
#include "geometric_math.hpp"
#include "fit_walls.hpp"
#include "track.hpp"

WallDetection::WallDetection()
    : Node("walldetection")
{
	/* setup parameters */
	voxel_size_ = declare_parameter<double>("voxel_size", 0.15);
	min_points_per_voxel_ = declare_parameter<int>("min_points_per_voxel", 0);
	//lidar_percentage_ = declare_parameter<double>("lidar_percentage", 0.5);
	filter_by_min_score_enabled_ = declare_parameter<bool>("filter_by_min_score_enabled", true);
	filter_by_min_score_ = declare_parameter<double>("filter_by_min_score", 0.002);
	sor_enabled_ = declare_parameter<bool>("sor_enabled", false);
	sor_mean_k_ = declare_parameter<double>("sor_mean_k", 2.0);
	sor_stddev_mul_thresh_ = declare_parameter<double>("sor_stddev_mul_thresh", 3.0);
	
	minimum_points_ = declare_parameter<int>("minimum_points", 4);
	epsilon_ = declare_parameter<double>("epsilon", 0.47);
	
	wall_radius_ = declare_parameter<double>("wall_radius", 0.5);
	use_prediction_for_walls_ = declare_parameter<bool>("use_prediction_for_walls", true);
	use_prediction_for_obstacles_ = declare_parameter<bool>("use_prediction_for_obstacles", true);
	distance_threshold_ = declare_parameter<double>("distance_threshold", 0.4);
	score_threshold_ = declare_parameter<int>("score_threshold", 3);
	minimum_confidence_ = declare_parameter<double>("minimum_confidence", 0.3);
	
	sac_max_iterations_ = declare_parameter<int>("sac_max_iterations", 99);
	sac_distance_to_model_threshold_ = declare_parameter<double>("sac_distance_to_model_threshold", 0.1);
	
	track_radius_propotions_ = declare_parameter<double>("track_radius_propotions", 1.2);
	track_in_front_threshold_ = declare_parameter<double>("track_in_front_threshold", 1.0);
	track_upper_wall_offset_ = declare_parameter<double>("track_upper_wall_offset", 1.5);
	
	RCLCPP_INFO_THROTTLE(
		get_logger(),
		*get_clock(),
		std::chrono::milliseconds(1000).count(),
		"voxel_size_: %lf, min_points_per_voxel: %d, filter_by_min_score_enabled: %s, filter_by_min_score: %lf, sor_enabled: %s, "
		"sor_mean_k: %lf, sor_stddev_mul_thresh: %lf, minimum_points: %d, epsilon: %lf, wall_radius: %lf, use_prediction_for_walls: %s, "
		"use_prediction_for_obstacles: %s, distance_threshold: %lf, score_threshold: %d, minimum_confidence: %lf, sac_max_iterations: %d, "
		"sac_distance_to_model_threshold: %lf, track_radius_propotions: %lf, track_in_front_threshold: %lf, track_upper_wall_offset: %lf",
		voxel_size_,
		min_points_per_voxel_,
		(filter_by_min_score_enabled_ ? "true" : "false"),
		filter_by_min_score_,
		(sor_enabled_ ? "true" : "false"),
		sor_mean_k_,
		sor_stddev_mul_thresh_,
		minimum_points_,
		epsilon_,
		wall_radius_,
		(use_prediction_for_walls_ ? "true" : "false"),
		(use_prediction_for_obstacles_ ? "true" : "false"),
		distance_threshold_,
		score_threshold_,
		minimum_confidence_,
		sac_max_iterations_,
		sac_distance_to_model_threshold_,
		track_radius_propotions_,
		track_in_front_threshold_,
		track_upper_wall_offset_
	);
	
	/* subscribers */
	using std::placeholders::_1;

	point_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
		"~/input/point_cloud",
		rclcpp::QoS{1},
		std::bind(&WallDetection::callbackPointCloud, this, _1)
	);

	/* publisher */

	track_pub_ = create_publisher<car_simulator_msgs::msg::Track>(
		"~/output/track",
		rclcpp::QoS{1}
	);
	
	obstacles_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
		"~/output/obstacles",
		rclcpp::QoS{1}
	);
}

std::pair<pcl::Indices, pcl::Indices> WallDetection::addClustersOnRegression(
	const pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr cloud,
    const std::unordered_map<uint32_t, pcl::IndicesPtr>& mapClusters,
	const std::unordered_set<uint32_t>& inputIgnoreIDs,
    const pcl::IndicesConstPtr leftWall,
	const pcl::IndicesConstPtr rightWall
)
{
	Circle<pcl::PointXYZRGBL> leftCircle = FitWalls::fitWall<pcl::PointXYZRGBL>(cloud, leftWall, sac_distance_to_model_threshold_, sac_max_iterations_);
	Circle<pcl::PointXYZRGBL> rightCircle = FitWalls::fitWall<pcl::PointXYZRGBL>(cloud, rightWall, sac_distance_to_model_threshold_, sac_max_iterations_);

    std::pair<pcl::Indices, pcl::Indices> additional_wall_ids;

    for (auto cluster_wall : mapClusters)
    {
        const uint32_t clusterID = cluster_wall.first;
		
		//Skip clusters already handled
        if (inputIgnoreIDs.find(clusterID) != inputIgnoreIDs.end()){
            continue;
		}

        const pcl::IndicesConstPtr cluster = cluster_wall.second;

		//Determine score
        int32_t leftScore = 0, rightScore = 0;
        for (size_t i = 0; i < cluster->size(); i++)
        {
            pcl::PointXYZRGBL p = cloud->at(cluster->at(i));

            if (leftCircle.getDistance(p) < distance_threshold_)
                leftScore++;

            if (rightCircle.getDistance(p) < distance_threshold_)
                rightScore++;
        }

		//If score is high enough, add clusters to wall clusters
        const double confidence = fabsf(1 - (static_cast<double>(leftScore + 1) / static_cast<double>(rightScore + 1)));
        if (confidence > minimum_confidence_ && (leftScore > score_threshold_ || rightScore > score_threshold_))
        {
            if (leftScore > rightScore)
            {
                additional_wall_ids.first.push_back(clusterID);
            }
            else if (rightScore > leftScore)
            {
                additional_wall_ids.second.push_back(clusterID);
            }
        }
    }

    return additional_wall_ids;
}

int64_t WallDetection::findLargestCluster(
	const std::unordered_map<uint32_t, pcl::IndicesPtr>& clusters,
    uint32_t ignoreID
)
{
    uint32_t largestClusterID = (std::max_element(clusters.begin(), clusters.end(), [&ignoreID](const std::pair<uint32_t, pcl::IndicesPtr>& a, const std::pair<uint32_t, pcl::IndicesPtr>& b){
		//IgnoredID is always smaller, so that it will not be the largest elemnt execpt if it is the only element
		return (
			(a.first == ignoreID)
			||(
				(b.first != ignoreID)
				&& (a.second->size() < b.second->size())
			)
		);
	}))->first;

    return largestClusterID;
}

std::pair<int64_t, int64_t> WallDetection::determineWallIDs(
	const pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr cloud,
    const std::unordered_map<uint32_t, pcl::IndicesPtr>& mapToCheck,
	double radius
)
{
    double maxLeft = 0;
    double minRight = 0;
    int64_t maxLeftID = -1;
    int64_t minRightID = -1;

    for (auto itr = mapToCheck.begin(); itr != mapToCheck.end(); ++itr)
    {
        for (size_t i = 0; i < itr->second->size(); ++i)
        {
			const pcl::PointXYZRGBL& point = cloud->at(itr->second->at(i));
			
            if (point.y < 0){
                continue;
			}
            if ((point.x > maxLeft) && (point.y <= radius))
            {
                maxLeft = point.x;
                maxLeftID = point.label;
            }
            if ((point.x < minRight) && (point.y <= radius))
            {
                minRight = point.x;
                minRightID = point.label;
            }
        }
    }

    if (maxLeftID == -1 && minRightID != -1)
    {
        // found a cluster for right but not for left. let's just choose the largest one for the left.
        maxLeftID = findLargestCluster(mapToCheck, minRightID);
    }
    else if (minRightID == -1 && maxLeftID != -1)
    {
        // same but reverse
        minRightID = findLargestCluster(mapToCheck, maxLeftID);
    }

    return std::pair<int64_t, int64_t>(minRightID, maxLeftID);
}

pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBL> WallDetection::boxing(
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr& cloud_in,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr& cloud_out
){
	//Remove outliers
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr output_cloud = cloud_in;
    if (sor_enabled_)
    {
		pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_sor_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZRGBL>>();
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBL> sor;
        sor.setInputCloud(cloud_in);
        sor.setMeanK(sor_mean_k_);
        sor.setStddevMulThresh(sor_stddev_mul_thresh_);
        sor.filter(*cloud_sor_ptr);
		output_cloud = cloud_sor_ptr;
    }
	
	//Initialize points as noise
	std::for_each(std::execution::par, output_cloud->begin(), output_cloud->end(), [](pcl::PointXYZRGBL& point){
		point.label = DBSCAN::NOISE;
	});
	
	//TODO: Only use a percentage of points based on lidar_percentage?
	
	//Downsample data, by generating link from voxels to original data

	//Octree for spatial optimization
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBL> octree(voxel_size_);
	octree.setInputCloud(output_cloud);
	octree.addPointsFromInputCloud();
	
	//Get largest intensity
	const pcl::octree::OctreeLeafNode<pcl::octree::OctreeContainerPointIndices>* largest_intensity_node = static_cast<const pcl::octree::OctreeLeafNode<pcl::octree::OctreeContainerPointIndices>*>(*(std::max_element(std::execution::par, octree.leaf_depth_begin(), octree.leaf_depth_end(), [this](const pcl::octree::OctreeNode* a, const pcl::octree::OctreeNode* b){
		return (static_cast<const pcl::octree::OctreeLeafNode<pcl::octree::OctreeContainerPointIndices>*>(a)->getContainer().getSize() < static_cast<const pcl::octree::OctreeLeafNode<pcl::octree::OctreeContainerPointIndices>*>(b)->getContainer().getSize());
	})));
	const float largest_intensity = static_cast<float>(largest_intensity_node->getContainer().getSize());
	
	//Discard voxels by marking them as unclassified
	std::for_each(std::execution::par, octree.leaf_depth_begin(), octree.leaf_depth_end(), [this, &largest_intensity, &output_cloud, &cloud_out](pcl::octree::OctreeNode* node){
		auto points = static_cast<const pcl::octree::OctreeLeafNode<pcl::octree::OctreeContainerPointIndices>*>(node)->getContainer();
		
		//If normalized intensity is lower than filter_by_min_score_ discard voxel, otherwise store it in new cloud
		if(!filter_by_min_score_enabled_ || (static_cast<float>(points.getSize()) / largest_intensity) >= filter_by_min_score_){
			const size_t index = points.getPointIndex();
			
			//Add voxel to cloud_out cloud
			pcl::PointXYZRGBL voxel;
			voxel.x = output_cloud->at(index).x;
			voxel.y = output_cloud->at(index).y;
			voxel.z = output_cloud->at(index).z;
			voxel.label = DBSCAN::UNCLASSIFIED;
			cloud_out->emplace_back(voxel);
		}
	});
	
	//Copy metadata
	cloud_out->header = cloud_in->header;

	return octree;//No std::move to allow copy elision

/*
    // now quantize this cloud for our system. this could probably also be implemented as a proper filter

    float largest_intensity = 0;

	//TODO: Maybe use differrent data stracture (2D/3D grid or hash structure)
    std::unordered_map<uint64_t, std::vector<size_t>> quantized_cloud_voxel_map;

	//We only analyze a certain amount of voxels
    size_t step_size = 1.0 / lidar_percentage_;

	if(step_size > 0){
		for (size_t i = 0; i < output_cloud->points.size(); i += step_size)
		{
			pcl::PointXYZ& point = output_cloud->at(i);
			float x = point.x - remainderf(point.x, voxel_size_);
			float y = point.y - remainderf(point.y, voxel_size_);
			float z = point.z; // - remainderf(point.z, m_voxel_size);

			// Search the voxel we want to increment
			// If we found the voxel, increase it's intensity (count of points in  that voxel)
			// Otherwise add a new voxel to our map
			uint64_t voxel_id = get_voxel_id(x, y, z);
			auto foundPair = quantized_cloud_voxel_map.find(voxel_id);
			if (foundPair != quantized_cloud_voxel_map.end())
			{
				pcl::PointXYZRGBL* foundPoint = &(foundPair->second);
				foundPoint->label++;

				largest_intensity = std::max(largest_intensity, foundPoint->label);
			}
			else
			{
				pcl::PointXYZRGBL point;
				point.x = x;
				point.y = y;
				point.z = z;
				point.label = 1;
				largest_intensity = std::max(largest_intensity, 1);
				quantized_cloud_voxel_map[voxel_id] = point;
			}
		}
	}
	
	//Filter out values with too low intensity
	if (m_filter_by_min_score_enabled)
	{
		//TODO:Use remove_if
		//TODO: Maybe directly remove on coppy with suitable function
		auto it = quantized_cloud_voxel_map.begin();
		while(it != quantized_cloud_voxel_map.end())
		{
			pcl::PointXYZRGBL point = it.second;
			if ((point.label / largest_intensity) < m_filter_by_min_score){
                it = quantized_cloud_voxel_map.erase(it);
			}else{
				it++;
			}
		}
	}

	//Create output point cloud
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr quantized_cloud(new pcl::PointCloud<pcl::PointXYZRGBL>);
	quantized_cloud->resize(quantized_cloud_voxel_map.size());
   
	//Copy data from map to point cloud
	std::transform(std::execution::par, quantized_cloud_voxel_map.begin(), quantized_cloud_voxel_map.end(), quantized_cloud.begin(), [](const std::unordered_map<uint64_t, pcl::PointXYZRGBL>::value_type value){
		pcl::PointXYZRGBL point = value.second;
		point.label = (point.label / largest_intensity) * static_cast<float>(std::numeric_limits<uint32_t>::max()); // uint32 max value
		return point;
	});
	
	//Copy header
	quantized_cloud->header = output_cloud->header;
*/
}

void WallDetection::classifyVoxels(
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr& cloud_in,
	const pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBL>& octree,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr& cloud_out
){
    DBSCAN ds(octree, cloud_in, minimum_points_, epsilon_ * epsilon_);
    ds.run();
	
	pcl::ConditionAnd<pcl::PointXYZRGBL>::Ptr filter_condition_ptr = std::make_shared<pcl::ConditionAnd<pcl::PointXYZRGBL>>();
	pcl::FieldComparison<pcl::PointXYZRGBL>::Ptr field_comparison_ptr = std::make_shared<pcl::FieldComparison<pcl::PointXYZRGBL>>("label", pcl::ComparisonOps::EQ, DBSCAN::NOISE);
	
	filter_condition_ptr->addComparison(field_comparison_ptr);
	
	pcl::ConditionalRemoval<pcl::PointXYZRGBL> conditional_removal;
	conditional_removal.setCondition(filter_condition_ptr);
	
	conditional_removal.setInputCloud(cloud_in);
	conditional_removal.filter(*cloud_out);

/*
	//Remove noise
	//TODO: Use remove_if
	auto it = cloud.begin();
	while(it != cloud.end())
	{
		if (it->label == NOISE){
			it = cloud.erase(it);
		}else{
			it++;
		}
	}
	*/
}

void WallDetection::publishObstacles(
	const pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr& input_cloud,
	const pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr& cluster_cloud,
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBL>& octree,
	const std::unordered_map<uint32_t, pcl::IndicesPtr>& mapClusters,
    const std::unordered_set<uint32_t>& ignoreIDs
)
{
	pcl::IndicesPtr output_cloud_indices = std::make_shared<pcl::Indices>();
	
	//Put all not ignored points into output cloud
	std::for_each(std::execution::seq, mapClusters.begin(), mapClusters.end(), [this, &ignoreIDs, &octree, &input_cloud, &cluster_cloud, &output_cloud_indices](const std::pair<uint32_t, pcl::IndicesPtr>& cluster){
		//Only handle clusters not ignored
		if(ignoreIDs.find(cluster.first) == ignoreIDs.end()){
			//Handle all voxels in cluster
			std::for_each(std::execution::seq, cluster.second->begin(), cluster.second->end(), [this, &octree, &input_cloud, &cluster_cloud, &output_cloud_indices](const size_t voxel_id){
				const pcl::PointXYZRGBL& voxel = cluster_cloud->at(voxel_id);
				
				pcl::Indices points;
				if(!octree.voxelSearch(voxel, points)){
					RCLCPP_ERROR(this->get_logger(), "Voxel not found");
				}
				//Add all points associated with voxel
				output_cloud_indices->insert(output_cloud_indices->end(), points.begin(), points.end());
			});
		}
	});
	
	//Extract points
	pcl::PointCloud<pcl::PointXYZRGBL> output_cloud;
	pcl::ExtractIndices<pcl::PointXYZRGBL> filter(false);
	filter.setIndices(output_cloud_indices);
	
	filter.setInputCloud(input_cloud);
	filter.filter(output_cloud);
	
	// Convert to ROS data type
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(output_cloud, output);

    // Publish the data
    this->obstacles_pub_->publish(output);
}

void WallDetection::publishTrack(
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr& input_cloud,
	const pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr& cluster_cloud,
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBL>& octree,
	const pcl::IndicesConstPtr wallLeft,
	const pcl::IndicesConstPtr wallRight
)
{
	//Extract voxels for walls from cluster_cloud
	pcl::PointCloud<pcl::PointXYZRGBL> cloud_left;
	pcl::PointCloud<pcl::PointXYZRGBL> cloud_right;
	
	pcl::ExtractIndices<pcl::PointXYZRGBL> extract_indices_left(false);
	extract_indices_left.setInputCloud(cluster_cloud);
	
	extract_indices_left.setIndices(wallLeft);
	extract_indices_left.filter(cloud_left);
	
	pcl::ExtractIndices<pcl::PointXYZRGBL> extract_indices_right(false);
	extract_indices_right.setInputCloud(cluster_cloud);
	
	extract_indices_right.setIndices(wallRight);
	extract_indices_right.filter(cloud_right);
	
	pcl::IndicesPtr left_cloud_indices = std::make_shared<pcl::Indices>();
	pcl::IndicesPtr right_cloud_indices = std::make_shared<pcl::Indices>();
	
	//Put all wall points into output cloud
	std::for_each(std::execution::seq, cloud_left.begin(), cloud_left.end(), [this, &octree, &input_cloud, &left_cloud_indices](const pcl::PointXYZRGBL& voxel){
		pcl::Indices points;
		if(!octree.voxelSearch(voxel, points)){
			//FIXME: Why is the voxel not found?
			//RCLCPP_ERROR(this->get_logger(), "Voxel not found");
		}
		
		//Set label for all points
		std::for_each(std::execution::par, points.begin(), points.end(), [&input_cloud](size_t index){
			input_cloud->at(index).label = WALL_DETECTION_WALL_ID_LEFT;
		});
		
		//Add all points associated with voxel
		left_cloud_indices->insert(left_cloud_indices->end(), points.begin(), points.end());
	});
	std::for_each(std::execution::seq, cloud_right.begin(), cloud_right.end(), [this, &octree, &input_cloud, &right_cloud_indices](const pcl::PointXYZRGBL& voxel){
		pcl::Indices points;
		if(!octree.voxelSearch(voxel, points)){
			//FIXME: Why is the voxel not found?
			//RCLCPP_ERROR(this->get_logger(), "Voxel not found");
		}
		
		//Set label for all points
		std::for_each(std::execution::par, points.begin(), points.end(), [&input_cloud](size_t index){
			input_cloud->at(index).label = WALL_DETECTION_WALL_ID_RIGHT;
		});
		
		//Add all points associated with voxel
		right_cloud_indices->insert(right_cloud_indices->end(), points.begin(), points.end());
	});
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_xyz = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	pcl::copyPointCloud(*input_cloud, *input_cloud_xyz);
	
	car_simulator_msgs::msg::Track track = TrackGenerator::generateTrack(input_cloud_xyz, left_cloud_indices, right_cloud_indices, track_radius_propotions_, track_in_front_threshold_, track_upper_wall_offset_, sac_distance_to_model_threshold_, sac_max_iterations_);

    // Publish the data
    this->track_pub_->publish(track);
}

void WallDetection::callbackPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr point_cloud_in)
{
	
	if(!point_cloud_in->data.empty()){
		// Container for original & filtered data
		pcl::PointCloud<pcl::PointXYZ> cloud_xyz;

		// Convert to PCL data type
		pcl::fromROSMsg(*point_cloud_in, cloud_xyz);
		
		pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZRGBL>>();
		
		pcl::copyPointCloud(cloud_xyz, *cloud_ptr);
		
		//Boxing
		pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_voxels_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZRGBL>>();
		pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBL> octree = boxing(cloud_ptr, cloud_voxels_ptr);
		
		//Voxel classification
		pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBL> octree_voxels(voxel_size_);
		octree_voxels.setInputCloud(cloud_voxels_ptr);
		octree_voxels.addPointsFromInputCloud();
		
		pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_clusters_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZRGBL>>();
		classifyVoxels(cloud_voxels_ptr, octree_voxels, cloud_clusters_ptr);

		std::unordered_map<uint32_t, pcl::IndicesPtr> clustersUsed;
		// map cluster ids in label to map with cluster id as key and pointvector as value
		for (size_t i = 0; i < cloud_clusters_ptr->size(); i++)
		{
			const uint32_t label = cloud_clusters_ptr->at(i).label;
			auto it = clustersUsed.find(label);
			if (it == clustersUsed.end()){
				it = clustersUsed.insert(std::unordered_map<uint32_t, pcl::IndicesPtr>::value_type(label, std::make_shared<pcl::Indices>())).first;
			}
			it->second->push_back(i);
		}

		// determine maximum left and right clusters in a radius
		std::pair<int64_t, int64_t> wallIds = determineWallIDs(cloud_clusters_ptr, clustersUsed, wall_radius_); // these ids are the walls

		std::unordered_set<uint32_t> ignoreIDs;

		if (wallIds.first >= 0 && wallIds.second >= 0)
		{
			pcl::IndicesPtr leftWall = clustersUsed[wallIds.first];
			pcl::IndicesPtr rightWall = clustersUsed[wallIds.second];
			
			if(leftWall->size() > 2 && rightWall->size() > 2){
				ignoreIDs.insert(wallIds.first);
				ignoreIDs.insert(wallIds.second);

				if (use_prediction_for_walls_ || use_prediction_for_obstacles_)
				{
					std::pair<pcl::Indices, pcl::Indices> additional_wall_ids = addClustersOnRegression(cloud_clusters_ptr, clustersUsed, ignoreIDs, leftWall, rightWall);

					if (use_prediction_for_walls_)
					{
						for (const uint32_t id : additional_wall_ids.first)
						{
							leftWall->insert(leftWall->end(), clustersUsed[id]->begin(), clustersUsed[id]->end());
						}

						for (const uint32_t  id : additional_wall_ids.second)
						{
							rightWall->insert(rightWall->end(), clustersUsed[id]->begin(), clustersUsed[id]->end());
						}
					}
					if (use_prediction_for_obstacles_)
					{
						ignoreIDs.insert(additional_wall_ids.first.begin(), additional_wall_ids.first.end());
						ignoreIDs.insert(additional_wall_ids.second.begin(), additional_wall_ids.second.end());
					}
				}
				
				// publish only the clusters with ids equal to the walls
				publishTrack(cloud_ptr, cloud_clusters_ptr, octree, leftWall, rightWall);
			}
		}
		
		// publish all other clusters
		publishObstacles(cloud_ptr, cloud_clusters_ptr, octree, clustersUsed, ignoreIDs);
	}
}

int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<WallDetection>());
	rclcpp::shutdown();
	return 0;
}
