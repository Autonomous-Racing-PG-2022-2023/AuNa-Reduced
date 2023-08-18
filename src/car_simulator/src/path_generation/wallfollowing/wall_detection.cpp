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

using namespace std::literals::chrono_literals;

WallDetection::WallDetection()
    : Node("walldetection"), preserved_voxels(std::make_shared<pcl::PointCloud<pcl::PointXYZRGBL>>())
{
	/* setup parameters */
	enable_debug_topics_ = declare_parameter<bool>("enable_debug_topics", true);
	
	voxel_size_ = declare_parameter<double>("voxel_size", 0.15);
	min_points_per_voxel_ = declare_parameter<int>("min_points_per_voxel", 0);
	//lidar_percentage_ = declare_parameter<double>("lidar_percentage", 0.5);
	filter_by_min_score_enabled_ = declare_parameter<bool>("filter_by_min_score_enabled", true);
	filter_by_min_score_ = declare_parameter<double>("filter_by_min_score", 0.002);
	sor_enabled_ = declare_parameter<bool>("sor_enabled", false);
	sor_mean_k_ = declare_parameter<double>("sor_mean_k", 2.0);
	sor_stddev_mul_thresh_ = declare_parameter<double>("sor_stddev_mul_thresh", 3.0);
	
	preserved_voxels_radius_ = declare_parameter<double>("preserved_voxels_radius", 3.0);
	
	db_search_minimum_points_ = declare_parameter<int>("db_search_minimum_points", 3);
	db_search_radius_ = declare_parameter<double>("db_search_radius", 0.25);
	
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
		"enable_debug_topics: %s, voxel_size_: %lf, min_points_per_voxel: %d, filter_by_min_score_enabled: %s, filter_by_min_score: %lf, sor_enabled: %s, "
		"sor_mean_k: %lf, sor_stddev_mul_thresh: %lf, preserved_voxels_radius: %lf, db_search_minimum_points: %d, db_search_radius: %lf, wall_radius: %lf, use_prediction_for_walls: %s, "
		"use_prediction_for_obstacles: %s, distance_threshold: %lf, score_threshold: %d, minimum_confidence: %lf, sac_max_iterations: %d, "
		"sac_distance_to_model_threshold: %lf, track_radius_propotions: %lf, track_in_front_threshold: %lf, track_upper_wall_offset: %lf",
		(enable_debug_topics_ ? "true" : "false"),
		voxel_size_,
		min_points_per_voxel_,
		(filter_by_min_score_enabled_ ? "true" : "false"),
		filter_by_min_score_,
		(sor_enabled_ ? "true" : "false"),
		sor_mean_k_,
		sor_stddev_mul_thresh_,
		preserved_voxels_radius_,
		db_search_minimum_points_,
		db_search_radius_,
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
	
	tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
	std::shared_ptr<tf2_ros::CreateTimerROS> timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
		this->get_node_base_interface(),
		this->get_node_timers_interface()
	);
	tf_buffer_->setCreateTimerInterface(timer_interface);
	
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	/* publisher */

	track_pub_ = create_publisher<car_simulator_msgs::msg::Track>(
		"~/output/track",
		rclcpp::QoS{1}
	);
	
	obstacles_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
		"~/output/obstacles",
		rclcpp::QoS{1}
	);
	
	voxels_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
		"~/debug/voxels",
		rclcpp::QoS{1}
	);
	
	clusters_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
		"~/debug/clusters",
		rclcpp::QoS{1}
	);
}


// filter the points, so that only a ring of points which is a certain margin from the car away is left
// calculate a new center from these points and connect this new center with the old track center.
// this should create a line, which approximates the road a head. Then it sums up the signed distance from the each cloud to this line and decides
// if this cloud is part of the left wall or the right wall

// when starting the process of detecting, it might miss assign the clusters, this might be due to not having enough points or due to missalignment
// but after correcting the cars posture it should be ralatively stable
std::pair<pcl::Indices, pcl::Indices> WallDetection::wallextensionByRadius(
	const pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr cloud,
    const std::unordered_map<uint32_t, pcl::IndicesPtr>& mapClusters,
	const std::unordered_set<uint32_t>& inputIgnoreIDs,
	int margin
)
// add margin to the length of the last centerpoint to calculate a radius by which to filter the wanted points
{
    std::pair<pcl::Indices, pcl::Indices> additional_wall_ids;

	const pcl::PointXYZRGBL track_direction_0 = getTrackDirection(cloud);
	const pcl::PointXYZRGBL track_direction_1 = getTrackDirectionByRadius(cloud,track_direction_0, margin);
	Line<pcl::PointXYZRGBL> track_line(track_direction_0, track_direction_1);

	// std::cout <<"track_direction_0: "<< track_direction_0 << std::endl;
	// std::cout <<"track_direction_1: "<< track_direction_1 << std::endl;
	// std::cout << "track_line length: " << track_line.length() <<std::endl;
	//connect these two center points and it should approximate the track better than just a single center point (from which a directions is determined).
    for (auto cluster_wall : mapClusters)
    {
        const uint32_t clusterID = cluster_wall.first;
		
		//Skip clusters already handled
        if (inputIgnoreIDs.find(clusterID) != inputIgnoreIDs.end()){
            continue;
		}

        const pcl::IndicesConstPtr cluster = cluster_wall.second;

		double sum_signed_distance = 0;

        for (size_t i = 0; i < cluster_wall.second->size(); ++i)
        {
			const pcl::PointXYZRGBL& point = cloud->at(cluster_wall.second->at(i));
			
			const double signed_distance = GeometricFunctions::calcSignedShortestDistanceToLine(point, track_line);
			
			sum_signed_distance = sum_signed_distance + signed_distance;
        }

		if (sum_signed_distance <= 0)
			{
				additional_wall_ids.first.push_back(clusterID);
			} else if (sum_signed_distance / cluster_wall.second->size() > 0)
			{
				additional_wall_ids.second.push_back(clusterID);
			}
    }
	// std::cout << "additional_wall_ids.first: ";
	// for (auto id : additional_wall_ids.first){
	// 	std::cout << id;
	// }
	// std::cout << std::endl << "additional_wall_ids.second: " << std::endl;
	// for (auto id : additional_wall_ids.second){
	// 	std::cout << id;
	// }
	// std::cout << std::endl;
    return additional_wall_ids;
}

// find the larges gap in the pointcloud out side a certain radius around the car.
pcl::PointXYZRGBL WallDetection::getTrackDirectionByRadius(const pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr cloud, const pcl::PointXYZRGBL cutoff, int margin){
	std::vector<pcl::PointXYZRGBL> sorted_points(cloud->begin(), cloud->end());

	const uint32_t radius = (std::sqrt(cutoff.x * cutoff.x + cutoff.y * cutoff.y + cutoff.z * cutoff.z));

	// std::cout << "radius:"<< radius << std::endl;

	//Remove all points behind the car
	sorted_points.erase(std::remove_if(std::execution::par_unseq, sorted_points.begin(), sorted_points.end(), [](const pcl::PointXYZRGBL& point){
		return (
			(point.x < 0)
		);
	}), sorted_points.end());

	// remove all the point inside the radius
	sorted_points.erase(std::remove_if(std::execution::par_unseq, sorted_points.begin(), sorted_points.end(), [&radius, &margin](const pcl::PointXYZRGBL& point){
	const double distance_from_origin = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
	return distance_from_origin < radius;
	}), sorted_points.end());

	// remove all the points outsite of (radius + margin), so there are only points in a ring form left
	sorted_points.erase(std::remove_if(std::execution::par_unseq, sorted_points.begin(), sorted_points.end(), [&radius, &margin](const pcl::PointXYZRGBL& point){
	const double distance_from_origin = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
	return distance_from_origin > (radius + margin);
	}), sorted_points.end());
	

	//Sort by angle
	std::sort(std::execution::par_unseq, sorted_points.begin(), sorted_points.end(), [](const pcl::PointXYZRGBL& a, const pcl::PointXYZRGBL& b){
		const double yaw_a = std::atan2(a.y, a.x);
		const double yaw_b = std::atan2(b.y, b.x);
		
		return yaw_a < yaw_b;
	});
	//Create segments
	std::vector<std::pair<pcl::PointXYZRGBL, pcl::PointXYZRGBL>> segments(sorted_points.size());
	std::transform(std::execution::par_unseq, sorted_points.begin(), sorted_points.end() - 1, sorted_points.begin() + 1, segments.begin(), [](const pcl::PointXYZRGBL& start, const pcl::PointXYZRGBL& end){
		return std::make_pair(start, end);
	});

	// maybe the problem is there are no points in the list so it throws an error
	//Then find segment with most distance between points
	std::pair<pcl::PointXYZRGBL, pcl::PointXYZRGBL> segment_in_front = *std::max_element(std::execution::par_unseq, segments.begin(), segments.end(), [](const std::pair<pcl::PointXYZRGBL, pcl::PointXYZRGBL>& a, const std::pair<pcl::PointXYZRGBL, pcl::PointXYZRGBL>& b){
		const double length_a = GeometricFunctions::distance(a.first, a.second);
		const double length_b = GeometricFunctions::distance(b.first, b.second);
		
		return length_a < length_b;
	});

	//Calculate it's center to get the track direction

	const pcl::PointXYZRGBL center(
		segment_in_front.second.x - segment_in_front.first.x,
		segment_in_front.second.y - segment_in_front.first.y,
		segment_in_front.second.z - segment_in_front.first.z
	);

	// std::cout <<"center: "<< center << std::endl;
	return center;
}

//Find track direction. Biggest holes in surrounding
pcl::PointXYZRGBL WallDetection::getTrackDirection(const pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr cloud){
	std::vector<pcl::PointXYZRGBL> sorted_points(cloud->begin(), cloud->end());
	
	//Remove all points behind the car

	sorted_points.erase(std::remove_if(std::execution::par_unseq, sorted_points.begin(), sorted_points.end(), [](const pcl::PointXYZRGBL& point){
		return (
			(point.x < 0)
		);
	}), sorted_points.end());

	//Sort by angle
	std::sort(std::execution::par_unseq, sorted_points.begin(), sorted_points.end(), [](const pcl::PointXYZRGBL& a, const pcl::PointXYZRGBL& b){
		const double yaw_a = std::atan2(a.y, a.x);
		const double yaw_b = std::atan2(b.y, b.x);
		
		return yaw_a < yaw_b;
	});
	
	//Create segments
	std::vector<std::pair<pcl::PointXYZRGBL, pcl::PointXYZRGBL>> segments(sorted_points.size());
	std::transform(std::execution::par_unseq, sorted_points.begin(), sorted_points.end() - 1, sorted_points.begin() + 1, segments.begin(), [](const pcl::PointXYZRGBL& start, const pcl::PointXYZRGBL& end){
		return std::make_pair(start, end);
	});
	
	//Then find segment with most distance between points
	std::pair<pcl::PointXYZRGBL, pcl::PointXYZRGBL> segment_in_front = *std::max_element(std::execution::par_unseq, segments.begin(), segments.end(), [](const std::pair<pcl::PointXYZRGBL, pcl::PointXYZRGBL>& a, const std::pair<pcl::PointXYZRGBL, pcl::PointXYZRGBL>& b){
		const double length_a = GeometricFunctions::distance(a.first, a.second);
		const double length_b = GeometricFunctions::distance(b.first, b.second);
		
		return length_a < length_b;
	});
	
	//Calculate it's center to get the track direction
	const float segment_length = GeometricFunctions::distance(segment_in_front.first, segment_in_front.second);
	const pcl::PointXYZRGBL center(
		segment_in_front.second.x - segment_in_front.first.x / segment_length,
		segment_in_front.second.y - segment_in_front.first.y / segment_length,
		segment_in_front.second.z - segment_in_front.first.z / segment_length
	);
	
	return center;
}

int64_t WallDetection::findLargestCluster(
	const std::unordered_map<uint32_t, pcl::IndicesPtr>& clusters,
    uint32_t ignoreID
)
{
    uint32_t largestClusterID = (std::max_element(clusters.begin(), clusters.end(), [&ignoreID](const std::pair<uint32_t, pcl::IndicesPtr>& a, const std::pair<uint32_t, pcl::IndicesPtr>& b){
		//IgnoredID is always smaller, so that it will not be the largest element execpt if it is the only element
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
	const pcl::PointXYZRGBL& track_direction,
	double radius
)
{
	//Define line that divides track in left and right
	Line<pcl::PointXYZRGBL> track_line(pcl::PointXYZRGBL(), track_direction);
	// std::cout << "line start: " << track_line.start << std::endl;
	// std::cout << "line end: " << track_line.end << std::endl;
	
    double maxRight = 0.0;
    double minLeft = 0.0;
    int64_t maxRightID = -1;
    int64_t minLeftID = -1;

    for (auto itr = mapToCheck.begin(); itr != mapToCheck.end(); ++itr)
    {
        for (size_t i = 0; i < itr->second->size(); ++i)
        {
			const pcl::PointXYZRGBL& point = cloud->at(itr->second->at(i));
			
			const double signed_distance = GeometricFunctions::calcSignedShortestDistanceToLine(point, track_line);
			
			if(point.x >= 0 && GeometricFunctions::distance(point, pcl::PointXYZRGBL(0.0f, 0.0f, 0.0f)) <= radius){
				if (signed_distance > maxRight)
				{
					maxRight = signed_distance;
					maxRightID = point.label;
				}else if (signed_distance < minLeft)
				{
					minLeft = signed_distance;
					minLeftID = point.label;
				}
			}
        }
    }

    if (minLeftID == -1 && maxRightID != -1)
    {
        // found a cluster for right but not for left. let's just choose the largest one for the left.
        minLeftID = findLargestCluster(mapToCheck, maxRightID);
    }else if (maxRightID == -1 && minLeftID != -1)
    {
        // same but reverse
        maxRightID = findLargestCluster(mapToCheck, minLeftID);
    }

    return std::pair<int64_t, int64_t>(maxRightID, minLeftID);
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
	std::for_each(std::execution::par_unseq, output_cloud->begin(), output_cloud->end(), [](pcl::PointXYZRGBL& point){
		point.label = DBSCAN::NOISE;
	});
	
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
	std::for_each(std::execution::par_unseq, octree.leaf_depth_begin(), octree.leaf_depth_end(), [this, &largest_intensity, &output_cloud, &cloud_out](pcl::octree::OctreeNode* node){
		auto points = static_cast<const pcl::octree::OctreeLeafNode<pcl::octree::OctreeContainerPointIndices>*>(node)->getContainer();
		
		if(points.getSize() > 0){
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
		}
	});
	
	//Copy metadata
	cloud_out->header = cloud_in->header;

	return octree;//No std::move to allow copy elision

}

void WallDetection::classifyVoxels(
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr& cloud_in,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr& cloud_out
){
    DBSCAN ds(cloud_in, db_search_minimum_points_, db_search_radius_);
	
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBL> octree_voxels(voxel_size_);
	octree_voxels.setInputCloud(cloud_in);
	octree_voxels.addPointsFromInputCloud();
	
    ds.run(octree_voxels, 1);
	
	//Remove noise
	pcl::ConditionAnd<pcl::PointXYZRGBL>::Ptr filter_condition_ptr = std::make_shared<pcl::ConditionAnd<pcl::PointXYZRGBL>>();
	pcl::FieldComparison<pcl::PointXYZRGBL>::Ptr field_comparison_ptr = std::make_shared<pcl::FieldComparison<pcl::PointXYZRGBL>>("label", pcl::ComparisonOps::EQ, DBSCAN::NOISE);
	
	filter_condition_ptr->addComparison(field_comparison_ptr);
	
	pcl::ConditionalRemoval<pcl::PointXYZRGBL> conditional_removal(true);
	conditional_removal.setCondition(filter_condition_ptr);
	
	conditional_removal.setInputCloud(cloud_in);
	conditional_removal.filter(*cloud_out);
	
	pcl::ExtractIndices<pcl::PointXYZRGBL> filter(false);
	filter.setIndices(conditional_removal.getRemovedIndices());
	
	filter.setInputCloud(cloud_in);
	filter.filter(*cloud_out);

}

//FIXME: Own octree search, cause existing one does currently not work: https://github.com/PointCloudLibrary/pcl/issues/5637
bool WallDetection::voxelSearch(pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBL>& octree, const pcl::PointXYZRGBL& voxel, pcl::Indices& points){
	const double voxel_side_len = this->voxel_size_;
		
	double min_x, min_y, min_z, max_x, max_y, max_z;
	octree.getBoundingBox(min_x, min_y, min_z, max_x, max_y, max_z);
	
	for(pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBL>::LeafNodeDepthFirstIterator it =  octree.leaf_depth_begin(); it != octree.leaf_depth_end(); ++it){
		auto container_points = static_cast<const pcl::octree::OctreeLeafNode<pcl::octree::OctreeContainerPointIndices>*>(*it)->getContainer();
		
		const pcl::PointXYZRGBL point_in_voxel = octree.getInputCloud()->at(container_points.getPointIndex());
		
		const size_t key_x = static_cast<size_t>((point_in_voxel.x - min_x) / this->voxel_size_);
		const size_t key_y = static_cast<size_t>((point_in_voxel.y - min_y) / this->voxel_size_);
		const size_t key_z = static_cast<size_t>((point_in_voxel.z - min_z) / this->voxel_size_);

		Eigen::Vector3f min_bounds;
		Eigen::Vector3f max_bounds;
		min_bounds(0) = static_cast<float>(static_cast<double>(key_x) * voxel_side_len + min_x);
		min_bounds(1) = static_cast<float>(static_cast<double>(key_y) * voxel_side_len + min_y);
		min_bounds(2) = static_cast<float>(static_cast<double>(key_z) * voxel_side_len +  min_z);

		max_bounds(0) = static_cast<float>(static_cast<double>(key_x + 1) * voxel_side_len + min_x);
		max_bounds(1) = static_cast<float>(static_cast<double>(key_y + 1) * voxel_side_len + min_y);
		max_bounds(2) = static_cast<float>(static_cast<double>(key_z + 1) * voxel_side_len + min_z);
		
		if(
			(voxel.x >= min_bounds.x())
			&& (voxel.y >= min_bounds.y())
			&& (voxel.z >= min_bounds.z())
			&& (voxel.x <= max_bounds.x())
			&& (voxel.y <= max_bounds.y())
			&& (voxel.z <= max_bounds.z())
		){
			container_points.getPointIndices(points);
			return true;
		}
	}
	return false;
}

//Preserve voxels, that are not already covered by new voxels and that are not too far away
//NOTE: We do this after classification sorted out noise
void WallDetection::preserveVoxel(const pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr& cloud){
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBL> octree(voxel_size_);
	octree.setInputCloud(cloud);
	octree.addPointsFromInputCloud();
	
	//Sort out voxels, that are too far away or that are already covered by new voxels
	preserved_voxels->erase(std::remove_if(std::execution::par_unseq, preserved_voxels->begin(), preserved_voxels->end(), [this, &octree](pcl::PointXYZRGBL& point){
		pcl::Indices points;
		
		return (
			(GeometricFunctions::distance(pcl::PointXYZRGBL(0.0f, 0.0f, 0.0f), point) > this->preserved_voxels_radius_)
			||(voxelSearch(octree, point, points))//(octree.voxelSearch(point, points));
		);
	}), preserved_voxels->end()); 
	
	//Then add the new voxels
	preserved_voxels->insert(preserved_voxels->end(), cloud->begin(), cloud->begin());
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
				if(!voxelSearch(octree, voxel, points)){//!octree.voxelSearch(voxel, points)){
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
		if(!voxelSearch(octree, voxel, points)){//!octree.voxelSearch(voxel, points)){
			RCLCPP_ERROR(this->get_logger(), "Voxel not found");
		}
		
		//Set label for all points
		std::for_each(std::execution::par_unseq, points.begin(), points.end(), [&input_cloud](size_t index){
			input_cloud->at(index).label = WALL_DETECTION_WALL_ID_LEFT;
		});
		
		//Add all points associated with voxel
		left_cloud_indices->insert(left_cloud_indices->end(), points.begin(), points.end());
	});
	
	std::for_each(std::execution::seq, cloud_right.begin(), cloud_right.end(), [this, &octree, &input_cloud, &right_cloud_indices](const pcl::PointXYZRGBL& voxel){
		pcl::Indices points;
		if(!voxelSearch(octree, voxel, points)){//!octree.voxelSearch(voxel, points)){
			RCLCPP_ERROR(this->get_logger(), "Voxel not found");
		}
		
		//Set label for all points
		std::for_each(std::execution::par_unseq, points.begin(), points.end(), [&input_cloud](size_t index){
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

void WallDetection::publishPointCloud(
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher,
	const pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr& input_cloud
)
{
	// Convert to ROS data type
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*input_cloud, output);

    // Publish the data
    publisher->publish(output);
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
		
		//Transform preserved points into current frame and reset their lable
		//NOTE: We are not using matrix inverse cause it is unstable. Instead we fetch the inverse transformation directly
		geometry_msgs::msg::TransformStamped transform_from_map_msg, transform_to_map_msg;
		try{
			transform_from_map_msg = tf_buffer_->lookupTransform(point_cloud_in->header.frame_id, "map", tf2::TimePointZero);
			transform_to_map_msg = tf_buffer_->lookupTransform("map", point_cloud_in->header.frame_id, tf2::TimePointZero);
		} catch (std::exception& e) {
			RCLCPP_ERROR(this->get_logger(), "%s", e.what());
			return;
		}
		tf2::Transform transform_from_map;
		tf2::fromMsg(transform_from_map_msg.transform, transform_from_map);
		
		const tf2::Transform transform = transform_from_map * last_transform_to_map;
		tf2::fromMsg(transform_to_map_msg.transform, last_transform_to_map);
		
		std::for_each(std::execution::par_unseq, preserved_voxels->begin(), preserved_voxels->end(), [&transform](pcl::PointXYZRGBL& point){
			const tf2::Vector3 tf_point(point.x, point.y, point.z);
			const tf2::Vector3 tf_point_transformed = transform * tf_point;
			
			point.x = tf_point_transformed.x();
			point.y = tf_point_transformed.y();
			point.z = tf_point_transformed.z();
			point.label = DBSCAN::UNCLASSIFIED;
		});
		
		//Boxing
		pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_voxels_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZRGBL>>();
		pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBL> octree = boxing(cloud_ptr, cloud_voxels_ptr);
		
		if(enable_debug_topics_){
			publishPointCloud(voxels_pub_, cloud_voxels_ptr);
		}
		
		//Add preserved voxels to current classification
		pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_voxels_combined_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZRGBL>>();
		pcl::PointCloud<pcl::PointXYZRGBL>::concatenate(*cloud_voxels_ptr, *preserved_voxels, *cloud_voxels_combined_ptr);
		
		//Voxel classification
		pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_clusters_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZRGBL>>();
		classifyVoxels(cloud_voxels_combined_ptr, cloud_clusters_ptr);
		
		//Sort out voxels, that are not from the current data set (the preserved points were only used to help correct clustering)
		cloud_clusters_ptr->erase(std::remove_if(std::execution::par_unseq, cloud_clusters_ptr->begin(), cloud_clusters_ptr->end(), [this, &octree](pcl::PointXYZRGBL& point){
			pcl::Indices points;
			
			return !voxelSearch(octree, point, points);//!octree.voxelSearch(point, points);
		}), cloud_clusters_ptr->end());
		
		if(enable_debug_topics_){
			publishPointCloud(clusters_pub_, cloud_clusters_ptr);
		}
		
		//Preserve voxel
		preserveVoxel(cloud_clusters_ptr);
		
		//Get Track direction
		const pcl::PointXYZRGBL track_direction = getTrackDirection(cloud_clusters_ptr);

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

		std::unordered_set<uint32_t> ignoreIDs;
		if(clustersUsed.size() > 1){
			// determine maximum left and right clusters in a radius
			std::pair<int64_t, int64_t> wallIds = determineWallIDs(cloud_clusters_ptr, clustersUsed, track_direction, wall_radius_); // these ids are the walls

			if (wallIds.first >= 0 && wallIds.second >= 0 && wallIds.first != wallIds.second)
			{
				pcl::IndicesPtr leftWall = clustersUsed[wallIds.first];
				pcl::IndicesPtr rightWall = clustersUsed[wallIds.second];
				
				if(leftWall->size() > 5 && rightWall->size() > 5){
					ignoreIDs.insert(wallIds.first);
					ignoreIDs.insert(wallIds.second);

					if (use_prediction_for_walls_ || use_prediction_for_obstacles_)
					{

						std::pair<pcl::Indices, pcl::Indices> additional_wall_ids = wallextensionByRadius(cloud_clusters_ptr, clustersUsed, ignoreIDs,30);//NOTE: Not used currently. Instead adjust DBSCAN parameters = addClustersOnRegression(cloud_clusters_ptr, clustersUsed, ignoreIDs, leftWall, rightWall);

						if (use_prediction_for_walls_)
						{
							// its not really a prediction, it adds additional clusters to the walls, for a more stable and complete wall
							for (const uint32_t id : additional_wall_ids.first)
							{
								//std::cout << "additional_wall_ids.first:" << id << std::endl;
								leftWall->insert(leftWall->end(), clustersUsed[id]->begin(), clustersUsed[id]->end());
							}

							for (const uint32_t  id : additional_wall_ids.second)
							{
								//std::cout << "additional_wall_ids.second:" << id << std::endl;
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
