#include "track.hpp"

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/sac_model_circle.h>

#include "pcl_conversions/pcl_conversions.h"

#include "geometric_math.hpp"
#include "fit_walls.hpp"

pcl::IndicesConstPtr TrackGenerator::cropPointcloud(
	const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
	float minimum_x
)
{
	pcl::PointCloud<pcl::PointXYZ> cloud_out;
	
	pcl::PassThrough<pcl::PointXYZ> filter(true);
	filter.setFilterFieldName("x");
	filter.setFilterLimits(std::numeric_limits<float>::lowest(), minimum_x);
	
	filter.setInputCloud(cloud);
	filter.filter(cloud_out);

    return filter.getRemovedIndices();//Negate by fetiching removed indices
}

pcl::PointXYZ TrackGenerator::calcNearestPointToPoint(
	const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
	const pcl::PointXYZ& point
)
{
    return *(std::min_element(cloud->begin(), cloud->end(), [&point](const pcl::PointXYZ& a, const pcl::PointXYZ& b){
	   return (GeometricFunctions::distance(a, point) < GeometricFunctions::distance(b, point));
   }));
}

bool TrackGenerator::isCurveEntryInFront(const pcl::PointXYZ& curve_entry_point, const pcl::PointXYZ& lowest_point, double threshold)
{
    if (lowest_point.y - curve_entry_point.y == 0)
    {
        return true;
    }
    double val = (lowest_point.x - curve_entry_point.x) / (lowest_point.y - curve_entry_point.y);
    return fabsf(val) > threshold;
}

pcl::PointXYZ TrackGenerator::getCurveEntry(
	const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud
)
{
   return *(std::max_element(cloud->begin(), cloud->end(), [](const pcl::PointXYZ& a, const pcl::PointXYZ& b){
	   return (a.x < b.x);
   }));
}

car_simulator_msgs::msg::Track TrackGenerator::generateTrack(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, pcl::IndicesConstPtr left_wall, pcl::IndicesConstPtr right_wall, double radius_propotions, double in_front_threshold, double upper_wall_offset, double sac_distance_to_model_threshold, int sac_max_iterations)
{
	(void) radius_propotions;
	
	pcl::IndicesConstPtr upper_wall;
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr upper_cloud_src_ptr;
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr left_cloud_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	pcl::PointCloud<pcl::PointXYZ>::Ptr right_cloud_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	pcl::PointCloud<pcl::PointXYZ>::Ptr upper_cloud_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    Circle<pcl::PointXYZ> left_circle;
    Circle<pcl::PointXYZ> right_circle;
    Circle<pcl::PointXYZ> upper_circle;

    car_simulator_msgs::msg::TrackCurveType curve_type;
    pcl::PointXYZ curve_entry;

    pcl::PointXYZ car_position;
	
	//Extract clouds
	pcl::ExtractIndices<pcl::PointXYZ> filter_left(false);
	filter_left.setIndices(left_wall);
	
	filter_left.setInputCloud(cloud);
	filter_left.filter(*left_cloud_ptr);
	
	pcl::ExtractIndices<pcl::PointXYZ> filter_right(false);
	filter_right.setIndices(right_wall);
	
	filter_right.setInputCloud(cloud);
	filter_right.filter(*right_cloud_ptr);
	
	left_circle = FitWalls::fitWall<pcl::PointXYZ>(cloud, left_wall, sac_distance_to_model_threshold, sac_max_iterations);
	right_circle = FitWalls::fitWall<pcl::PointXYZ>(cloud, right_wall, sac_distance_to_model_threshold, sac_max_iterations);
	
    car_position = pcl::PointXYZ(0.0f, 0.0f, 0.0f);
    curve_type.data = car_simulator_msgs::msg::TrackCurveType::CURVE_TYPE_STRAIGHT;

	// With radius_proportions can be checked whether the car is approaching a curve or is on a straight part of the
    // track.
	//TODO: Also somehow move in the middle of the track if we are not in a curve
    //double radius_proportions_left = left_circle.getRadius() / right_circle.getRadius();
    //double radius_proportions_right = right_circle.getRadius() / left_circle.getRadius();
    if (right_circle.getCenter().y > 0)//TODO: radius_proportions_left > radius_propotions)
    {
        curve_entry = getCurveEntry(left_cloud_ptr);
        pcl::PointXYZ nearest_point_to_car = calcNearestPointToPoint(left_cloud_ptr, car_position);
        if (isCurveEntryInFront(curve_entry, nearest_point_to_car, in_front_threshold))
        {
            upper_wall = cropPointcloud(right_cloud_ptr, curve_entry.y - upper_wall_offset);
			upper_cloud_src_ptr = right_cloud_ptr;
            curve_type.data = car_simulator_msgs::msg::TrackCurveType::CURVE_TYPE_LEFT;
        }
    }
    else if (left_circle.getCenter().y < 0)//TODO: radius_proportions_right > radius_propotions)
    {
        curve_entry = getCurveEntry(right_cloud_ptr);
        pcl::PointXYZ nearest_point_to_car = calcNearestPointToPoint(right_cloud_ptr, car_position);
        if (isCurveEntryInFront(curve_entry, nearest_point_to_car, in_front_threshold))
        {
            upper_wall = cropPointcloud(left_cloud_ptr, curve_entry.y - upper_wall_offset);
			upper_cloud_src_ptr = left_cloud_ptr;
            curve_type.data = car_simulator_msgs::msg::TrackCurveType::CURVE_TYPE_RIGHT;
        }
    }
	
	if(upper_wall && upper_wall->size() > 2){
		if (curve_type.data != car_simulator_msgs::msg::TrackCurveType::CURVE_TYPE_STRAIGHT)
		{
			upper_circle = FitWalls::fitWall<pcl::PointXYZ>(upper_cloud_src_ptr, upper_wall, sac_distance_to_model_threshold, sac_max_iterations);
		}
		
		//Generate upper cloud
		pcl::ExtractIndices<pcl::PointXYZ> filter(false);
		filter.setIndices(upper_wall);
		
		filter.setInputCloud(upper_cloud_src_ptr);
		filter.filter(*upper_cloud_ptr);
	}
	
	//Copy metadata
	left_cloud_ptr->header = cloud->header;
	right_cloud_ptr->header = cloud->header;
	upper_cloud_ptr->header = cloud->header;
	
	car_simulator_msgs::msg::Track track;
	track.header = pcl_conversions::fromPCL(cloud->header);
	
	track.curve_type = curve_type;
	track.curve_entry.x = curve_entry.x;
	track.curve_entry.y = curve_entry.y;
	track.curve_entry.z = curve_entry.z;
	  
    pcl::toROSMsg(*left_cloud_ptr, track.left_cloud);
	pcl::toROSMsg(*right_cloud_ptr, track.right_cloud);
	pcl::toROSMsg(*upper_cloud_ptr, track.upper_cloud);
	
	Circle<pcl::PointXYZ>::toROSMsg(left_circle, track.left_circle);
	Circle<pcl::PointXYZ>::toROSMsg(right_circle, track.right_circle);
	Circle<pcl::PointXYZ>::toROSMsg(upper_circle, track.upper_circle);
	
	return track;
}