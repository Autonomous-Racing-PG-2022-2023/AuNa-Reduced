#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "autoware_auto_planning_msgs/msg/path.hpp"

class PathConverter : public rclcpp::Node
{
private:
	rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
	rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_subscription_;
	
	rclcpp::Publisher<autoware_auto_planning_msgs::msg::Path>::SharedPtr publisher_;
	
	nav_msgs::msg::Path::ConstSharedPtr last_path_in;
	nav_msgs::msg::OccupancyGrid::ConstSharedPtr last_occupancy_grid_in;
	
	std::string path_src_topic_;
	std::string occupancy_grid_src_topic_;
	std::string dst_topic_;
	
	double max_longitudinal_velocity_mps_;
	double max_lateral_velocity_mps_;
	double max_heading_rate_rps_;
public:
	PathConverter()
	: Node("patch_converter")
	{
		path_src_topic_ = this->declare_parameter("path_src_topic", "~/input/path");
		occupancy_grid_src_topic_ = this->declare_parameter("occupancy_grid_src_topic", "~/input/occupancy_grid");
		dst_topic_ = this->declare_parameter("dst_topic", "~/output/path");
		
		max_longitudinal_velocity_mps_ = this->declare_parameter<double>("max_longitudinal_velocity_mps", 100.0);
		max_lateral_velocity_mps_ = this->declare_parameter<double>("max_lateral_velocity_mps", 100.0);
		max_heading_rate_rps_ = this->declare_parameter<double>("max_heading_rate_rps", 100.0);

		RCLCPP_INFO_THROTTLE(
			get_logger(),
			*get_clock(),
			std::chrono::milliseconds(1000).count(),
			"Coverting nav_msgs/msg/Path in topic %s and nav_msgs/msg/OccupancyGrid in topic %s to autoware_auto_planning_msgs/msg/Path in topic %s",
			path_src_topic_.c_str(),
			occupancy_grid_src_topic_.c_str(),
			dst_topic_.c_str()
		);
		
		path_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
			path_src_topic_.c_str(),
			10,
			std::bind(&PathConverter::pathCallback, this, std::placeholders::_1)
		);
		
		occupancy_grid_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
			occupancy_grid_src_topic_.c_str(),
			10,
			std::bind(&PathConverter::occupancyGridCallback, this, std::placeholders::_1)
		);
		
		publisher_ = create_publisher<autoware_auto_planning_msgs::msg::Path>(
			dst_topic_,
			rclcpp::QoS{1}
		);
	}

private:
	void handle(){
		nav_msgs::msg::Path::ConstSharedPtr path_in = last_path_in;
		nav_msgs::msg::OccupancyGrid::ConstSharedPtr occupancy_grid_in = last_occupancy_grid_in;
		
		std::vector<autoware_auto_planning_msgs::msg::PathPoint> points(path_in->poses.size());
		for(size_t i = 0; i < path_in->poses.size(); ++i){
			points[i].pose = path_in->poses[i].pose;
			//TODO: correct velocities? Done by later stages?
			points[i].longitudinal_velocity_mps = max_longitudinal_velocity_mps_;
			points[i].lateral_velocity_mps = max_lateral_velocity_mps_;
			points[i].heading_rate_rps = max_heading_rate_rps_;
		}
		//points[path_in->poses.size() - 1].is_final = true;
		
		autoware_auto_planning_msgs::msg::Path path_out;
		path_out.header = path_in->header;
		path_out.points = points;
		path_out.drivable_area = *occupancy_grid_in;
		
		publisher_->publish(path_out);
	}

	void pathCallback(const nav_msgs::msg::Path::ConstSharedPtr path_in)
	{
		last_path_in = path_in;
		if(last_occupancy_grid_in){
			handle();
		}
	}
	
	void occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr occupancy_grid_in)
	{
		last_occupancy_grid_in = occupancy_grid_in;
		if(last_path_in){
			handle();
		}
	}
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<PathConverter>());
	rclcpp::shutdown();
	return 0;
}