#pragma once

#include <rclcpp/rclcpp.hpp>

#include <rviz_common/message_filter_display.hpp>

#include <rviz_default_plugins/displays/pointcloud/point_cloud_common.hpp>
#include <rviz_default_plugins/displays/marker/marker_common.hpp>

#include "car_simulator_msgs/msg/track.hpp"


namespace rviz_common::properties
{
class ColorProperty;
class FloatProperty;
class IntProperty;
class BoolProperty;
class EnumProperty;
}  // namespace rviz_common::properties

namespace rviz_plugins
{

class TrackDisplay : public rviz_common::MessageFilterDisplay<car_simulator_msgs::msg::Track>
{
	Q_OBJECT

public:
	TrackDisplay();
	~TrackDisplay() override;
	TrackDisplay(const TrackDisplay &) = delete;
	TrackDisplay(const TrackDisplay &&) = delete;
	TrackDisplay& operator=(const TrackDisplay&) = delete;
	TrackDisplay& operator=(const TrackDisplay&&) = delete;
	
protected:
	/* callbacks */
	void onInitialize() override;
	void onEnable() override;
	void onDisable() override;
	void reset() override;
	void update(float wall_dt, float ros_dt) override;

private:
	/* data */
	std::unique_ptr<rviz_default_plugins::PointCloudCommon> left_point_cloud_;
	std::unique_ptr<rviz_default_plugins::PointCloudCommon> right_point_cloud_;
	std::unique_ptr<rviz_default_plugins::PointCloudCommon> upper_point_cloud_;
	
	std::unique_ptr<rviz_default_plugins::displays::MarkerCommon> curve_entry_;
	
	std::unique_ptr<rviz_default_plugins::displays::MarkerCommon> left_circle_;
	std::unique_ptr<rviz_default_plugins::displays::MarkerCommon> right_circle_;
	std::unique_ptr<rviz_default_plugins::displays::MarkerCommon> upper_circle_;

	rviz_common::Display* property_curve_entry_view_;
	
	rviz_common::Display* property_left_cloud_view_;
	rviz_common::Display* property_right_cloud_view_;
	rviz_common::Display* property_upper_cloud_view_;
	
	rviz_common::Display* property_left_circle_view_;
	rviz_common::Display* property_right_circle_view_;
	rviz_common::Display* property_upper_circle_view_;
	
	rviz_common::properties::FloatProperty* property_curve_entry_point_size_;
	
	rviz_common::properties::FloatProperty* property_curve_entry_alpha_;
	
	rviz_common::properties::FloatProperty* property_left_circle_alpha_;
	rviz_common::properties::FloatProperty* property_right_circle_alpha_;
	rviz_common::properties::FloatProperty* property_upper_circle_alpha_;
	
	rviz_common::properties::ColorProperty* property_curve_entry_color_;
	
	rviz_common::properties::ColorProperty* property_left_circle_color_;
	rviz_common::properties::ColorProperty* property_right_circle_color_;
	rviz_common::properties::ColorProperty* property_upper_circle_color_;
	

	/* functions */
	void subscribe() override;
	void unsubscribe() override;
	void processMessage(const car_simulator_msgs::msg::Track::ConstSharedPtr message) override;
	
	
};

}  // namespace rviz_plugins
