#include "track_display/track_display.hpp"

#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/parse_color.hpp>

#include "visualization_msgs/msg/marker.hpp"

namespace rviz_plugins
{

TrackDisplay::TrackDisplay()
{
	property_left_cloud_view_ = new rviz_common::Display();//("Left Cloud", true, "", this);
	property_left_cloud_view_->setName("Left Cloud");
	property_right_cloud_view_ = new rviz_common::Display();//("Right Cloud", true, "", this);
	property_right_cloud_view_->setName("Right Cloud");
	property_upper_cloud_view_ = new rviz_common::Display();//("Upper Cloud", true, "", this);
	property_upper_cloud_view_->setName("Upper Cloud");
	
	property_curve_entry_view_ = new rviz_common::Display();//("Curve Entry", true, "", this);
	property_curve_entry_view_->setName("Curve Entry");
	
	property_left_circle_view_ = new rviz_common::Display();//("Left Circle", true, "", this);
	property_left_circle_view_->setName("Left Circle");
	property_right_circle_view_ = new rviz_common::Display();//("Right Circle", true, "", this);
	property_right_circle_view_->setName("Right Circle");
	property_upper_circle_view_ = new rviz_common::Display();//("Upper Circle", true, "", this);
	property_upper_circle_view_->setName("Upper Circle");
	
	left_point_cloud_ = std::make_unique<rviz_default_plugins::PointCloudCommon>(property_left_cloud_view_);
	right_point_cloud_ = std::make_unique<rviz_default_plugins::PointCloudCommon>(property_right_cloud_view_);
	upper_point_cloud_ = std::make_unique<rviz_default_plugins::PointCloudCommon>(property_upper_cloud_view_);
	
	curve_entry_ = std::make_unique<rviz_default_plugins::displays::MarkerCommon>(property_curve_entry_view_);
	
	left_circle_ = std::make_unique<rviz_default_plugins::displays::MarkerCommon>(property_left_circle_view_);
	right_circle_ = std::make_unique<rviz_default_plugins::displays::MarkerCommon>(property_right_circle_view_);
	upper_circle_ = std::make_unique<rviz_default_plugins::displays::MarkerCommon>(property_upper_circle_view_);
	
	property_curve_entry_point_size_ = new rviz_common::properties::FloatProperty("Point Size", 0.1, "", property_curve_entry_view_);
	property_curve_entry_point_size_->setMin(0.0);
	
	property_curve_entry_alpha_ = new rviz_common::properties::FloatProperty("Alpha", 1.0, "", property_curve_entry_view_);
	property_curve_entry_alpha_->setMin(0.0);
	property_curve_entry_alpha_->setMax(1.0);
	
	property_left_circle_alpha_ = new rviz_common::properties::FloatProperty("Alpha", 1.0, "", property_left_circle_view_);
	property_left_circle_alpha_->setMin(0.0);
	property_left_circle_alpha_->setMax(1.0);
	property_right_circle_alpha_ = new rviz_common::properties::FloatProperty("Alpha", 1.0, "", property_right_circle_view_);
	property_right_circle_alpha_->setMin(0.0);
	property_right_circle_alpha_->setMax(1.0);
	property_upper_circle_alpha_ = new rviz_common::properties::FloatProperty("Alpha", 1.0, "", property_upper_circle_view_);
	property_upper_circle_alpha_->setMin(0.0);
	property_upper_circle_alpha_->setMax(1.0);
	
	property_curve_entry_color_ = new rviz_common::properties::ColorProperty("Color", Qt::white, "", property_curve_entry_view_);
	
	property_left_circle_color_ = new rviz_common::properties::ColorProperty("Color", Qt::white, "", property_left_circle_view_);
	property_right_circle_color_ = new rviz_common::properties::ColorProperty("Color", Qt::white, "", property_right_circle_view_);
	property_upper_circle_color_ = new rviz_common::properties::ColorProperty("Color", Qt::white, "", property_upper_circle_view_);
	
	this->addChild(property_left_cloud_view_);
	this->addChild(property_right_cloud_view_);
	this->addChild(property_upper_cloud_view_);
	this->addChild(property_left_circle_view_);
	this->addChild(property_right_circle_view_);
	this->addChild(property_upper_circle_view_);
	this->addChild(property_curve_entry_view_);
}

TrackDisplay::~TrackDisplay() = default;  // Properties are deleted by Qt

void TrackDisplay::onInitialize()
{
	MFDClass::onInitialize();
	
	property_left_cloud_view_->initialize(this->context_);
	property_right_cloud_view_->initialize(this->context_);
	property_upper_cloud_view_->initialize(this->context_);
	property_left_circle_view_->initialize(this->context_);
	property_right_circle_view_->initialize(this->context_);
	property_upper_circle_view_->initialize(this->context_);
	property_curve_entry_view_->initialize(this->context_);
	
	left_point_cloud_->initialize(this->context_, this->scene_node_);
	right_point_cloud_->initialize(this->context_, this->scene_node_);
	upper_point_cloud_->initialize(this->context_, this->scene_node_);
	
	curve_entry_->initialize(this->context_, this->scene_node_);
	
	left_circle_->initialize(this->context_, this->scene_node_);
	right_circle_->initialize(this->context_, this->scene_node_);
	upper_circle_->initialize(this->context_, this->scene_node_);
}

void TrackDisplay::onEnable() {
	this->subscribe();
	
	property_left_cloud_view_->setEnabled(true);
	property_right_cloud_view_->setEnabled(true);
	property_upper_cloud_view_->setEnabled(true);
	property_left_circle_view_->setEnabled(true);
	property_right_circle_view_->setEnabled(true);
	property_upper_circle_view_->setEnabled(true);
	property_curve_entry_view_->setEnabled(true);
}

void TrackDisplay::onDisable() {
	this->unsubscribe();
	
	property_left_cloud_view_->setEnabled(false);
	property_right_cloud_view_->setEnabled(false);
	property_upper_cloud_view_->setEnabled(false);
	property_left_circle_view_->setEnabled(false);
	property_right_circle_view_->setEnabled(false);
	property_upper_circle_view_->setEnabled(false);
	property_curve_entry_view_->setEnabled(false);
	
	left_point_cloud_->onDisable();
	right_point_cloud_->onDisable();
	upper_point_cloud_->onDisable();
}

void TrackDisplay::update(float wall_dt, float ros_dt)
{
	property_left_cloud_view_->update(wall_dt, ros_dt);
	property_right_cloud_view_->update(wall_dt, ros_dt);
	property_upper_cloud_view_->update(wall_dt, ros_dt);
	property_left_circle_view_->update(wall_dt, ros_dt);
	property_right_circle_view_->update(wall_dt, ros_dt);
	property_upper_circle_view_->update(wall_dt, ros_dt);
	property_curve_entry_view_->update(wall_dt, ros_dt);
	
	left_point_cloud_->update(wall_dt, ros_dt);
	right_point_cloud_->update(wall_dt, ros_dt);
	upper_point_cloud_->update(wall_dt, ros_dt);
	
	curve_entry_->update(wall_dt, ros_dt);
	
	left_circle_->update(wall_dt, ros_dt);
	right_circle_->update(wall_dt, ros_dt);
	upper_circle_->update(wall_dt, ros_dt);
}

void TrackDisplay::reset() {
	rviz_common::MessageFilterDisplay<car_simulator_msgs::msg::Track>::subscribe();
	
	property_left_cloud_view_->reset();
	property_right_cloud_view_->reset();
	property_upper_cloud_view_->reset();
	property_left_circle_view_->reset();
	property_right_circle_view_->reset();
	property_upper_circle_view_->reset();
	property_curve_entry_view_->reset();
	
	left_point_cloud_->reset();
	right_point_cloud_->reset();
	upper_point_cloud_->reset();
	
	curve_entry_->clearMarkers();
	
	left_circle_->clearMarkers();
	right_circle_->clearMarkers();
	upper_circle_->clearMarkers();
}

void TrackDisplay::subscribe() {
	MFDClass::subscribe();
}

void TrackDisplay::unsubscribe()
{
	MFDClass::unsubscribe();
}

void TrackDisplay::processMessage(const car_simulator_msgs::msg::Track::ConstSharedPtr message)
{
	//TODO:Validate message
	
	//Store message
	left_point_cloud_->addMessage(std::make_shared<sensor_msgs::msg::PointCloud2>(message->left_cloud));
	right_point_cloud_->addMessage(std::make_shared<sensor_msgs::msg::PointCloud2>(message->right_cloud));
	upper_point_cloud_->addMessage(std::make_shared<sensor_msgs::msg::PointCloud2>(message->upper_cloud));
	
	visualization_msgs::msg::Marker::SharedPtr curve_entry_marker_ptr = std::make_shared<visualization_msgs::msg::Marker>();
	curve_entry_marker_ptr->header = message->header;
	curve_entry_marker_ptr->ns = "curve_entry";
	curve_entry_marker_ptr->id = 0;
	curve_entry_marker_ptr->type = visualization_msgs::msg::Marker::SPHERE;
	curve_entry_marker_ptr->action = visualization_msgs::msg::Marker::MODIFY;
	curve_entry_marker_ptr->pose.position = message->curve_entry;
	curve_entry_marker_ptr->scale.x = property_curve_entry_point_size_->getValue().toFloat();
	curve_entry_marker_ptr->scale.y = property_curve_entry_point_size_->getValue().toFloat();
	curve_entry_marker_ptr->scale.z = property_curve_entry_point_size_->getValue().toFloat();
	curve_entry_marker_ptr->color.r = property_curve_entry_color_->getColor().red();
	curve_entry_marker_ptr->color.g = property_curve_entry_color_->getColor().green();
	curve_entry_marker_ptr->color.b = property_curve_entry_color_->getColor().blue();
	curve_entry_marker_ptr->color.a = property_curve_entry_alpha_->getValue().toFloat();
	curve_entry_marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.0);
	curve_entry_marker_ptr->frame_locked = false;
	
	curve_entry_->addMessage(curve_entry_marker_ptr);
	
	visualization_msgs::msg::Marker::SharedPtr left_circle_marker_ptr = std::make_shared<visualization_msgs::msg::Marker>();
	left_circle_marker_ptr->header = message->header;
	left_circle_marker_ptr->ns = "left_circle";
	left_circle_marker_ptr->id = 0;
	left_circle_marker_ptr->type = visualization_msgs::msg::Marker::CYLINDER;
	left_circle_marker_ptr->action = visualization_msgs::msg::Marker::MODIFY;
	left_circle_marker_ptr->pose.position = message->left_circle.center;
	left_circle_marker_ptr->scale.x = message->left_circle.radius;
	left_circle_marker_ptr->scale.y = message->left_circle.radius;
	left_circle_marker_ptr->scale.z = 0.001;
	left_circle_marker_ptr->color.r = property_left_circle_color_->getColor().red();
	left_circle_marker_ptr->color.g = property_left_circle_color_->getColor().green();
	left_circle_marker_ptr->color.b = property_left_circle_color_->getColor().blue();
	left_circle_marker_ptr->color.a = property_left_circle_alpha_->getValue().toFloat();
	left_circle_marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.0);
	left_circle_marker_ptr->frame_locked = false;
	
	visualization_msgs::msg::Marker::SharedPtr right_circle_marker_ptr = std::make_shared<visualization_msgs::msg::Marker>();
	right_circle_marker_ptr->header = message->header;
	right_circle_marker_ptr->ns = "right_circle";
	right_circle_marker_ptr->id = 0;
	right_circle_marker_ptr->type = visualization_msgs::msg::Marker::CYLINDER;
	right_circle_marker_ptr->action = visualization_msgs::msg::Marker::MODIFY;
	right_circle_marker_ptr->pose.position = message->right_circle.center;
	right_circle_marker_ptr->scale.x = message->right_circle.radius;
	right_circle_marker_ptr->scale.y = message->right_circle.radius;
	right_circle_marker_ptr->scale.z = 0.001;
	right_circle_marker_ptr->color.r = property_right_circle_color_->getColor().red();
	right_circle_marker_ptr->color.g = property_right_circle_color_->getColor().green();
	right_circle_marker_ptr->color.b = property_right_circle_color_->getColor().blue();
	right_circle_marker_ptr->color.a = property_right_circle_alpha_->getValue().toFloat();
	right_circle_marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.0);
	right_circle_marker_ptr->frame_locked = false;
	
	visualization_msgs::msg::Marker::SharedPtr upper_circle_marker_ptr = std::make_shared<visualization_msgs::msg::Marker>();
	upper_circle_marker_ptr->header = message->header;
	upper_circle_marker_ptr->ns = "upper_circle";
	upper_circle_marker_ptr->id = 0;
	upper_circle_marker_ptr->type = visualization_msgs::msg::Marker::CYLINDER;
	upper_circle_marker_ptr->action = visualization_msgs::msg::Marker::MODIFY;
	upper_circle_marker_ptr->pose.position = message->upper_circle.center;
	upper_circle_marker_ptr->scale.x = message->upper_circle.radius;
	upper_circle_marker_ptr->scale.y = message->upper_circle.radius;
	upper_circle_marker_ptr->scale.z = 0.001;
	upper_circle_marker_ptr->color.r = property_upper_circle_color_->getColor().red();
	upper_circle_marker_ptr->color.g = property_upper_circle_color_->getColor().green();
	upper_circle_marker_ptr->color.b = property_upper_circle_color_->getColor().blue();
	upper_circle_marker_ptr->color.a = property_upper_circle_alpha_->getValue().toFloat();
	upper_circle_marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.0);
	upper_circle_marker_ptr->frame_locked = false;
	
	left_circle_->addMessage(left_circle_marker_ptr);
	right_circle_->addMessage(right_circle_marker_ptr);
	upper_circle_->addMessage(upper_circle_marker_ptr);
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::TrackDisplay, rviz_common::Display)
