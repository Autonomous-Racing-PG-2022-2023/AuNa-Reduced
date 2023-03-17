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
: left_point_cloud_(std::make_unique<rviz_default_plugins::PointCloudCommon>(this))
, right_point_cloud_(std::make_unique<rviz_default_plugins::PointCloudCommon>(this))
, upper_point_cloud_(std::make_unique<rviz_default_plugins::PointCloudCommon>(this))
, curve_entry_(std::make_unique<rviz_default_plugins::displays::MarkerCommon>(this))
, car_position_(std::make_unique<rviz_default_plugins::displays::MarkerCommon>(this))
, left_circle_(std::make_unique<rviz_default_plugins::displays::MarkerCommon>(this))
, right_circle_(std::make_unique<rviz_default_plugins::displays::MarkerCommon>(this))
, upper_circle_(std::make_unique<rviz_default_plugins::displays::MarkerCommon>(this))
{
	property_curve_entry_view_ = new rviz_common::properties::BoolProperty("Curve Entry", true, "", this);
	property_car_position_view_ = new rviz_common::properties::BoolProperty("Car Position", true, "", this);
	
	/*
	property_left_cloud_view_ = new rviz_common::properties::BoolProperty("Left Cloud", true, "", this);
	property_right_cloud_view_ = new rviz_common::properties::BoolProperty("Right Cloud", true, "", this);
	property_upper_cloud_view_ = new rviz_common::properties::BoolProperty("Upper Cloud", true, "", this);
	*/
	
	property_left_circle_view_ = new rviz_common::properties::BoolProperty("Left Circle", true, "", this);
	property_right_circle_view_ = new rviz_common::properties::BoolProperty("Right Circle", true, "", this);
	property_upper_circle_view_ = new rviz_common::properties::BoolProperty("Upper Circle", true, "", this);
	
	property_curve_entry_point_size_ = new rviz_common::properties::FloatProperty("Point Size", 0.1, "", property_curve_entry_view_);
	property_curve_entry_point_size_->setMin(0.0);
	
	property_car_position_point_size_ = new rviz_common::properties::FloatProperty("Point Size", 0.1, "", property_car_position_view_);
	property_car_position_point_size_->setMin(0.0);
	
	property_left_circle_line_width_ = new rviz_common::properties::FloatProperty("Line Width", 0.1, "", property_left_circle_view_);
	property_left_circle_line_width_->setMin(0.0);
	property_right_circle_line_width_ = new rviz_common::properties::FloatProperty("Line Width", 0.1, "", property_right_circle_view_);
	property_right_circle_line_width_->setMin(0.0);
	property_upper_circle_line_width_ = new rviz_common::properties::FloatProperty("Line Width", 0.1, "", property_upper_circle_view_);
	property_upper_circle_line_width_->setMin(0.0);
	
	property_curve_entry_alpha_ = new rviz_common::properties::FloatProperty("Alpha", 1.0, "", property_curve_entry_view_);
	property_curve_entry_alpha_->setMin(0.0);
	property_curve_entry_alpha_->setMax(1.0);
	
	property_car_position_alpha_ = new rviz_common::properties::FloatProperty("Alpha", 1.0, "", property_car_position_view_);
	property_car_position_alpha_->setMin(0.0);
	property_car_position_alpha_->setMax(1.0);
	
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
	
	property_car_position_color_ = new rviz_common::properties::ColorProperty("Color", Qt::white, "", property_car_position_view_);
	
	property_left_circle_color_ = new rviz_common::properties::ColorProperty("Color", Qt::white, "", property_left_circle_view_);
	property_right_circle_color_ = new rviz_common::properties::ColorProperty("Color", Qt::white, "", property_right_circle_view_);
	property_upper_circle_color_ = new rviz_common::properties::ColorProperty("Color", Qt::white, "", property_upper_circle_view_);
	
	left_point_cloud_->selectable_property_->setName(QString("Left Cloud: ").append(left_point_cloud_->selectable_property_->getName()));
	left_point_cloud_->point_world_size_property_->setName(QString("Left Cloud: ").append(left_point_cloud_->point_world_size_property_->getName()));
	left_point_cloud_->point_pixel_size_property_->setName(QString("Left Cloud: ").append(left_point_cloud_->point_pixel_size_property_->getName()));
	left_point_cloud_->alpha_property_->setName(QString("Left Cloud: ").append(left_point_cloud_->alpha_property_->getName()));
	left_point_cloud_->xyz_transformer_property_->setName(QString("Left Cloud: ").append(left_point_cloud_->xyz_transformer_property_->getName()));
	left_point_cloud_->color_transformer_property_->setName(QString("Left Cloud: ").append(left_point_cloud_->color_transformer_property_->getName()));
	left_point_cloud_->style_property_->setName(QString("Left Cloud: ").append(left_point_cloud_->style_property_->getName()));
	left_point_cloud_->decay_time_property_->setName(QString("Left Cloud: ").append(left_point_cloud_->decay_time_property_->getName()));
	
	right_point_cloud_->selectable_property_->setName(QString("Right Cloud: ").append(right_point_cloud_->selectable_property_->getName()));
	right_point_cloud_->point_world_size_property_->setName(QString("Right Cloud: ").append(right_point_cloud_->point_world_size_property_->getName()));
	right_point_cloud_->point_pixel_size_property_->setName(QString("Right Cloud: ").append(right_point_cloud_->point_pixel_size_property_->getName()));
	right_point_cloud_->alpha_property_->setName(QString("Right Cloud: ").append(right_point_cloud_->alpha_property_->getName()));
	right_point_cloud_->xyz_transformer_property_->setName(QString("Right Cloud: ").append(right_point_cloud_->xyz_transformer_property_->getName()));
	right_point_cloud_->color_transformer_property_->setName(QString("Right Cloud: ").append(right_point_cloud_->color_transformer_property_->getName()));
	right_point_cloud_->style_property_->setName(QString("Right Cloud: ").append(right_point_cloud_->style_property_->getName()));
	right_point_cloud_->decay_time_property_->setName(QString("Right Cloud: ").append(right_point_cloud_->decay_time_property_->getName()));
	
	upper_point_cloud_->selectable_property_->setName(QString("Upper Cloud: ").append(upper_point_cloud_->selectable_property_->getName()));
	upper_point_cloud_->point_world_size_property_->setName(QString("Upper Cloud: ").append(upper_point_cloud_->point_world_size_property_->getName()));
	upper_point_cloud_->point_pixel_size_property_->setName(QString("Upper Cloud: ").append(upper_point_cloud_->point_pixel_size_property_->getName()));
	upper_point_cloud_->alpha_property_->setName(QString("Upper Cloud: ").append(upper_point_cloud_->alpha_property_->getName()));
	upper_point_cloud_->xyz_transformer_property_->setName(QString("Upper Cloud: ").append(upper_point_cloud_->xyz_transformer_property_->getName()));
	upper_point_cloud_->color_transformer_property_->setName(QString("Upper Cloud: ").append(upper_point_cloud_->color_transformer_property_->getName()));
	upper_point_cloud_->style_property_->setName(QString("Upper Cloud: ").append(upper_point_cloud_->style_property_->getName()));
	upper_point_cloud_->decay_time_property_->setName(QString("Upper Cloud: ").append(upper_point_cloud_->decay_time_property_->getName()));
	
	/*
	property_left_cloud_view_->addChild(left_point_cloud_->selectable_property_->getParent()->getParent()->takeChild(left_point_cloud_->selectable_property_));
	property_left_cloud_view_->addChild(left_point_cloud_->point_world_size_property_->getParent()->takeChild(left_point_cloud_->point_world_size_property_));
	property_left_cloud_view_->addChild(left_point_cloud_->point_pixel_size_property_->getParent()->takeChild(left_point_cloud_->point_pixel_size_property_));
	property_left_cloud_view_->addChild(left_point_cloud_->alpha_property_->getParent()->takeChild(left_point_cloud_->alpha_property_));
	property_left_cloud_view_->addChild(left_point_cloud_->xyz_transformer_property_->getParent()->takeChild(left_point_cloud_->xyz_transformer_property_));
	property_left_cloud_view_->addChild(left_point_cloud_->color_transformer_property_->getParent()->takeChild(left_point_cloud_->color_transformer_property_));
	property_left_cloud_view_->addChild(left_point_cloud_->style_property_->getParent()->takeChild(left_point_cloud_->style_property_));
	property_left_cloud_view_->addChild(left_point_cloud_->decay_time_property_->getParent()->takeChild(left_point_cloud_->decay_time_property_));
	
	property_right_cloud_view_->addChild(right_point_cloud_->selectable_property_->getParent()->takeChild(right_point_cloud_->selectable_property_));
	property_right_cloud_view_->addChild(right_point_cloud_->point_world_size_property_->getParent()->takeChild(right_point_cloud_->point_world_size_property_));
	property_right_cloud_view_->addChild(right_point_cloud_->point_pixel_size_property_->getParent()->takeChild(right_point_cloud_->point_pixel_size_property_));
	property_right_cloud_view_->addChild(right_point_cloud_->alpha_property_->getParent()->takeChild(right_point_cloud_->alpha_property_));
	property_right_cloud_view_->addChild(right_point_cloud_->xyz_transformer_property_->getParent()->takeChild(right_point_cloud_->xyz_transformer_property_));
	property_right_cloud_view_->addChild(right_point_cloud_->color_transformer_property_->getParent()->takeChild(right_point_cloud_->color_transformer_property_));
	property_right_cloud_view_->addChild(right_point_cloud_->style_property_->getParent()->takeChild(right_point_cloud_->style_property_));
	property_right_cloud_view_->addChild(right_point_cloud_->decay_time_property_->getParent()->takeChild(right_point_cloud_->decay_time_property_));
	
	property_upper_cloud_view_->addChild(upper_point_cloud_->selectable_property_->getParent()->takeChild(upper_point_cloud_->selectable_property_));
	property_upper_cloud_view_->addChild(upper_point_cloud_->point_world_size_property_->getParent()->takeChild(upper_point_cloud_->point_world_size_property_));
	property_upper_cloud_view_->addChild(upper_point_cloud_->point_pixel_size_property_->getParent()->takeChild(upper_point_cloud_->point_pixel_size_property_));
	property_upper_cloud_view_->addChild(upper_point_cloud_->alpha_property_->getParent()->takeChild(upper_point_cloud_->alpha_property_));
	property_upper_cloud_view_->addChild(upper_point_cloud_->xyz_transformer_property_->getParent()->takeChild(upper_point_cloud_->xyz_transformer_property_));
	property_upper_cloud_view_->addChild(upper_point_cloud_->color_transformer_property_->getParent()->takeChild(upper_point_cloud_->color_transformer_property_));
	property_upper_cloud_view_->addChild(upper_point_cloud_->style_property_->getParent()->takeChild(upper_point_cloud_->style_property_));
	property_upper_cloud_view_->addChild(upper_point_cloud_->decay_time_property_->getParent()->takeChild(upper_point_cloud_->decay_time_property_));
	*/
	
	
}

TrackDisplay::~TrackDisplay() = default;  // Properties are deleted by Qt

void TrackDisplay::onInitialize()
{
	MFDClass::onInitialize();
	
	left_point_cloud_->initialize(this->context_, this->scene_node_);
	right_point_cloud_->initialize(this->context_, this->scene_node_);
	upper_point_cloud_->initialize(this->context_, this->scene_node_);
	
	curve_entry_->initialize(this->context_, this->scene_node_);
	car_position_->initialize(this->context_, this->scene_node_);
	
	left_circle_->initialize(this->context_, this->scene_node_);
	right_circle_->initialize(this->context_, this->scene_node_);
	upper_circle_->initialize(this->context_, this->scene_node_);
}

void TrackDisplay::onEnable() {
	rviz_common::MessageFilterDisplay<car_simulator_msgs::msg::Track>::subscribe();
}

void TrackDisplay::onDisable() {
	rviz_common::MessageFilterDisplay<car_simulator_msgs::msg::Track>::unsubscribe();
	
	left_point_cloud_->onDisable();
	right_point_cloud_->onDisable();
	upper_point_cloud_->onDisable();
}

void TrackDisplay::update(float wall_dt, float ros_dt)
{
	left_point_cloud_->update(wall_dt, ros_dt);
	right_point_cloud_->update(wall_dt, ros_dt);
	upper_point_cloud_->update(wall_dt, ros_dt);
	
	curve_entry_->update(wall_dt, ros_dt);
	car_position_->update(wall_dt, ros_dt);
	
	left_circle_->update(wall_dt, ros_dt);
	right_circle_->update(wall_dt, ros_dt);
	upper_circle_->update(wall_dt, ros_dt);
}

void TrackDisplay::reset() {
	rviz_common::MessageFilterDisplay<car_simulator_msgs::msg::Track>::subscribe();
	
	left_point_cloud_->reset();
	right_point_cloud_->reset();
	upper_point_cloud_->reset();
	
	curve_entry_->clearMarkers();
	car_position_->clearMarkers();
	
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
	
	visualization_msgs::msg::Marker::SharedPtr car_position_marker_ptr = std::make_shared<visualization_msgs::msg::Marker>();
	car_position_marker_ptr->header = message->header;
	car_position_marker_ptr->ns = "car_position";
	car_position_marker_ptr->id = 0;
	car_position_marker_ptr->type = visualization_msgs::msg::Marker::SPHERE;
	car_position_marker_ptr->action = visualization_msgs::msg::Marker::MODIFY;
	car_position_marker_ptr->pose.position = message->car_position;
	car_position_marker_ptr->scale.x = property_car_position_point_size_->getValue().toFloat();
	car_position_marker_ptr->scale.y = property_car_position_point_size_->getValue().toFloat();
	car_position_marker_ptr->scale.z = property_car_position_point_size_->getValue().toFloat();
	car_position_marker_ptr->color.r = property_car_position_color_->getColor().red();
	car_position_marker_ptr->color.g = property_car_position_color_->getColor().green();
	car_position_marker_ptr->color.b = property_car_position_color_->getColor().blue();
	car_position_marker_ptr->color.a = property_car_position_alpha_->getValue().toFloat();
	car_position_marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.0);
	car_position_marker_ptr->frame_locked = false;
	
	curve_entry_->addMessage(curve_entry_marker_ptr);
	car_position_->addMessage(car_position_marker_ptr);
	
	visualization_msgs::msg::Marker::SharedPtr left_circle_marker_ptr = std::make_shared<visualization_msgs::msg::Marker>();
	left_circle_marker_ptr->header = message->header;
	left_circle_marker_ptr->ns = "left_circle";
	left_circle_marker_ptr->id = 0;
	left_circle_marker_ptr->type = visualization_msgs::msg::Marker::CYLINDER;
	left_circle_marker_ptr->action = visualization_msgs::msg::Marker::MODIFY;
	left_circle_marker_ptr->pose.position = message->left_circle.center;
	left_circle_marker_ptr->scale.x = property_left_circle_line_width_->getValue().toFloat();
	left_circle_marker_ptr->scale.y = property_left_circle_line_width_->getValue().toFloat();
	left_circle_marker_ptr->scale.z = 0.0;
	left_circle_marker_ptr->color.r = property_left_circle_color_->getColor().red();
	left_circle_marker_ptr->color.g = property_left_circle_color_->getColor().green();
	left_circle_marker_ptr->color.b = property_left_circle_color_->getColor().blue();
	left_circle_marker_ptr->color.a = property_left_circle_alpha_->getValue().toFloat();
	left_circle_marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.0);
	left_circle_marker_ptr->frame_locked = false;
	
	visualization_msgs::msg::Marker::SharedPtr right_circle_marker_ptr = std::make_shared<visualization_msgs::msg::Marker>();
	right_circle_marker_ptr->header = message->header;
	right_circle_marker_ptr->ns = "left_circle";
	right_circle_marker_ptr->id = 0;
	right_circle_marker_ptr->type = visualization_msgs::msg::Marker::CYLINDER;
	right_circle_marker_ptr->action = visualization_msgs::msg::Marker::MODIFY;
	right_circle_marker_ptr->pose.position = message->left_circle.center;
	right_circle_marker_ptr->scale.x = property_right_circle_line_width_->getValue().toFloat();
	right_circle_marker_ptr->scale.y = property_right_circle_line_width_->getValue().toFloat();
	right_circle_marker_ptr->scale.z = 0.0;
	right_circle_marker_ptr->color.r = property_right_circle_color_->getColor().red();
	right_circle_marker_ptr->color.g = property_right_circle_color_->getColor().green();
	right_circle_marker_ptr->color.b = property_right_circle_color_->getColor().blue();
	right_circle_marker_ptr->color.a = property_right_circle_alpha_->getValue().toFloat();
	right_circle_marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.0);
	right_circle_marker_ptr->frame_locked = false;
	
	visualization_msgs::msg::Marker::SharedPtr upper_circle_marker_ptr = std::make_shared<visualization_msgs::msg::Marker>();
	upper_circle_marker_ptr->header = message->header;
	upper_circle_marker_ptr->ns = "left_circle";
	upper_circle_marker_ptr->id = 0;
	upper_circle_marker_ptr->type = visualization_msgs::msg::Marker::CYLINDER;
	upper_circle_marker_ptr->action = visualization_msgs::msg::Marker::MODIFY;
	upper_circle_marker_ptr->pose.position = message->left_circle.center;
	upper_circle_marker_ptr->scale.x = property_upper_circle_line_width_->getValue().toFloat();
	upper_circle_marker_ptr->scale.y = property_upper_circle_line_width_->getValue().toFloat();
	upper_circle_marker_ptr->scale.z = 0.0;
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
