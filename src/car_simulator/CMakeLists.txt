cmake_minimum_required(VERSION 3.5)
project(car_simulator)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros_its_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)

find_package(tf2_ros REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(laser_geometry REQUIRED)

find_package(autoware_auto_control_msgs REQUIRED)
find_package(autoware_auto_perception_msgs REQUIRED)
find_package(autoware_auto_planning_msgs REQUIRED)
find_package(autoware_auto_vehicle_msgs REQUIRED)
find_package(tier4_autoware_utils REQUIRED)
find_package(autoware_ad_api_specs REQUIRED)
find_package(component_interface_utils REQUIRED)

find_package(vehicle_info_util REQUIRED)

find_package(message_filters REQUIRED)


add_executable(localization_pose src/transformations/localization_pose.cpp)
ament_target_dependencies(localization_pose rclcpp std_msgs geometry_msgs nav_msgs tf2_ros tf2_kdl)

add_executable(simulation_pose src/transformations/simulation_pose.cpp)
ament_target_dependencies(simulation_pose rclcpp std_msgs geometry_msgs nav_msgs gazebo_msgs ros_its_msgs)

add_executable(globalTF src/transformations/global_tf.cpp)
ament_target_dependencies(globalTF rclcpp std_msgs gazebo_msgs geometry_msgs tf2_ros tf2_kdl)

add_executable(vehicle_interface src/autoware/vehicle_interface/vehicle_interface.cpp src/autoware/vehicle_interface/vehicle_interface_node.cpp)
ament_target_dependencies(vehicle_interface rclcpp std_msgs gazebo_msgs nav_msgs tf2_ros geometry_msgs autoware_auto_control_msgs autoware_auto_vehicle_msgs message_filters vehicle_info_util)

add_executable(path_conversion src/autoware/conversions/path_conversion.cpp)
ament_target_dependencies(path_conversion rclcpp std_msgs nav_msgs geometry_msgs autoware_auto_planning_msgs message_filters)

add_executable(objects_provider src/autoware/stub/objects_provider.cpp)
ament_target_dependencies(objects_provider rclcpp std_msgs autoware_auto_perception_msgs)

add_executable(tf_remap src/transformations/tf_remap.cpp)
ament_target_dependencies(tf_remap rclcpp std_msgs geometry_msgs tf2_ros)

add_executable(tf_copy src/transformations/tf_copy.cpp)
ament_target_dependencies(tf_copy rclcpp std_msgs geometry_msgs tf2_ros)

add_executable(tf_project src/transformations/tf_project.cpp)
ament_target_dependencies(tf_project rclcpp std_msgs geometry_msgs tf2_ros tf2_geometry_msgs)

add_executable(laser_to_pointcloud src/laser_to_pointcloud/laser_to_pointcloud.cpp)
ament_target_dependencies(laser_to_pointcloud rclcpp std_msgs sensor_msgs geometry_msgs tf2_ros laser_geometry)

add_executable(default_pose_initializer src/autoware/ad_api/default_pose_initializer.cpp)
target_compile_options(default_pose_initializer PRIVATE -DROS_DISTRO_GALACTIC)
ament_target_dependencies(default_pose_initializer rclcpp std_msgs geometry_msgs tf2_geometry_msgs tier4_autoware_utils autoware_ad_api_specs component_interface_utils)

install(TARGETS
  localization_pose
  simulation_pose
  globalTF
  vehicle_interface
  path_conversion
  objects_provider
  tf_remap
  tf_copy
  tf_project
  laser_to_pointcloud
  default_pose_initializer
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  config
  launch
  models
  rviz
  worlds
  maps
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include/
  DESTINATION include
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/teleoperation/teleop_node.py
  scripts/waypoint_publisher.py
  scripts/tf_converter.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_package()
