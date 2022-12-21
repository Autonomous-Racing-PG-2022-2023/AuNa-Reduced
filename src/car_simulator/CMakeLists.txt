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

find_package(autoware_auto_control_msgs REQUIRED)
find_package(autoware_auto_vehicle_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(vehicle_info_util REQUIRED)


add_executable(localization_pose src/transformations/localization_pose.cpp)
ament_target_dependencies(localization_pose rclcpp std_msgs geometry_msgs nav_msgs tf2_ros tf2_kdl)

add_executable(simulation_pose src/transformations/simulation_pose.cpp)
ament_target_dependencies(simulation_pose rclcpp std_msgs geometry_msgs nav_msgs gazebo_msgs ros_its_msgs)

add_executable(globalTF src/transformations/global_tf.cpp)
ament_target_dependencies(globalTF rclcpp std_msgs gazebo_msgs geometry_msgs tf2_ros tf2_kdl)

add_executable(vehicle_interface src/autoware/vehicle_interface/vehicle_interface.cpp src/autoware/vehicle_interface/vehicle_interface_node.cpp)
ament_target_dependencies(vehicle_interface rclcpp std_msgs gazebo_msgs geometry_msgs autoware_auto_control_msgs autoware_auto_vehicle_msgs message_filters vehicle_info_util)

add_executable(frame_remap src/autoware/rviz_transform/frame_remap.cpp)
ament_target_dependencies(frame_remap rclcpp std_msgs geometry_msgs tf2_ros)

install(TARGETS
  localization_pose
  simulation_pose
  globalTF
  vehicle_interface
  frame_remap
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
