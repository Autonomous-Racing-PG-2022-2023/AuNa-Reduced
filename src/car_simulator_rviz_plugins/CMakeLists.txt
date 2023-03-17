cmake_minimum_required(VERSION 3.5)
project(car_simulator_rviz_plugins)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_rendering REQUIRED)

find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Core Qt5::Widgets)

find_package(car_simulator_msgs REQUIRED)

set(car_simulator_rviz_plugins_headers_to_moc
	include/track_display/track_display.hpp
)

foreach(header "${car_simulator_rviz_plugins_headers_to_moc}")
  qt5_wrap_cpp(car_simulator_rviz_plugins_moc_files "${header}")
endforeach()

add_library(car_simulator_rviz_plugins SHARED ${car_simulator_rviz_plugins_moc_files} src/track_display/track_display.cpp)
target_include_directories(car_simulator_rviz_plugins PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>${Qt5Widgets_INCLUDE_DIRS})
target_link_libraries(car_simulator_rviz_plugins ${QT_LIBRARIES})
target_compile_definitions(car_simulator_rviz_plugins PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")# Causes the visibility macros to use dllexport rather than dllimport (for Windows, when your plugin should be used as library)
ament_target_dependencies(car_simulator_rviz_plugins rclcpp std_msgs sensor_msgs visualization_msgs rviz_common rviz_default_plugins rviz_rendering car_simulator_msgs)

add_definitions(-DQT_NO_KEYWORDS)

# Export the plugin to be imported by rviz2
pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)

install(
  TARGETS car_simulator_rviz_plugins
  EXPORT car_simulator_rviz_plugins
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
  INCLUDES DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_targets(car_simulator_rviz_plugins HAS_LIBRARY_TARGET)
ament_export_dependencies(rosidl_default_runtime)

ament_package()

