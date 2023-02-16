#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Joep Tool

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Package Directories
    pkg_dir = get_package_share_directory('car_simulator')

    # Paths to folders and files
    gazebo_launch_file_dir = os.path.join(pkg_dir, 'launch', 'gazebo')
    spawn_launch_file_dir = os.path.join(pkg_dir, 'launch', 'spawn')
    tf_remap_file_dir = os.path.join(pkg_dir, 'launch', 'tf_remap')
    sensing_launch_file_dir = os.path.join(pkg_dir, 'launch', 'sensing')
    autowarelaunch_file_dir = os.path.join(pkg_dir, 'launch', 'autoware')
    nav2_file_dir = os.path.join(pkg_dir, 'launch', 'nav2')
    lidarslam_file_dir = os.path.join(pkg_dir, 'launch', 'lidarslam')
    rviz_file_dir = os.path.join(pkg_dir, 'launch', 'rviz')

    # Launch Argument Configurations
    world_name = LaunchConfiguration('world_name', default='racetrack_decorated')

    # Launch Arguments
    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='racetrack_decorated',
        description='Gazebo world file name'
    )
    
    # Nodes and other launch files
    world_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_launch_file_dir, 'gazebo_world.launch.py')),
        launch_arguments={
            'world_name': world_name
        }.items(),
    )
    
    spawn_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(spawn_launch_file_dir, 'spawn_single_robot.launch.py')),
        launch_arguments={
            'world_name': world_name
        }.items(),
    )
    
    laser_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sensing_launch_file_dir, 'laser.launch.py')),
        launch_arguments={
            'world_name': world_name
        }.items(),
    )
    
    tf_remap_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(autowarelaunch_file_dir, 'tf_remap.launch.py')),
        launch_arguments={
        }.items(),
    )
    
    vehicle_interface_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(autowarelaunch_file_dir, 'vehicle_interface.launch.py')),
        launch_arguments={
        }.items(),
    )
    
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(rviz_file_dir, 'rviz.launch.py')),
        launch_arguments={
        }.items(),
    )
    
    planning_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(autowarelaunch_file_dir, 'planning.launch.py')),
        launch_arguments={
        }.items(),
    )
    
    perception_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(autowarelaunch_file_dir, 'perception.launch.py')),
        launch_arguments={
        }.items(),
    )
    
    sensing_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(autowarelaunch_file_dir, 'sensing.launch.py')),
        launch_arguments={
        }.items(),
    )
    
    localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(autowarelaunch_file_dir, 'localization.launch.py')),
        launch_arguments={
        }.items(),
    )
    
    nav2_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_file_dir, 'localization.launch.py')),
        launch_arguments={
        }.items(),
    )
    
    lidarslam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(lidarslam_file_dir, 'slam.launch.py')),
        launch_arguments={
        }.items(),
    )
    
    control_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(autowarelaunch_file_dir, 'control.launch.py')),
        launch_arguments={
        }.items(),
    )
    
    system_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(autowarelaunch_file_dir, 'system.launch.py')),
        launch_arguments={
        }.items(),
    )
    
    # Launch Description
    ld = LaunchDescription()

    ld.add_action(world_name_arg)

    ld.add_action(world_cmd)
    ld.add_action(spawn_cmd)
    ld.add_action(tf_remap_cmd)
    ld.add_action(laser_cmd)
    ld.add_action(vehicle_interface_cmd)
    ld.add_action(planning_cmd)
    ld.add_action(perception_cmd)
    ld.add_action(sensing_cmd)
    ld.add_action(localization_cmd)
    #ld.add_action(nav2_localization_cmd)
    ld.add_action(lidarslam_cmd)
    ld.add_action(control_cmd)
    ld.add_action(system_cmd)
    ld.add_action(rviz_cmd)

    return ld