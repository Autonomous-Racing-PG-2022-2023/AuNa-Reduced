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
    autowarelaunch_file_dir = os.path.join(pkg_dir, 'launch', 'autoware')
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
    controller_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(autowarelaunch_file_dir, 'joy_controller.launch.py')),
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
    
    # Launch Description
    ld = LaunchDescription()

    ld.add_action(world_name_arg)

    ld.add_action(world_cmd)
    ld.add_action(spawn_cmd)
    #ld.add_action(controller_cmd)
    #ld.add_action(vehicle_interface_cmd)
    ld.add_action(rviz_cmd)

    return ld