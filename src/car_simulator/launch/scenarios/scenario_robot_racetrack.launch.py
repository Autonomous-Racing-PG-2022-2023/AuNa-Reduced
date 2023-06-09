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
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,OpaqueFunction
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from car_simulator import yaml_launch

def launch_actions(context: LaunchContext):

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
    path_generation_file_dir = os.path.join(pkg_dir, 'launch', 'path_generation')
    rviz_file_dir = os.path.join(pkg_dir, 'launch', 'rviz')
    communication_launch_file_dir = os.path.join(pkg_dir, 'launch', 'communication')

    # Launch Argument Configurations
    robot_number = LaunchConfiguration('robot_number', default='1')
    world_name = LaunchConfiguration('world_name', default='racetrack_decorated')
    
    # Nodes and other launch files
    robots = []
    for num in range(int(robot_number.perform(context))):
        robots.append(
            {
                'name': 'robot'+str(num), 
                'namespace': 'robot'+str(num), 
                'x_pose': yaml_launch.get_yaml_value("map_params", world_name.perform(context), ["spawn","offset","x"]) + num * yaml_launch.get_yaml_value("map_params", world_name.perform(context), ["spawn","linear","x"]),
                'y_pose': yaml_launch.get_yaml_value("map_params", world_name.perform(context), ["spawn","offset","y"]) + num * yaml_launch.get_yaml_value("map_params", world_name.perform(context), ["spawn","linear","y"]),
                'z_pose': yaml_launch.get_yaml_value("map_params", world_name.perform(context), ["spawn","offset","z"]) + num * yaml_launch.get_yaml_value("map_params", world_name.perform(context), ["spawn","linear","z"]),
            }
        )
    
    cmds = []
    for robot in robots:
        spawn_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(spawn_launch_file_dir, 'spawn_single_robot.launch.py')),
            launch_arguments={
                'world_name': world_name,
                'name': robot['name'],
                'namespace': robot['namespace']+'/',
                'urdf_namespace': robot['namespace']+'/',
                'x_pose': TextSubstitution(text=str(robot['x_pose'])),
                'y_pose': TextSubstitution(text=str(robot['y_pose'])),
                'z_pose': TextSubstitution(text=str(robot['z_pose']))
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
        
        wallfollowing_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(path_generation_file_dir, 'wallfollowing.launch.py')),
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

        communicaton_launch_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(communication_launch_file_dir, 'communication.launch.py')),
                launch_arguments = {        
                }.items(),
        )
        
        cmds.append(spawn_cmd);
        cmds.append(tf_remap_cmd);
        cmds.append(laser_cmd);
        cmds.append(vehicle_interface_cmd);
        cmds.append(planning_cmd);
        cmds.append(perception_cmd);
        cmds.append(sensing_cmd);
        cmds.append(localization_cmd);
        #cmds.append(nav2_localization_cmd);
        cmds.append(lidarslam_cmd);
        cmds.append(wallfollowing_cmd);
        cmds.append(control_cmd);
        cmds.append(system_cmd);
        cmds.append(rviz_cmd);
        cmds.append(communicaton_launch_cmd);

    return cmds

def generate_launch_description():
    # Package Directories
    pkg_dir = get_package_share_directory('car_simulator')

    # Paths to folders and files
    gazebo_launch_file_dir = os.path.join(pkg_dir, 'launch', 'gazebo')

    # Launch Argument Configurations
    robot_number = LaunchConfiguration('robot_number', default='2')
    world_name = LaunchConfiguration('world_name', default='racetrack_decorated')

    # Nodes and other launch files
    world_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_launch_file_dir, 'gazebo_world.launch.py')),
        launch_arguments={
            'world_name': world_name
        }.items(),
    )

    # Launch Arguments
    robot_number_arg = DeclareLaunchArgument(
        'robot_number',
        default_value='1',
        description='Number of spawned robots'
    )
    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='racetrack_decorated',
        description='Gazebo world file name'
    )
    
    # Launch Description
    ld = LaunchDescription()

    ld.add_action(robot_number_arg)
    ld.add_action(world_name_arg)
    
    ld.add_action(world_cmd)

    ld.add_action(OpaqueFunction(function=launch_actions))

    return ld
