import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch_ros.actions import Node, SetRemap, PushRosNamespace
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch import LaunchDescription
from car_simulator import yaml_launch, SetParametersFromFile

def generate_launch_description():
    # Package Directories
    pkg_dir = get_package_share_directory('car_simulator')

    # Launch Argument Configurations
    namespace = LaunchConfiguration('namespace', default='')
    
    #TODO:Use Autoware laserscan_to_pointcloud instead
    
    #TODO:Add other parameters
    #FIXME:Check correctness of parameters
    #TODO:Parameters in external file
    
    #TODO: GroupAction and remapps
    start_laserscan_to_pointcloud_cmd = Node(
        package='car_simulator',
        executable='laser_to_pointcloud',
        name='laserscan_to_pointcloud',
        namespace = namespace,
        output='screen',
        parameters=[
            {
                'target_frame': [namespace, 'base_link'],
                'src_topic': 'laser/out',
                'dst_topic': 'points_raw',
                'range_cutoff': -1.0,
                'channel_option': 0x1 | 0x8 #TODO: Use python lib enum
            }
        ]
    )

    # Launch Description
    ld = LaunchDescription()

    ld.add_action(start_laserscan_to_pointcloud_cmd)

    return ld