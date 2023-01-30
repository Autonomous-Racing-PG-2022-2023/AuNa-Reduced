import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch_ros.actions import Node, SetRemap
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription

def generate_launch_description():
     # Package Directories
    pkg_dir = get_package_share_directory('car_simulator')
    

    # Launch Argument Configurations
    namespace = LaunchConfiguration('namespace', default='')

    tf_remap_cmd = Node(
        package='car_simulator',
        executable='tf_remap',
        name='tf_remap',
        namespace = namespace,
        output='screen',
        parameters=[
            {
                'src': namespace,
                'dst': '',
                'src_tf': 'tf',
                'dst_tf': 'autoware/tf',
                'src_tf_static': 'tf_static',
                'dst_tf_static': 'autoware/tf_static'
            }
        ],
    )
        
    # Launch Description
    ld = LaunchDescription()

    ld.add_action(tf_remap_cmd)

    return ld