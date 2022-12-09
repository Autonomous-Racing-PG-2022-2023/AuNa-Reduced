import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription

def generate_launch_description():
    # Launch Argument Configurations
    namespace = LaunchConfiguration('namespace', default='autoware')

    # Nodes and other launch files
    start_autoware_joy_cmd = Node(
        package='joy_controller',
        executable='joy_controller',
        name='joy_controller',
        namespace = namespace,
        output='screen',
        arguments=[
        ]
    )

    # Launch Description
    ld = LaunchDescription()

    ld.add_action(start_autoware_joy_cmd)

    return ld