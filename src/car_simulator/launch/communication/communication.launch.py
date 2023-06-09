import os 
from ament_index_python.packages import get_package_share_directory 
from launch.substitutions import LaunchConfiguration 
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription 
from launch_ros.actions import Node

def generate_launch_description():

    pkg_dir = get_package_share_directory('car_simulator')

    namespace = LaunchConfiguration('namespace', default='')

    start_publisher_node_cmd = Node(
            package = 'communication',
            executable = 'publisher_node',
            name = 'publisher_node',
            output = 'screen'
            )

    start_subscriber_node_cmd = Node(
            package = 'communication',
            executable = 'subscriber_node',
            name = 'subscriber_node', 
            output = 'screen'
            )

    ld = LaunchDescription()

    ld.add_action(start_publisher_node_cmd)
    ld.add_action(start_subscriber_node_cmd)

    return ld
