from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    # Nodes and other launch files
    start_autoware_joy_cmd = Node(
        node_name='joy_controller',
    )

    # Launch Description
    ld = LaunchDescription()

    ld.add_action(start_autoware_joy_cmd)

    return ld