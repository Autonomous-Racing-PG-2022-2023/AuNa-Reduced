import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription

def generate_launch_description():
    # Launch Argument Configurations
    pkg_dir = get_package_share_directory('car_simulator')
    namespace = LaunchConfiguration('namespace', default='autoware')
    nav2_bringup_pkg_dir = get_package_share_directory('nav2_bringup')

    nav_launch_file_dir = os.path.join(nav2_bringup_pkg_dir, 'launch')
    default_rviz_config_file = os.path.join(pkg_dir, 'rviz','config.rviz')
    
    rviz_namespace = LaunchConfiguration('namespace', default='robot')
    rviz_config = LaunchConfiguration('rviz_config', default = default_rviz_config_file)
    use_namespace = LaunchConfiguration('use_namespace', default='true')

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
    
    rviz_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_launch_file_dir, 'rviz_launch.py')),
        launch_arguments={
            'namespace': rviz_namespace,
            'rviz_config': rviz_config,
            'use_namespace': use_namespace,
        }.items(),
    )

    # Launch Description
    ld = LaunchDescription()

    ld.add_action(start_autoware_joy_cmd)
    ld.add_action(rviz_launch_cmd)

    return ld