import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch_ros.actions import Node, SetRemap
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription

def generate_launch_description():
    # Launch Argument Configurations
    pkg_dir = get_package_share_directory('car_simulator')
    nav2_bringup_pkg_dir = get_package_share_directory('nav2_bringup')

    nav_launch_file_dir = os.path.join(nav2_bringup_pkg_dir, 'launch')
    default_rviz_config_file = os.path.join(pkg_dir, 'rviz','config.rviz')
    
    namespace = LaunchConfiguration('namespace', default='')
    rviz_config = LaunchConfiguration('rviz_config', default = default_rviz_config_file)
    use_namespace = LaunchConfiguration('use_namespace', default='true')
        
    remap_control_cmd = SetRemap(src='/external/selected/control_cmd', dst='/control/command/control_cmd');
    
    rviz_launch_cmd = GroupAction(
        actions = [
            remap_control_cmd,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav_launch_file_dir, 'rviz_launch.py')),
                launch_arguments={
                    'namespace': namespace,
                    'rviz_config': rviz_config,
                    'use_namespace': use_namespace,
                }.items(),
            )
        ]
    )
    
    frame_remap_cmd = Node(
        package='car_simulator',
        executable='frame_remap',
        name='frame_remap',
        namespace = namespace,
        output='screen',
        parameters=[
            {
                'src': namespace,
                'dst': ''
            }
        ]
    )

    # Launch Description
    ld = LaunchDescription()

    ld.add_action(rviz_launch_cmd)
    ld.add_action(frame_remap_cmd)

    return ld