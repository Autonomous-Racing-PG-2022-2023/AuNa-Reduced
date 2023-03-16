import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch_ros.actions import Node, SetRemap
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from car_simulator import SetParametersFromFile

def generate_launch_description():
    # Launch Argument Configurations
    pkg_dir = get_package_share_directory('car_simulator')
    nav2_bringup_pkg_dir = get_package_share_directory('nav2_bringup')

    nav_launch_file_dir = os.path.join(nav2_bringup_pkg_dir, 'launch')
    default_rviz_config_file = os.path.join(pkg_dir, 'rviz','config.rviz')
    
    config_file_vehicle = os.path.join(pkg_dir, 'config', 'model_params', 'vehicle_interface.yaml')
    
    namespace = LaunchConfiguration('namespace', default='')
    rviz_config = LaunchConfiguration('rviz_config', default = default_rviz_config_file)
    use_namespace = LaunchConfiguration('use_namespace', default='true')
    
    remap_tf = SetRemap(src='/tf', dst='autoware/tf');
    remap_tf_static = SetRemap(src='/tf_static', dst='autoware/tf_static');
    
    rviz_launch_cmd = GroupAction(
        actions = [
            remap_tf,
            remap_tf_static,
            SetRemap(src='/external/selected/control_cmd', dst='control/command/control_cmd'),
            SetRemap(src='/vehicle/status/velocity_status', dst='vehicle/status/velocity_status'),
            SetRemap(src='/vehicle/status/steering_status', dst='vehicle/status/steering_status'),
            SetParametersFromFile(config_file_vehicle),
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

    # Launch Description
    ld = LaunchDescription()

    ld.add_action(rviz_launch_cmd)

    return ld