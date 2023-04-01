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
            SetRemap(src='/api/autoware/get/emergency', dst='api/autoware/get/emergency'),
            SetRemap(src='/api/autoware/get/engage', dst='api/autoware/get/engage'),
            SetRemap(src='/api/fail_safe/mrm_state', dst='api/fail_safe/mrm_state'),
            SetRemap(src='/api/localization/initialization_state', dst='api/localization/initialization_state'),
            SetRemap(src='/api/motion/state', dst='api/motion/state'),
            SetRemap(src='/api/operation_mode/state', dst='api/operation_mode/state'),
            SetRemap(src='/api/routing/state', dst='api/routing/state'),
            SetRemap(src='/api/autoware/set/emergency', dst='api/autoware/set/emergency'),
            SetRemap(src='/api/autoware/set/engage', dst='api/autoware/set/engage'),
            SetRemap(src='/api/motion/accept_start', dst='api/motion/accept_start'),
            SetRemap(src='/api/operation_mode/change_to_autonomous', dst='api/operation_mode/change_to_autonomous'),
            SetRemap(src='/api/operation_mode/change_to_local', dst='api/operation_mode/change_to_local'),
            SetRemap(src='/api/operation_mode/change_to_remote', dst='api/operation_mode/change_to_remote'),
            SetRemap(src='/api/operation_mode/change_to_stop', dst='api/operation_mode/change_to_stop'),
            SetRemap(src='/api/operation_mode/disable_autoware_control', dst='api/operation_mode/disable_autoware_control'),
            SetRemap(src='/api/operation_mode/enable_autoware_control', dst='api/operation_mode/enable_autoware_control'),
            SetRemap(src='/api/routing/clear_route', dst='api/routing/clear_route'),
            SetRemap(src='/control/gate_mode_cmd', dst='control/gate_mode_cmd'),
            SetRemap(src='/control/current_gate_mode', dst='control/current_gate_mode'),
            SetRemap(src='/external/selected/gear_cmd', dst='external/selected/gear_cmd'),
            SetRemap(src='/external/selected/control_cmd', dst='external/selected/control_cmd'),
            SetRemap(src='/vehicle/status/gear_status', dst='vehicle/status/gear_status'),
            SetRemap(src='/vehicle/status/velocity_status', dst='vehicle/status/velocity_status'),
            SetRemap(src='/vehicle/status/steering_status', dst='vehicle/status/steering_status'),
            SetRemap(src='/planning/scenario_planning/current_max_velocity', dst='planning/scenario_planning/current_max_velocity'),
            SetRemap(src='/planning/scenario_planning/max_velocity_default', dst='planning/scenario_planning/max_velocity_default'),
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