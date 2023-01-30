import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch_ros.actions import Node, SetRemap, PushRosNamespace
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch import LaunchDescription

def generate_launch_description():
    # Package Directories
    pkg_dir = get_package_share_directory('car_simulator')
    
    # Paths to folders and files
    config_file_vehicle = os.path.join(pkg_dir, 'config', 'model_params', 'vehicle_interface.yaml')
    
    config_dir_trajectory_follower = os.path.join(pkg_dir, 'config', 'node_params', 'trajectory_follower')
    config_file_pure_pursuit = os.path.join(config_dir_trajectory_follower, 'pure_pursuit.yaml')
    config_file_pid = os.path.join(config_dir_trajectory_follower, 'pid.yaml')

    # Launch Argument Configurations
    namespace = LaunchConfiguration('namespace', default='')

    # Nodes and other launch files
    remap_tf = SetRemap(src='/tf', dst='autoware/tf');
    remap_tf_static = SetRemap(src='/tf_static', dst='autoware/tf_static');
    
    start_autoware_trajectory_follower_cmd = GroupAction(
        actions = [
            remap_tf,
            remap_tf_static,
            SetRemap(src='~/input/reference_trajectory', dst='motion_velocity_smoother/output/trajectory'),
            SetRemap(src='~/input/current_steering', dst='vehicle/status/steering_status'),
            SetRemap(src='~/input/current_odometry', dst='odom'),
            SetRemap(src='~/input/current_accel', dst='~/input/current_accel'),
            SetRemap(src='~/input/current_operation_mode', dst='~/input/current_operation_mode'),
            SetRemap(src='~/output/control_cmd', dst='control/command/control_cmd'),
            Node(
                package='trajectory_follower_nodes',
                executable='controller_node_exe',
                name='trajectory_follower',
                namespace = namespace,
                output='screen',
                parameters=[
                    config_file_vehicle,
                    config_file_pure_pursuit,
                    config_file_pid,
                    {
                        'lateral_controller_mode': 'pure_pursuit',
                        'longitudinal_controller_mode': 'pid',
                        'ctrl_period': 0.2,
                        'timeout_thr_sec': 1.0
                    }
                ],
            )
        ]
    )

    # Launch Description
    ld = LaunchDescription()

    ld.add_action(start_autoware_trajectory_follower_cmd)

    return ld