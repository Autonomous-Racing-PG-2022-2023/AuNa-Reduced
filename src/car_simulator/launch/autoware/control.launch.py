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
    
    config_dir_operation_mode_transition_manager = os.path.join(pkg_dir, 'config', 'node_params', 'operation_mode_transition_manager')
    config_file_operation_mode_transition_manager = os.path.join(config_dir_operation_mode_transition_manager, 'operation_mode_transition_manager.yaml')
    
    config_dir_vehicle_cmd_gate = os.path.join(pkg_dir, 'config', 'node_params', 'vehicle_cmd_gate')
    config_file_vehicle_cmd_gate = os.path.join(config_dir_vehicle_cmd_gate, 'vehicle_cmd_gate.yaml')

    # Launch Argument Configurations
    namespace = LaunchConfiguration('namespace', default='')

    # Nodes and other launch files
    remap_tf = SetRemap(src='/tf', dst='autoware/tf');
    remap_tf_static = SetRemap(src='/tf_static', dst='autoware/tf_static');
    
    start_operation_mode_transition_manager_cmd = GroupAction(
        actions = [
            remap_tf,
            remap_tf_static,
            SetRemap(src='/api/autoware/get/engage', dst='api/autoware/get/engage'),
            SetRemap(src='/control/external_cmd_selector/current_selector_mode', dst='control/external_cmd_selector/current_selector_mode'),
            SetRemap(src='control_mode_report', dst='vehicle/status/control_mode'),
            SetRemap(src='gate_operation_mode', dst='control/vehicle_cmd_gate/operation_mode'),
            SetRemap(src='control_cmd', dst='control/command/control_cmd'),
            SetRemap(src='/control/current_gate_mode', dst='control/current_gate_mode'),
            SetRemap(src='kinematics', dst='odom'),
            SetRemap(src='steering', dst='vehicle/status/steering_status'),
            SetRemap(src='trajectory', dst='obstacle_avoidance_planner/output/path'),
            SetRemap(src='is_autonomous_available', dst='control/is_autonomous_available'),
            SetRemap(src='control_mode_request', dst='control/control_mode_request'),
            SetRemap(src='/system/operation_mode/state', dst='system/operation_mode/state'),
            SetRemap(src='/autoware/engage', dst='autoware/engage'),
            SetRemap(src='/control/gate_mode_cmd', dst='control/gate_mode_cmd'),
            SetRemap(src='/control/external_cmd_selector/select_external_command', dst='control/external_cmd_selector/select_external_command'),
            SetRemap(src='/system/operation_mode/change_autoware_control', dst='system/operation_mode/change_autoware_control'),
            SetRemap(src='/system/operation_mode/change_operation_mode', dst='system/operation_mode/change_operation_mode'),
            Node(
                package='operation_mode_transition_manager',
                executable='operation_mode_transition_manager_exe',
                name='operation_mode_transition_manager',
                namespace = namespace,
                output='screen',
                parameters=[
                    config_file_vehicle,
                    config_file_operation_mode_transition_manager,
                    {
                    }
                ]
            )
        ]
    )
    
    start_vehicle_cmd_gate_cmd = GroupAction(
        actions = [
            remap_tf,
            remap_tf_static,
            SetRemap(src='input/steering', dst='vehicle/status/steering_status'),
            SetRemap(src='input/auto/control_cmd', dst='trajectory_follower/output/control_cmd'),
            SetRemap(src='input/external/control_cmd', dst='external/selected/control_cmd'),
            SetRemap(src='input/emergency/control_cmd', dst='system/emergency/control_cmd'),
            SetRemap(src='input/mrm_state', dst='system/fail_safe/mrm_state'),
            SetRemap(src='input/gate_mode', dst='control/gate_mode_cmd'),
            SetRemap(src='output/vehicle_cmd_emergency', dst='control/command/emergency_cmd'),
            SetRemap(src='output/control_cmd', dst='control/command/control_cmd'),
            SetRemap(src='output/external_emergency', dst='api/autoware/get/emergency'),
            SetRemap(src='~/service/engage', dst='api/autoware/set/engage'),
            SetRemap(src='~/service/external_emergency', dst='api/autoware/set/emergency'),
            SetRemap(src='output/gate_mode', dst='control/current_gate_mode'),
            SetRemap(src='output/engage', dst='api/autoware/get/engage'),
            SetRemap(src='/control/vehicle_cmd_gate/set_pause', dst='control/vehicle_cmd_gate/set_pause'),
            SetRemap(src='/control/vehicle_cmd_gate/is_paused', dst='control/vehicle_cmd_gate/is_paused'),
            SetRemap(src='/control/vehicle_cmd_gate/is_start_requested', dst='control/vehicle_cmd_gate/is_start_requested'),
            Node(
                package='vehicle_cmd_gate',
                executable='vehicle_cmd_gate',
                name='vehicle_cmd_gate',
                namespace = namespace,
                output='screen',
                parameters=[
                    config_file_vehicle,
                    config_file_vehicle_cmd_gate,
                    {
                    }
                ]
            )
        ]
    )

    
    start_autoware_trajectory_follower_cmd = GroupAction(
        actions = [
            remap_tf,
            remap_tf_static,
            SetRemap(src='~/input/reference_trajectory', dst='obstacle_avoidance_planner/output/path'),
            SetRemap(src='~/input/current_steering', dst='vehicle/status/steering_status'),
            SetRemap(src='~/input/current_odometry', dst='odom'),
            SetRemap(src='~/input/current_accel', dst='vehicle/status/acceleration'),
            SetRemap(src='~/input/current_operation_mode', dst='system/operation_mode/state'),
            SetRemap(src='~/output/control_cmd', dst='trajectory_follower/output/control_cmd'),
            Node(
                package='trajectory_follower_node',
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
                ]
            )
        ]
    )

    # Launch Description
    ld = LaunchDescription()

    ld.add_action(start_operation_mode_transition_manager_cmd)
    ld.add_action(start_vehicle_cmd_gate_cmd)
    ld.add_action(start_autoware_trajectory_follower_cmd)

    return ld