import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch_ros.actions import Node, SetRemap
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription

def generate_launch_description():
    # Package Directories
    pkg_dir = get_package_share_directory('car_simulator')

    # Paths to folders and files
    config_file = os.path.join(pkg_dir, 'config', 'model_params', 'vehicle_interface.yaml')

    # Launch Argument Configurations
    namespace = LaunchConfiguration('namespace', default='')
    base_frame_id = LaunchConfiguration('base_frame_id', default=[namespace, 'base_link']);
    steering_frame_id = LaunchConfiguration('steering_frame_id', default=[namespace, 'left_steering']);
    
    # Launch Arguments
    namespace_arg = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Robot namespace'
    )
    base_frame_id_arg = DeclareLaunchArgument(
        'base_frame_id',
        default_value=[namespace, 'base_link'],
        description='base_frame of the robot'
    )
    steering_frame_id_arg = DeclareLaunchArgument(
        'steering_frame_id',
        default_value=[namespace, 'left_steering'],
        description='steering_frame of the robot'
    )
    
    # Nodes and other launch files
    remap_tf = SetRemap(src='/tf', dst='autoware/tf');
    remap_tf_static = SetRemap(src='/tf_static', dst='autoware/tf_static');

    start_vehicle_interface_cmd = GroupAction(
        actions = [
            remap_tf,
            remap_tf_static,
            SetRemap(src='~/control/command/control_cmd', dst='control/command/control_cmd'),
            SetRemap(src='~/odom', dst='odom'),
            SetRemap(src='~/imu', dst='imu'),
            SetRemap(src='~/cmd_vel', dst='cmd_vel'),
            SetRemap(src='~/vehicle/status/velocity_status', dst='vehicle/status/velocity_status'),
            SetRemap(src='~/vehicle/status/steering_status', dst='vehicle/status/steering_status'),
            SetRemap(src='~/vehicle/status/acceleration', dst='vehicle/status/acceleration'),
            Node(
                package='car_simulator',
                executable='vehicle_interface',
                name='vehicle_interface',
                namespace = namespace,
                output='screen',
                parameters=[
                    config_file,
                    {
                        'base_frame_id': base_frame_id,
                        'steering_frame_id': steering_frame_id
                    }
                ]
            )
        ]
    )
    

    # Launch Description
    ld = LaunchDescription()
    
    ld.add_action(namespace_arg)
    ld.add_action(base_frame_id_arg)
    ld.add_action(steering_frame_id_arg)

    ld.add_action(start_vehicle_interface_cmd)

    return ld