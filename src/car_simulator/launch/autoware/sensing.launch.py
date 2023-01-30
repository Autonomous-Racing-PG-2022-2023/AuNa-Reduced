import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch_ros.actions import Node, SetRemap, PushRosNamespace, SetParameter
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch import LaunchDescription
from car_simulator import yaml_launch, SetParametersFromFile

def generate_launch_description():
    # Package Directories
    pkg_dir = get_package_share_directory('car_simulator')
    
    # Paths to folders and files
    
    config_dir_vehicle_velocity_converter = os.path.join(pkg_dir, 'config', 'node_params', 'vehicle_velocity_converter')
    config_file_vehicle_velocity_converter = os.path.join(config_dir_vehicle_velocity_converter, 'vehicle_velocity_converter.yaml')
   

    # Launch Argument Configurations
    namespace = LaunchConfiguration('namespace', default='')

    # Nodes and other launch files
    push_namespace = PushRosNamespace(namespace)
    
    remap_tf = SetRemap(src='/tf', dst='autoware/tf');
    remap_tf_static = SetRemap(src='/tf_static', dst='autoware/tf_static');
    
    #TODO:Add other parameters
    #FIXME:Check correctness of parameters
    #TODO:Parameters in external file
    start_vehicle_velocity_converter_cmd = GroupAction(
        actions = [
            remap_tf,
            remap_tf_static,
            SetRemap(src='velocity_status', dst='vehicle/status/velocity_status'),
            SetRemap(src='twist_with_covariance', dst='/ehicle/status/twist_with_covariance'),
            Node(
                package='vehicle_velocity_converter',
                executable='vehicle_velocity_converter',
                name='vehicle_velocity_converter',
                namespace = namespace,
                output='screen',
                parameters=[
                    config_file_vehicle_velocity_converter,
                    {
                        'frame_id': [namespace, 'odom'],
                    }
                ]
            )
        ]
    )

    # Launch Description
    ld = LaunchDescription()

    #ld.add_action(start_vehicle_velocity_converter_cmd)

    return ld