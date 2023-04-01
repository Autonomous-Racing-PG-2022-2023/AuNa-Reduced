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
    config_file_vehicle = os.path.join(pkg_dir, 'config', 'model_params', 'vehicle_interface.yaml')
    
    config_dir_path_generation = os.path.join(pkg_dir, 'config', 'node_params', 'path_generation')
    config_file_wall_detection = os.path.join(config_dir_path_generation, 'wall_detection.yaml')
    config_file_wallfollowing = os.path.join(config_dir_path_generation, 'wallfollowing.yaml')

    # Launch Argument Configurations
    namespace = LaunchConfiguration('namespace', default='')

    # Nodes and other launch files
    push_namespace = PushRosNamespace(namespace)
    
    remap_tf = SetRemap(src='/tf', dst='autoware/tf');
    remap_tf_static = SetRemap(src='/tf_static', dst='autoware/tf_static');
    
    #TODO:Add other parameters
    #FIXME:Check correctness of parameters
    start_wall_detection_cmd = GroupAction(
        actions = [
            remap_tf,
            remap_tf_static,
            SetRemap(src='~/input/point_cloud', dst='points_raw'),
            SetRemap(src='~/output/track', dst='wall_detection/output/track'),
            SetRemap(src='~/output/obstacles', dst='wall_detection/output/obstacles'),
            SetRemap(src='~/debug/voxels', dst='wall_detection/debug/voxels'),
            SetRemap(src='~/debug/clusters', dst='wall_detection/debug/clusters'),
             Node(
                package='car_simulator',
                executable='wall_detection',
                name='wall_detection',
                namespace = namespace,
                output='screen',
                parameters=[
                    config_file_wall_detection,
                    {
                    }
                ],
            )
        ]
    )
    
    #TODO:Add other parameters
    #FIXME:Check correctness of parameters
    start_wallfollowing_cmd = GroupAction(
        actions = [
            remap_tf,
            remap_tf_static,
            SetRemap(src='~/input/velocity_report', dst='vehicle/status/velocity_status'),
            SetRemap(src='~/input/track', dst='wall_detection/output/track'),
            SetRemap(src='~/output/path', dst='wallfollowing/output/path'),
            SetRemap(src='~/debug/path/poses', dst='wallfollowing/debug/path/poses'),
            SetRemap(src='~/debug/bounds/left', dst='wallfollowing/debug/bounds/left'),
            SetRemap(src='~/debug/bounds/right', dst='wallfollowing/debug/bounds/right'),
            Node(
                package='car_simulator',
                executable='wallfollowing',
                name='wallfollowing',
                namespace = namespace,
                output='screen',
                parameters=[
                    config_file_vehicle,
                    config_file_wallfollowing,
                    {
                        'frame_id': 'odom', #NOTE: Must be odom cause autoware nodes relie on /odom/topic for current pose (But maybe we can do the same?)
                    }
                ]
            )
        ]
    )

    # Launch Description
    ld = LaunchDescription()

    ld.add_action(start_wall_detection_cmd)
    ld.add_action(start_wallfollowing_cmd)

    return ld