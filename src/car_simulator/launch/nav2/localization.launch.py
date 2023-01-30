import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch_ros.actions import Node, SetRemap, PushRosNamespace, SetParameter
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch import LaunchDescription
from car_simulator import yaml_launch, SetParametersFromFile

def generate_launch_description():
    # Package Directories
    pkg_dir = get_package_share_directory('car_simulator')
    
    # Paths to folders and files
    config_dir_amcl = os.path.join(pkg_dir, 'config', 'node_params', 'amcl')
    config_file_amcl = os.path.join(config_dir_amcl, 'amcl.yaml')
    
    map_file = os.path.join(pkg_dir, 'maps', 'racetrack_decorated', 'map.yaml')
    
    # Launch Argument Configurations
    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Launch Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Nodes and other launch files
    push_namespace = PushRosNamespace(namespace)
    
    remap_tf = SetRemap(src='/tf', dst='tf');
    remap_tf_static = SetRemap(src='/tf_static', dst='tf_static');
    
    start_amcl_cmd = GroupAction(
        actions = [
            remap_tf,
            remap_tf_static,
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                namespace = namespace,
                output='screen',
                parameters=[
                    config_file_amcl,
                    {
                        'use_sim_time': use_sim_time,
                        'base_frame_id': [namespace, 'base_link'],
                        'odom_frame_id': [namespace, 'odom'],
                        'global_frame_id': 'map',
                        'scan_topic': 'laser/out',
                        'map_topic': 'map'
                    }
                ]
            )
        ]
    )
    
    start_map_server_cmd = GroupAction(
        actions = [
            remap_tf,
            remap_tf_static,
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                namespace = namespace,
                output='screen',
                parameters=[
                    {
                       'use_sim_time': use_sim_time,
                       'yaml_filename': map_file,
                       'topic_name': 'map',
                       'frame_id': 'map',
                    }
                ]
            )
        ]
    )
    
    start_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        namespace = namespace,
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['amcl', 'map_server'],
                'bond_timeout': 4.0,
                'attempt_respawn_reconnection': True,
                'bond_respawn_max_duration': 10.0,
            }
        ]
    )

    # Launch Description
    ld = LaunchDescription()

    ld.add_action(use_sim_time_arg)

    ld.add_action(start_amcl_cmd)
    ld.add_action(start_map_server_cmd)
    ld.add_action(start_lifecycle_manager_cmd)

    return ld