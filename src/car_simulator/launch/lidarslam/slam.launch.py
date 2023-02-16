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
    config_dir_lidarslam= os.path.join(pkg_dir, 'config', 'node_params', 'lidarslam')
    config_file_scanmatcher = os.path.join(config_dir_lidarslam, 'scanmatcher.yaml')
    
    # Launch Argument Configurations
    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Nodes and other launch files
    push_namespace = PushRosNamespace(namespace)
    
    remap_tf = SetRemap(src='/tf', dst='tf');
    remap_tf_static = SetRemap(src='/tf_static', dst='tf_static');
    
    #TODO:Somehow use odom
    start_scanmatcher_cmd = GroupAction(
        actions = [
            remap_tf,
            remap_tf_static,
            SetRemap(src='initial_pose', dst='initialpose'),
            SetRemap(src='input_cloud', dst='points_raw'),
            SetRemap(src='current_pose', dst='initialpose3d'),
            Node(
                package='scanmatcher',
                executable='scanmatcher_node',
                name='scanmatcher',
                namespace = namespace,
                output='screen',
                parameters=[
                    config_file_scanmatcher,
                    {
                        'global_frame_id': 'map',
                        'robot_frame_id': [namespace, 'slam_base_link'],
                        'odom_frame_id': [namespace, 'odom'],
                    }
                ]
            )
        ]
    )
    
    #Statically mapping from base_link to slam_base_link
    tf_remap_cmd = Node(
        package='car_simulator',
        executable='tf_remap',
        name='tf_remap_slam',
        namespace = namespace,
        output='screen',
        parameters=[
            {
                'src': [namespace, 'base_link'],
                'dst': [namespace, 'slam_base_link'],
                'src_tf': 'tf',
                'dst_tf': 'tf',
                'src_tf_static': 'tf_static',
                'dst_tf_static': 'tf_static'
            }
        ],
    )
    
    #Get map->odom by projecting map->slam_base_link onto odom->base_link
    tf_project_cmd = GroupAction(
        actions = [
            remap_tf,
            remap_tf_static,
             Node(
                package='car_simulator',
                executable='tf_project',
                name='tf_project',
                namespace = namespace,
                output='screen',
                parameters=[
                    {
                        'src_frame_id': 'map',
                        'src_child_frame_id': [namespace, 'slam_base_link'],
                        'dst_frame_id': [namespace, 'odom'],
                        'dst_child_frame_id': [namespace, 'base_link'],
                        'src_tf': 'tf',
                        'dst_tf': 'tf',
                        'src_tf_static': 'tf_static',
                        'dst_tf_static': 'tf_static'
                    }
                ],
            )
        ]
    )
    
    path_conversion_cmd = GroupAction(
        actions = [
            remap_tf,
            remap_tf_static,
             Node(
                package='car_simulator',
                executable='path_conversion',
                name='path_conversion',
                namespace = namespace,
                output='screen',
                parameters=[
                    {
                        'path_src_topic': 'path',
                        'occupancy_grid_src_topic': 'occupancy_grid_map',
                        'dst_topic': 'autoware_path',
                    }
                ],
            )
        ]
    )
   

    # Launch Description
    ld = LaunchDescription()

    ld.add_action(start_scanmatcher_cmd)
    ld.add_action(tf_remap_cmd)
    ld.add_action(tf_project_cmd)
    ld.add_action(path_conversion_cmd)

    return ld