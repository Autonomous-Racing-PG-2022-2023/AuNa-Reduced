import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch_ros.actions import Node, SetRemap, PushRosNamespace, SetParameter, ComposableNodeContainer, LoadComposableNodes
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription
from car_simulator import yaml_launch, SetParametersFromFile

def generate_launch_description():
    # Package Directories
    pkg_dir = get_package_share_directory('car_simulator')
    pkg_probabilistic_occupancy_grid_map = get_package_share_directory('probabilistic_occupancy_grid_map')
    
    # Paths to folders and files
    config_dir_probabilistic_occupancy_grid_map = os.path.join(pkg_dir, 'config', 'node_params', 'probabilistic_occupancy_grid_map')
    config_file_probabilistic_occupancy_grid_map = os.path.join(config_dir_probabilistic_occupancy_grid_map, 'probabilistic_occupancy_grid_map.yaml')

    # Launch Argument Configurations
    namespace = LaunchConfiguration('namespace', default='')

    # Nodes and other launch files
    push_namespace = PushRosNamespace(namespace)
    
    remap_tf = SetRemap(src='/tf', dst='autoware/tf');
    remap_tf_static = SetRemap(src='/tf_static', dst='autoware/tf_static');
    
    composable_nodes = [
        ComposableNode(
            package='probabilistic_occupancy_grid_map',
            plugin="occupancy_grid_map::LaserscanBasedOccupancyGridMapNode",
            name='probabilistic_occupancy_grid_map',
            namespace = namespace,
            parameters=[
                config_file_probabilistic_occupancy_grid_map,
                {
                    'map_frame': 'map',
                    'base_link_frame': [namespace, 'base_link'],
                    'scan_origin_frame': [namespace, 'base_link'],
                    'gridmap_origin_frame': [namespace, 'base_link'],
                    'input_obstacle_pointcloud': False,
                    'input_obstacle_and_raw_pointcloud': False,
                }
            ]
        )
    ];
    
    occupancy_grid_map_container = ComposableNodeContainer(
        name="occupancy_grid_map_container",
        namespace=namespace,
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=composable_nodes,
        output="screen",
    )
    
    #TODO:Add other parameters
    #FIXME:Check correctness of parameters
    start_probabilistic_occupancy_grid_map_cmd = GroupAction(
        actions = [
            remap_tf,
            remap_tf_static,
            SetRemap(src='~/input/obstacle_pointcloud', dst='probabilistic_occupancy_grid_map/input/obstacle_pointcloud'),
            SetRemap(src='~/input/raw_pointcloud', dst='points_raw'),
            SetRemap(src='~/input/laserscan', dst='laser/out'),
            SetRemap(src='~/output/occupancy_grid_map', dst='occupancy_grid_map'),
            occupancy_grid_map_container
        ]
    )

    # Launch Description
    ld = LaunchDescription()

    ld.add_action(start_probabilistic_occupancy_grid_map_cmd)

    return ld