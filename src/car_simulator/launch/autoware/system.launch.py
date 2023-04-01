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
    pkg_default_ad_api = get_package_share_directory('default_ad_api')
    
    config_dir_default_ad_api = os.path.join(pkg_dir, 'config', 'node_params', 'default_ad_api')
    config_file_default_ad_api = os.path.join(config_dir_default_ad_api, 'default_ad_api.yaml')

    # Launch Argument Configurations
    namespace = LaunchConfiguration('namespace', default='')

    # Nodes and other launch files
    push_namespace = PushRosNamespace(namespace)
    
    remap_tf = SetRemap(src='/tf', dst='autoware/tf');
    remap_tf_static = SetRemap(src='/tf_static', dst='autoware/tf_static');
    
    composable_nodes = [
        ComposableNode(
            package='default_ad_api',
            plugin="default_ad_api::FailSafeNode",
            name='fail_safe',
            namespace = namespace,
            parameters=[
                config_file_default_ad_api,
                {
                    'map_frame': 'map',
                    'base_link_frame': [namespace, 'base_link'],
                    'scan_origin_frame': [namespace, 'base_link'],
                    'gridmap_origin_frame': [namespace, 'base_link'],
                    'input_obstacle_pointcloud': False,
                    'input_obstacle_and_raw_pointcloud': False
                }
            ]
        ),
        ComposableNode(
            package='default_ad_api',
            plugin="default_ad_api::InterfaceNode",
            name='interface',
            namespace = namespace,
            parameters=[
                config_file_default_ad_api,
                {
                    'map_frame': 'map',
                    'base_link_frame': [namespace, 'base_link'],
                    'scan_origin_frame': [namespace, 'base_link'],
                    'gridmap_origin_frame': [namespace, 'base_link'],
                    'input_obstacle_pointcloud': False,
                    'input_obstacle_and_raw_pointcloud': False
                }
            ]
        ),
        ComposableNode(
            package='default_ad_api',
            plugin="default_ad_api::LocalizationNode",
            name='localization',
            namespace = namespace,
            parameters=[
                config_file_default_ad_api,
                {
                    'map_frame': 'map',
                    'base_link_frame': [namespace, 'base_link'],
                    'scan_origin_frame': [namespace, 'base_link'],
                    'gridmap_origin_frame': [namespace, 'base_link'],
                    'input_obstacle_pointcloud': False,
                    'input_obstacle_and_raw_pointcloud': False
                }
            ]
        ),
        ComposableNode(
            package='default_ad_api',
            plugin="default_ad_api::MotionNode",
            name='motion',
            namespace = namespace,
            parameters=[
                config_file_default_ad_api,
                {
                    'map_frame': 'map',
                    'base_link_frame': [namespace, 'base_link'],
                    'scan_origin_frame': [namespace, 'base_link'],
                    'gridmap_origin_frame': [namespace, 'base_link'],
                    'input_obstacle_pointcloud': False,
                    'input_obstacle_and_raw_pointcloud': False
                }
            ]
        ),
        ComposableNode(
            package='default_ad_api',
            plugin="default_ad_api::OperationModeNode",
            name='operation_mode',
            namespace = namespace,
            parameters=[
                config_file_default_ad_api,
                {
                    'map_frame': 'map',
                    'base_link_frame': [namespace, 'base_link'],
                    'scan_origin_frame': [namespace, 'base_link'],
                    'gridmap_origin_frame': [namespace, 'base_link'],
                    'input_obstacle_pointcloud': False,
                    'input_obstacle_and_raw_pointcloud': False
                }
            ]
        ),
        ComposableNode(
            package='default_ad_api',
            plugin="default_ad_api::PlanningNode",
            name='planning',
            namespace = namespace,
            parameters=[
                config_file_default_ad_api,
                {
                    'map_frame': 'map',
                    'base_link_frame': [namespace, 'base_link'],
                    'scan_origin_frame': [namespace, 'base_link'],
                    'gridmap_origin_frame': [namespace, 'base_link'],
                    'input_obstacle_pointcloud': False,
                    'input_obstacle_and_raw_pointcloud': False
                }
            ]
        ),
        ComposableNode(
            package='default_ad_api',
            plugin="default_ad_api::RoutingNode",
            name='routing',
            namespace = namespace,
            parameters=[
                config_file_default_ad_api,
                {
                    'map_frame': 'map',
                    'base_link_frame': [namespace, 'base_link'],
                    'scan_origin_frame': [namespace, 'base_link'],
                    'gridmap_origin_frame': [namespace, 'base_link'],
                    'input_obstacle_pointcloud': False,
                    'input_obstacle_and_raw_pointcloud': False
                }
            ]
        )
    ];
    
    default_ad_api_container = ComposableNodeContainer(
        name="default_ad_api_container",
        namespace=namespace,
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=composable_nodes,
        output="screen",
    )
    
    #TODO:Add other parameters
    #FIXME:Check correctness of parameters
    #TODO: Wildcards in remappings (but not yet supported)
    start_default_ad_api_cmd = GroupAction(
        actions = [
            remap_tf,
            remap_tf_static,
            SetRemap(src='/system/fail_safe/mrm_state', dst='system/fail_safe/mrm_state'),
            SetRemap(src='/system/component_state_monitor/component/autonomous/control', dst='system/component_state_monitor/component/autonomous/control'),
            SetRemap(src='/system/component_state_monitor/component/autonomous/localization', dst='system/component_state_monitor/component/autonomous/localization'),
            SetRemap(src='/system/component_state_monitor/component/autonomous/map', dst='system/component_state_monitor/component/autonomous/map'),
            SetRemap(src='/system/component_state_monitor/component/autonomous/perception', dst='system/component_state_monitor/component/autonomous/perception'),
            SetRemap(src='/system/component_state_monitor/component/autonomous/planning', dst='system/component_state_monitor/component/autonomous/planning'),
            SetRemap(src='/system/component_state_monitor/component/autonomous/sensing', dst='system/component_state_monitor/component/autonomous/sensing'),
            SetRemap(src='/system/component_state_monitor/component/autonomous/system', dst='system/component_state_monitor/component/autonomous/system'),
            SetRemap(src='/system/component_state_monitor/component/autonomous/vehicle', dst='system/component_state_monitor/component/autonomous/vehicle'),
            SetRemap(src='/system/operation_mode/state', dst='system/operation_mode/state'),
            SetRemap(src='/system/operation_mode/change_autoware_control', dst='system/operation_mode/change_autoware_control'),
            SetRemap(src='/system/operation_mode/change_operation_mode', dst='system/operation_mode/change_operation_mode'),
            SetRemap(src='/api/fail_safe/mrm_state', dst='api/fail_safe/mrm_state'),
            SetRemap(src='/api/interface/version', dst='api/interface/version'),
            SetRemap(src='/api/localization/initialization_state', dst='api/localization/initialization_state'),
            SetRemap(src='/api/localization/initialize', dst='api/localization/initialize'),
            SetRemap(src='/api/motion/state', dst='api/motion/state'),
            SetRemap(src='/api/motion/accept_start', dst='api/motion/accept_start'),
            SetRemap(src='/api/operation_mode/state', dst='api/operation_mode/state'),
            SetRemap(src='/api/operation_mode/change_to_autonomous', dst='api/operation_mode/change_to_autonomous'),
            SetRemap(src='/api/operation_mode/change_to_local', dst='api/operation_mode/change_to_local'),
            SetRemap(src='/api/operation_mode/change_to_remote', dst='api/operation_mode/change_to_remote'),
            SetRemap(src='/api/operation_mode/change_to_stop', dst='api/operation_mode/change_to_stop'),
            SetRemap(src='/api/operation_mode/disable_autoware_control', dst='api/operation_mode/disable_autoware_control'),
            SetRemap(src='/api/operation_mode/enable_autoware_control', dst='api/operation_mode/enable_autoware_control'),
            SetRemap(src='/api/planning/enable_autoware_control', dst='api/planning/steering_factors'),
            SetRemap(src='/api/planning/velocity_factors', dst='api/planning/velocity_factors'),
            SetRemap(src='/api/routing/route', dst='api/routing/route'),
            SetRemap(src='/api/routing/state', dst='api/routing/state'),
            SetRemap(src='/api/routing/clear_route', dst='api/routing/clear_route'),
            SetRemap(src='/api/routing/set_route', dst='api/routing/set_route'),
            SetRemap(src='/api/routing/set_route_points', dst='api/routing/set_route_points'),
            SetRemap(src='/localization/kinematic_state', dst='localization/kinematic_state'),
            SetRemap(src='/localization/initialization_state', dst='localization/initialization_state'),
            SetRemap(src='/localization/initialize', dst='localization/initialize'),
            SetRemap(src='/control/vehicle_cmd_gate/is_paused', dst='control/vehicle_cmd_gate/is_paused'),
            SetRemap(src='/control/vehicle_cmd_gate/is_start_requested', dst='control/vehicle_cmd_gate/is_start_requested'),
            SetRemap(src='/control/vehicle_cmd_gate/set_pause', dst='control/vehicle_cmd_gate/set_pause'),
            SetRemap(src='/planning/scenario_planning/trajectory', dst='planning/scenario_planning/trajectory'),
            SetRemap(src='/planning/steering_factor/avoidance', dst='planning/steering_factor/avoidance'),
            SetRemap(src='/planning/steering_factor/intersection', dst='planning/steering_factor/intersection'),
            SetRemap(src='/planning/steering_factor/lane_change', dst='planning/steering_factor/lane_change'),
            SetRemap(src='/planning/steering_factor/pull_out', dst='planning/steering_factor/pull_out'),
            SetRemap(src='/planning/steering_factor/pull_over', dst='planning/steering_factor/pull_over'),
            SetRemap(src='/planning/velocity_factors/blind_spot', dst='planning/velocity_factors/blind_spot'),
            SetRemap(src='/planning/velocity_factors/crosswalk', dst='planning/velocity_factors/crosswalk'),
            SetRemap(src='/planning/velocity_factors/detection_area', dst='planning/velocity_factors/detection_area'),
            SetRemap(src='/planning/velocity_factors/intersection', dst='planning/velocity_factors/intersection'),
            SetRemap(src='/planning/velocity_factors/merge_from_private', dst='planning/velocity_factors/merge_from_private'),
            SetRemap(src='/planning/velocity_factors/no_stopping_area', dst='planning/velocity_factors/no_stopping_area'),
            SetRemap(src='/planning/velocity_factors/obstacle_cruise', dst='planning/velocity_factors/obstacle_cruise'),
            SetRemap(src='/planning/velocity_factors/obstacle_stop', dst='planning/velocity_factors/obstacle_stop'),
            SetRemap(src='/planning/velocity_factors/occlusion_spot', dst='planning/velocity_factors/occlusion_spot'),
            SetRemap(src='/planning/velocity_factors/stop_line', dst='planning/velocity_factors/stop_line'),
            SetRemap(src='/planning/velocity_factors/surround_obstacle', dst='planning/velocity_factors/surround_obstacle'),
            SetRemap(src='/planning/velocity_factors/traffic_light', dst='planning/velocity_factors/traffic_light'),
            SetRemap(src='/planning/velocity_factors/virtual_traffic_light', dst='planning/velocity_factors/virtual_traffic_light'),
            SetRemap(src='/planning/velocity_factors/walkway', dst='planning/velocity_factors/walkway'),
            SetRemap(src='/planning/mission_planning/route', dst='planning/mission_planning/route'),
            SetRemap(src='/planning/mission_planning/route_state', dst='planning/mission_planning/route_state'),
            SetRemap(src='/planning/mission_planning/clear_route', dst='planning/mission_planning/clear_route'),
            SetRemap(src='/planning/mission_planning/set_route', dst='planning/mission_planning/set_route'),
            SetRemap(src='/planning/mission_planning/set_route_points', dst='planning/mission_planning/set_route_points'),
            SetParametersFromFile(config_file_default_ad_api),#TODO: Not sure why, but this seems to be necessary for composable nodes
            default_ad_api_container
        ]
    )
    
    start_default_pose_initializer_cmd = GroupAction(
        actions = [
            remap_tf,
            remap_tf_static,
               Node(
                package='car_simulator',
                executable='default_pose_initializer',
                name='default_pose_initializer',
                namespace = namespace,
                output='screen',
                parameters=[
                    {
                        'frame_id': 'map',
                        'position': {
                            'x': 0.0,
                            'y': 0.0,
                            'z': 0.0
                        },
                        'orientation': {
                            'roll': 0.0,
                            'yaw': 0.0,
                            'pitch': 0.0
                        }
                    }
                ]
            )
        ]
    )
    
    start_autonomous_availibility_manager_cmd = GroupAction(
        actions = [
            remap_tf,
            remap_tf_static,
            SetRemap(src='/system/component_state_monitor/component/autonomous/control', dst='system/component_state_monitor/component/autonomous/control'),
            SetRemap(src='/system/component_state_monitor/component/autonomous/localization', dst='system/component_state_monitor/component/autonomous/localization'),
            SetRemap(src='/system/component_state_monitor/component/autonomous/map', dst='system/component_state_monitor/component/autonomous/map'),
            SetRemap(src='/system/component_state_monitor/component/autonomous/perception', dst='system/component_state_monitor/component/autonomous/perception'),
            SetRemap(src='/system/component_state_monitor/component/autonomous/planning', dst='system/component_state_monitor/component/autonomous/planning'),
            SetRemap(src='/system/component_state_monitor/component/autonomous/sensing', dst='system/component_state_monitor/component/autonomous/sensing'),
            SetRemap(src='/system/component_state_monitor/component/autonomous/system', dst='system/component_state_monitor/component/autonomous/system'),
            SetRemap(src='/system/component_state_monitor/component/autonomous/vehicle', dst='system/component_state_monitor/component/autonomous/vehicle'),
            Node(
                package='car_simulator',
                executable='autonomous_availability_manager',
                name='autonomous_availability_manager',
                namespace = namespace,
                output='screen',
                parameters=[
                    {
                        'publish_period': 10
                    }
                ]
            )
        ]
    )
    
    start_vehicle_engager_cmd = GroupAction(
        actions = [
            remap_tf,
            remap_tf_static,
            SetRemap(src='/api/autoware/set/engage', dst='api/autoware/set/engage'),
            Node(
                package='car_simulator',
                executable='vehicle_engager',
                name='vehicle_engager',
                namespace = namespace,
                output='screen',
                parameters=[
                    {
                    }
                ]
            )
        ]
    )
    
    start_external_cmd_selector_conversion_cmd = GroupAction(
        actions = [
            remap_tf,
            remap_tf_static,
            SetRemap(src='~/input/operation_mode', dst='system/operation_mode/state'),
            SetRemap(src='~/output/selector_mode', dst='control/external_cmd_selector/current_selector_mode'),
            Node(
                package='car_simulator',
                executable='external_cmd_selector_conversion',
                name='external_cmd_selector_conversion',
                namespace = namespace,
                output='screen',
                parameters=[
                    {
                    }
                ]
            )
        ]
    )

    # Launch Description
    ld = LaunchDescription()
    
    ld.add_action(start_default_ad_api_cmd)
    #ld.add_action(start_default_pose_initializer_cmd)
    ld.add_action(start_autonomous_availibility_manager_cmd)
    ld.add_action(start_vehicle_engager_cmd)
    ld.add_action(start_external_cmd_selector_conversion_cmd)

    return ld