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
    pkg_external_velocity_limit_selector = get_package_share_directory('external_velocity_limit_selector')
    pkg_dir_obstacle_avoidance_planner = get_package_share_directory('obstacle_avoidance_planner')
    
    # Paths to folders and files
    config_file_vehicle = os.path.join(pkg_dir, 'config', 'model_params', 'vehicle_interface.yaml')
    
    config_dir_obstacle_avoidance_planner = os.path.join(pkg_dir, 'config', 'node_params', 'obstacle_avoidance_planner')
    config_file_obstacle_avoidance_planner = os.path.join(config_dir_obstacle_avoidance_planner, 'obstacle_avoidance_planner.yaml')

    config_dir_obstacle_cruise_planner = os.path.join(pkg_dir, 'config', 'node_params', 'obstacle_cruise_planner')
    config_file_obstacle_cruise_planner = os.path.join(config_dir_obstacle_cruise_planner, 'obstacle_cruise_planner.yaml')

    config_dir_obstacle_velocity_limiter = os.path.join(pkg_dir, 'config', 'node_params', 'obstacle_velocity_limiter')
    config_file_obstacle_velocity_limiter = os.path.join(config_dir_obstacle_velocity_limiter, 'obstacle_velocity_limiter.yaml')
    
    config_dir_motion_velocity_smoother = os.path.join(pkg_dir, 'config', 'node_params', 'motion_velocity_smoother')
    config_file_motion_velocity_smoother = os.path.join(config_dir_motion_velocity_smoother, 'motion_velocity_smoother.yaml')

    # Launch Argument Configurations
    namespace = LaunchConfiguration('namespace', default='')

    # Nodes and other launch files
    push_namespace = PushRosNamespace(namespace)

    
    remap_tf = SetRemap(src='/tf', dst='autoware/tf');
    remap_tf_static = SetRemap(src='/tf_static', dst='autoware/tf_static');

    robot_name = LaunchConfiguration('name')
    
    #TODO:Add other parameters
    #FIXME:Check correctness of parameters
    #TODO:Parameters in external file
    
    #TODO:Make node
    start_autoware_external_velocity_limit_selector_cmd = GroupAction(
        actions = [
            push_namespace,
            remap_tf,
            remap_tf_static,
            IncludeLaunchDescription(
                FrontendLaunchDescriptionSource(os.path.join(pkg_external_velocity_limit_selector, 'launch', 'external_velocity_limit_selector.launch.xml')),
                launch_arguments={
                }.items()
            )
        ]
    )
    
    #Provides empty PredictedObjects to make object avoidance planner work
    start_objects_provider_cmd = GroupAction(
        actions = [
            remap_tf,
            remap_tf_static,
             Node(
                package='car_simulator',
                executable='objects_provider',
                name='objects_provider',
                namespace = namespace,
                output='screen',
                parameters=[
                    {
                        'frame_id': 'map',
                        'publish_period': 100,
                        'dst_topic': 'predicted_objects',
                    }
                ],
                arguments = [robot_name]
            )
        ]
    )
    
    #TODO:Add other parameters
    #FIXME:Check correctness of parameters
    start_autoware_obstacle_avoidance_planner_cmd = GroupAction(
        actions = [
            remap_tf,
            remap_tf_static,
            SetRemap(src='~/input/odometry', dst='odom'),
            SetRemap(src='~/input/path', dst='wallfollowing/output/path'),
            SetRemap(src='~/output/path', dst='obstacle_avoidance_planner/output/trajectory'),
            Node(
                package='obstacle_avoidance_planner',
                executable='obstacle_avoidance_planner_node',
                name='obstacle_avoidance_planner',
                namespace = namespace,
                output='screen',
                parameters=[
                    config_file_vehicle,
                    config_file_obstacle_avoidance_planner,
                    {
                    }
                ]
            )
        ]
    )

    #TODO:Add other parameters
    #FIXME:Check correctness of parameters
    start_autoware_obstacle_cruise_planner_cmd = GroupAction(
        actions = [
            remap_tf,
            remap_tf_static,
            SetRemap(src='~/input/odometry', dst='odom'),
            SetRemap(src='~/input/objects', dst='predicted_objects'),
            SetRemap(src='~/input/trajectory', dst='obstacle_avoidance_planner/output/trajectory'),
             SetRemap(src='~/input/acceleration', dst='vehicle/status/acceleration'),
            SetRemap(src='~/output/trajectory', dst='obstacle_cruise_planner/output/trajectory'),
            Node(
                package='obstacle_cruise_planner',
                executable='obstacle_cruise_planner',
                name='obstacle_cruise_planner',
                namespace = namespace,
                output='screen',
                parameters=[
                    config_file_vehicle,
                    config_file_obstacle_cruise_planner,
                    {
                    }
                ]
            )
        ]
    )

    #TODO:Add other parameters
    #FIXME:Check correctness of parameters
    start_autoware_obstacle_velocity_limiter_cmd = GroupAction(
        actions = [
            remap_tf,
            remap_tf_static,
            SetRemap(src='~/input/odometry', dst='odom'),
            SetRemap(src='~/input/dynamic_obstacles', dst='predicted_objects'),
            SetRemap(src='~/input/trajectory', dst='obstacle_avoidance_planner/output/trajectory'),
            SetRemap(src='~/input/obstacle_pointcloud', dst='wall_detection/output/obstacles'),
            SetRemap(src='~/output/trajectory', dst='obstacle_velocity_limiter/output/trajectory'),
            Node(
                package='obstacle_velocity_limiter',
                executable='obstacle_velocity_limiter',
                name='obstacle_velocity_limiter',
                namespace = namespace,
                output='screen',
                parameters=[
                    config_file_vehicle,
                    config_file_obstacle_velocity_limiter,
                    {
                    }
                ]
            )
        ]
    )
    
    #TODO:Add other parameters
    #FIXME:Check correctness of parameters
    start_autoware_motion_velocity_smoother_cmd = GroupAction(
        actions = [
            remap_tf,
            remap_tf_static,
            SetRemap(src='/localization/kinematic_state', dst='odom'),
            SetRemap(src='~/input/trajectory', dst='obstacle_cruise_planner/output/trajectory'),
            SetRemap(src='~/output/trajectory', dst='motion_velocity_smoother/output/trajectory'),
            SetRemap(src='~/input/external_velocity_limit_mps', dst='planning/scenario_planning/max_velocity'),
            SetRemap(src='~/output/current_velocity_limit_mps', dst='motion_velocity_smoother/output/current_max_velocity'),
            Node(
                package='motion_velocity_smoother',
                executable='motion_velocity_smoother',
                name='motion_velocity_smoother',
                namespace = namespace,
                output='screen',
                parameters=[
                    config_file_vehicle,
                    config_file_motion_velocity_smoother,
                    {
                    }
                ]
            )
        ]
    )

    # Launch Description
    ld = LaunchDescription()

    #ld.add_action(start_autoware_external_velocity_limit_selector_cmd)
    ld.add_action(start_objects_provider_cmd)
    ld.add_action(start_autoware_obstacle_avoidance_planner_cmd)
    ld.add_action(start_autoware_obstacle_cruise_planner_cmd)
    #ld.add_action(start_autoware_obstacle_velocity_limiter_cmd)
    ld.add_action(start_autoware_motion_velocity_smoother_cmd)

    return ld