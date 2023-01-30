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
    pkg_gyro_odometer = get_package_share_directory('gyro_odometer')
    pkg_pose_initializer = get_package_share_directory('pose_initializer')
    pkg_ndt_scan_matcher = get_package_share_directory('ndt_scan_matcher')
    pkg_ekf_localizer = get_package_share_directory('ekf_localizer')
    
    # Paths to folders and files
    config_dir_gyro_odometer = os.path.join(pkg_dir, 'config', 'node_params', 'gyro_odometer')
    config_file_gyro_odometer = os.path.join(config_dir_gyro_odometer, 'gyro_odometer.yaml')
    
    config_dir_ndt_scan_matcher = os.path.join(pkg_dir, 'config', 'node_params', 'ndt_scan_matcher')
    config_file_ndt_scan_matcher = os.path.join(config_dir_ndt_scan_matcher, 'ndt_scan_matcher.yaml')
    
    config_dir_ekf_localizer = os.path.join(pkg_dir, 'config', 'node_params', 'ekf_localizer')
    config_file_ekf_localizer = os.path.join(config_dir_ekf_localizer, 'ekf_localizer.yaml')
   

    # Launch Argument Configurations
    namespace = LaunchConfiguration('namespace', default='')

    # Nodes and other launch files
    push_namespace = PushRosNamespace(namespace)
    
    gyro_odometer_parameters = SetParametersFromFile(config_file_gyro_odometer);
    ndt_scan_matcher_parameters = SetParametersFromFile(config_file_ndt_scan_matcher);
    ekf_localizer_parameters = SetParametersFromFile(config_file_ekf_localizer);
    
    remap_tf = SetRemap(src='/tf', dst='autoware/tf');
    remap_tf_static = SetRemap(src='/tf_static', dst='autoware/tf_static');
    
    #TODO:Add other parameters
    #FIXME:Check correctness of parameters
    #TODO:Parameters in external file
    
    #TODO:Make node
    start_gyro_odometer_cmd = GroupAction(
        actions = [
            push_namespace,
            remap_tf,
            remap_tf_static,
            gyro_odometer_parameters,
            SetParameter('input_imu_topic', 'imu'),
            IncludeLaunchDescription(
                FrontendLaunchDescriptionSource(os.path.join(pkg_gyro_odometer, 'launch', 'gyro_odometer.launch.xml')),
                launch_arguments={
                    'input_imu_topic': 'imu',
                }.items()
            )
        ]
    )
    
    #TODO:Add other parameters
    #FIXME:Check correctness of parameters
    #TODO:Parameters in external file
    start_pose_initializer_cmd = GroupAction(
        actions = [
            remap_tf,
            remap_tf_static,
            SetRemap(src='ndt_align', dst='ndt_align'),
            SetRemap(src='gnss_pose_cov', dst='sensing/gnss/pose_with_covariance'),
            SetRemap(src='pose_reset', dst='initialpose3d'),
            SetRemap(src='ekf_trigger_node', dst='ekf_trigger_node'),
            SetRemap(src='ndt_trigger_node', dst='ndt_trigger_node'),
            Node(
                package='pose_initializer',
                executable='pose_initializer_node',
                name='pose_initializer',
                namespace = namespace,
                output='screen',
                parameters=[
                    {
                        'ndt_enabled': True,
                        'stop_check_enabled': False,
                        'stop_check_duration': 0.1,
                        'gnss_enabled': True,
                        'gnss_pose_timeout': 1.0,
                        'output_pose_covariance': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        'gnss_particle_covariance': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    }
                ]
            )
        ]
    )
    
    #TODO:Add other parameters
    #FIXME:Check correctness of parameters
    #TODO:Parameters in external file
    start_ndt_scan_matcher_cmd = GroupAction(
        actions = [
            remap_tf,
            remap_tf_static,
            SetRemap(src='points_raw', dst='points_raw'),
            SetRemap(src='ekf_pose_with_covariance', dst='ekf_pose_with_covariance'),
            SetRemap(src='pointcloud_map', dst='pointcloud_map'),
            SetRemap(src='ndt_pose', dst='ndt_pose'),
            SetRemap(src='ndt_pose_with_covariance', dst='ndt_pose_with_covariance'),
            SetRemap(src='regularization_pose_with_covariance', dst='sensing/gnss/pose_with_covariance'),
            SetRemap(src='trigger_node_srv', dst='ndt_trigger_node'),
            SetRemap(src='ndt_align_srv', dst='ndt_align'),
            Node(
                package='ndt_scan_matcher',
                executable='ndt_scan_matcher',
                name='ndt_scan_matcher',
                namespace = namespace,
                output='screen',
                parameters=[
                    config_file_ndt_scan_matcher,
                    {
                        'base_frame': [namespace, 'base_link']
                    }
                ]
            )
        ]
    )
    
    #TODO:Add other parameters
    #FIXME:Check correctness of parameters
    start_ekf_localizer_cmd = GroupAction(
        actions = [
            remap_tf,
            remap_tf_static,
            SetRemap(src='in_pose_with_covariance', dst='ndt_pose_with_covariance'),
            SetRemap(src='in_twist_with_covariance', dst='gyro_twist_with_covariance'),
            SetRemap(src='initialpose', dst='initialpose3d'),
            SetRemap(src='trigger_node_srv', dst='ekf_trigger_node'),
            SetRemap(src='ekf_odom', dst='ekf_odom'),
            SetRemap(src='ekf_pose', dst='ekf_pose'),
            SetRemap(src='ekf_pose_with_covariance', dst='ekf_pose_with_covariance'),
            SetRemap(src='ekf_biased_pose', dst='ekf_biased_pose'),
            SetRemap(src='ekf_biased_pose_with_covariance', dst='ekf_biased_pose_with_covariance'),
            SetRemap(src='ekf_twist', dst='ekf_twist'),
            SetRemap(src='ekf_twist_with_covariance', dst='ekf_twist_with_covariance'),
            Node(
                package='ekf_localizer',
                executable='ekf_localizer',
                name='ekf_localizer',
                namespace = namespace,
                output='screen',
                parameters=[
                    config_file_ekf_localizer,
                    {
                        'pose_frame_id': 'map'
                    }
                ]
            )
        ]
    )

    # Launch Description
    ld = LaunchDescription()

    #ld.add_action(start_gyro_odometer_cmd)
    #ld.add_action(start_pose_initializer_cmd)
    #ld.add_action(start_ndt_scan_matcher_cmd)
    #ld.add_action(start_ekf_localizer_cmd)

    return ld