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
    pkg_default_ad_api = get_package_share_directory('default_ad_api')

    # Launch Argument Configurations
    namespace = LaunchConfiguration('namespace', default='')

    # Nodes and other launch files
    push_namespace = PushRosNamespace(namespace)
    
    remap_tf = SetRemap(src='/tf', dst='autoware/tf');
    remap_tf_static = SetRemap(src='/tf_static', dst='autoware/tf_static');
    
    #TODO:Add other parameters
    #FIXME:Check correctness of parameters
    start_default_ad_api_cmd = GroupAction(
        actions = [
            push_namespace,
            remap_tf,
            remap_tf_static,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_default_ad_api, 'launch', 'default_ad_api.launch.py')),
                launch_arguments={
                }.items()
            )
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

    # Launch Description
    ld = LaunchDescription()
    
    #ld.add_action(start_default_ad_api_cmd)
    #ld.add_action(start_default_pose_initializer_cmd)

    return ld