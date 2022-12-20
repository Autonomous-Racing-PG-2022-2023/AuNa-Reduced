import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription

def generate_launch_description():
    # Package Directories
    pkg_dir = get_package_share_directory('car_simulator')

    # Paths to folders and files
    config_file = os.path.join(pkg_dir, 'config', 'model_params', 'vehicle_interface.yaml')

     # Launch Argument Configurations
    namespace = LaunchConfiguration('namespace', default='')
    base_frame_id = LaunchConfiguration('base_frame_id', default=[namespace, '/base_link']);
    
    # Launch Arguments
    namespace_arg = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Robot namespace'
    )
    base_frame_id_arg = DeclareLaunchArgument(
        'base_frame_id',
        default_value=[namespace, '/base_link'],
        description='base_frame_id is the target_frame for status messages'
    )

    # Nodes and other launch files
    start_vehicle_interface_cmd = Node(
        package='car_simulator',
        executable='vehicle_interface',
        name='vehicle_interface',
        namespace = namespace,
        output='screen',
        parameters=[
            config_file,
            {'base_frame_id': base_frame_id}
        ],
        remappings=[
            ('/control/command/control_cmd', '/external/selected/control_cmd'),
            ('/control/command/control_cmd', '/external/selected/control_cmd'),
            ('base_link', 'robot/base_link')
        ]
    )
    

    # Launch Description
    ld = LaunchDescription()
    
    ld.add_action(namespace_arg)
    ld.add_action(base_frame_id_arg)

    ld.add_action(start_vehicle_interface_cmd)

    return ld