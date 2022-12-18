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
    namespace = LaunchConfiguration('namespace', default='robot')

    # Nodes and other launch files
    start_vehicle_interface_cmd = Node(
        package='car_simulator',
        executable='vehicle_interface',
        name='vehicle_interface',
        namespace = namespace,
        output='screen',
        parameters=[
            config_file,
            {'base_frame_id': "robot/odom/base_link"}
        ],
        remappings=[
            #('/external/selected/control_cmd', '/control/command/control_cmd'),
            ('/control/command/control_cmd', '/external/selected/control_cmd'),
            ('/api/autoware/set/engage', '/control/command/engage'),
            ('/external/selected/gear_cmd', '/control/command/gear_cmd')
        ]
    )

    # Launch Description
    ld = LaunchDescription()

    ld.add_action(start_vehicle_interface_cmd)

    return ld