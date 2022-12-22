import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch_ros.actions import Node, SetRemap
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch import LaunchDescription

def generate_launch_description():
    # Package Directories
    pkg_dir = get_package_share_directory('trajectory_follower_nodes')

    # Launch Argument Configurations
    namespace = LaunchConfiguration('namespace', default='')

    # Nodes and other launch files
    
    remap_odometry = SetRemap(src=[namespace, 'odom'], dst='input/kinematics');
    
    #FIXME: Not yet pure_pursuit, cause not yet supported? (But files do exist, just no launch)
    start_autoware_pure_pursuit_cmd = GroupAction(
        actions = [
            remap_odometry,
            IncludeLaunchDescription(
                FrontendLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'simple_trajectory_follower.launch.xml')),
                launch_arguments={
                }.items()
            )
        ]
    )

    # Launch Description
    ld = LaunchDescription()

    ld.add_action(start_autoware_pure_pursuit_cmd)

    return ld