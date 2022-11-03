import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    
    caramel_gait_planner_pkg_share = FindPackageShare('caramel_gait_planner').find('caramel_gait_planner')

    default_gait_planner_config = os.path.join(caramel_gait_planner_pkg_share, 'config', 'gait_planner.yaml')
    gait_planner_config = LaunchConfiguration('gait_planner_config', default=default_gait_planner_config)

    gait_traj_server = Node(
        package='caramel_gait_planner',
        executable='gait_traj_server',
        name='gait_traj_server',
        output='both',
    )

    trot_gait = Node(
        package='caramel_gait_planner',
        executable='trot_gait.py',
        output='both',
        parameters=[gait_planner_config]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(name='gait_planner_config', default_value=default_gait_planner_config, 
                              description='Absolute path to gait planner config file'),
        gait_traj_server,
        trot_gait
    ])
