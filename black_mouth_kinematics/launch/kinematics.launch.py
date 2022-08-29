import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    bm_kinematics_pkg_share = FindPackageShare('black_mouth_kinematics').find('black_mouth_kinematics')

    default_quadruped_config = os.path.join(bm_kinematics_pkg_share, 'config', 'quadruped.yaml')
    quadruped_config = LaunchConfiguration('quadruped_config', default=default_quadruped_config)

    ik_server = Node(
        package='black_mouth_kinematics',
        executable='inv_kinematics_server',
        name='compute_ik_server',
        output='both',
        parameters=[quadruped_config]
    )

    ik_node = Node(
        package='black_mouth_kinematics',
        executable='inv_kinematics_node',
        name='ik_node',
        output='both',
        parameters=[quadruped_config]
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='quadruped_config', default_value=default_quadruped_config, 
                              description='Absolute path to quadruped config file'),
        ik_server,
        ik_node
    ])
