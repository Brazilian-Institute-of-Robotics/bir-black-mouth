import os
import launch
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    black_mouth_pkg_share = FindPackageShare('black_mouth_kinematics').find('black_mouth_kinematics')
    default_quadruped_config = os.path.join(black_mouth_pkg_share, 'config', 'quadruped.yaml')

    ik_server = Node(
        package='black_mouth_kinematics',
        executable='inv_kinematics_server',
        name='compute_ik_server',
        output='both',
        parameters=[default_quadruped_config]
    )

    ik_node = Node(
        package='black_mouth_kinematics',
        executable='inv_kinematics_node',
        name='ik_node',
        output='both',
        parameters=[default_quadruped_config]
    )

    return launch.LaunchDescription([
        ik_server,
        ik_node
    ])
