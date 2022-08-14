import launch
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable


def generate_launch_description():

    ik_server = Node(
        package='black_mouth_kinematics',
        executable='inv_kinematics_server',
        name='compute_ik_server',
        output='both'
    )

    ik_node = Node(
        package='black_mouth_kinematics',
        executable='inv_kinematics_node',
        name='ik_node',
        output='both'
    )

    return launch.LaunchDescription([
        ik_server,
        ik_node
    ])
