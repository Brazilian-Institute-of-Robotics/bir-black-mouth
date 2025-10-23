from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('caramel_cpg')
    config_path = os.path.join(pkg_share, 'config', 'trot_config.yaml')

    return LaunchDescription([
        Node(
            package='caramel_cpg',
            executable='trot_node',
            name='trot_node',
            output='screen',
            parameters=[config_path]
        )
    ])
