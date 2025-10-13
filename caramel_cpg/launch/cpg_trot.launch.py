from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Caminho absoluto para o arquivo de parâmetros
    pkg_share = os.path.join(
        os.getenv('COLCON_PREFIX_PATH', '/'),
        'share',
        'caramel_cpg',
        'config'
    )
    config_path = os.path.join(pkg_share, 'trot_config.yaml')

    return LaunchDescription([
        Node(
            package='caramel_cpg',
            executable='trot_node',
            name='hopf_cpg_node',
            output='screen',
            parameters=[config_path]
        )
    ])
