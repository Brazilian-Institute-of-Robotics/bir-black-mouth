import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='caramel_cpg',
            executable='cpg_node',
            name='cpg_node',
            output='screen',
        )
    ])