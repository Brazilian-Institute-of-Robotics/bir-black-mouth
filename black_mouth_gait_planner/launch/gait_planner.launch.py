from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    gait_traj_server = Node(
        package='black_mouth_gait_planner',
        executable='gait_traj_server',
        name='gait_traj_server',
        output='both',
    )
    
    return LaunchDescription([
        gait_traj_server
    ])
