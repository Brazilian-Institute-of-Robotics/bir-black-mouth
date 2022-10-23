from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    gait_traj_server = Node(
        package='black_mouth_gait_planner',
        executable='gait_traj_server',
        name='gait_traj_server',
        output='both',
    )

    trot_gait = Node(
        package='black_mouth_gait_planner',
        executable='trot_gait.py',
        name='trot_gait',
        output='both',
    )
    
    return LaunchDescription([
        gait_traj_server,
        trot_gait
    ])
