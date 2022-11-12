import os
from datetime import datetime
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, TimerAction, RegisterEventHandler, EmitEvent
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown


def generate_launch_description():
    use_stabilization_arg = LaunchConfiguration('use_stabilization', default='0.0')

    path = '/home/ubuntu/bm_ws/src/bir-black-mouth/caramel_data_analysis/bags/'
    currentDateAndTime = datetime.now()
    currentTime = currentDateAndTime.strftime("%H:%M:%S")
    bag_name = LaunchConfiguration('bag_name', default=currentTime)

    vel_stability_node = Node(
        package="caramel_data_analysis",
        executable="stability_velocity",
        parameters=[{'use_stabilization': use_stabilization_arg}],
        output="both",
    )

    # Load controller to publish joint data
    init_bag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '--include-hidden-topics', '-o', [TextSubstitution(text=path), bag_name],
             '/cmd_vel', '/cmd_ik', '/feet_poses', '/imu/data'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='use_stabilization', default_value='False',
                              description='Whether to use or not'),
        DeclareLaunchArgument(name='bag_name', default_value=currentTime,
                              description='Name of the bag'),
        init_bag_record,
        TimerAction(period=1.0, actions=[vel_stability_node]),
        
        RegisterEventHandler(
            OnProcessExit(
                target_action=vel_stability_node,
                on_exit=[EmitEvent(event=Shutdown(
                        reason='Velocity Stability Test Finished!'))
                ]
            )
        )
    ])
