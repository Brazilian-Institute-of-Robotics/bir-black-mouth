import os
from datetime import datetime
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, TimerAction, RegisterEventHandler, EmitEvent
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown


def generate_launch_description():
    x_arg = LaunchConfiguration('x', default='0.0')
    y_arg = LaunchConfiguration('y', default='0.0')

    path = '/home/ubuntu/bm_ws/src/bir-black-mouth/caramel_data_analysis/bags/'
    currentDateAndTime = datetime.now()
    currentTime = currentDateAndTime.strftime("%H:%M:%S")
    bag_name = LaunchConfiguration('bag_name', default=currentTime)

    traj_test_node = Node(
        package="caramel_data_analysis",
        executable="foot_follow_trajectory",
        parameters=[{'x': x_arg, 'y': y_arg}],
        output="both",
    )

    # Load controller to publish joint data
    init_bag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '--include-hidden-topics', '-o', [TextSubstitution(text=path), bag_name],
             '/cmd_vel', '/cmd_ik', '/feet_poses', '/imu/data'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='x', default_value='0.0',
                              description='Distante in X'),
        DeclareLaunchArgument(name='y', default_value='0.0',
                              description='Distante in Y'),
        DeclareLaunchArgument(name='bag_name', default_value=currentTime,
                              description='Name of the bag'),
        init_bag_record,
        TimerAction(period=1.0, actions=[traj_test_node]),
        
        RegisterEventHandler(
            OnProcessExit(
                target_action=traj_test_node,
                on_exit=[EmitEvent(event=Shutdown(
                        reason='Trajectory Finished!'))
                ]
            )
        )
    ])
