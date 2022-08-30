import os
import launch
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, TimerAction

def generate_launch_description():

    black_mouth_pkg_share = FindPackageShare('black_mouth_kinematics').find('black_mouth_kinematics')

    # TODO: Fix config file for x360 joystick 
    # TODO: Create config file for ps4 joystick
    joy_type = LaunchConfiguration('joy_type', default="generic")
    joystick_config = [TextSubstitution(text=os.path.join(black_mouth_pkg_share, 'config', '')), 
                       joy_type, TextSubstitution(text='_joystick.yaml')]

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='both',
        parameters=[{
          'deadzone': 0.05,
          'autorepeat_rate': 1.0,
          'sticky_buttons': False,
          'coalesce_interval_ms': 1
        }],
    )

    joy_body_ik = Node(
        package='black_mouth_kinematics',
        executable='joy_body_ik',
        name='joy_body_ik_node',
        parameters=[joystick_config]
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument(name='joy_type', default_value='generic', 
                              description='Set the joystick type (generic, x360 or ps4)'),
        joy_node,
        TimerAction(period=3.0, actions=[joy_body_ik])
    ])
