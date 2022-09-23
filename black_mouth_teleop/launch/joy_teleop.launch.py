import os
import launch
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, TextSubstitution, PythonExpression
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition

def generate_launch_description():

    bm_teleop_pkg_share = FindPackageShare('black_mouth_teleop').find('black_mouth_teleop')

    launch_joy_node = LaunchConfiguration('launch_joy')

    joy_type = LaunchConfiguration('joy_type', default="generic")
    joystick_config = [TextSubstitution(text=os.path.join(bm_teleop_pkg_share, 'config', '')), 
                       joy_type, TextSubstitution(text='_joystick.yaml')]

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='both',
        parameters=[{
          'deadzone': 0.05,
          'autorepeat_rate': 0.0,
          'sticky_buttons': False,
          'coalesce_interval_ms': 1
        }],
        condition=IfCondition(PythonExpression([launch_joy_node, ' == True']))
    )

    teleop_state_server = Node(
        package='black_mouth_teleop',
        executable='teleop_state_server',
        name='teleop_state_server',
        output='both',
    )

    joy_teleop = Node(
        package='black_mouth_teleop',
        executable='joy_teleop',
        name='joy_teleop_node',
        parameters=[joystick_config]
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument('launch_joy', default_value='True',
                               description="Whether to launch joy node or not"),
        DeclareLaunchArgument(name='joy_type', default_value='generic', 
                              description='Set the joystick type (generic, x360 or ps4)'),
        joy_node,
        teleop_state_server,
        TimerAction(period=3.0, actions=[joy_teleop])
    ])
