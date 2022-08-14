import os
import launch
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction

def generate_launch_description():

  black_mouth_pkg_share = FindPackageShare('black_mouth_kinematics').find('black_mouth_kinematics')
  default_joystick_config = os.path.join(black_mouth_pkg_share, 'config', 'generic_joystick.yaml')

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

  # TODO: Create other config file for x360 joystick 
  joy_body_ik = Node(
    package='black_mouth_kinematics',
    executable='joy_body_ik',
    name='joy_body_ik_node',
    parameters=[
      default_joystick_config
    ]
  )

  return launch.LaunchDescription([
    joy_node,
    TimerAction(period=3.0, actions=[joy_body_ik])
  ])
