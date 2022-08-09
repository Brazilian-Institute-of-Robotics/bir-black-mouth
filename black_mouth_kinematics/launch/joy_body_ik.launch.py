import launch
from launch_ros.actions import Node

def generate_launch_description():

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
    }]
  )

  # TODO: Put these values in a config file, create one for genetic joytic and other for x360 
  joy_body_ik = Node(
    package='black_mouth_kinematics',
    executable='joy_body_ik',
    name='joy_body_ik',
    parameters=[
      {"move_linear_x": 1},
      {"move_linear_y": 0},
      {"move_linear_z": 2},
      {"move_angular_roll": 4},
      {"move_angular_pitch": 5},
      {"move_angular_yaw": 3},
    ]
  )

  # TODO: Wait before run joy_body_ik
  return launch.LaunchDescription([
    joy_node,
    joy_body_ik
  ])
