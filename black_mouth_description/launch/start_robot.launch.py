import os
import launch
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
  
  bm_description_pkg_share = FindPackageShare('black_mouth_description').find('black_mouth_description')
  bm_control_pkg_share = FindPackageShare('black_mouth_control').find('black_mouth_control')
  bm_kinematics_pkg_share = FindPackageShare('black_mouth_kinematics').find('black_mouth_kinematics')

  default_model = os.path.join(bm_description_pkg_share, "urdf", "black_mouth.urdf.xacro")

  use_sim_time = LaunchConfiguration('use_sim_time', default='true')
  robot_model = LaunchConfiguration('model', default=default_model)


  # Publish TF
  robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': Command(['xacro ', robot_model])}],
    )
  
  bm_controllers = IncludeLaunchDescription(
              PythonLaunchDescriptionSource(
                os.path.join(bm_control_pkg_share, 'launch', 'bm_load_controller.launch.py')),
            #   launch_arguments={'': ''}.items(),
  )

  inverse_kinematics = IncludeLaunchDescription(
              PythonLaunchDescriptionSource(
                os.path.join(bm_kinematics_pkg_share, 'launch', 'kinematics.launch.py')),
            #   launch_arguments={'': ''}.items(),
  )

  return LaunchDescription([
    DeclareLaunchArgument(name='use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),
    DeclareLaunchArgument(name='model', default_value=default_model, description='Absolute path to robot urdf file'),
    robot_state_publisher,
    bm_controllers,
    inverse_kinematics
  ])
