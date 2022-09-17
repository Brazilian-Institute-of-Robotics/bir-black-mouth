import os
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

    default_model = os.path.join(bm_description_pkg_share, "urdf", "black_mouth_real.urdf.xacro")
    robot_model = LaunchConfiguration('model', default=default_model)

    default_controllers = os.path.join(bm_control_pkg_share, "config", "leg_controllers_real_robot.yaml")
    robot_controllers = LaunchConfiguration('controllers', default=default_controllers)

    default_quadruped_config = os.path.join(bm_kinematics_pkg_share, 'config', 'quadruped.yaml')
    quadruped_config = LaunchConfiguration('quadruped_config', default=default_quadruped_config)

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

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
        launch_arguments={'model': robot_model,
                          'controllers': robot_controllers}.items(),
    )

    inverse_kinematics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bm_kinematics_pkg_share, 'launch', 'kinematics.launch.py')),
        launch_arguments={'quadruped_config': quadruped_config}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='use_sim_time', default_value='false', 
                              description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(name='model', default_value=default_model, 
                              description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(name='controllers', default_value=default_controllers, 
                              description='Absolute path to robot controllers file'),
        DeclareLaunchArgument(name='quadruped_config', default_value=default_quadruped_config, 
                              description='Absolute path to quadruped config file'),
        robot_state_publisher,
        bm_controllers,
        inverse_kinematics
    ])
