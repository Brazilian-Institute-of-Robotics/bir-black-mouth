import os
import launch
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_description = FindPackageShare('black_mouth_description').find('black_mouth_description')


    inverse_kinematics_pkg_share = FindPackageShare('black_mouth_kinematics').find('black_mouth_kinematics')

    default_model = os.path.join(
        pkg_description, "urdf", "black_mouth.urdf.xacro")
        
    default_rviz_config_path = os.path.join(
        pkg_description, 'rviz/urdf_config.rviz')
    
    # Load controller to publish joint data
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    # Load controller of the front left leg joints
    load_front_left_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'front_left_joint_trajectory_controller'],
        output='screen'
    )

    # Load controller of the front right leg joints
    load_front_right_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'front_right_joint_trajectory_controller'],
        output='screen'
    )

    # Load controller of the back left leg joints
    load_back_left_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'back_left_joint_trajectory_controller'],
        output='screen'
    )

    # Load controller of the back right leg joints
    load_back_right_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'back_right_joint_trajectory_controller'],
        output='screen'
    )

    load_all_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'all_joint_trajectory_controller'],
        output='screen'
    )

    inverse_kinematics = IncludeLaunchDescription(
              PythonLaunchDescriptionSource(
                os.path.join(inverse_kinematics_pkg_share, 'launch', 'kinematics.launch.py')),
            #   launch_arguments={'': ''}.items(),
  )


    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(name='gui', default_value='True',
                                             description='Flag to enable joint_state_publisher_gui'),
        DeclareLaunchArgument(
            name='model', default_value=default_model, description='Absolute path to robot urdf file'),

        DeclareLaunchArgument(name='rviz_config', default_value=default_rviz_config_path,
                                             description='Absolute path to rviz config file'),

        load_joint_state_broadcaster,
        load_front_left_joint_trajectory_controller,
        load_front_right_joint_trajectory_controller,
        load_back_left_joint_trajectory_controller,
        load_back_right_joint_trajectory_controller,
        # load_all_joint_trajectory_controller,

        inverse_kinematics
    ])
