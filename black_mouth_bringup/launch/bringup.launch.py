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

    default_model = os.path.join(
        pkg_description, "urdf", "black_mouth_real.urdf.xacro")

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Publish TF
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': Command(['xacro ', default_model])}],
    )

    # Load controller to publish joint data
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("black_mouth_description"),
                    "urdf",
                    "black_mouth_real.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description2 = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("black_mouth_control"),
            "config",
            "leg_controllers_real_robot.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description2, robot_controllers],
        output="both",
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

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(name='gui', default_value='false',
                                             description='Flag to enable joint_state_publisher_gui'),
        DeclareLaunchArgument(
            name='model', default_value=default_model, description='Absolute path to robot urdf file'),
        node_robot_state_publisher,
        control_node,
        load_joint_state_broadcaster,
        load_front_left_joint_trajectory_controller,
        load_front_right_joint_trajectory_controller,
        load_back_left_joint_trajectory_controller,
        load_back_right_joint_trajectory_controller,
    ])
