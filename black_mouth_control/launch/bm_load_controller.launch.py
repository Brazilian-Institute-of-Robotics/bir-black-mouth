import os 
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    bm_description_pkg_share = FindPackageShare('black_mouth_description').find('black_mouth_description')
    bm_control_pkg_share = FindPackageShare('black_mouth_control').find('black_mouth_control')

    default_controllers = os.path.join(bm_control_pkg_share, "config", "leg_controllers.yaml")
    robot_controllers = LaunchConfiguration('controllers', default=default_controllers)

    default_model = os.path.join(bm_description_pkg_share, "urdf", "black_mouth.urdf.xacro")    
    robot_model = LaunchConfiguration('model', default=default_model)

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': Command(['xacro ', robot_model])}, robot_controllers],
        output="both",
    )

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


    return LaunchDescription([
        DeclareLaunchArgument(name='model', default_value=default_model, 
                              description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(name='controllers', default_value=default_controllers, 
                              description='Absolute path to robot controllers file'),
        load_joint_state_broadcaster,
        load_front_left_joint_trajectory_controller,
        load_front_right_joint_trajectory_controller,
        load_back_left_joint_trajectory_controller,
        load_back_right_joint_trajectory_controller,
        # load_all_joint_trajectory_controller,
        control_node
    ])
