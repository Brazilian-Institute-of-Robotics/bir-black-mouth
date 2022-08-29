from launch.actions import ExecuteProcess
from launch import LaunchDescription


def generate_launch_description():

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
        load_joint_state_broadcaster,
        load_front_left_joint_trajectory_controller,
        load_front_right_joint_trajectory_controller,
        load_back_left_joint_trajectory_controller,
        load_back_right_joint_trajectory_controller,
        # load_all_joint_trajectory_controller,
    ])
