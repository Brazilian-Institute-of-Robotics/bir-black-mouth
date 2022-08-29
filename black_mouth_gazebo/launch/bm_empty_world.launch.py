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

    pkg_gazebo_ros = FindPackageShare('gazebo_ros').find('gazebo_ros')

    inverse_kinematics_pkg_share = FindPackageShare('black_mouth_kinematics').find('black_mouth_kinematics')

    default_model = os.path.join(
        pkg_description, "urdf", "black_mouth.urdf.xacro")
        
    default_rviz_config_path = os.path.join(
        pkg_description, 'rviz/urdf_config.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    gzserver = ExecuteProcess(
        cmd=['gzserver',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             '/home/workspaces/black_mouth_humble/src/bir-black-mouth/black_mouth_gazebo/worlds/empty.world'
             ''],
        output='screen',
    )

    # Gazebo client
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        # condition=IfCondition(LaunchConfiguration('gui')),
    )

    # Publish TF
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': Command(['xacro ', default_model])}],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        output='screen',
        arguments=['-entity','quadruped',
                     '-topic', '/robot_description',
                     '-x', '0',
                     '-y', '0',
                     '-z', '0.0'
                     ],
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
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
                    "black_mouth.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description2 = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("black_mouth_control"),
            "config",
            "leg_controllers.yaml",
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
        gzserver,
        gzclient,
        # joint_state_publisher_gui,
        node_robot_state_publisher,
        # joint_state_publisher,
        spawn_robot,
        # rviz,
        control_node,
        load_joint_state_broadcaster,
        load_front_left_joint_trajectory_controller,
        load_front_right_joint_trajectory_controller,
        load_back_left_joint_trajectory_controller,
        load_back_right_joint_trajectory_controller,
        inverse_kinematics
    ])
