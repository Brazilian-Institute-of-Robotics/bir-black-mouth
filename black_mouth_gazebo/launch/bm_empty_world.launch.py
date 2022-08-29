import os
import launch
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    bm_description_pkg_share = FindPackageShare('black_mouth_description').find('black_mouth_description')

    pkg_gazebo_ros = FindPackageShare('gazebo_ros').find('gazebo_ros')

    default_model = os.path.join(
        bm_description_pkg_share, "urdf", "black_mouth.urdf.xacro")
        
    default_rviz_config_path = os.path.join(
        bm_description_pkg_share, 'rviz/urdf_config.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    gzserver = ExecuteProcess(
        cmd=['gzserver',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             '/home/devel_ws/bm/src/bir-black-mouth/black_mouth_gazebo/worlds/empty.world'
             ''],
        output='screen',
    )

    # Gazebo client
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        # condition=IfCondition(LaunchConfiguration('gui')),
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
                     '-z', '0.0'],
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

    start_robot = IncludeLaunchDescription(
              PythonLaunchDescriptionSource(
                os.path.join(bm_description_pkg_share, 'launch', 'start_robot.launch.py')),
            #   launch_arguments={'': ''}.items(),
    )


    return LaunchDescription([
        DeclareLaunchArgument(name='use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(name='gui', default_value='True', description='Flag to enable joint_state_publisher_gui'),
        DeclareLaunchArgument(name='model', default_value=default_model, description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(name='rviz_config', default_value=default_rviz_config_path, description='Absolute path to rviz config file'),
        gzserver,
        gzclient,
        # joint_state_publisher_gui,
        # joint_state_publisher,
        start_robot,
        spawn_robot,
        # rviz,
        control_node,

    ])
