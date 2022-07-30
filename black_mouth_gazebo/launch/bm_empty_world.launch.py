import os
import launch
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    pkg_description=get_package_share_directory('black_mouth_description')
    default_model = os.path.join(pkg_description, "urdf","black_mouth.urdf.xacro")
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    default_rviz_config_path = os.path.join(pkg_description, 'rviz/urdf_config.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Gazebo server
    gzserver = ExecuteProcess(
        cmd=['gzserver',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             '-u',
             ''],
        output='screen',
    )

    # Gazebo client
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        # condition=IfCondition(LaunchConfiguration('gui')),
    )

    node_robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{'use_sim_time':use_sim_time,'robot_description': Command(['xacro ', default_model])}],
            )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    spawn_robot = Node(
    	package='gazebo_ros', 
    	executable='spawn_entity.py',
        name= 'urdf_spawner',
        output='screen',
        arguments=['-entity','quadruped', '-topic', '/robot_description'],
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


    return LaunchDescription([
        launch.actions.DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                                    description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model',          default_value=default_model,description='Absolute path to robot urdf file'),

        launch.actions.DeclareLaunchArgument(name='rviz_config', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        gzserver,
        gzclient,
        joint_state_publisher_gui,
        node_robot_state_publisher,
        joint_state_publisher,
        spawn_robot,
        rviz

    ])
