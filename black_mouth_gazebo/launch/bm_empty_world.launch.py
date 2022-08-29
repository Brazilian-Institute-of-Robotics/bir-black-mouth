import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    bm_description_pkg_share = FindPackageShare('black_mouth_description').find('black_mouth_description')

    pkg_gazebo_ros = FindPackageShare('gazebo_ros').find('gazebo_ros')

    default_model = os.path.join(bm_description_pkg_share, "urdf", "black_mouth.urdf.xacro")
        
    default_rviz_config_path = os.path.join(bm_description_pkg_share, 'rviz/urdf_config.rviz')

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
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.0'],
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
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
    ])
