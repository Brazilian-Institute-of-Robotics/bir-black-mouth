import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    bm_gazebo_pkg_share = FindPackageShare('black_mouth_gazebo').find('black_mouth_gazebo')
    bm_description_pkg_share = FindPackageShare('black_mouth_description').find('black_mouth_description')
    bm_control_pkg_share = FindPackageShare('black_mouth_control').find('black_mouth_control')
    bm_kinematics_pkg_share = FindPackageShare('black_mouth_kinematics').find('black_mouth_kinematics')
        
    default_world = os.path.join(bm_gazebo_pkg_share, 'worlds/empty.world')
    default_rviz_config = os.path.join(bm_description_pkg_share, 'rviz/urdf_config.rviz')
    default_model = os.path.join(bm_description_pkg_share, "urdf", "black_mouth.urdf.xacro")
    default_controllers = os.path.join(bm_control_pkg_share, "config", "leg_controllers.yaml")
    default_quadruped_config = os.path.join(bm_kinematics_pkg_share, 'config', 'quadruped.yaml')
    
    gzserver = ExecuteProcess(
        cmd=['gzserver',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             LaunchConfiguration('world'),
             ''],
        output='screen',
    )

    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
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

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    start_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bm_description_pkg_share, 'launch', 'start_robot.launch.py')),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'),
                          'model': LaunchConfiguration('model'),
                          'controllers': LaunchConfiguration('controllers'),
                          'quadruped_config': LaunchConfiguration('quadruped_config'),
                          }.items(),
    )


    return LaunchDescription([
        DeclareLaunchArgument(name='use_sim_time', default_value='false', 
                              description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(name='world', default_value=default_world, 
                              description='Absolute path to world file'),
        DeclareLaunchArgument(name='model', default_value=default_model, 
                              description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(name='controllers', default_value=default_controllers, 
                              description='Absolute path to robot controllers file'),
        DeclareLaunchArgument(name='quadruped_config', default_value=default_quadruped_config, 
                              description='Absolute path to quadruped config file'),
        DeclareLaunchArgument(name='use_rviz', default_value='false', 
                              description='Start rviz if true'),
        DeclareLaunchArgument(name='rviz_config', default_value=default_rviz_config, 
                              description='Absolute path to rviz config file'),
        gzserver,
        gzclient,
        spawn_robot,
        start_robot,
        rviz,
    ])
