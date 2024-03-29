from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os

def generate_launch_description():

    caramel_description_pkg_share = FindPackageShare(package='caramel_description').find('caramel_description')
    caramel_kinematics_pkg_description=FindPackageShare('caramel_kinematics').find('caramel_kinematics')
    
    default_model_path = os.path.join(caramel_description_pkg_share, 'urdf/caramel.urdf.xacro')
    default_rviz_config_path = os.path.join(caramel_kinematics_pkg_description, 'rviz/test_grid.rviz')

    robot_description = {'robot_description': Command(['xacro ', LaunchConfiguration('model')])}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description]
    )
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
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

    return LaunchDescription([
        DeclareLaunchArgument(name='gui', default_value='True',
                              description='Flag to enable joint_state_publisher_gui'),
        DeclareLaunchArgument(name='model', default_value=default_model_path,
                              description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(name='rviz_config', default_value=default_rviz_config_path,
                              description='Absolute path to rviz config file'),
        DeclareLaunchArgument(name='use_sim_time',default_value='true',
                              description='Use simulation (Gazebo) clock if true'),
        joint_state_publisher_gui,
        joint_state_publisher,
        robot_state_publisher,
        TimerAction(period=4.0, actions=[rviz])
    ])
