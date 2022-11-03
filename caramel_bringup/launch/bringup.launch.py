import os
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
  
    caramel_description_pkg_share = FindPackageShare('caramel_description').find('caramel_description')
    caramel_kinematics_pkg_share = FindPackageShare('caramel_kinematics').find('caramel_kinematics')
    caramel_control_pkg_share = FindPackageShare('caramel_control').find('caramel_control')
    caramel_gait_planner_pkg_share = FindPackageShare('caramel_gait_planner').find('caramel_gait_planner')
    caramel_teleop_pkg_share = FindPackageShare('caramel_teleop').find('caramel_teleop')
    caramel_bringup_pkg_share = FindPackageShare('caramel_bringup').find('caramel_bringup')


    default_model = os.path.join(caramel_description_pkg_share, "urdf", "caramel_real.urdf.xacro")
    robot_model = LaunchConfiguration('model', default=default_model)

    default_controllers = os.path.join(caramel_control_pkg_share, "config", "leg_controllers_real_robot.yaml")
    robot_controllers = LaunchConfiguration('controllers', default=default_controllers)

    default_quadruped_config = os.path.join(caramel_kinematics_pkg_share, 'config', 'quadruped.yaml')
    quadruped_config = LaunchConfiguration('quadruped_config', default=default_quadruped_config)

    default_body_control_config = os.path.join(caramel_control_pkg_share, 'config', 'body_control.yaml')
    body_control_config = LaunchConfiguration('body_control_config', default=default_body_control_config)

    use_sim_time = LaunchConfiguration('use_sim_time', default='False')
    joy_type = LaunchConfiguration('joy_type', default="generic")


    feet_listener = Node(
        package="caramel_description",
        executable="default_feet_poses_transform.py",
        output="screen",
    )
    
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': Command(['xacro ', robot_model])}],
    )

    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(caramel_bringup_pkg_share, 'launch', 'mpu6050.launch.py')),
         condition=UnlessCondition(use_sim_time)
    )
    
    caramel_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(caramel_control_pkg_share, 'launch', 'caramel_load_controller.launch.py')),
        launch_arguments={'model': robot_model,
                          'controllers': robot_controllers}.items(),
    )

    inverse_kinematics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(caramel_kinematics_pkg_share, 'launch', 'kinematics.launch.py')),
        launch_arguments={'quadruped_config': quadruped_config}.items(),
    )

    body_control = Node(
        package="caramel_control",
        executable="body_control",
        name="body_control_node",
        parameters=[body_control_config],
        output="screen",
        condition=UnlessCondition(PythonExpression([use_sim_time, ' == True']))
    )
    body_control_remapped = Node(
        package="caramel_control",
        executable="body_control",
        name="body_control_node",
        parameters=[body_control_config],
        output="screen",
        remappings=[('imu/data', 'imu/out')],
        condition=IfCondition(PythonExpression([use_sim_time, ' == True']))
    )

    gait_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(caramel_gait_planner_pkg_share, 'launch', 'gait_planner.launch.py')),
    )

    joy_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(caramel_teleop_pkg_share, 'launch', 'joy_teleop.launch.py')),
        launch_arguments={'launch_joy_node': LaunchConfiguration('launch_joy_node'),
                          'joy_type': joy_type}.items(),
        condition=IfCondition(LaunchConfiguration('launch_joy_teleop')),
    )


    return LaunchDescription([
        DeclareLaunchArgument(name='use_sim_time', default_value='False', 
                              description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(name='model', default_value=default_model, 
                              description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(name='controllers', default_value=default_controllers, 
                              description='Absolute path to robot controllers file'),
        DeclareLaunchArgument(name='quadruped_config', default_value=default_quadruped_config, 
                              description='Absolute path to quadruped config file'),
        DeclareLaunchArgument(name='body_control_config', default_value=default_body_control_config, 
                              description='Absolute path to body control config file'),
        DeclareLaunchArgument(name='launch_joy_node', default_value='True',
                              description='Whether to launch joy node or not'),
        DeclareLaunchArgument(name='launch_joy_teleop', default_value='True',
                              description="Whether to launch bm joy teleop or not"),
        DeclareLaunchArgument(name='joy_type', default_value='generic', 
                              description='Set the joystick type (generic, x360 or ps4)'),
        DeclareLaunchArgument(name='launch_imu', default_value='True', 
                              description='Whether to launch imu or not'),
        imu,
        feet_listener,
        robot_state_publisher,
        TimerAction(period=1.0, actions=[caramel_controllers]),
        TimerAction(period=2.0, actions=[inverse_kinematics]),
        TimerAction(period=3.0, actions=[body_control, body_control_remapped]),
        TimerAction(period=4.0, actions=[gait_planner]),
        TimerAction(period=5.0, actions=[joy_teleop])
    ])
