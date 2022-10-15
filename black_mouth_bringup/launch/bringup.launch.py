import os
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
  
    bm_description_pkg_share = FindPackageShare('black_mouth_description').find('black_mouth_description')
    bm_kinematics_pkg_share = FindPackageShare('black_mouth_kinematics').find('black_mouth_kinematics')
    bm_control_pkg_share = FindPackageShare('black_mouth_control').find('black_mouth_control')
    bm_gait_planner_pkg_share = FindPackageShare('black_mouth_gait_planner').find('black_mouth_gait_planner')
    bm_teleop_pkg_share = FindPackageShare('black_mouth_teleop').find('black_mouth_teleop')
    imu_pkg_share = FindPackageShare('mpu6050_driver_ros2').find('mpu6050_driver_ros2')


    default_model = os.path.join(bm_description_pkg_share, "urdf", "black_mouth_real.urdf.xacro")
    robot_model = LaunchConfiguration('model', default=default_model)

    default_controllers = os.path.join(bm_control_pkg_share, "config", "leg_controllers_real_robot.yaml")
    robot_controllers = LaunchConfiguration('controllers', default=default_controllers)

    default_quadruped_config = os.path.join(bm_kinematics_pkg_share, 'config', 'quadruped.yaml')
    quadruped_config = LaunchConfiguration('quadruped_config', default=default_quadruped_config)

    default_body_control_config = os.path.join(bm_control_pkg_share, 'config', 'body_control.yaml')
    body_control_config = LaunchConfiguration('body_control_config', default=default_body_control_config)

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    joy_type = LaunchConfiguration('joy_type', default="generic")


    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(imu_pkg_share, 'launch', 'mpu6050_driver_with_filter.launch.py')),
        condition=IfCondition(LaunchConfiguration('launch_imu')),
    )
    
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': Command(['xacro ', robot_model])}],
    )
    
    bm_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bm_control_pkg_share, 'launch', 'bm_load_controller.launch.py')),
        launch_arguments={'model': robot_model,
                          'controllers': robot_controllers}.items(),
    )

    inverse_kinematics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bm_kinematics_pkg_share, 'launch', 'kinematics.launch.py')),
        launch_arguments={'quadruped_config': quadruped_config}.items(),
    )

    body_control = Node(
        package="black_mouth_control",
        executable="body_control",
        name="body_control_node",
        parameters=[body_control_config],
        output="screen",
        remappings=[('imu/data', 'imu/out' if use_sim_time == "True" else 'imu/data')]
    )

    gait_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bm_gait_planner_pkg_share, 'launch', 'gait_planner.launch.py')),
    )

    joy_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bm_teleop_pkg_share, 'launch', 'joy_teleop.launch.py')),
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
        robot_state_publisher,
        bm_controllers,
        inverse_kinematics,
        body_control,
        gait_planner,
        TimerAction(period=5.0, actions=[joy_teleop])
    ])
