from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    feet_broadcaster = Node(
        package="black_mouth_data_analysis",
        executable="feet_default_poses_broadcaster",
        name="feet_default_poses_broadcaster",
        output="screen",
        parameters=[{'use_sim_time': use_sim_time}],
    )

    feet_listener = Node(
        package="black_mouth_data_analysis",
        executable="feet_listener",
        name="feet_listener",
        output="screen",
        parameters=[{'use_sim_time': use_sim_time}],
    )

    body_control_setpoint = Node(
        package="black_mouth_data_analysis",
        executable="body_control_setpoint",
        name="body_control_setpoint",
        output="screen",
        remappings=[('imu/data', 'imu/out' if PythonExpression([use_sim_time, ' == True']) else 'imu/data')]
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='use_sim_time', default_value='False', 
                              description='Use simulation (Gazebo) clock if true'),
        feet_broadcaster,
        feet_listener,
        body_control_setpoint
    ])
