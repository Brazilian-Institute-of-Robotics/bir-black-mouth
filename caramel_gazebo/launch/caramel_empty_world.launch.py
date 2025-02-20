import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    # Caminhos dos pacotes
    ros_ign_gazebo_pkg_share = FindPackageShare('ros_ign_gazebo').find('ros_ign_gazebo')
    caramel_gazebo_pkg_share = FindPackageShare('caramel_gazebo').find('caramel_gazebo')
    caramel_bringup_pkg_share = FindPackageShare('caramel_bringup').find('caramel_bringup')
    caramel_description_pkg_share = FindPackageShare('caramel_description').find('caramel_description')
    caramel_control_pkg_share = FindPackageShare('caramel_control').find('caramel_control')
    caramel_kinematics_pkg_share = FindPackageShare('caramel_kinematics').find('caramel_kinematics')
    
    # Caminhos padrão dos arquivos
    default_world = os.path.join(caramel_gazebo_pkg_share, 'worlds/mario_world.sdf')  # Mudança para .sdf
    default_rviz_config = os.path.join(caramel_description_pkg_share, 'rviz/urdf_config.rviz')
    default_model = os.path.join(caramel_description_pkg_share, "urdf", "caramel.urdf.xacro")
    default_controllers = os.path.join(caramel_control_pkg_share, "config", "leg_controllers.yaml")
    default_quadruped_config = os.path.join(caramel_kinematics_pkg_share, 'config', 'quadruped.yaml')
    default_body_control_config = os.path.join(caramel_control_pkg_share, 'config', 'body_control.yaml')
    

    ignition = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(ros_ign_gazebo_pkg_share, 'launch', 'ign_gazebo.launch.py')),
                launch_arguments={'ign_args': ['-r ', LaunchConfiguration('world')]}.items(),
    )

    # Spawn do robô
    spawn_ignition = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                        os.path.join(caramel_gazebo_pkg_share, 'launch', 'spawn_ignition.launch.py')),
                        launch_arguments={'model': LaunchConfiguration('model'),
                                        'use_rviz': LaunchConfiguration('use_rviz'),
                                        'rviz_config': LaunchConfiguration('rviz_config')}.items(),
    )

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(caramel_bringup_pkg_share, 'launch', 'bringup.launch.py')),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'),
                          'model': LaunchConfiguration('model'),
                          'controllers': LaunchConfiguration('controllers'),
                          'quadruped_config': LaunchConfiguration('quadruped_config'),
                          'body_control_config': LaunchConfiguration('body_control_config'),
                          'launch_joy_node': LaunchConfiguration('launch_joy_node'),
                          'launch_joy_teleop': LaunchConfiguration('launch_joy_teleop'),
                          'joy_type': LaunchConfiguration('joy_type'),
                          'launch_imu': LaunchConfiguration('launch_imu'),
                          }.items(),
    )


    return LaunchDescription([
        # Declaração de parâmetros
        DeclareLaunchArgument(name='use_sim_time', default_value='True', 
                              description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(name='world', default_value=default_world, 
                              description='Absolute path to world file'),
        DeclareLaunchArgument(name='model', default_value=default_model, 
                              description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(name='controllers', default_value=default_controllers, 
                              description='Absolute path to robot controllers file'),
        DeclareLaunchArgument(name='quadruped_config', default_value=default_quadruped_config, 
                              description='Absolute path to quadruped config file'),
        DeclareLaunchArgument(name='body_control_config', default_value=default_body_control_config, 
                              description='Absolute path to body control config file'),
        DeclareLaunchArgument(name='use_rviz', default_value='false', 
                              description='Start rviz if true'),
        DeclareLaunchArgument(name='rviz_config', default_value=default_rviz_config, 
                              description='Absolute path to rviz config file'),
        DeclareLaunchArgument(name='launch_joy_node', default_value='True',
                              description='Whether to launch joy node or not'),
        DeclareLaunchArgument(name='launch_joy_teleop', default_value='True',
                              description="Whether to launch bm joy teleop or not"),
        DeclareLaunchArgument(name='joy_type', default_value='generic', 
                              description='Set the joystick type (generic, x360 or ps4)'),
        DeclareLaunchArgument(name='launch_imu', default_value='False', 
                              description='Whether to launch imu or not'),
        
        # Ações para iniciar simulação
        ignition,
        spawn_ignition,
        bringup,
    ])