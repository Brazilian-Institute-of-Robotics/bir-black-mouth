import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    caramel_description_pkg_share = FindPackageShare('caramel_description').find('caramel_description')
    caramel_control_pkg_share = FindPackageShare('caramel_control').find('caramel_control')

    default_controllers = os.path.join(caramel_control_pkg_share, "config", "leg_controllers.yaml")
    robot_controllers = LaunchConfiguration('controllers', default=default_controllers)

    default_model = os.path.join(caramel_description_pkg_share, "urdf", "caramel.urdf.xacro")    
    robot_model = LaunchConfiguration('model', default=default_model)

    # 1. O Gerenciador Principal (Carrega o driver C++)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': Command(['xacro ', robot_model])}, robot_controllers],
        output="both",
    )

    # 2. Joint State Broadcaster
    # Esse DEVE nascer ATIVO para você ver o robô no Rviz mesmo parado
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # 3. Controladores das Pernas 
    # ATENÇÃO: A flag "--inactive" é o segredo para o JoyTeleop ligar tudo junto depois
    
    front_left_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["front_left_joint_trajectory_controller", "--controller-manager", "/controller_manager", "--inactive"],
    )

    front_right_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["front_right_joint_trajectory_controller", "--controller-manager", "/controller_manager", "--inactive"],
    )

    back_left_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["back_left_joint_trajectory_controller", "--controller-manager", "/controller_manager", "--inactive"],
    )

    back_right_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["back_right_joint_trajectory_controller", "--controller-manager", "/controller_manager", "--inactive"],
    )

    # O Controller Toggler (Seu código C++ novo)
    # Ele não precisa de args especiais, pois já configuramos os tópicos no código
    controller_toggler = Node(
        package="caramel_control",
        executable="controller_toggler_node",
        name="controller_toggler_node",
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='model', default_value=default_model, 
                              description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(name='controllers', default_value=default_controllers, 
                              description='Absolute path to robot controllers file'),
        
        control_node,
        
        # Spawners
        joint_state_broadcaster_spawner,
        front_left_spawner,
        front_right_spawner,
        back_left_spawner,
        back_right_spawner,
        
        controller_toggler,
    ])