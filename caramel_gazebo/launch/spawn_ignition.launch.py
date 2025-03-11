import launch
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

  caramel_description_pkg_share = FindPackageShare('caramel_description').find('caramel_description')

  default_model_path = os.path.join(caramel_description_pkg_share, 'urdf/caramel.urdf.xacro')
  default_rviz_config_path = os.path.join(caramel_description_pkg_share, 'rviz/config.rviz')

  spawn_robot = Node(
    package='ros_ign_gazebo',
    executable='create',
    arguments=['-name', 'caramel',
               '-topic', 'robot_description',
               '-z', '0.6',  # Argumento para posição
              ],
    output='screen'
  )

  return launch.LaunchDescription([
    launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                         description='Absolute path to robot urdf file'),
    launch.actions.DeclareLaunchArgument(name='use_rviz', default_value='true',
                                         description='Use RViz if true'),
    launch.actions.DeclareLaunchArgument(name='rviz_config', default_value=default_rviz_config_path,
                                         description='Absolute path to rviz config file'),
    spawn_robot,
  ])