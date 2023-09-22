import os
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    imu_pkg_share = FindPackageShare('mpu6050_driver_ros2').find('mpu6050_driver_ros2')

    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(imu_pkg_share, 'launch', 'mpu6050_driver_with_filter.launch.py')),
        condition=IfCondition(LaunchConfiguration('launch_imu')),
    )

    return LaunchDescription([imu])
