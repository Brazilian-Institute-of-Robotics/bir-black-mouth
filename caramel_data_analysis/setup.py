from setuptools import setup
import os
from glob import glob

package_name = 'caramel_data_analysis'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'body_control_setpoint = caramel_data_analysis.body_control_setpoint:main',
            'foot_follow_trajectory = caramel_data_analysis.foot_follow_trajectory:main'
        ],
    },
)
