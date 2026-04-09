from setuptools import setup
from glob import glob
import os

package_name = 'omnibot_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='makarGon',
    maintainer_email='makar6002@mail.ru',
    description='LIDAR and SLAM perception package for omnibot using RPLiDAR A1M8.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_node = omnibot_perception.lidar_node:main',
            'tf_broadcaster = omnibot_perception.tf_broadcaster:main',
            'servo_joint_state_node = omnibot_perception.servo_joint_state_node:main',
        ],
    },
)
