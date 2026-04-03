from glob import glob
import os

from setuptools import setup

package_name = 'omnibot_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='makarGon',
    maintainer_email='makar6002@mail.ru',
    description='USB camera capture and ArUco pose estimation for omnibot.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = omnibot_vision.camera_node:main',
            'aruco_pose_node = omnibot_vision.aruco_pose_node:main',
            'camera_calibration_capture = omnibot_vision.camera_calibration_capture:main',
            'fisheye_calibrate = omnibot_vision.fisheye_calibrate:main',
        ],
    },
)
