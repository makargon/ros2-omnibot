from setuptools import setup
from glob import glob
import os

package_name = 'omnibot_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'missions'), glob(os.path.join('missions', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='makarGon',
    maintainer_email='makar6002@mail.ru',
    description='Common launch files for omnibot stack.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'robot_launch = omnibot_bringup.launch.robot_launch:generate_launch_description',
    ]},
    scripts=[
        'missions/demo.py',
    ]
)
