from setuptools import setup
from glob import glob
import os

package_name = 'omnibot_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, f'{package_name}.missions'],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='omnibot',
    maintainer_email='makar6002@mail.ru',
    description='Mission programming toolkit and examples for Omnibot.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_template = omnibot_mission.mission_template:main',
            'pickup_delivery_mission = omnibot_mission.missions.pickup_delivery_mission:main',
        ],
    },
)
