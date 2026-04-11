from setuptools import setup

package_name = 'omnibot_movement'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/movement.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='elf',
    maintainer_email='so02s@mail.ru',
    description='movement control node for 3-wheel omnibot.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run = omnibot_movement.movement_node:main',
        ],
    },
)
