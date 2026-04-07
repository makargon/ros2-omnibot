from setuptools import setup

package_name = 'omnibot_actuator'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/actuator.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/actuator.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='makarGon',
    maintainer_email='makar6002@mail.ru',
    description='Motor control node for 3-wheel omnibot.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run = omnibot_actuator.actuator_node:main',
            'tester = omnibot_actuator.actuator_test_node:main',
        ],
    },
)
