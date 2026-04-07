from setuptools import setup

package_name = 'omnibot_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/motor_controller.launch.py',
            'launch/servo_controller.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/motor_controller.yaml',
            'config/servo_controller.yaml',
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
            'motor_controller = omnibot_control.motor_controller_node:main',
            'servo_controller = omnibot_control.servo_controller_node:main',
            'manual_control = omnibot_control.manual_control:main',
            'wheel_test = omnibot_control.wheel_test:main',
        ],
    },
)
