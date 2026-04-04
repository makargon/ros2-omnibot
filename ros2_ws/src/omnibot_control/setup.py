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
            'launch/actuator.launch.py',
            'launch/teleop.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/motor_controller.yaml',
            'config/servo_controller.yaml',
            'config/actuator.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='omnibot',
    maintainer_email='maintainer@example.com',
    description='Motor control node for 3-wheel omnibot.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'actuator = omnibot_control.actuator.actuator_node:main',
            'actuator_tester = omnibot_control.actuator.actuator_test_node:main',
            'motor_controller = omnibot_control.motor_controller_node:main',
            'servo_controller = omnibot_control.servo_controller_node:main',
            'manual_control = omnibot_control.manual_control:main',
            'wheel_test = omnibot_control.wheel_test:main',
        ],
    },
)
