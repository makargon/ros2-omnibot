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
    maintainer='omnibot',
    maintainer_email='maintainer@example.com',
    description='Actuator control node for 3-wheel omnibot.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run = ' + package_name + '.actuator_node:main',
            'tester = ' + package_name + '.actuator_test_node:main',
        ],
    },
)
