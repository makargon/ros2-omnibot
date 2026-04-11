from setuptools import setup

package_name = 'omnibot_startup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/startup.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='elf',
    maintainer_email='so02s@mail.ru',
    description='Startup Node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run = omnibot_startup.startup_node:main',
        ],
    },
)
