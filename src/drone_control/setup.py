from setuptools import find_packages, setup

package_name = 'drone_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sozureke',
    maintainer_email='sozure.synergy@gmail.com',
    description='Low-level controller for drone control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'command_gateway = drone_control.command_gateway:main',
            'state_publisher = drone_control.state_publisher:main',
            'fsm_action_server = drone_control.fsm_action_server:main',
        ],
    },
)
