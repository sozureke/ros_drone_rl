from setuptools import setup, find_packages
import glob
import os

package_name = 'drone_rl_agent'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         [os.path.join('resource', package_name)]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         glob.glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=[
        'setuptools',
        'gym',
        'numpy',
        'stable-baselines3',
    ],
    zip_safe=True,
    maintainer='sozureke',
    maintainer_email='sozure.synergy@gmail.com',
    description='RL-agent landing a drone on a moving platform',
    license='MIT',
    entry_points={
        'console_scripts': [
            'train_agent = drone_rl_agent.training:main',
            'rl_inference = drone_rl_agent.env:run_inference_node',
        ],
    },
)
