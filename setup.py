from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'RobotProject'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/Robot_Project.launch.py']),
        ('share/' + package_name, ['urdf/platform2.urdf'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jalma',
    maintainer_email='jalma@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'Kinematics = RobotProject.Kinematics_node:main',
                'Controller = RobotProject.Controller_node:main',
                'ball_detection_node = RobotProject.ball_detection_node:main',
                'Camera = RobotProject.Camera_node:main',
                'Sim_node = RobotProject.sim_node:main' 
        ],
    },
)
