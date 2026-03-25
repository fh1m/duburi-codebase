import os
from glob import glob
from setuptools import setup

package_name = 'duburi_planner'

setup(
    name=package_name,
    version='1.0.0',
    packages=[
        package_name,
        f'{package_name}.states',
        f'{package_name}.missions',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='BRACU Duburi',
    maintainer_email='duburi@bracu.edu.bd',
    description='YASMIN FSM mission planner for BRACU Duburi AUV — RoboSub 2026',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mission_node = duburi_planner.mission_node:main',
            'demo_node = duburi_planner.demo_node:main',
        ],
    },
)
