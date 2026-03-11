from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vision_inspector'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/cameras.yaml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'opencv-python', 'numpy', 'pyyaml'],
    zip_safe=True,
    maintainer='BRACU Duburi',
    maintainer_email='duburi@bracu.edu.bd',
    description='Camera orchestrator for Duburi 4.2 – multi-camera streaming, reconnection, calibration, recording, and tools',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'camera_manager = vision_inspector.camera_manager_node:main',
            'camera_enum = vision_inspector.camera_enumerator:main',
            'camera_test = vision_inspector.camera_tester:main',
            'camera_calibrate = vision_inspector.camera_calibrator:main',
            'camera_record = vision_inspector.camera_recorder:main',
            'camera_playback = vision_inspector.camera_playback:main',
        ],
    },
)
