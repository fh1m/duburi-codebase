from setuptools import find_packages, setup

package_name = 'vision'

setup(
    name=package_name,
    version='1.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/vision.launch.py']),
    ],
    install_requires=[
        'setuptools',
        'ultralytics',
        'supervision',
        'opencv-python',
        'numpy',
        'simple-pid',
    ],
    zip_safe=True,
    maintainer='BRACU Duburi',
    maintainer_email='duburi@bracu.edu.bd',
    description='YOLO object detection, Kalman tracking, and visual servo alignment for Duburi 4.2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'detector_node = vision.detector_node:main',
            'detector_standalone = vision.detector_standalone:main',
            'alignment_controller = vision.alignment_controller:main',
        ],
    },
)
