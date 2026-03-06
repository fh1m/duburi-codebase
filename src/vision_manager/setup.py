from setuptools import find_packages, setup

package_name = 'vision_manager'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/camera.launch.py']),
    ],
    install_requires=['setuptools', 'opencv-python', 'numpy', 'pyyaml'],
    zip_safe=True,
    maintainer='BRACU Duburi',
    maintainer_email='duburi@bracu.edu.bd',
    description='Camera management for Duburi 4.2 – enumeration, testing, streaming, calibration',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'camera_node = vision_manager.camera_node:main',
            'camera_enum = vision_manager.camera_enumerator:main',
            'camera_test = vision_manager.camera_tester:main',
            'camera_calibrate = vision_manager.camera_calibrator:main',
        ],
    },
)
