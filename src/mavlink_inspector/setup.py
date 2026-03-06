from setuptools import find_packages, setup

package_name = 'mavlink_inspector'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/duburi_control.launch.py']),
    ],
    install_requires=['setuptools', 'pymavlink'],
    zip_safe=True,
    maintainer='BRACU Duburi',
    maintainer_email='duburi@bracu.edu.bd',
    description='MAVLink/Pixhawk connection inspector for Duburi 4.2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'inspector = mavlink_inspector.inspector_node:main',
        ],
    },
)
