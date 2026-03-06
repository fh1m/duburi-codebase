from setuptools import find_packages, setup

package_name = 'mavlink_logger'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='BRACU Duburi',
    maintainer_email='duburi@bracu.edu.bd',
    description='MAVLink logger for Duburi 4.2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'logger = mavlink_logger.logger_node:main',
        ],
    },
)
