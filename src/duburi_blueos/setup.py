import glob

from setuptools import find_packages, setup

package_name = 'duburi_blueos'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.py')),
        ('share/' + package_name + '/config', glob.glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'requests', 'aiohttp', 'websocket-client'],
    zip_safe=True,
    maintainer='BRACU Duburi',
    maintainer_email='duburi@bracu.edu.bd',
    description='BlueOS integration for Duburi AUV 4.2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'blueos_monitor = duburi_blueos.blueos_monitor_node:main',
            'mavlink_bridge = duburi_blueos.mavlink_bridge_node:main',
            'mavros_bridge = duburi_blueos.mavros_bridge_node:main',
        ],
    },
)
