from setuptools import find_packages, setup

package_name = 'mavlink_runner'

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
    description='Duburi CLI for AUV testing',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'runner = mavlink_runner.runner:main',
        ],
    },
)
