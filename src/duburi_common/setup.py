from setuptools import setup

package_name = 'duburi_common'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='BRACU Duburi',
    maintainer_email='duburi@bracu.edu.bd',
    description='Shared constants and command vocabulary for Duburi AUV',
    license='Apache-2.0',
)
