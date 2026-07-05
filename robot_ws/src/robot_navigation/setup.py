import os
from glob import glob
from setuptools import setup

package_name = 'robot_navigation'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='PKHuy1455',
    maintainer_email='huyphan1455@gmail.com',
    description='Navigation and SLAM package for crawler robot',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)
