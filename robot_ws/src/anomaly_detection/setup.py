import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'anomaly_detection'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='PK Huy',
    maintainer_email='huy@todo.todo',
    description='Crack/anomaly detection node for inspection robot on Raspberry Pi 4',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'anomaly_detector = anomaly_detection.anomaly_detector:main',
            'position_bridge = anomaly_detection.position_bridge:main',
            'coordinator = anomaly_detection.coordinator:main',
        ],
    },
)
