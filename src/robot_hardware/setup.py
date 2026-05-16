from setuptools import find_packages, setup

package_name = 'robot_hardware'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/hardware_driver.launch.py']),
        ('share/' + package_name + '/config', ['config/hardware_params.yaml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.com',
    description='Hardware driver node for differential drive tracked robot.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hardware_driver = robot_hardware.hardware_driver:main',
        ],
    },
)
