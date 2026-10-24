from glob import glob

from setuptools import setup

package_name = 'agv_odometry'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='riccardosch',
    maintainer_email='riccardoschiavo003@gmail.com',
    description='Differential-drive wheel odometry and /cmd_vel to wheel targets for the AGV.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': ['odometry_node = agv_odometry.odometry_node:main'],
    },
)
