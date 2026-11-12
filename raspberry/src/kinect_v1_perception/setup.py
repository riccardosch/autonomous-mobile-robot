from setuptools import find_packages, setup

package_name = 'kinect_v1_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Riccardo Schiavo',
    maintainer_email='riccardo.schiavo.1@studenti.unipd.it',
    description='Perception nodes on top of the Kinect v1 driver.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sound_localizer = kinect_v1_perception.sound_localizer:main',
            'sound_classifier = kinect_v1_perception.sound_classifier:main',
        ],
    },
)
