import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'maze_navigator'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='William D\'Olier',
    maintainer_email='william@dolier.net',
    description='Navigator package for robocup maze robot',
    license='GPL',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigator_node = maze_navigator.navigator_node:main',
        ],
    },
)
