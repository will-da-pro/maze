from setuptools import find_packages, setup

package_name = 'movement'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer="William D'Olier",
    maintainer_email='william@dolier.net',
    description='Package for interacting with motor controller over serial',
    license='GPL',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'twist_subscriber = movement.twist_subscriber:main',
            'odom_publisher = movement.odom_publisher:main',
        ],
    },
)
