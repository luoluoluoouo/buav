import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'buav'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        # ('share/ament_index/resource_index/packages',
        #     ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')), 
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='ROS2 node for multi drone control',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'main = buav.main:main',
            'sim = buav.main:sim'
        ],
    },
)
