from setuptools import find_packages, setup

import os
import glob

package_name = 'drone_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # get all launch files
        (os.path.join('share', package_name, 'launch'), glob.glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # get config files
        (os.path.join('share', package_name, 'config'), glob.glob(os.path.join('config', '*.yaml'))),
        # rviz files
        (os.path.join('share', package_name, 'rviz2'), glob.glob(os.path.join('rviz2', '*.rviz'))),
        # urdf files
        (os.path.join('share', package_name, 'urdf'), glob.glob(os.path.join('urdf', '*.xacro'))),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xtrana',
    maintainer_email='atabassezgin@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
