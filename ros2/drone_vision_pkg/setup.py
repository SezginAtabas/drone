from setuptools import find_packages, setup

import os
import glob

package_name = 'drone_vision_pkg'

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
        # urdf files
        (os.path.join('share', package_name, 'urdf'), glob.glob(os.path.join('urdf', '*.xacro'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sezgin_atabas',
    maintainer_email='atabassezgin@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_vision_node = drone_vision_pkg.drone_vision_node:main',
            'trt_node = drone_vision_pkg.trt_node:main',
            'vision_pose_node = drone_vision_pkg.vision_pose_node:main',
        ],
    },
)
