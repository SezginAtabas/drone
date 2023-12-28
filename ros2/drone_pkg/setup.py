from setuptools import find_packages, setup

package_name = 'drone_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'drone_controller_node = drone_pkg.drone_controller_node:main',
            'drone_example = drone_pkg.drone_example:main',
            'trt_run = drone_pkg.trt_node:main',
        ],
    },
)
