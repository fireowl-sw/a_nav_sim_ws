from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'pointcloud2_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fireowl',
    maintainer_email='fireowl@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'point_transform_node = pointcloud2_slam.1_point_pub_map:main',
            'obstacle_grid_node = pointcloud2_slam.2_map_pub:main',
        ],
    },
)
