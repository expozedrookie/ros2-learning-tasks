from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'tortoisebot_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arz-1013',
    maintainer_email='arz-1013@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'follow_the_closest_object = tortoisebot_nodes.ball_follower_script:main',
            'find_closest_object_distance= tortoisebot_nodes.lidar_filter_script:main',
            'find_distance_using_laser_filters=tortoisebot_nodes.filter_using_laser_filters:main'
        
        ],
    },
)
