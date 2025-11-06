from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'tortoisebot_actions'

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
            'follow_the_closest_object = tortoisebot_actions.ball_follower_script:main',
            'find_the_closest_object = tortoisebot_actions.closest_object_finder:main',
            'action_server = tortoisebot_actions.action_server_test:main',
            'action_clientt = tortoisebot_actions.action_client_test:main',
        ],
    },
)
