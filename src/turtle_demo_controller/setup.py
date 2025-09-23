from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtle_demo_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vedantknaik',
    maintainer_email='vnaik79014@gmail.com',
    description='ROS2 Turtlesim PID Controller with Distance and Position Movement',
    license='TODO: License declaration',
    tests_require=['pytest'],
        entry_points={
        'console_scripts': [
            'turt_controller = turtle_demo_controller.turtle_controller:main',
            'client = turtle_demo_controller.action_client:main',
            'distance_controller = turtle_demo_controller.distance_controller:main',
            'distance_client = turtle_demo_controller.distance_client:main',
            'mcp_bridge_service = turtle_demo_controller.mcp_bridge_service:main'
        ],
    },
)
