from glob import glob
from setuptools import find_packages, setup

package_name = 'ros2_topic_monitor'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config/', glob('config/*', recursive=True)),
        ('share/' + package_name + '/launch/', glob('launch/*', recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ibrahim',
    maintainer_email='ibrahim.hroub7@gmail.com',
    description='ROS2 package to monitor multiple topics and update a GUI',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'monitor.py = ros2_topic_monitor.monitor:main',
            'record.py = ros2_topic_monitor.record:main'
        ],
    },
)
