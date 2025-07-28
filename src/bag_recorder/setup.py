from setuptools import find_packages, setup

package_name = 'bag_recorder'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/bag_record.launch.py',
            'launch/bag_record_yaml.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/bag_config.yaml',
            'config/compressed_config.yaml',
            'config/extended_sensors_config.yaml',
        ]),
        ('lib/' + package_name, [
            'bag_recorder/bag_recorder_node.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='liziye725@gmail.com',
    description='ROS2 package for recording sensor data to bag files (GNSS, IMU, and extensible for camera, wheel odometry, etc.)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bag_recorder = bag_recorder.bag_recorder:main',
        ],
    },
)
