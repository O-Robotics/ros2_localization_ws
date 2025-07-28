from setuptools import find_packages, setup

package_name = 'imu_offset_calibration'

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
    maintainer='dev',
    maintainer_email='liziye725@gmail.com',
    description='ROS2 package for IMU offset calibration and pitch reading',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_pitch_reader = imu_offset_calibration.imu_pitch_reader:main',
        ],
    },
)
