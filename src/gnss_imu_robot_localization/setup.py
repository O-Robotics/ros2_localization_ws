from setuptools import setup

package_name = 'gnss_imu_robot_localization'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/ekf_odm.launch.py',
            'launch/ekf_map.launch.py',
            'launch/navsat_transform.launch.py',
            'launch/bringup.launch.py',
            'launch/combo_bringup.launch.py', 
        ]),
        ('share/' + package_name + '/config', [
            'config/ekf_odm.yaml',
            'config/ekf_map.yaml',
            'config/navsat_transform.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Li Ziye',
    maintainer_email='liziye725@gmail.com',
    description='Launch + config for GNSS + IMU localization',
    license='MIT',
    tests_require=['pytest'],
    entry_points={},
)