from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('gnss_imu_robot_localization'), 'config', 'navsat_transform.yaml')

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[config_file],
            remappings=[
                ('/imu/data', '/imu/data'),
                ('/gps/fix', '/fix'),
                ('/odometry/filtered', '/odometry/filtered'),
            ]
        )
    ])