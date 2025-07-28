from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('gnss_imu_robot_localization'), 'config', 'ekf_odm.yaml')

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_odm_node',
            output='screen',
            parameters=[config_file],
            remappings=[('/odometry/filtered', '/odometry/filtered/local')]
        )
    ])