from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # AMR Sweeper Description
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('amr_sweeper_description'),
                    'launch',
                    'rsp.launch.py'
                ])
            )
        ),
        # WIT IMU and RViz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('wit_ros2_imu'),
                    'launch',
                    'rviz_and_imu.launch.py'
                ])
            )
        ),
        # Ublox GNSS
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('ublox_dgnss'),
                    'launch',
                    'ublox_rover_hpposllh_navsatfix.launch.py'
                ])
            )
        ),
    ])
