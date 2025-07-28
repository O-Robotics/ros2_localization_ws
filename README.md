# Localization Workspace

ROS2 workspace for autonomous mobile robot localization using GNSS and IMU sensor fusion.

## Quick Start

```bash
# Build workspace
cd /home/dev/ORobotics/localization_ws
colcon build
source install/setup.bash

# Launch localization
ros2 launch gnss_imu_robot_localization bringup.launch.py

# Launch with data recording
ros2 launch gnss_imu_robot_localization bringup.launch.py \
    enable_bag_recording:=true \
    bag_config_file:="compressed_config.yaml"
```

## Packages

- **gnss_imu_robot_localization** - Main localization system
- **bag_recorder** - Sensor data recording
- **wit_ros2_imu** - IMU driver
- **ublox_dgnss*** - GNSS drivers
- **amr_sweeper_description** - Robot description

## Additional Dependencies

These packages need to be installed separately (not included in ROS2 Humble desktop):

```bash
# ROS2 bag recording tools
sudo apt install -y ros-humble-rosbag2-storage-default-plugins
sudo apt install -y ros-humble-rosbag2-storage-sqlite3
sudo apt install -y ros-humble-rosbag2-compression
sudo apt install -y ros-humble-rosbag2-compression-zstd

# Robot localization (EKF)
sudo apt install -y ros-humble-robot-localization

# Serial communication for IMU
sudo apt install -y python3-serial

# Build tools (if not already installed)
sudo apt install -y python3-colcon-common-extensions
```

## File Structure

```
localization_ws/
├── src/
│   ├── gnss_imu_robot_localization/        # Main localization package
│   │   ├── launch/
│   │   │   ├── bringup.launch.py           # Main system launch
│   │   │   └── ekf_odm.launch.py           # EKF-only launch
│   │   ├── config/
│   │   │   └── ekf_params.yaml             # EKF configuration
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── README.md
│   ├── bag_recorder/                       # Sensor data recording
│   │   ├── bag_recorder/
│   │   │   ├── __init__.py
│   │   │   ├── bag_recorder.py             # Main recording node
│   │   │   └── bag_recorder_node.py        # ROS2 executable
│   │   ├── config/
│   │   │   ├── bag_config.yaml             # Default config
│   │   │   ├── compressed_config.yaml      # Compressed recording
│   │   │   └── extended_sensors_config.yaml # Future sensors
│   │   ├── launch/
│   │   │   ├── bag_record.launch.py        # Parameter-based launch
│   │   │   └── bag_record_yaml.launch.py   # YAML-based launch
│   │   ├── scripts/
│   │   │   ├── start_recording.sh          # Convenience script
│   │   │   └── start_recording_yaml.sh     # YAML convenience script
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── README.md
│   ├── wit_ros2_imu/                       # IMU driver
│   │   ├── wit_ros2_imu/
│   │   │   ├── __init__.py
│   │   │   └── wit_ros2_imu.py             # IMU driver (10Hz)
│   │   ├── launch/
│   │   │   └── rviz_and_imu.launch.py      # IMU launch
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── README.md
│   ├── ublox_dgnss/                        # GNSS base driver
│   ├── ublox_dgnss_node/                   # GNSS node
│   ├── ublox_nav_sat_fix_hp_node/          # High-precision GNSS
│   └── amr_sweeper_description/            # Robot description
│       ├── urdf/
│       ├── launch/
│       │   └── rsp.launch.py               # Robot state publisher
│       └── package.xml
├── build/                                  # Build artifacts (auto-generated)
├── install/                                # Installation files (auto-generated)
├── log/                                    # Build logs (auto-generated)
├── rebuild.sh                              # Workspace rebuild script
└── README.md                               # This file
```

## Build Commands

```bash
# Build all packages
colcon build

# Build specific package
colcon build --packages-select package_name

# Clean specific package
rm -rf build/package_name install/package_name

# Use rebuild script
./rebuild.sh                    # Build all
./rebuild.sh package_name       # Build specific
./rebuild.sh --clean package    # Clean and build
./rebuild.sh --help             # Show options
```

## Development

```bash
# Selective building (recommended)
colcon build --packages-select gnss_imu_robot_localization
source install/setup.bash

# Clean specific packages only (avoid rm -rf build/ install/ log/)
rm -rf build/package_name install/package_name
```

## Hardware

- ublox F9P GNSS receiver
- WIT motion sensor IMU
- USB connections

---

**Status**: Production Ready
