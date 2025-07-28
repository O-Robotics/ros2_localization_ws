# GNSS IMU Robot Localization

A ROS2 package for autonomous mobile robot localization using GNSS and IMU sensor fusion with Extended Kalman Filter (EKF).

## Features

- **GNSS Integration**: High-precision ublox DGNSS receiver support
- **IMU Integration**: WIT motion sensor IMU data processing
- **Sensor Fusion**: Extended Kalman Filter for robust localization
- **Robot Description**: URDF-based robot model and transforms
- **Data Recording**: Optional bag recording for sensor data analysis
- **Production Ready**: Tested and validated for autonomous robot deployment

## Quick Start

### Basic Localization
```bash
# Source workspace
source install/setup.bash

# Start localization system
ros2 launch gnss_imu_robot_localization bringup.launch.py
```

### With Data Recording
```bash
# Start localization with bag recording
ros2 launch gnss_imu_robot_localization bringup.launch.py \
    enable_bag_recording:=true \
    bag_config_file:="compressed_config.yaml"
```

## System Components

### Hardware Requirements
- **GNSS Receiver**: ublox F9P or compatible DGNSS receiver
- **IMU Sensor**: WIT motion sensor or compatible 9-DOF IMU
- **Robot Platform**: AMR sweeper robot or compatible mobile platform

### Software Components
- **ublox_dgnss**: GNSS receiver driver and configuration
- **wit_ros2_imu**: IMU sensor driver and data processing
- **robot_localization**: Extended Kalman Filter for sensor fusion
- **amr_sweeper_description**: Robot URDF and transform tree
- **bag_recorder**: Optional data recording (standalone package)

## Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `enable_bag_recording` | `false` | Enable automatic bag recording |
| `bag_config_file` | `bag_config.yaml` | Configuration file for bag recording |

## Topics

### Published Topics
- `/fix` - GNSS position data (sensor_msgs/NavSatFix)
- `/imu/data` - IMU sensor data (sensor_msgs/Imu)
- `/odometry/filtered` - Filtered localization output (nav_msgs/Odometry)
- `/tf` - Transform tree (tf2_msgs/TFMessage)

### Subscribed Topics
- `/imu/raw_data` - Raw IMU data from sensor
- `/fix` - Raw GNSS data from receiver

## Configuration

### GNSS Configuration
The ublox DGNSS receiver is configured for:
- 10Hz measurement rate
- RTK high-precision positioning
- USB output protocol optimization

### IMU Configuration
The WIT IMU sensor provides:
- 9-DOF motion data (accelerometer, gyroscope, magnetometer)
- Calibrated orientation and angular velocity
- Serial communication via USB

### EKF Configuration
The Extended Kalman Filter fuses:
- GNSS position measurements
- IMU orientation and angular velocity
- Robot kinematic model constraints

## Installation

```bash
# Navigate to workspace
cd /home/dev/ORobotics/localization_ws

# Build the package
colcon build --packages-select gnss_imu_robot_localization

# Source the workspace
source install/setup.bash
```

## Dependencies

- ROS2 Humble
- robot_localization
- ublox_dgnss_node
- ublox_nav_sat_fix_hp_node
- wit_ros2_imu
- amr_sweeper_description
- bag_recorder (optional)

## Usage Examples

### Development Testing
```bash
# Test individual components
ros2 launch amr_sweeper_description rsp.launch.py
ros2 launch wit_ros2_imu rviz_and_imu.launch.py
ros2 launch gnss_imu_robot_localization ekf_odm.launch.py
```

### Production Deployment
```bash
# Full system with data recording
ros2 launch gnss_imu_robot_localization bringup.launch.py \
    enable_bag_recording:=true \
    bag_config_file:="compressed_config.yaml"
```

### Data Analysis
```bash
# Record data for analysis
ros2 launch gnss_imu_robot_localization bringup.launch.py \
    enable_bag_recording:=true

# Analyze recorded data
ros2 bag info /home/dev/ros2_bags/sensor_data
ros2 bag play /home/dev/ros2_bags/sensor_data
```

## Troubleshooting

### Common Issues

**GNSS not receiving fix**:
- Check antenna connection and placement
- Verify RTCM correction data availability
- Monitor signal strength and satellite count

**IMU calibration issues**:
- Ensure IMU is properly mounted and oriented
- Check serial communication and baud rate
- Verify coordinate frame transformations

**EKF divergence**:
- Check sensor data quality and timing
- Verify transform tree consistency
- Adjust EKF parameters if needed

### Debug Commands
```bash
# Check topics and data flow
ros2 topic list
ros2 topic echo /fix
ros2 topic echo /imu/data

# Monitor transforms
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_link odom

# Check node status
ros2 node list
ros2 node info /robot_localization_ekf_node
```

## Package Structure

```
gnss_imu_robot_localization/
├── launch/
│   ├── bringup.launch.py      # Main system launch
│   └── ekf_odm.launch.py      # EKF-only launch
├── config/
│   └── ekf_params.yaml        # EKF configuration
├── package.xml                # Package manifest
├── setup.py                   # Python setup
└── README.md                  # This file
```

## Contributing

When modifying this package:
1. Test individual components before integration
2. Verify sensor data quality and timing
3. Check transform tree consistency
4. Update configuration documentation
5. Test with real hardware in representative environments

---

**Built for autonomous mobile robot localization**
