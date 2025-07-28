# WIT ROS2 IMU Driver

ROS2 driver package for WIT motion sensor IMU devices. Provides 9-DOF sensor data (accelerometer, gyroscope, magnetometer) for robot localization.

## Quick Start

```bash
# Source workspace
source install/setup.bash

# Launch IMU driver
ros2 launch wit_ros2_imu rviz_and_imu.launch.py

# Check IMU data
ros2 topic echo /imu/data_raw
ros2 topic hz /imu/data_raw
```

## Hardware Setup

### Requirements
- WIT motion sensor IMU (9-DOF)
- USB serial connection
- Device mounted as `/dev/imu_usb`

### Connection
1. Connect IMU via USB
2. Verify device: `ls -l /dev/imu_usb`
3. Check permissions: `sudo chmod 666 /dev/imu_usb`

## Configuration

### Serial Settings
- **Port**: `/dev/imu_usb`
- **Baud Rate**: 9600
- **Timeout**: 0.5 seconds

### Publishing
- **Topic**: `/imu/data_raw`
- **Message Type**: `sensor_msgs/msg/Imu`
- **Frame ID**: `imu_link`
- **Frequency**: 10Hz (configurable)

## Topics

### Published
- `/imu/data_raw` - Raw IMU sensor data
  - Linear acceleration (m/s²)
  - Angular velocity (rad/s)
  - Orientation (quaternion)

## Launch Files

### `rviz_and_imu.launch.py`
```bash
ros2 launch wit_ros2_imu rviz_and_imu.launch.py
```

Launches:
- IMU driver node
- Optional RViz visualization

### Parameters
```python
parameters=[
    {'port': '/dev/imu_usb'},
    {'baud': 9600}
]
```

## Frequency Configuration

### Current Setting: 10Hz
The driver is configured to publish at 10Hz for optimal performance.

### To Change Frequency
Edit the installed file:
`/home/dev/ORobotics/localization_ws/install/wit_ros2_imu/lib/python3.10/site-packages/wit_ros2_imu/wit_ros2_imu.py`

Modify the sleep value:
```python
# 10Hz (current)
time.sleep(0.1)

# Other frequencies:
time.sleep(0.05)  # 20Hz
time.sleep(0.02)  # 50Hz
time.sleep(0.2)   # 5Hz
```

**No rebuild required** - changes are immediate.

## Integration

### With Localization System
The IMU integrates with the main localization bringup:
```bash
ros2 launch gnss_imu_robot_localization bringup.launch.py
```

### Data Flow
```
IMU Hardware → /imu/data_raw → EKF Node → /odometry/filtered/local
```

## Troubleshooting

### Common Issues

**No data publishing**:
```bash
# Check device
ls -l /dev/imu_usb

# Check permissions
sudo chmod 666 /dev/imu_usb

# Check node status
ros2 node list | grep imu
```

**Serial connection failed**:
```bash
# Check if device exists
lsusb

# Check serial devices
ls /dev/ttyUSB*

# Test serial communication
screen /dev/imu_usb 9600
```

**Wrong frequency**:
- Check the sleep value in the installed Python file
- Verify with: `ros2 topic hz /imu/data_raw`

### Debug Commands
```bash
# Check topics
ros2 topic list | grep imu

# Monitor data
ros2 topic echo /imu/data_raw --once

# Check frequency
ros2 topic hz /imu/data_raw

# Node info
ros2 node info /imu
```

## Data Format

### IMU Message Structure
```yaml
header:
  frame_id: "imu_link"
  stamp: <timestamp>
linear_acceleration:
  x: <accel_x>  # m/s²
  y: <accel_y>
  z: <accel_z>
angular_velocity:
  x: <gyro_x>   # rad/s
  y: <gyro_y>
  z: <gyro_z>
orientation:
  x: <quat_x>   # quaternion
  y: <quat_y>
  z: <quat_z>
  w: <quat_w>
```

## Development

### File Structure
```
wit_ros2_imu/
├── wit_ros2_imu/
│   └── wit_ros2_imu.py     # Main driver
├── launch/
│   └── rviz_and_imu.launch.py
├── package.xml
├── setup.py
└── README.md
```

### Key Functions
- `IMUDriverNode.__init__()` - Initialize publisher and serial
- `driver_loop()` - Serial data reading loop
- `handle_serial_data()` - Parse IMU packets
- `imu_data()` - Publish IMU messages

---

**Status**: Production Ready
**Frequency**: 10Hz
**Hardware**: WIT Motion Sensor IMU
