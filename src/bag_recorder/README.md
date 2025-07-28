# Bag Recorder

A production-ready, scalable ROS2 package for recording sensor data to bag files. Designed for autonomous mobile robots with GNSS and IMU sensors, with extensible architecture for future sensor integration.

## Features

- **Current Sensors**: Records `/imu/raw_data` and `/fix` topics by default
- **Extensible Architecture**: Easy to add camera, wheel odometry, lidar, etc.
- **Configurable Parameters**: Output directory, bag naming, duration/size limits
- **Compression Support**: File-level compression with zstd
- **Multiple Usage Methods**: YAML configs, launch files, convenience scripts
- **Robust Design**: Graceful shutdown, error handling, atexit cleanup
- **ROS2 Integration**: Easy integration with existing launch systems
- **Production Tested**: All parameters validated and working

## Quick Start

### Basic Recording (Default Settings)
```bash
source install/setup.bash
ros2 launch bag_recorder bag_record_yaml.launch.py
```

### Compressed Recording (Recommended)
```bash
ros2 launch bag_recorder bag_record_yaml.launch.py config_file:="compressed_config.yaml"
```

### Using Convenience Script
```bash
./src/bag_recorder/scripts/start_recording_yaml.sh compressed_config.yaml
```

## Installation

1. Clone this package to your ROS2 workspace:
```bash
cd /path/to/your/ros2_ws/src
# Package should already be in your workspace
```

2. Build the package:
```bash
cd /path/to/your/ros2_ws
colcon build --packages-select gnss_imu_bag_recorder
source install/setup.bash
```

## Tested Parameters

All parameters have been tested and validated:

 **Default Parameters**: Basic recording with automatic timestamped naming  
 **Custom Output Directory**: `/tmp/test_bags`  
 **Custom Bag Name**: `custom_test`  
 **Duration Limit**: `max_duration:="5"` (5 seconds)  
 **Size Limit**: `max_size:="100"` (100 MB)  
 **Compression**: `compression_mode:="file"` with `compression_format:="zstd"`  
 **Convenience Script**: All parameters working via shell script  

## Usage

### Method 1: Using Launch File

```bash
# Basic usage with default settings
ros2 launch bag_recorder bag_record.launch.py

# With custom parameters (all tested)
ros2 launch bag_recorder bag_record.launch.py \
    output_dir:="~/my_bags" \
    bag_name:="test_recording" \
    max_duration:="300" \
    max_size:="1000" \
    compression_mode:="file"
```

### Method 2: Using Convenience Script

```bash
cd /path/to/gnss_imu_bag_recorder/scripts
./start_recording.sh [output_dir] [bag_name] [max_duration] [max_size_mb] [compression_mode]

# Examples:
./start_recording.sh                                    # Use all defaults
./start_recording.sh ~/my_bags                          # Custom output directory
./start_recording.sh ~/my_bags test_run 300 1000 file  # All parameters
```

### Method 3: Direct Node Execution

```bash
ros2 run gnss_imu_bag_recorder bag_recorder \
    --ros-args \
    -p output_dir:="~/ros2_bags" \
    -p bag_name:="my_recording" \
    -p max_bag_duration:=300 \
    -p max_bag_size:=1000
```

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `output_dir` | string | `~/ros2_bags` | Directory to save bag files |
| `bag_name` | string | `""` | Name of bag file (timestamp added if empty) |
| `topics` | string[] | `["/imu/raw_data", "/fix"]` | Topics to record |
| `max_bag_duration` | int | `0` | Max recording duration in seconds (0 = unlimited) |
| `max_bag_size` | int | `0` | Max bag size in MB (0 = unlimited) |
| `compression_mode` | string | `none` | Compression mode: none, file, message |
| `compression_format` | string | `zstd` | Compression format: zstd, lz4 |

## Integration with Other Packages

### Using with gnss_imu_robot_localization

To integrate this bag recorder with the `gnss_imu_robot_localization` package's bringup, you can include it in your launch files:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Your existing launch configuration...
    
    # Include bag recording
    bag_recorder = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('gnss_imu_bag_recorder'),
            '/launch/bag_record.launch.py'
        ]),
        launch_arguments={
            'output_dir': '~/localization_bags',
            'bag_name': 'localization_data',
            'compression_mode': 'file'
        }.items()
    )
    
    return LaunchDescription([
        # Your existing nodes...
        bag_recorder,
    ])
```

## Topics Recorded

By default, this package records:
- `/imu/raw_data` - Raw IMU sensor data
- `/fix` - GNSS fix data

## Output

Bag files are saved with the following naming convention:
- If `bag_name` is specified: `{output_dir}/{bag_name}`
- If `bag_name` is empty: `{output_dir}/gnss_imu_data_{timestamp}`

Where `{timestamp}` is in format `YYYYMMDD_HHMMSS`.

## Stopping Recording

Press `Ctrl+C` to gracefully stop the recording. The node will properly terminate the bag recording process and clean up resources.

## Dependencies

- rclpy
- std_msgs
- sensor_msgs
- geometry_msgs

## License

MIT

## Configuration Files

The package includes three YAML configuration files:

### `bag_config.yaml` (Default)
- Basic recording settings
- No compression
- No size/time limits
- Output: `/home/dev/ros2_bags/sensor_data`

### `compressed_config.yaml` (Recommended)
- File-level zstd compression
- 5-minute duration limit
- 1GB size limit
- Output: `/home/dev/ros2_bags/compressed_recording`

### `extended_sensors_config.yaml` (Template)
- Template for future sensor expansion
- Includes placeholder topics for camera, lidar, etc.
- Use as starting point for custom configurations

##  Usage Methods

### Method 1: YAML Configuration (Recommended)

```bash
# Basic recording with default settings
ros2 launch bag_recorder bag_record_yaml.launch.py

# Compressed recording (recommended for production)
ros2 launch bag_recorder bag_record_yaml.launch.py config_file:="compressed_config.yaml"

# Extended sensors template
ros2 launch bag_recorder bag_record_yaml.launch.py config_file:="extended_sensors_config.yaml"
```

### Method 2: Direct Launch Parameters

```bash
ros2 launch bag_recorder bag_record.launch.py \
    output_dir:="/tmp/my_bags" \
    bag_name:="test_recording" \
    max_duration:="600" \
    compression_mode:="file"
```

### Method 3: Convenience Scripts

```bash
# YAML-based recording
./src/bag_recorder/scripts/start_recording_yaml.sh compressed_config.yaml

# Parameter-based recording
./src/bag_recorder/scripts/start_recording.sh /tmp/bags test_bag 300
```

##  Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `output_dir` | string | `/home/dev/ros2_bags` | Directory for bag files |
| `bag_name` | string | `sensor_data` | Base name for bag files |
| `topics` | string[] | `['/imu/raw_data', '/fix']` | Topics to record |
| `max_duration` | int | `0` | Max recording time (seconds, 0=unlimited) |
| `max_size` | int | `0` | Max bag size (bytes, 0=unlimited) |
| `compression_mode` | string | `none` | Compression mode (`none`, `file`, `message`) |
| `compression_format` | string | `zstd` | Compression format (`zstd`) |
| `storage_format` | string | `sqlite3` | Storage format (`sqlite3`) |

##  Integration Examples

### With `gnss_imu_robot_localization`

```bash
# Start localization with bag recording
ros2 launch gnss_imu_robot_localization bringup.launch.py \
    enable_bag_recording:=true \
    bag_config_file:="compressed_config.yaml"
```

### Custom Launch File Integration

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    bag_recorder = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('bag_recorder'),
                'launch',
                'bag_record_yaml.launch.py'
            ])
        ),
        launch_arguments={'config_file': 'compressed_config.yaml'}.items()
    )
    
    return LaunchDescription([bag_recorder])
```

##  Adding New Sensors

To extend for new sensors (camera, lidar, wheel odometry):

1. **Edit YAML Config**:
   ```yaml
   topics:
     - '/imu/raw_data'    # IMU data
     - '/fix'             # GNSS data
     - '/camera/image'    # Camera data
     - '/scan'            # Lidar data
     - '/odom'            # Wheel odometry
   ```

2. **Extension Points**: Code includes marked extension points for easy expansion

3. **Test**: Verify recording with `ros2 bag info /path/to/bag`

##  Troubleshooting

### Common Issues

**Storage format error**:
```bash
# Use sqlite3 instead of mcap if you see storage format errors
# Edit config files to use storage_format: "sqlite3"
```

**Permission denied**:
```bash
# Create output directory or change output_dir parameter
mkdir -p /home/dev/ros2_bags
```

**No topics found**:
```bash
# Check if sensor nodes are running
ros2 topic list
ros2 topic echo /imu/raw_data
```

### Debug Mode
```bash
ros2 launch bag_recorder bag_record_yaml.launch.py --ros-args --log-level DEBUG
```

##  Performance Tips

-  **Use compression** for long recordings
-  **Set size limits** to prevent disk overflow  
-  **Set time limits** for automated stops
-  **Use fast storage** (SSD) for high-frequency sensors
-  **Monitor disk space** during recording

##  Examples

### Quick Test (30 seconds)
```bash
ros2 launch bag_recorder bag_record.launch.py bag_name:="test_30s" max_duration:="30"
```

### Production Recording
```bash
ros2 launch bag_recorder bag_record_yaml.launch.py config_file:="compressed_config.yaml"
```

### Custom Location
```bash
ros2 launch bag_recorder bag_record.launch.py \
    output_dir:="/media/usb/robot_data" \
    bag_name:="mission_$(date +%Y%m%d_%H%M%S)"
```

## Package Structure

```
bag_recorder/
├── bag_recorder/           # Python package
├── config/                 # YAML configurations
├── launch/                 # Launch files
├── scripts/                # Convenience scripts
├── package.xml             # ROS2 package manifest
├── setup.py                # Python setup
└── README.md               # This file
```

