#!/bin/bash

# Script to start sensor data bag recording using YAML configuration
# Usage: ./start_recording_yaml.sh [config_file]

# Default config file
CONFIG_FILE=${1:-"bag_config.yaml"}

echo "Starting sensor data bag recording with YAML config..."
echo "Config file: $CONFIG_FILE"
echo ""
echo "Available config files:"
echo "  - bag_config.yaml (default settings)"
echo "  - compressed_config.yaml (with compression and limits)"
echo "  - extended_sensors_config.yaml (for future sensor expansion)"
echo ""
echo "Press Ctrl+C to stop recording"
echo ""

# Launch the bag recorder with YAML config
ros2 launch bag_recorder bag_record_yaml.launch.py \
    config_file:="$CONFIG_FILE"
