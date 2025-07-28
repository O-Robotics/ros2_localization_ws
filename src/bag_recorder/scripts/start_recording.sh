#!/bin/bash

# Script to start sensor data bag recording
# Usage: ./start_recording.sh [output_dir] [bag_name] [max_duration] [max_size_mb] [compression_mode] [storage_format]

# Default values
OUTPUT_DIR=${1:-"~/ros2_bags"}
BAG_NAME=${2:-""}
MAX_DURATION=${3:-"0"}
MAX_SIZE=${4:-"0"}
COMPRESSION_MODE=${5:-"none"}
STORAGE_FORMAT=${6:-"sqlite3"}

echo "Starting sensor data bag recording..."
echo "Output directory: $OUTPUT_DIR"
echo "Bag name: $BAG_NAME"
echo "Max duration: $MAX_DURATION seconds (0 = unlimited)"
echo "Max size: $MAX_SIZE MB (0 = unlimited)"
echo "Compression: $COMPRESSION_MODE"
echo "Storage format: $STORAGE_FORMAT"
echo ""
echo "Recording topics: /imu/raw_data, /fix (extensible for camera, wheel odometry, etc.)"
echo "Press Ctrl+C to stop recording"
echo ""

# Launch the bag recorder
ros2 launch bag_recorder bag_record.launch.py \
    output_dir:="$OUTPUT_DIR" \
    bag_name:="$BAG_NAME" \
    max_duration:="$MAX_DURATION" \
    max_size:="$MAX_SIZE" \
    compression_mode:="$COMPRESSION_MODE" \
    storage_format:="$STORAGE_FORMAT"
