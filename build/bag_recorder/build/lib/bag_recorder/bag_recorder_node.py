#!/usr/bin/env python3

"""
ROS2 executable entry point for bag_recorder node.
This file serves as the main executable for ROS2 launch compatibility.
"""

from bag_recorder.bag_recorder import main

if __name__ == '__main__':
    main()
