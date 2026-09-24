#!/usr/bin/env bash

sudo -S chmod +777 /dev/ttyAMA0
ros2 run micro_ros_agent micro_ros_agent serial -b 115200 --dev /dev/ttyAMA0