#!/bin/bash
# Run on RPI: starts tmux sessions for micro_ros_connect, imu+barometer, and camera

# Configuration
SESSION_NAME="nereo_services"
ROS_SETUP_PATH="/opt/ros/humble/setup.bash"
MICRO_ROS_SCRIPT="~/micro_ros_connect.sh"
CAM_SCRIPT="~/start_cam.sh"

# Source ROS setup
source ${ROS_SETUP_PATH}
# Source rpi_ws workspace setup
source ~/rpi_ws/install/setup.bash

# Create tmux session and windows
tmux -f /dev/null new-session -d -s ${SESSION_NAME}

# micro_ros_connect.sh
# sudo chmod +777 /dev/ttyAMA0
# ros2 run micro_ros_agent micro_ros_agent serial -b 115200 --dev /dev/ttyAMA0

# start_cam.sh
# gst-launch-1.0 v4l2src ! videoconvert ! x264enc tune=zerolatency ! rtph264pay ! udpsink host=10.0.0.69 port=5000

# Window 1: Micro ROS Connect
tmux new-window -t ${SESSION_NAME} -n "micro_ros"
tmux send-keys -t ${SESSION_NAME}:micro_ros "$MICRO_ROS_SCRIPT" Enter

# Window 2: IMU and Barometer
tmux new-window -t ${SESSION_NAME} -n "sensors"
tmux send-keys -t ${SESSION_NAME}:sensors "ros2 launch nereo_sensors_pkg sensors.launch.py" Enter

# Window 3: Camera (GStreamer)
tmux new-window -t ${SESSION_NAME} -n "camera"
tmux send-keys -t ${SESSION_NAME}:camera "$CAM_SCRIPT" Enter

# Attach to session
tmux attach-session -t ${SESSION_NAME}
