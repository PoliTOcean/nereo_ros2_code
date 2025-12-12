#!/bin/bash
# Run on RPI: starts tmux sessions for micro_ros_connect, imu+barometer, and camera

# Configuration
SESSION_NAME="nereo_services"
ROS_SETUP_PATH="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP="$HOME/nereo_ros2_code/rpi_ws/install/setup.bash"
MICRO_ROS_SCRIPT="$HOME/micro_ros_connect.sh"
CAM_SCRIPT="$HOME/start_cam.sh"

echo "Starting Nereo RPI services..."

# Check if tmux session already exists
if tmux has-session -t ${SESSION_NAME} 2>/dev/null; then
    echo "Session ${SESSION_NAME} already exists. Attaching..."
    tmux attach-session -t ${SESSION_NAME}
    exit 0
fi

# Create tmux session with first window
tmux -f /dev/null new-session -d -s ${SESSION_NAME} -n "micro_ros"

# Window 1: Micro ROS Connect
tmux send-keys -t ${SESSION_NAME}:micro_ros "$MICRO_ROS_SCRIPT" Enter
# Window 2: IMU and Barometer
tmux new-window -t ${SESSION_NAME} -n "sensors"
tmux send-keys -t ${SESSION_NAME}:sensors "source $WORKSPACE_SETUP && ros2 run nereo_sensors_pkg imu_pub & ros2 run nereo_sensors_pkg barometer_pub" Enter

# Window 3: Camera (GStreamer)
tmux new-window -t ${SESSION_NAME} -n "camera"
tmux send-keys -t ${SESSION_NAME}:camera "$CAM_SCRIPT" Enter

echo "Tmux session '${SESSION_NAME}' created with 3 windows: micro_ros, sensors, camera"
echo "Attaching to session..."

# Attach to session
tmux attach-session -t ${SESSION_NAME}