#!/usr/bin/env bash
set -e

log() {
    echo "[GUI_START] $1"
}

UBUNTU_VERSION=$(lsb_release -rs)

if [ "$UBUNTU_VERSION" = "24.04" ]; then
    ROS_DISTRO="jazzy"
elif [ "$UBUNTU_VERSION" = "22.04" ]; then
    ROS_DISTRO="humble"
else
    log "Unsupported Ubuntu version: $UBUNTU_VERSION"
    exit 1
fi

ROS_SETUP="/opt/ros/$ROS_DISTRO/setup.bash"
WORKSPACE_SETUP="$HOME/nereo_ros2_code/gui_ws/install/setup.bash"

if [ ! -f "$ROS_SETUP" ]; then
    log "ERROR: ROS2 $ROS_DISTRO not installed."
    exit 1
fi

source "$ROS_SETUP"

if [ ! -f "$WORKSPACE_SETUP" ]; then
    log "Workspace not built. Building gui_ws..."

    cd "$HOME/nereo_ros2_code/gui_ws"
    colcon build --symlink-install
fi

source "$WORKSPACE_SETUP"

log "Launching GUI..."
ros2 launch gui_pkg workstation.launch.py