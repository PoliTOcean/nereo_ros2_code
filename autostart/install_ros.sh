#!/usr/bin/env bash
set -e

log() {
    echo "[ROS_SETUP] $1"
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

log "Detected Ubuntu $UBUNTU_VERSION → ROS $ROS_DISTRO"

if [ ! -f "$ROS_SETUP" ]; then
    log "ROS2 $ROS_DISTRO not found. Installing base ROS setup..."

    sudo -v

    sudo apt update
    sudo apt install -y software-properties-common curl gnupg lsb-release

    sudo mkdir -p /usr/share/keyrings

    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg

    UBUNTU_CODENAME=$(. /etc/os-release && echo "$UBUNTU_CODENAME")

    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $UBUNTU_CODENAME main" | \
        sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update
    sudo apt install -y "ros-$ROS_DISTRO-desktop"

    log "ROS2 $ROS_DISTRO installed."
else
    log "ROS2 $ROS_DISTRO already installed."
fi

log "Ensuring required ROS packages are installed..."
sudo apt update
sudo apt install -y \
    "ros-$ROS_DISTRO-rosbridge-server" \
    "ros-$ROS_DISTRO-joy"

if ! command -v colcon >/dev/null 2>&1; then
    log "colcon not found. Installing..."
    sudo apt install -y python3-colcon-common-extensions
else
    log "colcon already installed."
fi

log "ROS setup completed."