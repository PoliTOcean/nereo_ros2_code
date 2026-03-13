#!/bin/bash
# Wrapper comune per tutti i service Nereo
# Sourcia l'ambiente ROS e lancia il comando passato come argomento
source /opt/ros/jazzy/setup.bash
source /home/pi/micro_ros_agent_ws/install/local_setup.bash
source /home/pi/nereo_interfaces/install/setup.bash
exec "$@"
