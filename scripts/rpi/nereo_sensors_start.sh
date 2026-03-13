#!/bin/bash
# Avvia IMU e barometro in parallelo
source /home/pi/nereo_ros2_code/rpi_ws/install/setup.bash
ros2 run nereo_sensors_pkg imu_pub &
ros2 run nereo_sensors_pkg barometer_pub
wait
