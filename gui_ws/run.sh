source install/setup.bash

echo "Sourced setup.bash"

python3 SocketStream/server.py && ros2 run gui_pkg gui_node &

echo "Started server and gui node"
