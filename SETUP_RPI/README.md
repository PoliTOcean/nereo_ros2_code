# Configure Main Computer: Raspberry Pi 4
1. Flash the image on the SD card: using Raspberry PI Imager, flash the Ubuntu Server 22.04 LTS (64bit) image on the SD card, at least 32GB. User: `politocean`, Password: `politoceanPI`.
2. After the image is flashed, boot the raspberry PI and connect to a shell, either through monitor and keyboard, or connecting it to a router with DHCP, or by configuring the access to a WiFi during the 1st step.
3. Network configuration: you should edit the `/etc/netplan/50-cloud-init.yaml` and add this part:
```yaml
	ethernets:
		renderer: networkd
		optional: false
		dhcp4: false
		addresses: [10.0.0.3/24]	
```
Then run `sudo netplan apply`. You should then be able to shh into your PI from the ethernet connection, using `ssh politocean@10.0.0.3`. (do not forget to configure the network ip on your laptop too).
4. Connect to the internet. Connect to your wifi of choice, LAN or whatever you prefer.
5. Run ```git clone https://github.com/PoliTOcean/nereo_ros2_code.git ~/nereo_ros2_code```
6. Execute the setup script: ```cd ~/nereo_ros2_code/SETUP_RPI && ./setup_rpi.sh```. This will install all the dependencies and utilities of PoliTOcean Nereo software, making the Raspberry Pi ready to run the ROV. Please make sure to follow the instructions displayed on the screen. If any, they should be colored to distinguish them from normal log text.
7. To start the video camera, launch ```gst-launch-1.0 v4l2src ! videoconvert ! x264enc tune=zerolatency ! rtph264pay ! udpsink host=10.0.0.69 port=5000```