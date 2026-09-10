# Configure Main Computer: Raspberry Pi 5
1. Flash the image on the SD card: using Raspberry PI Imager, flash the Ubuntu Server 24.04 LTS (noble, 64bit) image on the SD card, at least 32GB. User: `pi`, Password: `raspberry`. The Pi runs ROS 2 Jazzy, not Humble as the workstation does.
2. After the image is flashed, boot the raspberry PI and connect to a shell, either through monitor and keyboard, or connecting it to a router with DHCP, or by configuring the access to a WiFi during the 1st step.
3. Network configuration: you should edit the `/etc/netplan/50-cloud-init.yaml` and add this part:
```yaml
	ethernets:
		renderer: networkd
		optional: false
		dhcp4: false
		addresses: [10.0.0.3/24]	
```
Then run `sudo netplan apply`. You should then be able to shh into your PI from the ethernet connection, using `ssh pi@10.0.0.3`. (do not forget to configure the network ip on your laptop too: you MUST set it to 10.0.0.69/24).
4. Connect to the internet. Connect to your wifi of choice, LAN or whatever you prefer.
5. Run ```git clone https://github.com/PoliTOcean/nereo_ros2_code.git ~/nereo_ros2_code```
6. Execute the setup script: ```cd ~/nereo_ros2_code/SETUP_RPI && ./setup_rpi.sh```. This will install all the dependencies and utilities of PoliTOcean Nereo software, making the Raspberry Pi ready to run the ROV. Please make sure to follow the instructions displayed on the screen. If any, they should be colored to distinguish them from normal log text.
7. Install the camera stack. Cameras are served by DWE OS 2, a daemon that starts at boot — there is nothing to launch by hand any more. On a Pi with bench internet access run ```cd ~/nereo_ros2_code/SETUP_RPI && ./install_dweos.sh``` once. This takes the pre-install snapshot, installs the pinned DWE OS release, patches two vendor defects and disables the bundled web terminal. Re-run ```./install_dweos.sh configure``` after any camera is moved between USB ports. Note that DWE OS saves each stream as enabled but does not restart the pipelines when its service comes back, so the install also writes a drop-in that re-runs the configure step after every start of the service; ```./install_dweos.sh autostart``` installs just that drop-in on a Pi where DWE OS is already set up. The three streams go to `10.0.0.69` (the laptop from step 3) on UDP ports 5001, 5002 and 5003 — exactly the ports the workstation GUI already listens on, so nothing has to be changed on that side.
8. The DWE OS web interface is on **port 80**: browse to `http://10.0.0.3/` to adjust resolution, framerate or bitrate per camera. Note the port: any documentation naming 5000 for the DWE OS interface describes DWE OS 1, not the version installed here. Port 80 is taken on this Pi — a later diagnostics service must pick a different one.
9. Camera identity is procedural, not automatic. DWE OS persists each camera's stream configuration against `bus_info`, a USB bus-topology string, and exposes no serial-number field anywhere in its device model. So three USB ports on the Pi are dedicated and physically labelled `main_cam`, `cam_1` and `cam_2`, and each exploreHD stays in its own port. Moving a camera to a different port strands its configuration on the old topology string: update the `*_BUS_INFO` settings at the top of `install_dweos.sh` and re-run `./install_dweos.sh configure`. To read the values back, run ```./install_dweos.sh identify```, which asks you to unplug one camera per role and reports the `bus_info` that disappeared; `v4l2-ctl -d /dev/videoN --info` gives the same string directly. A udev serial-symlink layer was considered and rejected — DWE OS would persist against `bus_info` regardless, so the layer would be one more component to maintain without closing the gap.

| Physical port label | Camera | bus_info | UDP port |
|---|---|---|---|
| main_cam | front | usb-xhci-hcd.1-1 | 5001 |
| cam_1 | right | usb-xhci-hcd.0-2 | 5002 |
| cam_2 | bottom | usb-xhci-hcd.1-2 | 5003 |

10. DWE OS bundles a full browser terminal, unauthenticated and running as root, enabled by default on that same port 80 surface. `install_dweos.sh` turns it off through a systemd drop-in at `/etc/systemd/system/dwe_os_2.service.d/override.conf`. It is off by default because the competition tether is a shared venue, the team already works over SSH, and an unauthenticated root shell there is a far larger surface than camera configuration. If you genuinely need it for a bench session, drop `--no-ttyd` from the `ExecStart=` line in that drop-in, then ```sudo systemctl daemon-reload && sudo systemctl restart dwe_os_2``` — and put it back afterwards. Turn it on knowingly; do not delete the drop-in.
11. Rollback. The previous hand-rolled pipelines are still in version control, unmodified. Stop DWE OS with ```sudo systemctl stop dwe_os_2```, then run ```./stream_cam.sh /dev/videoN <port>``` once per camera. Find each camera's `/dev/videoN` first: every exploreHD exposes four nodes and only one of them advertises H264, so run ```for d in /dev/video*; do v4l2-ctl -d $d --list-formats 2>/dev/null | grep -q H264 && echo $d; done``` and pick from those. The numbers change whenever a camera is replugged, which is why DWE OS binds to the USB bus instead in the tmux session `setup_rpi.sh` sets up, as the team did before this migration. These streams are started by hand over SSH and have no systemd unit to recover them — that is the current operating model, not an oversight. Restore DWE OS with ```sudo systemctl start dwe_os_2```.

Expected behaviour, not a fault. After a GUI restart, and after the vehicle is powered on, a video pane can stay blank for up to about ten seconds before the first picture arrives. DWE OS repeats the H.264 parameter sets every ten seconds where the previous pipeline repeated them every second, and a decoder cannot show anything until it has seen them.

Verified on the vehicle: with the battery disconnected and reconnected, DWE OS comes back on its own, reconfigures all three streams through the drop-in described in step 7, and the three panes fill without anyone running anything. Cold boot to login is about twenty seconds, of which DWE OS and the stream reconfiguration are six.
