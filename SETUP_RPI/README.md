# Configure Main Computer: Raspberry Pi
## Installing Ubuntu Server
1. Flash SD card with Ubuntu Server 22.04 LTS, make sure to enable ssh (if you cannot enable it from the Pi Imager, you will have to enable it when connecting with the external keyboard).
## Network configurations
In order to ssh into your new ubuntu server Pi, you first have to connect using an external monitor and keyboard.
Run:
```bash
cd /etc/netplan
```
```bash
sudo vim 50-cloud-init.yaml
```
Then edit it until it looks like this:
```yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      dhcp4: false
      optional: true
      addresses: [10.0.0.3/24]
```
Save and run ```sudo netplan try```. If it shows no errors, press Enter to accept the configurations. Now you can ssh into your system.
# Connect to WiFi:
Run ```sudo vim /etc/netplan/50-cloud-init.yaml```, at the end of the file, append something like this:
```yaml
  wifis:
    wlan0:
      dhcp4: yes
      access-points:
        "YOUR_SSID":
          auth:
            method: peap
            identity: "YOUR_USERNAME"
            password: "YOUR_PASSWORD"
```
Adapt this to your needs: in case of "polito" WiFi, SSID is "polito", "Username" is your student email and "Password" is your password.
3. run ```sudo vim /etc/wpa_supplicant/wpa_supplicant.conf```, make it look something like:
```
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1
country=US

network={
    ssid="YOUR_SSID"
    key_mgmt=WPA-EAP
    eap=PEAP
    identity="YOUR_USERNAME"
    password="YOUR_PASSWORD"
    phase1="peaplabel=0"
    phase2="auth=MSCHAPV2"
}
```
4. run ```sudo killall wpa_supplicant```, then run
```bash
sudo wpa_supplicant -B -i wlan0 -c /etc/wpa_supplicant/wpa_supplicant.conf && sudo systemctl restart systemd-networkd
```
5. Run ```sudo netplan apply```, then see if you are connected using: ```sudo ip a```.
6. For every system boot, you need to repeat point 4, IDK why, I might try to find a solution in the future.
## Clone the GitHub repository.
1. Run ```git clone https://github.com/Z4nna/nereo_rov_code.git ~/```
2. Execute the setup script: ```cd ~/nereo_rov_code && ./setup_rpi.sh```. This will install all the dependencies and utilities of PoliTOcean Nereo software, making the Raspberry Pi ready to run the ROV. The script is yet to be tested, so if you have any problems, please contact me.
