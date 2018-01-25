# Canopy Client
This is the ROS node for the client-side application of Canopy.

## Installation
```
cd <YOUR CATKIN WORKSPACE>/src
git clone https://github.com/baalexander/rospy_message_converter
git clone https://github.com/canopy-ros/canopy_client
cd canopy_client
. ./canopy_client_install.sh
```

## Clock Synchronization
This section describes the _client-side_ instructions for synchronizing the clocks of your Canopy clients and the Canopy server using `chrony`. 

### Setting up chrony
Modify the chrony configuration file `/etc/chrony/chrony_canopy_client.conf` to reflect your Canopy server's IP address:
```
# /etc/chrony/chrony_canopy_client.conf
server 128.31.37.168
...
```
Disable and stop the default chrony service:
```
sudo systemctl stop chrony
sudo systemctl disable chrony
```

Enable and start the Canopy chrony service:
```
sudo systemctl enable chrony_canopy_client
sudo systemctl start chrony_canopy_client
```

### Checking chrony setup
To check whether the service is running correctly, run `systemctl status chrony_canopy_client` which should show an active running status:
```
chrony_canopy_client.service - chrony service for canopy client
Loaded: loaded (/lib/systemd/system/chrony_canopy_client.service; static; vendor preset: enabled)
Active: active (running)
...
```

To check whether the chrony daemon `chronyd` is working correctly, first setup chrony on the [Canopy server](https://github.com/canopy-ros/canopy_server_startup/) and ensure that it is running correctly. Then run `chronyc activity` on the client which should return the following:
```
200 OK
1 sources online
0 sources offline
0 sources doing burst (return to online)
0 sources doing burst (return to offline)
0 sources with unknown address
```
If you see the message `506 Cannot talk to daemon`, then `chronyd` is not running properly.

Finally run `chronyc sources` and you should see your Canopy server's IP address listed. If you are connecting the chrony client to the chrony server for the first time, it may take several minutes before the correct IP is listed under sources. 
