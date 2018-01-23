# Canopy Client
This is the ROS node for the client-side application of Canopy.

## Installation
See the [Canopy docs](http://canopy-docs.readthedocs.io/en/latest/client-docs.html#installation). 

## Time Synchronization
This section describes the steps for synchronizing timestamps between your Canopy clients and the Canopy server.

Install chrony:
```
sudo apt-get install chrony
```

Modify the chrony configuration file `chrony_canopy_client.conf` to reflect your Canopy server's IP address:
```
server 123.45.67.890
```

Restart chrony with this configuration file:
```
sudo chronyd -f <path_to_canopy_client>/chrony_canopy_client.conf
```