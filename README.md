# Canopy Client
This is the ROS node for the client-side application of Canopy.

## Installation
See the [Canopy docs](http://canopy-docs.readthedocs.io/en/latest/client-docs.html#installation). 

## Clock Synchronization
This section describes the _client-side_ instructions for synchronizing the clocks of your Canopy clients and the Canopy server using `chrony`. 

Install chrony:
```
sudo apt-get install chrony
```

Modify the chrony configuration file `chrony_canopy_client.conf` to reflect your Canopy server's IP address:
```
# chrony_canopy_client.conf
server 128.31.37.168
...
```

Restart chrony with this configuration file:
```
sudo chronyd -f <absolute_path_to_config_file>/chrony_canopy_client.conf -r -s
```
