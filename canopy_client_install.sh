#!/bin/sh
CANOPY_CLIENT_DIR=$(pwd)

# trap errors
function err_fn() {
    echo 'ERROR - aborting script.'
    cd $CANOPY_CLIENT_DIR
    trap - ERR
}
trap "err_fn; return" ERR

# install python packages
pip install -r requirements.txt

# setup chrony
sudo apt-get update
sudo apt-get install chrony
sudo cp chrony_canopy_client.service /lib/systemd/system
sudo cp chrony_canopy_client.conf /etc/chrony

# make ros package
cd ../..
catkin_make
. ./devel/setup.bash
cd $CANOPY_CLIENT_DIR
