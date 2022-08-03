#!/bin/bash

if [[ -z "$1" || -z "$2" ]]
then
    echo "Usage: ./run.sh <dragonfly apm namespace> <SYSID_THISMAV for this instance>"
    exit 1
fi

source /opt/ros/galactic/setup.bash
cd /home/ubuntu/dev/dragonfly/
source ./install/setup.bash

./run.py  --name $1 --sysid_thismav $2 --cyclone_network wlan1 &

