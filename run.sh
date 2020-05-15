#!/bin/bash

if [[ -z "$1" ]]
then
    echo "Usage: ./run.sh <dragonfly apm namespace>"
    exit 1
fi

source ./devel/setup.bash

exec 3>&1 1>>logs/script.log 2>&1

sudo route add -net 224.0.0.0 netmask 240.0.0.0 wlan0
rosrun master_discovery_fkie master_discovery _mcast_group:=224.0.0.0 &
P1=$!
sleep 3
rosrun master_sync_fkie master_sync &
P2$!
roslaunch config/apm.launch &
P3=$!
rosrun dragonfly co2publisher.py $1 &
P4=$!
rosrun dragonfly logger.py $1 >> logs/run.log &
P5=$!
rosrun dragonfly command.py $1 1 0 >> logs/command.log &
P6=$!
wait $P1 $P2 $P3 $P4 $P5 $P6
