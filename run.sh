#!/bin/bash

if [[ -z "$1" || -z "$2" ]]
then
    echo "Usage: ./run.sh <dragonfly apm namespace> <SYSID_THISMAV for this instance>"
    exit 1
fi

source /opt/ros/kinetic/setup.bash
source /etc/ubiquity/env.sh
source /home/ubuntu/dev/dragonfly/devel/setup.bash

#sudo route add -net 224.0.0.0 netmask 240.0.0.0 wlan0
rosrun master_discovery_fkie master_discovery _mcast_group:=224.0.0.0 &
P1=$!
rosrun master_sync_fkie master_sync &
P2=$!
/opt/ros/kinetic/bin/roslaunch /home/ubuntu/dev/dragonfly/config/apm.launch name:=$1 tgt_system:=$2 &
P3=$!
ros2 run dragonfly co2publisher $1 &
P4=$!
ros2 run dragonfly logger $1 >> /home/ubuntu/dev/dragonfly/logs/run.log &
P5=$!
ros2 run dragonfly command $1 >> /home/ubuntu/dev/dragonfly/logs/command.log &
P6=$!
ros2 run dragonfly announce $1 &
P7=$!
wait $P1 $P2 $P3 $P4 $P5 $P6 $P7
