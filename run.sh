#!/bin/bash

if [[ -z "$1" || -z "$2" ]]
then
    echo "Usage: ./run.sh <dragonfly apm namespace> <SYSID_THISMAV for this instance>"
    exit 1
fi

source /opt/ros/galactic/setup.bash
source /home/ubuntu/dev/dragonfly/install/setup.bash

P2=$!
ros2 launch /home/ubuntu/dev/dragonfly/config/apm.launch name:=$1 tgt_system:=$2 fcu_url:=/dev/ttypixhawk:921600 &
P3=$!
ros2 run dragonfly co2publisher $1 &
P4=$!
ros2 run dragonfly logger $1 >> /home/ubuntu/dev/dragonfly/logs/run.log &
P5=$!
ros2 run dragonfly command $1 >> /home/ubuntu/dev/dragonfly/logs/command.log &
P6=$!
ros2 run dragonfly announce $1 &
P7=$!
wait $P2 $P3 $P4 $P5 $P6 $P7
