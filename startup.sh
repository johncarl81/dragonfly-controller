#!/bin/bash

if [[ -z "$1" || -z "$2" || -z "$3" || -z "$4" ]]
then
    echo "Usage: ./startup.sh <dragonfly apm namespace> <unique altitude> <index in swarm> <swarm size>"
    exit 1
fi

source /opt/ros/galactic/setup.bash
source /home/ubuntu/dev/dragonfly/install/setup.bash

exec 3>&1 1>>logs/script.log 2>&1
cd /home/ubuntu/dev/dragonfly

ros2 launch config/apm.launch & 
ros2 run dragonfly co2publisher.py $1 &
ros2 run dragonfly logger.py $1 >> logs/output.log &
ros2 run dragonfly command.py $1 $2 $3 $4 &
