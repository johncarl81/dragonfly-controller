#!/bin/bash

if [[ -z "$1" || -z "$2" || -z "$3" || -z "$4" ]]
then
    echo "Usage: ./startup.sh <dragonfly apm namespace> <unique altitude> <index in swarm> <swarm size>"
    exit 1
fi

source /opt/ros/kinetic/setup.bash
source /home/ubuntu/catkin_ws/devel/setup.bash
source ./devel/setup.bash

exec 3>&1 1>>logs/script.log 2>&1
#roscore&
sleep 10
roslaunch config/apm.launch & 
rosrun dragonfly co2publisher.py $1 &
rosrun dragonfly logger.py $1 >> logs/output.log &
rosrun dragonfly command.py $1 $2 $3 $4 &
