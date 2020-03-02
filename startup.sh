#!/bin/bash

if [[ -z "$1" ]]
then
    echo "Usage: ./startup.sh <dragonfly apm namespace>"
    exit 1
fi

source /opt/ros/kinetic/setup.bash
source /home/ubuntu/catkin_ws/devel/setup.bash

exec 3>&1 1>>logs/script.log 2>&1
#roscore&
sleep 10
roslaunch apm.launch & 
python co2publisher.py $1 &
python logger.py $1 >> logs/output.log &
