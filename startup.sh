#!/bin/bash
source /opt/ros/kinetic/setup.bash
source /home/ubuntu/catkin_ws/devel/setup.bash

exec 3>&1 1>>logs/script.log 2>&1
#roscore&
sleep 10
roslaunch apm.launch & 
python co2publisher.py &
python logger.py >> logs/output.log &
