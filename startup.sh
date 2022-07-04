#!/bin/bash

source /opt/ros/galactic/setup.bash
source /home/ubuntu/dev/dragonfly/install/setup.bash

exec 3>&1 1>>logs/script.log 2>&1
cd /home/ubuntu/dev/dragonfly

ros2 launch config/apm.launch & 
ros2 run dragonfly co2publisher $HOSTNAME &
#logger node is looking for led_logging.json
#ros2 run dragonfly logger $HOSTNAME >> logs/output.log &
ros2 run dragonfly command $HOSTNAME
