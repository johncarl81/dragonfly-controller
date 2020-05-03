#!/bin/bash

if [[ -z "$1" ]]
then
    echo "Usage: ./run.sh <dragonfly apm namespace>"
    exit 1
fi

source ./devel/setup.bash

roscore &
P1=$!
roslaunch config/apm.launch &
P2=$!
rosrun dragonfly co2publisher.py $1 &
P3=$!
rosrun dragonfly logger.py $1 > output.log &
P4=$!
wait $P1 $P2 $P3 $P4
