#!/bin/bash

if [[ -z "$1" ]]
then
    echo "Usage: ./run.sh <dragonfly apm namespace>"
    exit 1
fi

roscore &
P1=$!
roslaunch apm.launch &
P2=$!
python co2publisher.py $1 &
P3=$!
python logger.py $1 > output.log &
P4=$!
wait $P1 $P2 $P3 $P4
