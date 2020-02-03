#!/bin/bash
roscore &
P1=$!
roslaunch apm.launch &
P2=$!
python co2publisher.py JUAV1 &
P3=$!
python logger.py JUAV1 > output.log &
P4=$!
wait $P1 $P2 $P3 $P4
