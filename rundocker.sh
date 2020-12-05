#!/bin/bash
docker run -it \
    --network ros-net \
    dragonfly-controller:latest \
    rosrun dragonfly command.py JUAV1
