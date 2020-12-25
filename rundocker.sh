#!/bin/bash
docker run -it \
    --network ros-net \
    dragonfly-controller:latest \
    /bin/sh -c 'rosrun dragonfly announce.py JUAV1 & rosrun dragonfly command.py JUAV1'
