#!/bin/bash

docker network inspect ros-net >/dev/null 2>&1 || \
    docker network create ros-net --subnet=172.18.0.0/16

docker run -it \
    --network ros-net \
    dragonfly-controller:latest --drones 4
    
