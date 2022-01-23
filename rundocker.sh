#!/bin/bash

docker network inspect ros-net >/dev/null 2>&1 || \
    docker network create ros-net --subnet=172.18.0.0/16

docker run -it \
    --network ros-net \
    dragonfly-controller:latest \
    /bin/sh -c 'for i in {1..3}
    do
      rosrun dragonfly announce.py dragonfly$i &
      rosrun dragonfly command.py dragonfly$i &
      rosrun dragonfly virtualco2publisher.py dragonfly$i &
    done;
    rosrun dragonfly announce.py dragonfly4 &
    rosrun dragonfly command.py dragonfly4 &
    rosrun dragonfly virtualco2publisher.py dragonfly4'
