#!/bin/bash
docker run -it \
    --network ros-net \
    dragonfly-controller:latest \
    /bin/sh -c 'for i in {1..3}
    do
      rosrun dragonfly announce.py JUAV$i &
      rosrun dragonfly command.py JUAV$i &
    done; rosrun dragonfly announce.py JUAV4 & rosrun dragonfly command.py JUAV4'
