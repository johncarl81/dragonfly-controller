#!/bin/bash
set -e

source /opt/ros/melodic/setup.bash
source /workspace/devel/setup.bash

export ROS_MASTER_URI=http://172.18.0.2:11311

exec "$@"
