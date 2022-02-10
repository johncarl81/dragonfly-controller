#!/bin/bash
set -e

source /opt/ros/rolling/setup.bash
source /workspace/install/setup.bash

export ROS_MASTER_URI=http://172.18.0.2:11311

exec "$@"
