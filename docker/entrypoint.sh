#!/bin/bash
set -e

source /opt/ros/galactic/setup.bash
source /workspace/install/setup.bash

exec "$@"
