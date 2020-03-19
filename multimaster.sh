#!/bin/bash

roscore&
rosrun master_discovery_fkie master_discovery >/dev/null 2>&1 &
rosrun master_sync_fkie master_sync >/dev/null 2>&1 &
