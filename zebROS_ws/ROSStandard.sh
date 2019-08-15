#!/usr/bin/env bash

# Setup ROS for Local Development
source /opt/ros/melodic/setup.bash
source ~/2019Offseason/zebROS_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:5802
export ROS_IP=`ip route get 10.9.0.1 | head -1 | cut -d ' ' -f 8`
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

exec "$@"
