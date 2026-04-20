#!/usr/bin/env bash

# Edit these two IPs before sourcing this file.
export ROBOT_STATION_IP=192.168.1.20
export AI_STATION_IP=192.168.1.10

export ROS_MASTER_URI=http://${ROBOT_STATION_IP}:11311
export ROS_IP=${AI_STATION_IP}

cd ~/sagittarius_ws || exit 1
source /mnt/d/ai_models/groundingdino-venv/bin/activate
source /opt/ros/noetic/setup.bash
source devel/setup.bash

export PYTHONPATH=/mnt/d/ai_models/GroundingDINO:${PYTHONPATH}
export MPLCONFIGDIR=/tmp/matplotlib-cfg
mkdir -p "${MPLCONFIGDIR}"
