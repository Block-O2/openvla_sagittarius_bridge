#!/usr/bin/env bash

# Edit these two IPs before sourcing this file.
export ROBOT_STATION_IP=192.168.1.20
export AI_STATION_IP=192.168.1.10

export ROS_MASTER_URI=http://${ROBOT_STATION_IP}:11311
export ROS_IP=${ROBOT_STATION_IP}

cd ~/sagittarius_ws || exit 1
source /opt/ros/noetic/setup.bash
source devel/setup.bash
