#!/usr/bin/env bash
set -e

roslaunch sagittarius_object_color_detector language_guided_robot_station.launch \
  robot_name:=sgr532 \
  video_dev:=/dev/video0 \
  image_width:=640 \
  image_height:=480 \
  framerate:=10 \
  pixel_format:=mjpeg \
  observation_topic:=/language_guided_grasp/target_observation \
  execution_feedback_topic:=/language_guided_grasp/execution_feedback \
  drop_after_grasp:=false
