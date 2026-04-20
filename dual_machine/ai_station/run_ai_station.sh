#!/usr/bin/env bash
set -e

roslaunch sagittarius_object_color_detector language_guided_ai_station.launch \
  device:=cpu \
  image_topic:=/usb_cam/image_raw \
  target_topic:=/grasp_target_text \
  observation_topic:=/language_guided_grasp/target_observation \
  execution_feedback_topic:=/language_guided_grasp/execution_feedback \
  groundingdino_config:=/mnt/d/ai_models/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py \
  groundingdino_weights:=/mnt/d/ai_models/GroundingDINO/weights/groundingdino_swint_ogc.pth
