#!/usr/bin/env bash
set -euo pipefail

# Single-machine language-guided grasp runner.
# Default is dry-run for safety. Set EXECUTE_GRASP=true for real grasp.

WS_DIR="${SAGITTARIUS_WS:-$HOME/sagittarius_ws}"
GROUNDINGDINO_ROOT="${GROUNDINGDINO_ROOT:-/mnt/d/ai_models/GroundingDINO}"
GROUNDINGDINO_VENV="${GROUNDINGDINO_VENV:-/mnt/d/ai_models/groundingdino-venv}"
GROUNDINGDINO_CONFIG="${GROUNDINGDINO_CONFIG:-$GROUNDINGDINO_ROOT/groundingdino/config/GroundingDINO_SwinT_OGC.py}"
GROUNDINGDINO_WEIGHTS="${GROUNDINGDINO_WEIGHTS:-$GROUNDINGDINO_ROOT/weights/groundingdino_swint_ogc.pth}"

VIDEO_DEV="${VIDEO_DEV:-/dev/video0}"
PIXEL_FORMAT="${PIXEL_FORMAT:-mjpeg}"
IMAGE_WIDTH="${IMAGE_WIDTH:-640}"
IMAGE_HEIGHT="${IMAGE_HEIGHT:-480}"
FRAMERATE="${FRAMERATE:-10}"

DEVICE="${DEVICE:-cuda}"
EXECUTE_GRASP="${EXECUTE_GRASP:-false}"
TARGET_TEXT="${TARGET_TEXT:-}"
SEARCH_POSE_MODE="${SEARCH_POSE_MODE:-define_stay}"
RETURN_TO_SEARCH_POSE_AFTER_GRASP="${RETURN_TO_SEARCH_POSE_AFTER_GRASP:-false}"
PICK_ORIENTATION_MODE="${PICK_ORIENTATION_MODE:-auto}"
DROP_AFTER_GRASP="${DROP_AFTER_GRASP:-false}"

cd "$WS_DIR"
source "$GROUNDINGDINO_VENV/bin/activate"
source /opt/ros/noetic/setup.bash
source "$WS_DIR/devel/setup.bash"

export PYTHONPATH="$GROUNDINGDINO_ROOT:${PYTHONPATH:-}"
export MPLCONFIGDIR="${MPLCONFIGDIR:-/tmp/matplotlib-cfg}"
mkdir -p "$MPLCONFIGDIR"

roslaunch sagittarius_object_color_detector language_guided_grasp.launch \
  device:="$DEVICE" \
  video_dev:="$VIDEO_DEV" \
  pixel_format:="$PIXEL_FORMAT" \
  image_width:="$IMAGE_WIDTH" \
  image_height:="$IMAGE_HEIGHT" \
  framerate:="$FRAMERATE" \
  min_detection_interval:=0.5 \
  stable_required:=3 \
  center_tolerance:=12.0 \
  min_grasp_score:=0.35 \
  execute_grasp:="$EXECUTE_GRASP" \
  search_pose_mode:="$SEARCH_POSE_MODE" \
  return_to_search_pose_after_grasp:="$RETURN_TO_SEARCH_POSE_AFTER_GRASP" \
  pick_orientation_mode:="$PICK_ORIENTATION_MODE" \
  drop_after_grasp:="$DROP_AFTER_GRASP" \
  save_annotated_image:=true \
  annotated_image_path:=/tmp/language_guided_grasp_single_gpu.jpg \
  groundingdino_config:="$GROUNDINGDINO_CONFIG" \
  groundingdino_weights:="$GROUNDINGDINO_WEIGHTS" \
  default_target_text:="$TARGET_TEXT"
