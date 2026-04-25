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
# Use the old color-grasp XYZ/RPY search pose by default so the arm camera
# looks toward the table. Avoid the legacy DEFINE_STAY joint preset because it
# can fold the arm upward and point the camera at the ceiling.
SEARCH_POSE_MODE="${SEARCH_POSE_MODE:-camera_down}"
SEARCH_POSE_X="${SEARCH_POSE_X:-0.20}"
SEARCH_POSE_Y="${SEARCH_POSE_Y:-0.00}"
SEARCH_POSE_Z="${SEARCH_POSE_Z:-0.15}"
SEARCH_POSE_ROLL="${SEARCH_POSE_ROLL:-0.0}"
SEARCH_POSE_PITCH="${SEARCH_POSE_PITCH:-1.57}"
SEARCH_POSE_YAW="${SEARCH_POSE_YAW:-0.0}"
RETURN_TO_SEARCH_POSE_AFTER_GRASP="${RETURN_TO_SEARCH_POSE_AFTER_GRASP:-false}"
PICK_ORIENTATION_MODE="${PICK_ORIENTATION_MODE:-auto}"
DROP_AFTER_GRASP="${DROP_AFTER_GRASP:-false}"
SCAN_VIEW_ORDER="${SCAN_VIEW_ORDER:-front,left,right}"
SCAN_ATTEMPTS_PER_VIEW="${SCAN_ATTEMPTS_PER_VIEW:-5}"
SCAN_STABLE_REQUIRED="${SCAN_STABLE_REQUIRED:-3}"
SCAN_SETTLE_SEC="${SCAN_SETTLE_SEC:-0.8}"
LEFT_VIEW_ENABLED="${LEFT_VIEW_ENABLED:-false}"
LEFT_VIEW_VISION_CONFIG="${LEFT_VIEW_VISION_CONFIG:-}"
LEFT_VIEW_X="${LEFT_VIEW_X:-0.20}"
LEFT_VIEW_Y="${LEFT_VIEW_Y:-0.08}"
LEFT_VIEW_Z="${LEFT_VIEW_Z:-0.20}"
LEFT_VIEW_ROLL="${LEFT_VIEW_ROLL:-0.0}"
LEFT_VIEW_PITCH="${LEFT_VIEW_PITCH:-1.57}"
LEFT_VIEW_YAW="${LEFT_VIEW_YAW:-0.45}"
RIGHT_VIEW_ENABLED="${RIGHT_VIEW_ENABLED:-false}"
RIGHT_VIEW_VISION_CONFIG="${RIGHT_VIEW_VISION_CONFIG:-}"
RIGHT_VIEW_X="${RIGHT_VIEW_X:-0.20}"
RIGHT_VIEW_Y="${RIGHT_VIEW_Y:--0.08}"
RIGHT_VIEW_Z="${RIGHT_VIEW_Z:-0.20}"
RIGHT_VIEW_ROLL="${RIGHT_VIEW_ROLL:-0.0}"
RIGHT_VIEW_PITCH="${RIGHT_VIEW_PITCH:-1.57}"
RIGHT_VIEW_YAW="${RIGHT_VIEW_YAW:--0.45}"
SAVE_RAW_IMAGE="${SAVE_RAW_IMAGE:-true}"
RAW_IMAGE_PATH="${RAW_IMAGE_PATH:-/tmp/language_guided_grasp_raw_single_gpu.jpg}"
SAVE_ANNOTATED_IMAGE="${SAVE_ANNOTATED_IMAGE:-true}"
ANNOTATED_IMAGE_PATH="${ANNOTATED_IMAGE_PATH:-/tmp/language_guided_grasp_single_gpu.jpg}"
HF_HUB_OFFLINE="${HF_HUB_OFFLINE:-1}"
TRANSFORMERS_OFFLINE="${TRANSFORMERS_OFFLINE:-1}"

cd "$WS_DIR"
source "$GROUNDINGDINO_VENV/bin/activate"
source /opt/ros/noetic/setup.bash
source "$WS_DIR/devel/setup.bash"

export PYTHONPATH="$GROUNDINGDINO_ROOT:${PYTHONPATH:-}"
export MPLCONFIGDIR="${MPLCONFIGDIR:-/tmp/matplotlib-cfg}"
export HF_HUB_OFFLINE
export TRANSFORMERS_OFFLINE
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
  search_pose_x:="$SEARCH_POSE_X" \
  search_pose_y:="$SEARCH_POSE_Y" \
  search_pose_z:="$SEARCH_POSE_Z" \
  search_pose_roll:="$SEARCH_POSE_ROLL" \
  search_pose_pitch:="$SEARCH_POSE_PITCH" \
  search_pose_yaw:="$SEARCH_POSE_YAW" \
  return_to_search_pose_after_grasp:="$RETURN_TO_SEARCH_POSE_AFTER_GRASP" \
  pick_orientation_mode:="$PICK_ORIENTATION_MODE" \
  drop_after_grasp:="$DROP_AFTER_GRASP" \
  dynamic_place_z:="${DYNAMIC_PLACE_Z:-0.20}" \
  scan_view_order:="$SCAN_VIEW_ORDER" \
  scan_attempts_per_view:="$SCAN_ATTEMPTS_PER_VIEW" \
  scan_stable_required:="$SCAN_STABLE_REQUIRED" \
  scan_settle_sec:="$SCAN_SETTLE_SEC" \
  left_view_enabled:="$LEFT_VIEW_ENABLED" \
  left_view_vision_config:="$LEFT_VIEW_VISION_CONFIG" \
  left_view_x:="$LEFT_VIEW_X" \
  left_view_y:="$LEFT_VIEW_Y" \
  left_view_z:="$LEFT_VIEW_Z" \
  left_view_roll:="$LEFT_VIEW_ROLL" \
  left_view_pitch:="$LEFT_VIEW_PITCH" \
  left_view_yaw:="$LEFT_VIEW_YAW" \
  right_view_enabled:="$RIGHT_VIEW_ENABLED" \
  right_view_vision_config:="$RIGHT_VIEW_VISION_CONFIG" \
  right_view_x:="$RIGHT_VIEW_X" \
  right_view_y:="$RIGHT_VIEW_Y" \
  right_view_z:="$RIGHT_VIEW_Z" \
  right_view_roll:="$RIGHT_VIEW_ROLL" \
  right_view_pitch:="$RIGHT_VIEW_PITCH" \
  right_view_yaw:="$RIGHT_VIEW_YAW" \
  save_raw_image:="$SAVE_RAW_IMAGE" \
  raw_image_path:="$RAW_IMAGE_PATH" \
  save_annotated_image:="$SAVE_ANNOTATED_IMAGE" \
  annotated_image_path:="$ANNOTATED_IMAGE_PATH" \
  groundingdino_config:="$GROUNDINGDINO_CONFIG" \
  groundingdino_weights:="$GROUNDINGDINO_WEIGHTS" \
  default_target_text:="$TARGET_TEXT"
