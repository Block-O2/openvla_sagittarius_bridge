#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${SAGITTARIUS_WS:-$HOME/sagittarius_ws}"
PACKAGE_DIR="$WS_DIR/src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector"
CSV_PATH="${1:-$PACKAGE_DIR/config/manual_calibration_points.csv}"
VISION_CONFIG="${2:-$PACKAGE_DIR/config/vision_config.yaml}"

cd "$WS_DIR"
source /opt/ros/noetic/setup.bash
source "$WS_DIR/devel/setup.bash"

python3 "$PACKAGE_DIR/nodes/manual_vision_calibration.py" \
  --csv "$CSV_PATH" \
  --vision-config "$VISION_CONFIG"
