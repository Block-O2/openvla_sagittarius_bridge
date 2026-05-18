# Sagittarius Semantic Grasp

Language-guided semantic grasping system for a Sagittarius robotic arm using ROS1, MoveIt, GroundingDINO, camera calibration, HSV/contour center refinement, and real-robot pick/place execution.

## Overview

This project turns the original Sagittarius color-grasping example into a real-robot semantic manipulation pipeline. A user provides a natural-language target or a simple task such as `put each block into the bucket of the same color`; the system captures a camera frame, detects language-conditioned objects with GroundingDINO, refines grasp centers with HSV/contour processing, maps image coordinates into the calibrated robot work plane, and executes grasp/place actions through the existing Sagittarius ROS1, MoveIt, and `sgr_ctrl` control chain.

## Demo

### Demo 1 — Single-target pick/place
[![Demo 1](https://img.youtube.com/vi/4V7Y5Qn2Jmw/0.jpg)](https://www.youtube.com/watch?v=4V7Y5Qn2Jmw)

Baseline end-to-end run: detect a blue block by language prompt and place it into a red bucket on the real arm.

---

### Demo 2 — Multi-step stress test with safe failure behavior
[![Demo 2](https://img.youtube.com/vi/vSotKdQC2tA/0.jpg)](https://www.youtube.com/watch?v=vSotKdQC2tA)

Task: `put each block into the bucket of the same color` (red, blue, green).

| Step | Result | Reason |
|------|--------|--------|
| Red block → red bucket | **Failed** | Arm observation angle caused incomplete bucket detection; no stable center found — system rejected execution |
| Blue block → blue bucket | **Succeeded** | Detection stable, placement completed |
| Green block → green bucket | **Failed (intentional)** | Bucket was moved during the task to prevent stable frame accumulation — system correctly refused to execute |

The test was designed to push edge cases. The failures show the stability and confidence gates working as intended: the arm only moves when detection is reliable enough to act on.

---

### Demo 3 — Open-vocabulary generalization: placing onto a palm
[![Demo 3](https://img.youtube.com/vi/i96AO-EEe4g/0.jpg)](https://www.youtube.com/watch?v=i96AO-EEe4g)

Target: a human palm — not a bucket, not a trained object category. GroundingDINO identified it correctly from the language prompt, and the arm completed the placement. Shows that the open-vocabulary detection backend generalizes beyond the block-and-bucket setup without any retraining.

---

## Highlights

- Real Sagittarius robotic arm project, not only an offline vision demo.
- Language-conditioned object detection with GroundingDINO.
- ROS1 integration through `/grasp_target_text`, camera topics, state topics, launch files, and action execution.
- Pick/place task flow for block-to-bucket semantic sorting.
- Separate `pick_front` and `place_front` camera calibration configurations.
- 3x3 manual calibration workflow for mapping pixel centers to robot work-plane coordinates.
- HSV/contour center refinement inside GroundingDINO boxes to improve grasp point stability.
- Stability checks and confidence filtering before robot execution.
- Safe rejection behavior when text is empty, confidence is low, no target is detected, or the placement target is not visible.
- Debug outputs including annotated images and saved raw/annotated frames.

## My Contributions

My work focused on turning the baseline Sagittarius examples into an integrated semantic grasping system:

- Integrated GroundingDINO as the language-conditioned perception backend.
- Added the ROS text-command interface and task parsing for single-object and simple pick/place commands.
- Built the perception-to-execution pipeline: detection, target selection, stability checks, center refinement, coordinate mapping, and Sagittarius execution dispatch.
- Added manual camera calibration workflows and maintained separate calibration files for pick and place viewpoints.
- Added HSV/contour-based center refinement to make the detected center more useful for physical grasping than a raw bounding-box center.
- Preserved the original Sagittarius MoveIt / SDK / `sgr_ctrl` execution path instead of replacing the low-level robot control stack.
- Tested the pipeline on the real arm and documented failure cases instead of hiding them.

## Tech Stack

| Area | Tools / Components |
| --- | --- |
| Robot | Sagittarius arm, gripper, `sgr_ctrl`, MoveIt |
| Middleware | ROS1 Noetic, actionlib, roslaunch, rostopic |
| Perception | GroundingDINO, OpenCV, HSV segmentation, contour analysis |
| Calibration | Manual 3x3 point collection, YAML calibration configs, pixel-to-plane linear mapping |
| Execution | Pick/place state machine, stable-frame locking, safe execution gates |
| Platform | Ubuntu 20.04 / WSL2 Ubuntu 20.04, CUDA-capable GPU recommended |

## Limitations and Next Steps

This is a research/engineering prototype rather than an industrial grasping product.

Current limitations:

- The most reliable tested mode is front-view operation.
- Left/right multi-view support exists in the configuration structure but needs more real-robot calibration and validation.
- Pixel-to-robot mapping currently uses a planar linear mapping, so it is best suited to tabletop tasks rather than full 3D scenes.
- No depth camera or full 6D pose estimation is used.
- Placement depends heavily on whether the bucket is visible after grasping.
- GroundingDINO remains sensitive to prompt wording, lighting, occlusion, and object appearance.
- WSL2 USB camera/serial passthrough can be unstable.
- Task parsing is rule-based, not a general LLM planner.

Next steps:

- Validate left/right viewpoints with real 3x3 calibration data.
- Replace or augment box centers with segmentation-mask or grasp-point estimation.
- Add depth sensing or AprilTag/hand-eye calibration for stronger spatial reasoning.
- Improve failure recovery, such as rescanning from another viewpoint when the place target is missing.
- Migrate the most stable workflow from WSL2 to native Ubuntu for more reliable USB/camera behavior.

---

# Detailed Technical Notes

The sections below preserve the setup, calibration, launch, testing, and debugging notes for reproducing the system.

## Original Chinese Summary

基于 Sagittarius 机械臂、ROS1、MoveIt、GroundingDINO 与相机标定的语言引导语义抓取 / 放置项目。

本项目的目标不是单纯做一个"颜色块识别 demo"，而是把原始 Sagittarius 机械臂的 HSV 颜色抓取例程改造成一个更接近真实机器人应用的语义抓取系统：用户用自然语言指定目标，视觉模型在相机图像中识别目标，系统将像素位置映射到机械臂工作平面坐标，并调用原有 MoveIt / `sgr_ctrl` 执行抓取与放置。

## System Flow / 项目概览

```text
Natural-language task
  -> task parsing into pick/place targets
  -> camera image capture
  -> GroundingDINO semantic detection
  -> HSV / contour center refinement
  -> pixel-center to robot-plane coordinate mapping
  -> Sagittarius MoveIt / sgr_ctrl pick or place execution
```

Typical target command:

```text
red block
```

Task-level command:

```text
put each block into the bucket of the same color
```

The task-level command is decomposed into steps such as:

```text
1. red block   -> red bucket
2. blue block  -> blue bucket
3. green block -> green bucket
```

Each step has two phases:

```text
pick phase: detect and grasp the block
place phase: observe again, detect the bucket, and place near/above it
```

## Current Main-Branch Status

The current `main` branch includes multi-stage pick/place, front-view 3x3 calibration, HSV center refinement, and real-robot testing notes.

Implemented capabilities:

- GroundingDINO language-conditioned object detection backend.
- Natural-language target input through `/grasp_target_text`.
- Single-target grasp commands such as `red block` and `blue block`.
- Simple task decomposition such as `red block -> red bucket`.
- Two-stage pick/place execution.
- Separate `pick_front` and `place_front` calibration configurations.
- 3x3 manual calibration data.
- HSV/contour center refinement inside the GroundingDINO detection box.
- Stable-frame locking before execution.
- Safe rejection for low confidence, missing target, and empty text.
- Annotated-image output for debugging.
- Raw and annotated image saving for post-run inspection.
- Original Sagittarius robot execution chain preserved.

## Hardware and Software Environment

Robot hardware:

- Sagittarius robotic arm.
- Sagittarius MoveIt configuration.
- Sagittarius SDK / `sgr_ctrl`.
- ROS namespace: `/sgr532`.
- Serial device usually mapped as `/dev/ttyACM0` or `/dev/sagittarius -> /dev/ttyACM0`.
- Under WSL2, USB serial passthrough usually requires Windows `usbipd`.

Camera hardware:

- UVC camera through ROS `usb_cam`.
- Default device: `/dev/video0`.
- Default image settings: 640x480, 10 FPS, MJPEG.

Compute platform:

- Ubuntu 20.04 / WSL2 Ubuntu 20.04.
- ROS Noetic.
- CUDA-capable GPU recommended for GroundingDINO.
- CPU inference is possible but too slow for comfortable real-robot testing.

## Repository Structure

Main package path:

```text
src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/
```

Important files:

| File | Role |
| --- | --- |
| `language_guided_grasp.py` | Main node for task parsing, detection, stability checks, coordinate mapping, grasp/place scheduling |
| `language_guided_calibration.py` | Language-guided calibration node for collecting calibration points from viewpoints |
| `manual_vision_calibration.py` | Fits `vision_config*.yaml` from manually collected pixel/robot coordinate pairs |
| `center_refinement.py` | HSV/contour refinement inside detection boxes |
| `task_parsing.py` | Converts simple language tasks into pick/place steps |
| `coordinate_mapping.py` | Loads calibration YAML and maps image pixels into the robot work plane |
| `execution.py` | Thin wrapper over the original `SGRCtrlAction` execution path |
| `grounding_dino.py` | GroundingDINO backend |
| `run_single_machine_gpu_grasp.sh` | Single-machine GPU real-robot test launcher |
| `ensure_sagittarius_serial.sh` | Checks/fixes Sagittarius serial mapping |

## Architecture

```text
Input layer
  - camera image /usb_cam/image_raw
  - text task /grasp_target_text

Perception layer
  - BasePerceptionBackend
  - GroundingDinoBackend
  - HSV / contour center refinement

Decision layer
  - target selection
  - confidence filtering
  - stable-frame detection
  - task state machine

Mapping layer
  - VisionPlaneMapper
  - vision_config_pick_front.yaml
  - vision_config_place_front.yaml
  - vision_config_left.yaml / right.yaml

Execution layer
  - SagittariusGraspExecutor
  - SGRCtrlAction
  - MoveIt
  - Sagittarius SDK
```

Main state flow:

```text
waiting_for_target -> detecting -> target_locked -> grasping -> placing -> done
```

Failure state:

```text
failed
```

Common failure reasons:

- Empty target text.
- No detection.
- Detection confidence below threshold.
- Unstable target center.
- MoveIt planning failure.
- Placement target not visible.
- Robot execution failure.

State topic:

```bash
rostopic echo /language_guided_grasp/state
```

## Coordinate Mapping and Calibration

The vision model outputs image pixel coordinates:

```text
pixel_x, pixel_y
```

The robot needs work-plane coordinates:

```text
robot_x, robot_y, robot_z
```

The current system follows the original Sagittarius linear mapping style:

```text
robot_x = k1 * pixel_y + b1
robot_y = k2 * pixel_x + b2
```

YAML format:

```yaml
LinearRegression:
  k1: ...
  b1: ...
  k2: ...
  b2: ...
```

Recommended calibration files:

| File | Usage |
| --- | --- |
| `vision_config_pick_front.yaml` | Front-view pick-stage calibration |
| `vision_config_place_front.yaml` | Front-view place-stage calibration |
| `vision_config_left.yaml` | Left-view calibration scaffold; requires site-specific validation |
| `vision_config_right.yaml` | Right-view calibration scaffold; requires site-specific validation |
| `vision_config_front.yaml` | Compatibility front-view config |
| `vision_config.yaml` | Original compatibility entry point |

The most reliable current real-robot results use:

```text
vision_config_pick_front.yaml
vision_config_place_front.yaml
```

Current 3x3 calibration data:

```text
manual_calibration_points_pick_front.csv
manual_calibration_points_place_front.csv
```

The calibration grid covers:

```text
x = 0.22, 0.24, 0.26
y = -0.03, 0.00, 0.03
```

Do not casually reuse `pick_front` calibration for `place_front`; the robot pose, camera view, and object type can differ.

Manual calibration flow:

1. Move the arm to a fixed observation pose.
2. Place a calibration object on the table, such as a blue block or bucket.
3. Detect the target with GroundingDINO.
4. Refine the center using HSV/contour processing inside the box.
5. Record `pixel_x, pixel_y, robot_x, robot_y, score, label`.
6. Save as `manual_calibration_points_*.csv`.
7. Fit `vision_config_*.yaml` with the manual calibration script.

Example calibration row:

```csv
pixel_x,pixel_y,robot_x,robot_y,score,label
383.7,376.9,0.22,-0.03,0.95,blue block
```

Calibration notes:

- Points should cover the actual grasp/place area, not only the image center.
- `pick_front` should cover where blocks usually appear.
- `place_front` should cover where buckets usually appear.
- Recalibrate if the camera position, arm observation pose, or resolution changes.
- If the detection is correct but the robot lands in the wrong location, check calibration before blaming GroundingDINO.

## Test Modes

Recommended test order is from safe to risky.

### Static Image Test

Purpose: verify GroundingDINO, text prompts, detection results, and visualization without real robot execution.

Relevant entry points:

```text
language_guided_grasp_image_test.launch
publish_test_image.py
```

### Live Camera Detection Without Execution

Purpose: verify the real camera feed, GroundingDINO detection, center refinement, and coordinate mapping without moving the arm.

Recommended setting:

```text
execute_grasp:=false
```

Check that:

- The annotated image shows the correct target box.
- The refined center is near the object center.
- Mapped x/y values are reasonable.
- Mapped x/y changes with target movement.

### Single-Object Real Grasp

Suggested targets:

```text
red block
blue block
green block
```

Start with placement disabled:

```text
drop_after_grasp:=false
```

### Pick/Place Test

Example commands:

```text
pick red block and place it into red bucket
put each block into the bucket of the same color
```

## Single-Machine GPU Launch

Safe detection mode, with no real arm motion:

```bash
bash src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/scripts/run_single_machine_gpu_grasp.sh
```

Equivalent explicit form:

```bash
EXECUTE_GRASP=false bash src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/scripts/run_single_machine_gpu_grasp.sh
```

Real grasp mode, only after validating detection and mapping:

```bash
EXECUTE_GRASP=true bash src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/scripts/run_single_machine_gpu_grasp.sh
```

Override camera device:

```bash
VIDEO_DEV=/dev/video2 EXECUTE_GRASP=true bash src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/scripts/run_single_machine_gpu_grasp.sh
```

Start with a target text:

```bash
TARGET_TEXT="red block" EXECUTE_GRASP=true bash src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/scripts/run_single_machine_gpu_grasp.sh
```

Full same-color sorting task:

```bash
TARGET_TEXT="put each block into the bucket of the same color" EXECUTE_GRASP=true bash src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/scripts/run_single_machine_gpu_grasp.sh
```

Manual launch fallback:

```bash
roslaunch sagittarius_object_color_detector language_guided_grasp.launch execute_grasp:=false
```

Send target text manually:

```bash
rostopic pub -1 /grasp_target_text std_msgs/String "data: 'red block'"
rostopic pub -1 /grasp_target_text std_msgs/String "data: 'pick red block and place it into red bucket'"
rostopic pub -1 /grasp_target_text std_msgs/String "data: 'put each block into the bucket of the same color'"
```

## Key Parameters

Perception parameters:

| Parameter | Meaning | Suggested value |
| --- | --- | --- |
| `device` | GroundingDINO inference device | `cuda` |
| `box_threshold` | Detection-box threshold | start around `0.35` |
| `text_threshold` | Text matching threshold | start around `0.25` |
| `min_grasp_score` | Minimum confidence before execution | start around `0.35` |
| `stable_required` | Required stable frames | `5` |
| `center_tolerance` | Pixel tolerance for center stability | `8.0` |

Execution parameters:

| Parameter | Meaning | Recommendation |
| --- | --- | --- |
| `execute_grasp` | Whether to move the real robot | debug with `false`, execute with `true` only after checks |
| `pick_z` | Grasp height | tune for table/object height |
| `drop_after_grasp` | Whether to place after grasp | keep `false` for early single-object tests |
| `dynamic_place_z` | Dynamic placement height | tune for bucket rim height |
| `return_to_search_pose_after_grasp` | Return to search pose after grasp | usually `false` in current tests |

Observation pose parameters include:

```text
search_pose_x/y/z/roll/pitch/yaw
place_front_view_x/y/z/roll/pitch/yaw
left_view_x/y/z/roll/pitch/yaw
right_view_x/y/z/roll/pitch/yaw
place_scan_view_order
```

The most validated setting is:

```text
place_scan_view_order:=front
```

## Debug Outputs

State topic:

```bash
rostopic echo /language_guided_grasp/state
```

Annotated image topic:

```text
/language_guided_grasp/annotated_image
```

Common saved images:

```text
/tmp/language_guided_grasp_raw_single_gpu.jpg
/tmp/language_guided_grasp_single_gpu.jpg
/tmp/language_guided_grasp_latest.jpg
```

If grasping is inaccurate, check:

1. Whether the raw image clearly shows the target.
2. Whether the annotated detection box is correct.
3. Whether the refined center is near the target center.
4. Whether the logged pixel center is reasonable.
5. Whether mapped x/y matches the actual table position.
6. Whether the active `vision_config` matches the current observation pose.

## Common Issues

### GroundingDINO detects correctly but the arm misses

Check calibration first. Common causes:

- Wrong `vision_config*.yaml`.
- Camera moved without recalibration.
- Observation pose changed without recalibration.
- Runtime center definition differs from calibration-time center definition.

### MoveIt plans but does not execute

Errors such as `Computed path is not valid`, `Start state appears to be in collision`, or contact between links usually indicate self-collision or bad initial state. Inspect the robot model in RViz before changing collision settings.

### WSL cannot find the arm serial device

Check:

```bash
lsusb
ls -l /dev/ttyACM* /dev/sagittarius
```

Then run:

```bash
bash src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/scripts/ensure_sagittarius_serial.sh
```

Under WSL2, confirm the device is attached through Windows `usbipd`.

### Camera and arm connection instability

This is common under WSL2 USB passthrough. Prefer native Ubuntu for demos, a stable UVC camera, or a two-machine ROS deployment where one machine handles GPU inference and the robot host handles arm control.

## Suggested Demo Flow

1. Start the arm and camera.
2. Run with `execute_grasp:=false` and verify detection/mapping.
3. Send `red block` and inspect the annotated image.
4. Confirm mapped x/y values are reasonable.
5. Enable `execute_grasp:=true` for a single-object grasp.
6. Test `pick red block and place it into red bucket`.
7. Test `put each block into the bucket of the same color`.

## Project Summary

The current `main` branch implements a working language-guided robotic grasping prototype:

- GroundingDINO connects natural-language targets to image objects.
- HSV/contour refinement improves the detected center for grasping.
- 3x3 calibration maps image pixels into the robot work plane.
- Sagittarius MoveIt / SDK / `sgr_ctrl` remains the execution backbone.
- Real-robot tests show partial success on multi-step same-color grasp/place tasks with documented failure behavior.

Most mature current configuration:

```text
front-only + separate pick_front/place_front calibration + GroundingDINO + HSV refinement
```
