# AI Station Guide

This machine runs language-conditioned perception. It does not connect directly to the Sagittarius arm.

## What This Machine Needs

- This repository: `openvla_sagittarius_bridge`
- ROS Noetic workspace environment
- GroundingDINO source code
- GroundingDINO model weights
- `groundingdino-venv` Python environment
- Network access to the Robot Station ROS master

This machine does not need:

- Sagittarius arm serial connection
- USB camera connection

## Responsibility

```text
/usb_cam/image_raw
        |
        v
language_guided_perception.py
        |
        v
/language_guided_grasp/target_observation
```

Inputs:

- `/usb_cam/image_raw`: camera frames published by the Robot Station
- `/grasp_target_text`: text prompt such as `red block`, `banana`, or `bottle`
- `/language_guided_grasp/execution_feedback`: execution feedback from the Robot Station

Outputs:

- `/language_guided_grasp/target_observation`: lightweight JSON detection result
- `/language_guided_perception/annotated_image`: annotated debug image
- `/language_guided_perception/state`: perception node state

## Network Setup

Example:

```text
Robot Station IP: 192.168.1.20
AI Station IP:    192.168.1.10
```

On the AI Station:

```bash
export ROS_MASTER_URI=http://192.168.1.20:11311
export ROS_IP=192.168.1.10
```

Check connectivity:

```bash
ping 192.168.1.20
rostopic list
```

## Environment

Typical setup on the current machine:

```bash
cd ~/sagittarius_ws
source /mnt/d/ai_models/groundingdino-venv/bin/activate
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export PYTHONPATH=/mnt/d/ai_models/GroundingDINO:$PYTHONPATH
export MPLCONFIGDIR=/tmp/matplotlib-cfg
mkdir -p "$MPLCONFIGDIR"
```

## Start AI Station

```bash
roslaunch sagittarius_object_color_detector language_guided_ai_station.launch \
  device:=cpu \
  image_topic:=/usb_cam/image_raw \
  target_topic:=/grasp_target_text \
  observation_topic:=/language_guided_grasp/target_observation \
  execution_feedback_topic:=/language_guided_grasp/execution_feedback \
  groundingdino_config:=/mnt/d/ai_models/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py \
  groundingdino_weights:=/mnt/d/ai_models/GroundingDINO/weights/groundingdino_swint_ogc.pth
```

If CUDA is available, use:

```bash
device:=cuda
```

## Send Target Text

The target text can be published from any machine connected to the same ROS master.

```bash
rostopic pub /grasp_target_text std_msgs/String "data: 'red block'" -1
rostopic pub /grasp_target_text std_msgs/String "data: 'banana'" -1
rostopic pub /grasp_target_text std_msgs/String "data: 'bottle'" -1
```

## Checks

```bash
rostopic hz /usb_cam/image_raw
rostopic echo /language_guided_perception/state
rostopic echo /language_guided_grasp/target_observation
rostopic hz /language_guided_perception/annotated_image
```

If `/usb_cam/image_raw` has no rate, the Robot Station camera stream is not reaching this machine.

If `target_observation` has no output, check:

- `/grasp_target_text` has been published
- GroundingDINO paths are correct
- `PYTHONPATH` includes the GroundingDINO source tree
- The AI Station can receive `/usb_cam/image_raw`

## Git Setup

Use the dual-machine branch:

```bash
git fetch origin
git switch dual-machine-station-split
git pull origin dual-machine-station-split
catkin_make
source devel/setup.bash
```

## Complete AI Station Command Checklist

Use this checklist on the machine that runs GroundingDINO inference. The Robot Station should already be hosting the ROS master, or `ROS_MASTER_URI` should point to the Robot Station.

### 1. Configure Network and Workspace

Copy the environment template and edit the IP addresses:

```bash
cd ~/sagittarius_ws
cp dual_machine/ai_station/env.example.sh dual_machine/ai_station/env.local.sh
nano dual_machine/ai_station/env.local.sh
```

Confirm these values:

```bash
export ROBOT_STATION_IP=192.168.1.20
export AI_STATION_IP=192.168.1.10
```

Load the environment:

```bash
source dual_machine/ai_station/env.local.sh
```

### 2. Network Connectivity Test

```bash
ping $ROBOT_STATION_IP
rostopic list
```

If `rostopic list` fails, check:

- The Robot Station has started `roscore` or the robot launch
- `ROS_MASTER_URI` points to the Robot Station
- Both machines are on the same network
- `ROS_IP` is the correct network-interface IP

### 3. Camera Stream Test

After the Robot Station starts its camera node, run:

```bash
rostopic hz /usb_cam/image_raw
```

Do not run a live grasp demo until this topic has a stable rate.

### 4. GroundingDINO Static-Image Smoke Test

This test does not require the robot or the lab machine. It only verifies the local model paths, Python environment, and backend inference:

```bash
python3 src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/nodes/perception_backend_smoke_test.py \
  --image src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/test_data/sample_red_block.ppm \
  --text "red block" \
  --backend grounding_dino \
  --config /mnt/d/ai_models/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py \
  --weights /mnt/d/ai_models/GroundingDINO/weights/groundingdino_swint_ogc.pth \
  --device cpu \
  --output /tmp/ai_station_smoke_red_block.jpg
```

Expected output:

```text
source_model: grounding_dino
decision_status: selected
selected_score: ...
selected_center: ...
annotated_output: /tmp/ai_station_smoke_red_block.jpg
```

Open the annotated image:

```bash
xdg-open /tmp/ai_station_smoke_red_block.jpg
```

### 5. Start AI Station Test Mode

Recommended script:

```bash
source dual_machine/ai_station/env.local.sh
bash dual_machine/ai_station/run_ai_station.sh
```

Manual launch command:

```bash
roslaunch sagittarius_object_color_detector language_guided_ai_station.launch \
  device:=cpu \
  image_topic:=/usb_cam/image_raw \
  target_topic:=/grasp_target_text \
  observation_topic:=/language_guided_grasp/target_observation \
  execution_feedback_topic:=/language_guided_grasp/execution_feedback \
  groundingdino_config:=/mnt/d/ai_models/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py \
  groundingdino_weights:=/mnt/d/ai_models/GroundingDINO/weights/groundingdino_swint_ogc.pth
```

In another AI Station terminal:

```bash
source dual_machine/ai_station/env.local.sh
rostopic echo /language_guided_perception/state
```

To inspect the lightweight detection result sent to the Robot Station:

```bash
source dual_machine/ai_station/env.local.sh
rostopic echo /language_guided_grasp/target_observation
```

### 6. Official Run Command

For the full dual-machine demo, keep this running on the AI Station:

```bash
source dual_machine/ai_station/env.local.sh
bash dual_machine/ai_station/run_ai_station.sh
```

Then publish a target text from any terminal connected to the same ROS master:

```bash
rostopic pub /grasp_target_text std_msgs/String "data: 'red block'" -1
```

## New Demo Scenarios

All demos use the same pipeline: text target -> GroundingDINO -> bbox center -> JSON observation -> Robot Station stability check -> coordinate mapping -> `sgr_ctrl` grasp.

### Demo 1: Grasp `red block`

```bash
rostopic pub /grasp_target_text std_msgs/String "data: 'red block'" -1
```

Expected behavior:

- AI Station publishes a `selected` observation
- Annotated image shows the target bbox and center point
- Robot Station executes after stable detection

### Demo 2: Grasp `banana`

```bash
rostopic pub /grasp_target_text std_msgs/String "data: 'banana'" -1
```

This demonstrates category-level grasping beyond HSV color thresholding.

### Demo 3: Grasp `bottle`

```bash
rostopic pub /grasp_target_text std_msgs/String "data: 'bottle'" -1
```

This is useful for showing everyday object categories.

### Demo 4: Multi-Object Target Selection

Place multiple objects on the table and request only one:

```bash
rostopic pub /grasp_target_text std_msgs/String "data: 'bottle'" -1
```

The system should select the highest-confidence target that matches the text prompt.

### Demo 5: Safe No-Grasp for Missing Target

```bash
rostopic pub /grasp_target_text std_msgs/String "data: 'airplane'" -1
```

Expected behavior:

- `target_observation.status` is not `selected`
- Robot Station enters a safe failed/no-grasp state
- The arm does not execute a grasp

### Demo 6: Visualization / Debug Output

```bash
rostopic hz /language_guided_perception/annotated_image
rostopic echo /language_guided_grasp/target_observation
rostopic echo /language_guided_perception/state
```

To save annotated output, add these launch parameters:

```bash
save_annotated_image:=true annotated_image_path:=/tmp/language_guided_ai_station_latest.jpg
```

### Demo 7: Optional Pick-and-Place

If the Robot Station is launched with:

```bash
drop_after_grasp:=true
```

the robot will reuse the existing `sgr_ctrl` put action after a successful pick. For the first live demo, keep `drop_after_grasp:=false` until the grasp motion is stable.

## Common Debug Commands

```bash
rostopic list | grep -E 'grasp_target_text|target_observation|annotated_image|state|image_raw'
rostopic echo /language_guided_grasp/target_observation
rostopic echo /language_guided_grasp/execution_feedback
rostopic echo /language_guided_perception/state
rostopic hz /usb_cam/image_raw
```

If the AI Station does not publish observations, check:

- `/usb_cam/image_raw` is visible and has a rate
- `/grasp_target_text` has been published
- GroundingDINO config and weights paths are correct
- `PYTHONPATH` includes the GroundingDINO source tree
- Use `device:=cpu` first if CUDA is uncertain
