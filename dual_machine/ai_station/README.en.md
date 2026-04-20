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
