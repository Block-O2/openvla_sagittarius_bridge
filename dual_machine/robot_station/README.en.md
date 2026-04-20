# Robot Station Guide

This machine owns the real hardware: Sagittarius arm, camera, coordinate mapping, and grasp execution.

## What This Machine Needs

- This repository: `openvla_sagittarius_bridge`
- Sagittarius ROS / MoveIt / SDK environment
- `sgr_ctrl.py` action server
- Camera driver, for example `usb_cam`
- `vision_config.yaml`
- USB/serial access to the robot arm and camera

This machine does not need:

- GroundingDINO source code
- GroundingDINO model weights
- CUDA / GPU
- Large-model Python environment

## Responsibility

```text
Camera /usb_cam/image_raw  ---> AI Station

AI Station /language_guided_grasp/target_observation
        |
        v
language_guided_executor.py
        |
        v
vision_config.yaml -> sgr_ctrl -> Sagittarius arm
```

Inputs:

- `/language_guided_grasp/target_observation`: JSON detection result from the AI Station

Outputs:

- `/usb_cam/image_raw`: camera frames
- `/language_guided_grasp/execution_feedback`: execution feedback
- `/language_guided_executor/state`: executor state

## Network Setup

The Robot Station should usually host the ROS master because it owns both the camera and robot arm.

Example:

```text
Robot Station IP: 192.168.1.20
AI Station IP:    192.168.1.10
```

On the Robot Station:

```bash
export ROS_MASTER_URI=http://192.168.1.20:11311
export ROS_IP=192.168.1.20
roscore
```

In another terminal:

```bash
ping 192.168.1.10
```

## Environment

```bash
cd ~/sagittarius_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

## Start Robot Station

```bash
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
```

If another node already publishes the camera topic:

```bash
start_camera:=false
```

## Checks

```bash
rostopic hz /usb_cam/image_raw
rostopic echo /language_guided_executor/state
rostopic echo /language_guided_grasp/target_observation
rostopic echo /language_guided_grasp/execution_feedback
```

If the robot does not move, check:

- Sagittarius demo works on its own
- `sgr_ctrl_node` exists
- `/sgr532/sgr_ctrl` action is available
- AI Station publishes `target_observation`
- `target_observation.status` is `selected`

## Safety Notes

For the first test:

- Use `drop_after_grasp:=false`
- Put only one obvious object on the table, such as a red block
- Watch `/language_guided_executor/state`
- Observe the arm only after `target_locked`

## Git Setup

```bash
git clone -b dual-machine-station-split https://github.com/Block-O2/openvla_sagittarius_bridge.git ~/sagittarius_ws
cd ~/sagittarius_ws
catkin_make
source devel/setup.bash
```

If the repository already exists:

```bash
git fetch origin
git switch dual-machine-station-split
git pull origin dual-machine-station-split
catkin_make
source devel/setup.bash
```
