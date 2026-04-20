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

## Complete Robot Station Command Checklist

Use this checklist on the lab machine that connects to the Sagittarius arm and USB camera. The Robot Station should normally host the ROS master.

### 1. Configure Network and Workspace

Copy the environment template and edit the IP addresses:

```bash
cd ~/sagittarius_ws
cp dual_machine/robot_station/env.example.sh dual_machine/robot_station/env.local.sh
nano dual_machine/robot_station/env.local.sh
```

Confirm these values:

```bash
export ROBOT_STATION_IP=192.168.1.20
export AI_STATION_IP=192.168.1.10
```

Load the environment:

```bash
source dual_machine/robot_station/env.local.sh
```

### 2. Network Connectivity Test

```bash
hostname -I
ping $AI_STATION_IP
```

If the AI Station cannot reach the Robot Station, check that both machines are on the same network and that `ROS_IP` is not `127.0.0.1`.

### 3. Hardware Detection Test

Check the robot serial device:

```bash
ls -l /dev/sagittarius /dev/ttyACM* /dev/ttyUSB*
```

Check the camera:

```bash
ls -l /dev/video*
v4l2-ctl --list-formats-ext -d /dev/video0
```

If the camera is not `/dev/video0`, update `video_dev:=/dev/video0` in the launch command.

### 4. Camera-Only Test

Start only the camera first:

```bash
source dual_machine/robot_station/env.local.sh
roslaunch sagittarius_object_color_detector usb_cam.launch \
  video_dev:=/dev/video0 \
  pixel_format:=mjpeg \
  image_width:=640 \
  image_height:=480 \
  framerate:=10
```

In another terminal:

```bash
source dual_machine/robot_station/env.local.sh
rostopic hz /usb_cam/image_raw
```

After confirming a stable rate, stop this camera-only launch with `Ctrl-C` and start the full Robot Station launch.

If `mjpeg` repeatedly prints decode errors, try:

```bash
roslaunch sagittarius_object_color_detector usb_cam.launch \
  video_dev:=/dev/video0 \
  pixel_format:=yuyv \
  image_width:=640 \
  image_height:=480 \
  framerate:=10
```

Use whichever setting gives a stable `/usb_cam/image_raw` rate.

### 5. Robot-Only Test

Before the full dual-machine demo, make sure the existing Sagittarius demo or color-classification demo still works on the lab machine. This confirms that serial mapping, power, SDK, and MoveIt are healthy.

Useful checks:

```bash
rosnode list | grep -E 'sdk_sagittarius_arm|move_group|sgr_ctrl'
rostopic echo -n 1 /sgr532/joint_states
```

If the lab machine can already run the original color-classification demo, keep that baseline working. The dual-machine setup only moves perception inference to the AI Station; it does not replace the robot execution stack.

### 6. Start Robot Station Test Mode

Recommended script:

```bash
source dual_machine/robot_station/env.local.sh
bash dual_machine/robot_station/run_robot_station.sh
```

Manual launch command:

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
  stable_required:=5 \
  center_tolerance:=8.0 \
  min_grasp_score:=0.35 \
  drop_after_grasp:=false
```

In another terminal:

```bash
source dual_machine/robot_station/env.local.sh
rostopic echo /language_guided_executor/state
```

### 7. Safe No-Grasp Communication Test

This checks that the Robot Station receives the observation topic without triggering a grasp:

```bash
rostopic pub /language_guided_grasp/target_observation std_msgs/String \
  'data: "{\"status\":\"no_detection\",\"target_text\":\"test\",\"selected\":null}"' -1
```

Expected behavior:

- `/language_guided_executor/state` shows `failed: no_detection`
- The arm does not move
- The safe no-grasp path is working

### 8. Official Run Command

For the full dual-machine demo, keep this running on the Robot Station:

```bash
source dual_machine/robot_station/env.local.sh
bash dual_machine/robot_station/run_robot_station.sh
```

After the AI Station starts, publish a target from any terminal on the ROS network:

```bash
rostopic pub /grasp_target_text std_msgs/String "data: 'red block'" -1
```

## New Demo Scenarios

The AI Station performs language grounding. The Robot Station performs stability checks, coordinate mapping, and robot execution.

### Demo 1: Grasp `red block`

```bash
rostopic pub /grasp_target_text std_msgs/String "data: 'red block'" -1
```

Watch:

```bash
rostopic echo /language_guided_executor/state
rostopic echo /language_guided_grasp/target_observation
```

Expected behavior: after stable detection, the state enters `target_locked`, then `grasping`.

### Demo 2: Grasp `banana`

```bash
rostopic pub /grasp_target_text std_msgs/String "data: 'banana'" -1
```

This demonstrates category-level grasping, not HSV color thresholding.

### Demo 3: Grasp `bottle`

```bash
rostopic pub /grasp_target_text std_msgs/String "data: 'bottle'" -1
```

Useful for everyday-object demos.

### Demo 4: Multi-Object Target Selection

Place several objects on the table and request only one:

```bash
rostopic pub /grasp_target_text std_msgs/String "data: 'bottle'" -1
```

Expected behavior: the Robot Station executes only the target selected by the AI Station.

### Demo 5: Safe No-Grasp for Missing Target

```bash
rostopic pub /grasp_target_text std_msgs/String "data: 'airplane'" -1
```

Expected behavior:

- Robot Station state shows `failed: no_detection` or `failed: low_confidence`
- The arm does not execute

### Demo 6: Visualization Support

Annotated images are published by the AI Station, but the Robot Station can confirm the topic:

```bash
rostopic list | grep annotated_image
rostopic hz /language_guided_perception/annotated_image
```

For presentations, show the annotated image or observation output on the AI Station and keep the Robot Station terminal focused on robot state.

### Demo 7: Optional Pick-and-Place

After grasp-only behavior is stable, enable drop-after-grasp:

```bash
roslaunch sagittarius_object_color_detector language_guided_robot_station.launch \
  robot_name:=sgr532 \
  video_dev:=/dev/video0 \
  pixel_format:=mjpeg \
  drop_after_grasp:=true
```

This reuses the existing `sgr_ctrl` put action after a successful pick. For the first live demo, keep `drop_after_grasp:=false`.

## Common Debug Commands

```bash
rostopic list | grep -E 'image_raw|target_observation|execution_feedback|executor|sgr_ctrl'
rostopic hz /usb_cam/image_raw
rostopic echo /language_guided_executor/state
rostopic echo /language_guided_grasp/target_observation
rostopic echo /language_guided_grasp/execution_feedback
rosnode list | grep -E 'sdk_sagittarius_arm|move_group|sgr_ctrl|language_guided_executor|usb_cam'
```

If the arm does not move, check in this order:

1. The original Sagittarius demo works
2. `/dev/sagittarius` exists
3. `/usb_cam/image_raw` has a stable rate
4. AI Station publishes `target_observation`
5. `target_observation.status` is `selected`
6. `/language_guided_executor/state` is not stuck at `failed`, `waiting_for_observation`, or `target_locked`

If the arm keeps beeping or the SDK repeatedly restarts, stop the language-guided demo and verify power, serial mapping, robot connection, and the lab machine baseline demo first.
