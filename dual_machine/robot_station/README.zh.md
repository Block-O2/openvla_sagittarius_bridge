# Robot Station 实验室电脑使用说明

这台机器负责真实硬件：机械臂、摄像头、坐标映射和抓取执行。

## 这台机器要放什么

- 本仓库 `openvla_sagittarius_bridge`
- Sagittarius ROS / MoveIt / SDK 环境
- `sgr_ctrl.py` action server
- 摄像头驱动，例如 `usb_cam`
- `vision_config.yaml`
- 可以连接机械臂串口和摄像头 USB

不需要：

- GroundingDINO 源码
- GroundingDINO 权重
- CUDA / GPU
- 大模型 Python 环境

## 这台机器负责什么

```text
摄像头 /usb_cam/image_raw  ---> AI Station

AI Station /language_guided_grasp/target_observation
        |
        v
language_guided_executor.py
        |
        v
vision_config.yaml -> sgr_ctrl -> Sagittarius 机械臂
```

输入：

- `/language_guided_grasp/target_observation`：AI Station 发布的 JSON 检测结果

输出：

- `/usb_cam/image_raw`：摄像头图像
- `/language_guided_grasp/execution_feedback`：抓取完成反馈
- `/language_guided_executor/state`：执行侧状态

## 推荐网络设置

推荐让实验室电脑作为 ROS master，因为机械臂和相机都在这台机器上。

假设：

```text
实验室电脑 Robot Station IP: 192.168.1.20
你的电脑 AI Station IP:     192.168.1.10
```

在实验室电脑执行：

```bash
export ROS_MASTER_URI=http://192.168.1.20:11311
export ROS_IP=192.168.1.20
roscore
```

另开终端确认网络：

```bash
ping 192.168.1.10
```

## 启动前环境

```bash
cd ~/sagittarius_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

## 启动 Robot Station

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

如果摄像头已经由别的节点发布，可以关闭本 launch 里的摄像头：

```bash
start_camera:=false
```

## 检查是否正常

```bash
rostopic hz /usb_cam/image_raw
rostopic echo /language_guided_executor/state
rostopic echo /language_guided_grasp/target_observation
rostopic echo /language_guided_grasp/execution_feedback
```

如果机械臂没有动作，先检查：

- Sagittarius demo 是否能单独跑通
- `sgr_ctrl_node` 是否存在
- `/sgr532/sgr_ctrl` action 是否可用
- AI Station 是否正在发布 `target_observation`
- `target_observation` 里的 `status` 是否为 `selected`

## 安全建议

第一次测试建议：

- `drop_after_grasp:=false`
- 桌面只放一个明显目标，例如红色方块
- 先观察 `/language_guided_executor/state`
- 确认 `target_locked` 后再靠近机械臂观察动作

## Git 拉取方式

```bash
git clone -b dual-machine-station-split https://github.com/Block-O2/openvla_sagittarius_bridge.git ~/sagittarius_ws
cd ~/sagittarius_ws
catkin_make
source devel/setup.bash
```

如果实验室电脑已经有仓库：

```bash
git fetch origin
git switch dual-machine-station-split
git pull origin dual-machine-station-split
catkin_make
source devel/setup.bash
```
