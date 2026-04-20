# AI Station 你的电脑使用说明

这台机器负责大模型推理，不直接连接机械臂。

## 这台机器要放什么

- 本仓库 `openvla_sagittarius_bridge`
- ROS Noetic 工作空间环境
- GroundingDINO 源码
- GroundingDINO 权重
- `groundingdino-venv` Python 环境
- 网络能访问实验室电脑的 ROS master

不需要连接：

- Sagittarius 机械臂串口
- 摄像头 USB

## 这台机器负责什么

```text
/usb_cam/image_raw
        |
        v
language_guided_perception.py
        |
        v
/language_guided_grasp/target_observation
```

输入：

- `/usb_cam/image_raw`：实验室电脑发布的摄像头图像
- `/grasp_target_text`：目标文本，例如 `red block`、`banana`、`bottle`
- `/language_guided_grasp/execution_feedback`：实验室电脑执行完成反馈

输出：

- `/language_guided_grasp/target_observation`：JSON 格式的轻量检测结果
- `/language_guided_perception/annotated_image`：标注图
- `/language_guided_perception/state`：AI 推理状态

## 推荐网络设置

假设：

```text
实验室电脑 Robot Station IP: 192.168.1.20
你的电脑 AI Station IP:     192.168.1.10
```

在你的电脑执行：

```bash
export ROS_MASTER_URI=http://192.168.1.20:11311
export ROS_IP=192.168.1.10
```

先确认能连到实验室电脑：

```bash
ping 192.168.1.20
rostopic list
```

## 启动前环境

根据你的当前安装路径，常用命令是：

```bash
cd ~/sagittarius_ws
source /mnt/d/ai_models/groundingdino-venv/bin/activate
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export PYTHONPATH=/mnt/d/ai_models/GroundingDINO:$PYTHONPATH
export MPLCONFIGDIR=/tmp/matplotlib-cfg
mkdir -p "$MPLCONFIGDIR"
```

## 启动 AI Station

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

如果有 NVIDIA GPU 并且环境支持 CUDA，可以把 `device:=cpu` 改成：

```bash
device:=cuda
```

## 发送目标文本

目标文本可以从任意一台连接到同一个 ROS master 的机器发布。

```bash
rostopic pub /grasp_target_text std_msgs/String "data: 'red block'" -1
rostopic pub /grasp_target_text std_msgs/String "data: 'banana'" -1
rostopic pub /grasp_target_text std_msgs/String "data: 'bottle'" -1
```

## 检查是否正常

```bash
rostopic hz /usb_cam/image_raw
rostopic echo /language_guided_perception/state
rostopic echo /language_guided_grasp/target_observation
rostopic hz /language_guided_perception/annotated_image
```

如果 `/usb_cam/image_raw` 没有频率，说明相机图像还没有从实验室电脑过来。

如果 `target_observation` 没有输出，优先检查：

- 是否已经发布 `/grasp_target_text`
- GroundingDINO 路径是否正确
- `PYTHONPATH` 是否包含 GroundingDINO 源码
- AI Station 是否能看到 `/usb_cam/image_raw`

## Git 拉取方式

今天去实验室前，建议使用双机分支：

```bash
git fetch origin
git switch dual-machine-station-split
git pull origin dual-machine-station-split
catkin_make
source devel/setup.bash
```
