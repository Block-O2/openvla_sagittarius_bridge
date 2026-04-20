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

## AI Station 完整启动命令清单

下面这组命令适合你的电脑，也就是负责 GroundingDINO 推理的机器。实验室电脑需要先启动 ROS master，或者至少保证 `ROS_MASTER_URI` 指向实验室电脑。

### 1. 设置网络和工作空间

先复制环境模板并按实际 IP 修改：

```bash
cd ~/sagittarius_ws
cp dual_machine/ai_station/env.example.sh dual_machine/ai_station/env.local.sh
nano dual_machine/ai_station/env.local.sh
```

需要重点确认：

```bash
export ROBOT_STATION_IP=192.168.1.20
export AI_STATION_IP=192.168.1.10
```

然后加载环境：

```bash
source dual_machine/ai_station/env.local.sh
```

### 2. 网络连通性测试

```bash
ping $ROBOT_STATION_IP
rostopic list
```

如果 `rostopic list` 报错，一般是：

- 实验室电脑还没启动 `roscore` 或 robot launch
- `ROS_MASTER_URI` 没指向实验室电脑
- 两台电脑不在同一网络，或者防火墙阻断
- `ROS_IP` 设置成了错误网卡 IP

### 3. 确认能收到相机图像

实验室电脑启动 Robot Station 后，在你的电脑运行：

```bash
rostopic hz /usb_cam/image_raw
```

正常时应该能看到稳定频率，例如 5 Hz、10 Hz 或 30 Hz。没有频率时不要启动正式抓取，先排查相机和 ROS 网络。

### 4. GroundingDINO 静态图片冒烟测试

这个测试不需要机械臂，也不需要实验室电脑，只验证你的电脑上的模型路径、Python 环境和后端推理能不能工作：

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

成功时会看到类似：

```text
source_model: grounding_dino
decision_status: selected
selected_score: ...
selected_center: ...
annotated_output: /tmp/ai_station_smoke_red_block.jpg
```

查看标注图：

```bash
xdg-open /tmp/ai_station_smoke_red_block.jpg
```

### 5. 启动 AI Station 测试模式

推荐先直接用脚本启动：

```bash
source dual_machine/ai_station/env.local.sh
bash dual_machine/ai_station/run_ai_station.sh
```

如果你想临时改参数，也可以手动启动：

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

另开一个 AI Station 终端，观察推理状态：

```bash
source dual_machine/ai_station/env.local.sh
rostopic echo /language_guided_perception/state
```

再开一个终端，观察发给实验室电脑的轻量检测结果：

```bash
source dual_machine/ai_station/env.local.sh
rostopic echo /language_guided_grasp/target_observation
```

### 6. 正式运行命令

正式运行时，AI Station 只需要保持推理节点运行：

```bash
source dual_machine/ai_station/env.local.sh
bash dual_machine/ai_station/run_ai_station.sh
```

然后从任意连接到同一个 ROS master 的终端发送目标文本：

```bash
rostopic pub /grasp_target_text std_msgs/String "data: 'red block'" -1
```

## 新增 Demo 场景

这些 demo 都走同一条链路：文本目标 -> GroundingDINO -> bbox 中心 -> JSON observation -> Robot Station 稳定判断 -> 坐标映射 -> `sgr_ctrl` 抓取。

### Demo 1：抓取 red block

```bash
rostopic pub /grasp_target_text std_msgs/String "data: 'red block'" -1
```

预期现象：

- AI Station 发布 `selected` observation
- 标注图显示红色方块 bbox 和中心点
- Robot Station 连续稳定检测后执行抓取

### Demo 2：抓取 banana

```bash
rostopic pub /grasp_target_text std_msgs/String "data: 'banana'" -1
```

适合展示系统已经不是旧的 HSV 颜色分类，而是类别级语言目标。

### Demo 3：抓取 bottle

```bash
rostopic pub /grasp_target_text std_msgs/String "data: 'bottle'" -1
```

适合展示常见物体类别抓取。

### Demo 4：多物体场景指定目标

桌面同时放多个物体，例如方块、瓶子、香蕉，然后只发布一个目标：

```bash
rostopic pub /grasp_target_text std_msgs/String "data: 'bottle'" -1
```

预期系统选择与文本最匹配且置信度最高的目标，不会按颜色阈值乱抓。

### Demo 5：不存在目标安全不抓

```bash
rostopic pub /grasp_target_text std_msgs/String "data: 'airplane'" -1
```

预期现象：

- `target_observation.status` 不是 `selected`
- Robot Station 状态变成安全失败原因，例如 `no_detection` 或 `low_confidence`
- 机械臂不执行抓取

### Demo 6：查看标注图和状态

```bash
rostopic hz /language_guided_perception/annotated_image
rostopic echo /language_guided_grasp/target_observation
rostopic echo /language_guided_perception/state
```

如果需要保存标注图，启动 AI Station 时增加：

```bash
save_annotated_image:=true annotated_image_path:=/tmp/language_guided_ai_station_latest.jpg
```

### Demo 7：可选 pick-and-place

如果 Robot Station 启动时设置：

```bash
drop_after_grasp:=true
```

抓取成功后会复用原有 `sgr_ctrl` 放置动作，把物体放到固定位置。第一次正式演示建议先保持 `drop_after_grasp:=false`，确认抓取稳定后再打开。

## 常用调试命令

```bash
rostopic list | grep -E 'grasp_target_text|target_observation|annotated_image|state|image_raw'
rostopic echo /language_guided_grasp/target_observation
rostopic echo /language_guided_grasp/execution_feedback
rostopic echo /language_guided_perception/state
rostopic hz /usb_cam/image_raw
```

如果 AI Station 一直没有输出 observation，优先看：

- 是否收到 `/usb_cam/image_raw`
- 是否发布了 `/grasp_target_text`
- GroundingDINO config / weights 路径是否正确
- `PYTHONPATH` 是否包含 GroundingDINO 源码
- `device:=cuda` 是否在当前环境可用；不确定时先用 `device:=cpu`
