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

## Robot Station 完整启动命令清单

下面这组命令适合实验室电脑，也就是连接机械臂和摄像头的机器。推荐实验室电脑作为 ROS master。

### 1. 设置网络和工作空间

先复制环境模板并按实验室实际 IP 修改：

```bash
cd ~/sagittarius_ws
cp dual_machine/robot_station/env.example.sh dual_machine/robot_station/env.local.sh
nano dual_machine/robot_station/env.local.sh
```

需要重点确认：

```bash
export ROBOT_STATION_IP=192.168.1.20
export AI_STATION_IP=192.168.1.10
```

然后加载环境：

```bash
source dual_machine/robot_station/env.local.sh
```

### 2. 网络连通性测试

```bash
hostname -I
ping $AI_STATION_IP
```

如果 AI Station 访问不到 Robot Station，先检查两台电脑是否在同一个局域网，且 `ROS_IP` 是真实网卡 IP，不要用 `127.0.0.1`。

### 3. 硬件识别测试

确认机械臂串口：

```bash
ls -l /dev/sagittarius /dev/ttyACM* /dev/ttyUSB*
```

确认摄像头：

```bash
ls -l /dev/video*
v4l2-ctl --list-formats-ext -d /dev/video0
```

如果摄像头不是 `/dev/video0`，正式启动时把 `video_dev:=/dev/video0` 改成实际设备号。

### 4. 相机单独测试

先只启动相机，确认能发布图像：

```bash
source dual_machine/robot_station/env.local.sh
roslaunch sagittarius_object_color_detector usb_cam.launch \
  video_dev:=/dev/video0 \
  pixel_format:=mjpeg \
  image_width:=640 \
  image_height:=480 \
  framerate:=10
```

另开终端：

```bash
source dual_machine/robot_station/env.local.sh
rostopic hz /usb_cam/image_raw
```

能看到稳定频率后，按 `Ctrl-C` 关闭这个相机单独测试，再启动完整 Robot Station。

如果 `mjpeg` 一直刷解码错误，可以试：

```bash
roslaunch sagittarius_object_color_detector usb_cam.launch \
  video_dev:=/dev/video0 \
  pixel_format:=yuyv \
  image_width:=640 \
  image_height:=480 \
  framerate:=10
```

以 `rostopic hz /usb_cam/image_raw` 有稳定输出为准。

### 5. 机械臂单独测试

正式双机前，先确认实验室电脑上的原始 Sagittarius demo 或颜色分类 demo 仍然能跑。这个步骤是为了确认问题不在串口、供电或 SDK。

常用检查命令：

```bash
rosnode list | grep -E 'sdk_sagittarius_arm|move_group|sgr_ctrl'
rostopic echo -n 1 /sgr532/joint_states
```

如果之前实验室电脑可以跑颜色分类，建议先保持那套流程可运行；双机方案只是把“感知推理”移到 AI Station，不改机械臂底层执行链路。

### 6. 启动 Robot Station 测试模式

推荐使用脚本：

```bash
source dual_machine/robot_station/env.local.sh
bash dual_machine/robot_station/run_robot_station.sh
```

或者手动启动：

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

启动后另开终端观察：

```bash
source dual_machine/robot_station/env.local.sh
rostopic echo /language_guided_executor/state
```

### 7. 安全 no-grasp 通信测试

这个测试只验证 Robot Station 能收到 observation topic，且不会触发机械臂抓取：

```bash
rostopic pub /language_guided_grasp/target_observation std_msgs/String \
  'data: "{\"status\":\"no_detection\",\"target_text\":\"test\",\"selected\":null}"' -1
```

预期：

- `/language_guided_executor/state` 显示 `failed: no_detection`
- 机械臂不动
- 这说明“无目标不抓取”的安全路径是通的

### 8. 正式运行命令

正式运行时，Robot Station 保持以下命令运行：

```bash
source dual_machine/robot_station/env.local.sh
bash dual_machine/robot_station/run_robot_station.sh
```

AI Station 启动后，在任意一台 ROS 网络内的机器发送目标：

```bash
rostopic pub /grasp_target_text std_msgs/String "data: 'red block'" -1
```

## 新增 Demo 场景

这些 demo 由 AI Station 负责语言检测，Robot Station 负责稳定判断、坐标映射和机械臂执行。

### Demo 1：抓取 red block

```bash
rostopic pub /grasp_target_text std_msgs/String "data: 'red block'" -1
```

观察：

```bash
rostopic echo /language_guided_executor/state
rostopic echo /language_guided_grasp/target_observation
```

预期：检测稳定后进入 `target_locked`，随后进入 `grasping`。

### Demo 2：抓取 banana

```bash
rostopic pub /grasp_target_text std_msgs/String "data: 'banana'" -1
```

这是类别级目标抓取，不依赖 HSV 颜色阈值。

### Demo 3：抓取 bottle

```bash
rostopic pub /grasp_target_text std_msgs/String "data: 'bottle'" -1
```

适合演示多种日常物体目标。

### Demo 4：多物体场景指定目标

桌面放多个物体，只发布一个目标：

```bash
rostopic pub /grasp_target_text std_msgs/String "data: 'bottle'" -1
```

预期：Robot Station 只执行 AI Station 选中的最高置信度目标。

### Demo 5：不存在目标安全不抓

```bash
rostopic pub /grasp_target_text std_msgs/String "data: 'airplane'" -1
```

预期：

- Robot Station 状态显示 `failed: no_detection` 或 `failed: low_confidence`
- 机械臂不执行抓取

### Demo 6：标注图辅助演示

标注图由 AI Station 发布，但 Robot Station 可以检查 topic 是否存在：

```bash
rostopic list | grep annotated_image
rostopic hz /language_guided_perception/annotated_image
```

如果演示时需要投屏，建议在 AI Station 上打开标注图或 echo observation，Robot Station 终端只保留机械臂状态。

### Demo 7：可选 pick-and-place

确认抓取稳定后，可以启用放置动作：

```bash
roslaunch sagittarius_object_color_detector language_guided_robot_station.launch \
  robot_name:=sgr532 \
  video_dev:=/dev/video0 \
  pixel_format:=mjpeg \
  drop_after_grasp:=true
```

这会在抓取成功后复用原有 put action，把物体放到固定位置。第一次实机演示建议先用 `drop_after_grasp:=false`。

## 常用调试命令

```bash
rostopic list | grep -E 'image_raw|target_observation|execution_feedback|executor|sgr_ctrl'
rostopic hz /usb_cam/image_raw
rostopic echo /language_guided_executor/state
rostopic echo /language_guided_grasp/target_observation
rostopic echo /language_guided_grasp/execution_feedback
rosnode list | grep -E 'sdk_sagittarius_arm|move_group|sgr_ctrl|language_guided_executor|usb_cam'
```

如果机械臂不动，按这个顺序排查：

1. 原始 Sagittarius demo 是否能跑
2. `/dev/sagittarius` 是否存在
3. `/usb_cam/image_raw` 是否有频率
4. AI Station 是否发布 `target_observation`
5. `target_observation.status` 是否为 `selected`
6. `/language_guided_executor/state` 是否卡在 `failed`、`waiting_for_observation` 或 `target_locked`

如果机械臂一直哔哔响或 SDK 反复重启，先不要继续语言抓取，回到原始 demo 验证供电、串口映射、机械臂连接和实验室电脑本地环境。
