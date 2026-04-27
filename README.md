# Sagittarius Semantic Grasp

基于 Sagittarius 机械臂、ROS1、MoveIt、GroundingDINO 与相机标定的语言引导语义抓取 / 放置项目。

本项目的目标不是单纯做一个“颜色块识别 demo”，而是把原始 Sagittarius 机械臂的 HSV 颜色抓取例程改造成一个更接近真实机器人应用的语义抓取系统：用户用自然语言指定目标，视觉模型在相机图像中识别目标，系统将像素位置映射到机械臂工作平面坐标，并调用原有 MoveIt / `sgr_ctrl` 执行抓取与放置。

---

## 1. 项目概览

当前系统的主流程是：

```text
自然语言任务
  -> 任务解析，拆成 pick/place 目标
  -> 相机采图
  -> GroundingDINO 语义检测
  -> HSV / 轮廓中心精修
  -> 像素中心到机械臂平面坐标映射
  -> Sagittarius MoveIt / sgr_ctrl 执行抓取或放置
```

典型输入可以是：

```text
red block
```

也可以是更完整的任务级命令：

```text
put each block into the bucket of the same color
```

当前系统会将后者拆成多步任务，例如：

```text
1. red block  -> red bucket
2. blue block -> blue bucket
3. green block -> green bucket
```

每一步包含两个阶段：

```text
pick 阶段：识别并抓取 block
place 阶段：重新观察，识别 bucket，并把物体放到 bucket 附近 / 上方
```

---

## 2. 当前 main 分支状态

当前 `main` 分支已经合入了多阶段 pick/place、前视角九点标定、HSV 中心精修和实机测试相关内容。

### 已经实现并合入 main 的能力

* 使用 GroundingDINO 作为语言条件目标检测后端。
* 支持通过 `/grasp_target_text` 话题输入自然语言目标。
* 支持单目标抓取，例如 `red block`、`blue block`。
* 支持简单任务拆解，例如同色分类任务：`red block -> red bucket`。
* 支持 pick/place 两阶段流程。
* 支持 `pick_front` 与 `place_front` 两套独立前视角标定。
* 支持 3×3 九点手动标定数据。
* 支持在 GroundingDINO 检测框内部使用 HSV / 轮廓方法精修目标中心。
* 支持稳定帧锁定，避免瞬时误检直接触发机械臂。
* 支持低置信度、目标未检测到、文本为空等情况下安全拒绝执行。
* 支持输出 annotated image，用于观察模型选中了哪里。
* 支持保存原始图像和标注图，便于调试。
* 保留原有 Sagittarius 机械臂执行链路，不重写底层运动控制。

### 最近一次实机测试结果

在前视角、九点标定与 HSV 中心精修版本中，系统完成了以下实机流程：

```text
任务：put each block into the bucket of the same color

1. red block  -> red bucket   成功抓取并放置
2. blue block -> blue bucket  成功抓取并放置
3. green block -> green bucket
   - green block 抓取成功
   - green bucket 在放置阶段未检测到，系统安全失败，没有继续盲目执行
```

这个结果说明：

* 前视角语义抓取链路已经可以完成连续多步任务。
* GroundingDINO + HSV 中心精修比单纯检测框中心更适合抓取落点。
* `pick_front` / `place_front` 分离标定是有效的。
* 放置阶段仍然更依赖目标可见性，bucket 没有进入清晰视野时会失败。

---

## 3. 硬件与软件环境

### 机械臂硬件

当前项目围绕 Sagittarius 机械臂运行，ROS namespace 使用：

```text
/sgr532
```

相关链路包括：

* Sagittarius 机械臂本体
* Sagittarius MoveIt 配置
* Sagittarius 底层 SDK / `sgr_ctrl`
* 机械臂串口设备，通常映射为：

```text
/dev/ttyACM0
/dev/sagittarius -> /dev/ttyACM0
```

如果使用 WSL2，机械臂 USB 串口通常需要通过 Windows `usbipd` 透传。

### 相机硬件

当前测试使用 UVC 相机，通过 ROS `usb_cam` 发布图像，默认设备为：

```text
/dev/video0
```

默认图像参数：

```text
image_width  = 640
image_height = 480
framerate    = 10
pixel_format = mjpeg
```

### 计算平台

当前项目主要在单机 GPU 模式下调试：

* 操作系统：Ubuntu 20.04 / WSL2 Ubuntu 20.04
* ROS：ROS Noetic
* GPU：NVIDIA RTX 4060 级别显卡
* 推理后端：GroundingDINO，推荐使用 CUDA

推荐正式测试时使用：

```text
device:=cuda
```

不建议在正式实机抓取时使用 CPU 推理，因为 GroundingDINO 在 CPU 上速度较慢，容易影响闭环体验。

---

## 4. 仓库结构

主要代码位于：

```text
src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/
```

核心结构如下：

```text
sagittarius_object_color_detector/
├── action/
│   └── SGRCtrl.action
├── config/
│   ├── vision_config.yaml
│   ├── vision_config_front.yaml
│   ├── vision_config_pick_front.yaml
│   ├── vision_config_place_front.yaml
│   ├── vision_config_left.yaml
│   ├── vision_config_right.yaml
│   ├── manual_calibration_points.example.csv
│   ├── manual_calibration_points_front.csv
│   ├── manual_calibration_points_pick_front.csv
│   ├── manual_calibration_points_place_front.csv
│   ├── manual_calibration_points_left.csv
│   └── manual_calibration_points_right.csv
├── launch/
│   ├── language_guided_grasp.launch
│   ├── language_guided_grasp_image_test.launch
│   ├── language_guided_calibration_base.launch
│   ├── usb_cam.launch
│   ├── color_classification.launch
│   └── color_classification_image_test.launch
├── nodes/
│   ├── language_guided_grasp.py
│   ├── language_guided_calibration.py
│   ├── manual_vision_calibration.py
│   ├── publish_test_image.py
│   ├── sgr_ctrl.py
│   └── perception_framework/
│       ├── detection_types.py
│       ├── backend_factory.py
│       ├── decision.py
│       ├── selection.py
│       ├── stability.py
│       ├── coordinate_mapping.py
│       ├── execution.py
│       ├── center_refinement.py
│       ├── task_parsing.py
│       ├── visualization.py
│       └── backends/
│           ├── base.py
│           └── grounding_dino.py
├── scripts/
│   ├── ensure_sagittarius_serial.sh
│   ├── run_single_machine_gpu_grasp.sh
│   └── run_manual_vision_calibration.sh
├── test_data/
│   └── sample_red_block.ppm
├── CMakeLists.txt
└── package.xml
```

### 重要文件说明

| 文件                                | 作用                                       |
| --------------------------------- | ---------------------------------------- |
| `language_guided_grasp.py`        | 主节点，负责任务解析、检测、稳定判断、坐标映射、抓取与放置调度          |
| `language_guided_calibration.py`  | 语言引导标定节点，可在不同观察位下采集标定点                   |
| `manual_vision_calibration.py`    | 根据手动采集的像素点和机器人坐标拟合 `vision_config*.yaml` |
| `center_refinement.py`            | 在检测框内部用 HSV / 轮廓方法精修目标中心                 |
| `task_parsing.py`                 | 将自然语言任务拆解为 pick/place 步骤                 |
| `coordinate_mapping.py`           | 读取 `vision_config*.yaml`，执行像素到机械臂平面的线性映射 |
| `execution.py`                    | 对原有 `SGRCtrlAction` 的轻量封装                |
| `grounding_dino.py`               | GroundingDINO 后端                         |
| `run_single_machine_gpu_grasp.sh` | 单机 GPU 实机测试启动脚本                          |
| `ensure_sagittarius_serial.sh`    | 检查和修复 Sagittarius 串口映射                   |

---

## 5. 系统架构

系统分为五层：

```text
输入层
  - 相机图像 /usb_cam/image_raw
  - 文本任务 /grasp_target_text

感知层
  - BasePerceptionBackend
  - GroundingDinoBackend
  - HSV / contour center refinement

决策层
  - 目标选择
  - 置信度过滤
  - 连续帧稳定检测
  - 任务状态机

映射层
  - VisionPlaneMapper
  - vision_config_pick_front.yaml
  - vision_config_place_front.yaml
  - vision_config_left.yaml / right.yaml

执行层
  - SagittariusGraspExecutor
  - SGRCtrlAction
  - MoveIt
  - Sagittarius SDK
```

主节点维护的状态流大致为：

```text
waiting_for_target
  -> detecting
  -> target_locked
  -> grasping
  -> placing
  -> done
```

失败时进入：

```text
failed
```

常见失败原因包括：

* 目标文本为空
* 没有检测到目标
* 检测置信度过低
* 目标中心不稳定
* MoveIt 规划失败
* 放置目标不可见
* 机械臂执行失败

状态会发布到：

```text
/language_guided_grasp/state
```

查看状态：

```bash
rostopic echo /language_guided_grasp/state
```

---

## 6. 坐标映射与标定

### 6.1 为什么需要标定

视觉模型输出的是图像像素位置，例如：

```text
pixel_x, pixel_y
```

机械臂执行需要的是工作平面坐标，例如：

```text
robot_x, robot_y, robot_z
```

所以必须通过标定建立映射关系。

当前系统沿用原 Sagittarius 例程中的线性映射形式：

```text
robot_x = k1 * pixel_y + b1
robot_y = k2 * pixel_x + b2
```

对应 YAML 文件中的参数：

```yaml
LinearRegression:
  k1: ...
  b1: ...
  k2: ...
  b2: ...
```

### 6.2 当前可用标定文件

当前 main 分支中建议使用以下标定文件：

| 文件                               | 用途                     |
| -------------------------------- | ---------------------- |
| `vision_config_pick_front.yaml`  | 抓取阶段前视角标定              |
| `vision_config_place_front.yaml` | 放置阶段前视角标定              |
| `vision_config_left.yaml`        | 左观察位标定，框架已支持，需根据现场实测确认 |
| `vision_config_right.yaml`       | 右观察位标定，框架已支持，需根据现场实测确认 |
| `vision_config_front.yaml`       | 通用前视角配置，兼容保留           |
| `vision_config.yaml`             | 原始兼容入口，不建议作为最新主流程唯一配置  |

当前最可靠的实机结果来自：

```text
vision_config_pick_front.yaml
vision_config_place_front.yaml
```

### 6.3 当前九点标定数据

当前 main 分支中包含两套关键九点标定数据：

```text
manual_calibration_points_pick_front.csv
manual_calibration_points_place_front.csv
```

它们都是 3×3 网格标定，机器人平面点覆盖：

```text
x = 0.22, 0.24, 0.26
y = -0.03, 0.00, 0.03
```

这两套数据分别用于：

* 抓取阶段：`pick_front`
* 放置阶段：`place_front`

注意：抓取和放置时机械臂姿态、相机视角、目标类型可能不同，所以不要随便把 `pick_front` 的标定直接用于 `place_front`。

### 6.4 手动标定流程

推荐标定流程：

1. 让机械臂移动到固定观察位。
2. 在桌面上放置标定物，例如蓝色方块或桶。
3. 用 GroundingDINO 检测目标。
4. 在检测框内部使用 HSV / 轮廓方法得到更稳定的中心点。
5. 记录：

```text
pixel_x, pixel_y, robot_x, robot_y, score, label
```

6. 保存为 `manual_calibration_points_*.csv`。
7. 使用手动标定脚本拟合 `vision_config_*.yaml`。

典型标定数据格式：

```csv
pixel_x,pixel_y,robot_x,robot_y,score,label
383.7,376.9,0.22,-0.03,0.95,blue block
...
```

推荐至少使用 5 个点，当前主流程使用 9 个点更稳定。

### 6.5 标定注意事项

* 标定点不要全部集中在图像中心。
* 标定区域应覆盖实际抓取 / 放置常用区域。
* `pick_front` 应覆盖 block 常出现的区域。
* `place_front` 应覆盖 bucket 常出现的区域。
* 如果相机位置、机械臂观察位姿、分辨率发生变化，需要重新标定。
* 如果从 WSL2 切换到原生 Ubuntu，理论上标定参数可以保留，但只要相机物理安装位置变化，就必须重标。
* 如果发现模型框选正确但机械臂落点偏，优先检查标定，而不是优先怀疑 GroundingDINO。

---

## 7. 可用测试方式

本项目建议按从安全到危险的顺序测试，不要一上来就执行真实抓取。

### 7.1 静态图片测试

用途：验证 GroundingDINO 后端、任务文本、检测结果和可视化，不需要真实机械臂执行。

适合场景：

* 没有接机械臂
* 相机不稳定
* 只想验证模型是否能识别目标
* 想调试文本 prompt、置信度阈值、标注图输出

相关入口：

```text
language_guided_grasp_image_test.launch
publish_test_image.py
```

### 7.2 相机实时检测但不执行

用途：验证真实相机画面、GroundingDINO 检测、中心精修和坐标映射，但不让机械臂动作。

建议设置：

```text
execute_grasp:=false
```

这是实机前最重要的一步。你应该确认：

* annotated image 中目标框正确
* refined center 落在目标几何中心附近
* 日志中的 mapped x/y 合理
* 目标移动时 mapped x/y 会跟着变化

### 7.3 单目标真实抓取测试

用途：只测试一个目标的完整抓取链路。

建议输入：

```text
red block
blue block
green block
```

建议先关闭放置：

```text
drop_after_grasp:=false
```

确认抓取成功后，再开启 place 相关逻辑。

### 7.4 两阶段 pick/place 测试

用途：测试抓取后重新寻找放置目标。

示例命令：

```text
pick red block and place it into red bucket
```

或者：

```text
put red block into red bucket
```

推荐先只使用前视角：

```text
place_scan_view_order:=front
```

当前前视角是已验证最充分的模式。

### 7.5 同色分类多步任务测试

用途：测试任务拆解、多步连续执行和失败保护。

示例命令：

```text
put each block into the bucket of the same color
```

当前系统会拆成：

```text
red block -> red bucket
blue block -> blue bucket
green block -> green bucket
```

最近实机结果显示：红色和蓝色完整成功，绿色抓取成功，但绿色桶未检测到，因此放置阶段安全失败。

---

## 8. 推荐启动方式

### 8.1 环境准备

进入工作空间：

```bash
cd ~/sagittarius_ws
```

加载 ROS 环境：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

如果使用 GroundingDINO 虚拟环境：

```bash
source /mnt/d/ai_models/groundingdino-venv/bin/activate
export PYTHONPATH=/mnt/d/ai_models/GroundingDINO:$PYTHONPATH
export MPLCONFIGDIR=/tmp/matplotlib-cfg
mkdir -p "$MPLCONFIGDIR"
```

### 8.2 检查机械臂串口

先运行：

```bash
bash src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/scripts/ensure_sagittarius_serial.sh
```

正常情况下应该看到：

```text
/dev/ttyACM0
/dev/sagittarius -> /dev/ttyACM0
```

如果在 WSL2 中使用机械臂，需要先在 Windows PowerShell 中通过 `usbipd` attach 机械臂 USB 设备。

### 8.3 推荐单机 GPU 测试脚本

当前推荐入口：

```bash
bash src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/scripts/run_single_machine_gpu_grasp.sh
```

该脚本适合单机模式：

```text
同一台电脑连接机械臂 + 相机 + 运行 GroundingDINO
```

第一次运行建议保持：

```text
EXECUTE_GRASP=false
```

确认检测、中心点和坐标映射都合理后，再改为：

```text
EXECUTE_GRASP=true
```

### 8.4 直接 roslaunch 示例

可以直接启动主 launch：

```bash
roslaunch sagittarius_object_color_detector language_guided_grasp.launch \
  device:=cuda \
  video_dev:=/dev/video0 \
  pixel_format:=mjpeg \
  image_width:=640 \
  image_height:=480 \
  framerate:=10 \
  vision_config:=$(rospack find sagittarius_object_color_detector)/config/vision_config_pick_front.yaml \
  place_front_view_vision_config:=$(rospack find sagittarius_object_color_detector)/config/vision_config_place_front.yaml \
  execute_grasp:=false
```

确认无误后再把：

```text
execute_grasp:=true
```

### 8.5 发送目标文本

新开一个终端，加载环境后发送目标：

```bash
rostopic pub -1 /grasp_target_text std_msgs/String "data: 'red block'"
```

任务级命令示例：

```bash
rostopic pub -1 /grasp_target_text std_msgs/String "data: 'put each block into the bucket of the same color'"
```

---

## 9. 常用参数说明

### 感知相关参数

| 参数                 | 含义                 | 推荐值       |
| ------------------ | ------------------ | --------- |
| `device`           | GroundingDINO 推理设备 | `cuda`    |
| `box_threshold`    | 检测框阈值              | `0.35` 起调 |
| `text_threshold`   | 文本匹配阈值             | `0.25` 起调 |
| `min_grasp_score`  | 执行前最低置信度           | `0.35` 起调 |
| `stable_required`  | 单目标抓取稳定帧数          | `5`       |
| `center_tolerance` | 中心点稳定容差，像素单位       | `8.0`     |

### 执行相关参数

| 参数                                  | 含义          | 建议                     |
| ----------------------------------- | ----------- | ---------------------- |
| `execute_grasp`                     | 是否真实执行机械臂动作 | 调试先 `false`，确认后 `true` |
| `pick_z`                            | 抓取高度        | 根据桌面和物体高度调整            |
| `drop_after_grasp`                  | 抓取后是否执行放置   | 单目标测试先 `false`         |
| `dynamic_place_z`                   | 动态放置高度      | 根据桶口高度设置               |
| `return_to_search_pose_after_grasp` | 抓取后是否回观察位   | 当前通常建议 `false`         |

### 观察位相关参数

| 参数                                      | 含义          |
| --------------------------------------- | ----------- |
| `search_pose_x/y/z/roll/pitch/yaw`      | 抓取前观察位姿     |
| `place_front_view_x/y/z/roll/pitch/yaw` | 放置阶段前视角观察位姿 |
| `left_view_x/y/z/roll/pitch/yaw`        | 左侧观察位姿      |
| `right_view_x/y/z/roll/pitch/yaw`       | 右侧观察位姿      |
| `place_scan_view_order`                 | 放置阶段扫描顺序    |

当前最推荐、验证最充分的是：

```text
place_scan_view_order:=front
```

left/right 多视角框架已经存在，但需要根据现场相机视野和机械臂可达空间继续实测和标定。

---

## 10. 调试输出

### 状态话题

```bash
rostopic echo /language_guided_grasp/state
```

### 标注图话题

```text
/language_guided_grasp/annotated_image
```

### 常见保存图片

```text
/tmp/language_guided_grasp_raw_single_gpu.jpg
/tmp/language_guided_grasp_single_gpu.jpg
/tmp/language_guided_grasp_latest.jpg
```

含义：

* `raw`：原始相机画面
* `single_gpu` / `latest`：带检测框、置信度、中心点和状态的标注图

如果机械臂抓偏，优先检查：

1. 原始图中目标是否清晰可见。
2. 标注图中检测框是否正确。
3. refined center 是否落在目标中心附近。
4. 日志中的 pixel center 是否合理。
5. mapped x/y 是否符合桌面实际位置。
6. 当前使用的 `vision_config` 是否对应当前观察位。

---

## 11. 常见问题

### 11.1 GroundingDINO 检测到了目标，但机械臂抓偏

优先检查标定。

如果检测框正确、中心点正确，但机械臂落点偏，问题通常不是模型，而是：

* 使用了错误的 `vision_config*.yaml`
* 相机位置变化但没有重新标定
* 观察位姿变化但仍使用旧标定
* 像素中心和标定时的中心定义不一致

当前版本已经尽量让标定和运行时都使用 refined center，但如果你手动改过中心计算逻辑，需要重新标定。

### 11.2 MoveIt 规划成功但不执行

如果出现：

```text
Computed path is not valid
Start state appears to be in collision
Found a contact between link1 and link5
```

说明 MoveIt 认为当前状态或目标路径存在自碰撞。常见原因：

* 初始姿态不合适
* 观察位姿太低或太折叠
* joint state 与真实机械臂不一致
* 某些关节接近限位

优先在 RViz 中查看机器人模型是否一开始就处于碰撞状态。不要直接禁用碰撞检测，除非确认只是碰撞模型误判。

### 11.3 WSL 中找不到机械臂串口

先检查：

```bash
lsusb
ls -l /dev/ttyACM* /dev/sagittarius
```

再运行：

```bash
bash src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/scripts/ensure_sagittarius_serial.sh
```

如果使用 WSL2，需要在 Windows PowerShell 中确认 `usbipd list` 里机械臂已 attach。

### 11.4 相机和机械臂同时连接不稳定

这是 WSL2 + USB 透传环境下的常见问题。建议：

* 正式演示优先使用原生 Ubuntu。
* 或者使用更稳定的 UVC 摄像头。
* 或者把大模型推理和机械臂控制拆到两台机器上，通过 ROS 通信。

---

## 12. 当前限制

当前系统还不是完整工业级抓取系统，主要限制包括：

* 当前最稳定的是前视角 `front-only` 模式。
* left/right 多观察位框架已经支持，但仍需要更多实机标定和验证。
* 目前像素到机械臂坐标使用线性平面映射，适合桌面平面任务，不适合复杂 3D 场景。
* 当前没有深度相机，无法直接估计目标高度和完整 6D pose。
* 放置阶段对 bucket 的可见性比较敏感。
* GroundingDINO 对 prompt、光照、遮挡和目标外观仍然敏感。
* WSL2 下 USB 相机和串口透传可能不稳定。
* 目前任务解析是轻量规则式拆解，不是完整 LLM planner。

---

## 13. 未来方向

### 13.1 感知方向

* 接入 YOLO-World、OWL-ViT、Grounded-SAM 等可替换后端。
* 引入 segmentation mask，而不是只依赖 bounding box。
* 将 HSV center refinement 升级为更通用的 mask center / grasp point estimation。
* 增加目标跟踪，减少每帧重复检测带来的抖动。

### 13.2 标定与空间理解

* 完成 left/right 观察位的实机九点标定。
* 增加更多桌面点，评估不同区域误差。
* 从线性平面映射升级到相机外参标定。
* 引入深度相机或 AprilTag 标定板。
* 支持更稳定的 hand-eye calibration。

### 13.3 任务规划

* 将当前规则式 `task_parsing.py` 升级为更通用的语言任务解析器。
* 支持更多任务形式，例如：

```text
put the red block to the left of the blue bucket
stack the blue block on the red block
move all blocks to the tray
```

* 增加失败恢复策略，例如目标未找到时自动换视角或重新扫描。

### 13.4 运动执行

* 优化 MoveIt 规划参数，减少自碰撞和奇异姿态。
* 增加 pre-grasp、grasp、lift、place 的明确阶段。
* 为不同物体类型设置不同抓取高度和夹爪宽度。
* 增加执行前仿真检查和可达性验证。

### 13.5 系统部署

* 从 WSL2 迁移到原生 Ubuntu，提高相机和串口稳定性。
* 支持双机部署：GPU 电脑负责大模型推理，机械臂主机负责 ROS 控制。
* 将实验流程整理为一键启动脚本。
* 增加更规范的日志记录、实验结果保存和 demo 复现实验文档。

---

## 14. 建议演示流程

推荐演示时按以下顺序：

1. 启动机械臂和相机。
2. 设置 `execute_grasp:=false`，只看检测和映射。
3. 发送：

```text
red block
```

4. 查看 annotated image，确认检测框和中心点正确。
5. 确认 mapped x/y 合理。
6. 设置 `execute_grasp:=true`，执行单目标抓取。
7. 再测试：

```text
pick red block and place it into red bucket
```

8. 最后测试多步任务：

```text
put each block into the bucket of the same color
```

这样可以逐步暴露问题，避免一开始就让机械臂执行复杂任务。

---

## 15. 项目总结

当前 `main` 分支已经实现了一个可运行的语言引导机械臂抓取原型。它的特点是：

* 使用 GroundingDINO 将自然语言目标和图像目标联系起来。
* 使用 HSV / 轮廓方法对检测中心进行工程化修正。
* 使用九点标定建立图像像素和机械臂工作平面的映射。
* 保留 Sagittarius 原有 MoveIt / SDK / `sgr_ctrl` 执行链路。
* 已经在真实机械臂上完成多步同色抓取 / 放置的部分成功验证。

当前最成熟的模式是：

```text
front-only + pick_front/place_front 分离标定 + GroundingDINO + HSV
```
