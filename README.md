# openvla_sagittarius_bridge

## 项目定位

这个仓库基于 Sagittarius ROS1 机械臂栈改造，当前重点是把原来的“HSV 颜色识别抓取”升级为“语言目标识别抓取”：

```text
目标文本 -> 感知后端 -> 检测框 -> 像素中心 -> vision_config.yaml 坐标映射 -> sgr_ctrl 抓取
```

当前已实现的感知后端是 GroundingDINO。架构上已经预留了可插拔后端接口，后续可以继续接入 YOLO-World、OWL-ViT、Grounded-SAM 等模型。

机械臂执行侧尽量保持原样，仍然复用 Sagittarius MoveIt、底层 SDK 和 `sgr_ctrl` action，不重写整套运动控制。

## 当前状态

已确认可以工作的部分：

- GroundingDINO 环境和权重可以加载
- 文本目标话题 `/grasp_target_text` 可以驱动检测
- 测试图片代替相机的静态验证链路已经跑通
- 检测结果可以继续进入原有 `sgr_ctrl` 抓取执行链路
- `vision_config.yaml` 线性回归映射仍然保留

当前主要限制：

- 在当前 WSL2 + usbipd 环境下，同时透传机械臂串口和摄像头时可能不稳定
- 真实相机闭环建议优先在原生 Ubuntu 或更稳定的 UVC 摄像头环境下验证

## 主要入口

推荐使用的新入口：

- 主节点：`src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/nodes/language_guided_grasp.py`
- 主 launch：`src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/launch/language_guided_grasp.launch`
- 图片测试 launch：`src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/launch/language_guided_grasp_image_test.launch`

兼容保留的旧入口：

- `nodes/color_classification.py`
- `launch/color_classification.launch`
- `launch/color_classification_image_test.launch`

旧入口只是为了兼容以前的命令和脚本。新代码已经不再是颜色分类逻辑，推荐以后都使用 `language_guided_grasp` 命名。

## 代码结构

```text
src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/
├── action/
│   └── SGRCtrl.action
├── config/
│   ├── HSVParams.cfg
│   └── vision_config.yaml
├── launch/
│   ├── language_guided_grasp.launch
│   ├── language_guided_grasp_image_test.launch
│   ├── color_classification.launch
│   ├── color_classification_image_test.launch
│   └── usb_cam.launch
├── nodes/
│   ├── language_guided_grasp.py
│   ├── color_classification.py
│   ├── perception_backend_smoke_test.py
│   ├── publish_test_image.py
│   ├── sgr_ctrl.py
│   └── perception_framework/
│       ├── detection_types.py
│       ├── backend_factory.py
│       ├── selection.py
│       ├── stability.py
│       ├── coordinate_mapping.py
│       ├── execution.py
│       └── backends/
│           ├── base.py
│           └── grounding_dino.py
├── test_data/
│   └── sample_red_block.ppm
├── CMakeLists.txt
└── package.xml
```

核心职责：

- `language_guided_grasp.py`：ROS 编排节点，负责接收图像和文本、调用感知后端、稳定判断、坐标映射、触发抓取
- `perception_framework/detection_types.py`：统一检测结果结构 `DetectionResult` 和 `DetectionBox`
- `perception_framework/backends/base.py`：所有感知后端的抽象接口 `BasePerceptionBackend`
- `perception_framework/backends/grounding_dino.py`：GroundingDINO 后端实现
- `perception_framework/backend_factory.py`：根据 `perception_backend` 参数创建后端
- `perception_framework/selection.py`：目标选择逻辑，当前默认选择最高分框
- `perception_framework/stability.py`：连续帧中心点稳定检测
- `perception_framework/coordinate_mapping.py`：读取 `vision_config.yaml`，把像素中心映射到机械臂平面坐标
- `perception_framework/execution.py`：对原有 `SGRCtrlAction` 的轻量封装
- `publish_test_image.py`：把本地图片发布成 `/usb_cam/image_raw`，用于不接真实相机时验证链路
- `perception_backend_smoke_test.py`：不启动 ROS action，只验证感知后端推理输出

## 架构说明

当前内部流程分成 5 层：

```text
输入层
  图像 /usb_cam/image_raw
  文本 /grasp_target_text

感知层
  BasePerceptionBackend
  GroundingDinoBackend

决策层
  select_highest_score
  CenterStabilityFilter

映射层
  VisionPlaneMapper
  vision_config.yaml

执行层
  SagittariusGraspExecutor
  SGRCtrlAction
```

统一感知输出结构：

```text
DetectionResult
├── source_model
├── timestamp
├── image_size
├── boxes
├── selected_box
├── selected_center
├── mask
└── metadata
```

每个检测框：

```text
DetectionBox
├── bbox_xyxy
├── score
├── label
└── metadata
```

下游抓取逻辑只依赖统一结构，不直接依赖 GroundingDINO 的 tensor 输出。这样以后换模型时，尽量只新增 backend，不动抓取执行侧。

## 环境准备

当前调试机器上 GroundingDINO 实际使用路径：

```text
/mnt/d/ai_models/GroundingDINO
/mnt/d/ai_models/groundingdino-venv
/mnt/d/ai_models/GroundingDINO/weights/groundingdino_swint_ogc.pth
```

启动前建议：

```bash
cd ~/sagittarius_ws
source /mnt/d/ai_models/groundingdino-venv/bin/activate
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export PYTHONPATH=/mnt/d/ai_models/GroundingDINO:$PYTHONPATH
export MPLCONFIGDIR=/tmp/matplotlib-cfg
mkdir -p "$MPLCONFIGDIR"
```

当前额外安装过的 Python 依赖：

```text
rospkg
catkin_pkg
empy
netifaces
```

## 启动方式

### 真实相机闭环

```bash
roslaunch sagittarius_object_color_detector language_guided_grasp.launch \
  device:=cpu \
  video_dev:=/dev/video0 \
  pixel_format:=mjpeg \
  image_width:=1280 \
  image_height:=720 \
  framerate:=30 \
  groundingdino_config:=/mnt/d/ai_models/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py \
  groundingdino_weights:=/mnt/d/ai_models/GroundingDINO/weights/groundingdino_swint_ogc.pth \
  drop_after_grasp:=false
```

### 测试图片代替相机

```bash
roslaunch sagittarius_object_color_detector language_guided_grasp_image_test.launch \
  device:=cpu \
  groundingdino_config:=/mnt/d/ai_models/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py \
  groundingdino_weights:=/mnt/d/ai_models/GroundingDINO/weights/groundingdino_swint_ogc.pth \
  drop_after_grasp:=false
```

默认测试图：

```text
src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/test_data/sample_red_block.ppm
```

换自己的图片：

```bash
roslaunch sagittarius_object_color_detector language_guided_grasp_image_test.launch \
  test_image:=/绝对路径/你的测试图片.jpg \
  device:=cpu \
  groundingdino_config:=/mnt/d/ai_models/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py \
  groundingdino_weights:=/mnt/d/ai_models/GroundingDINO/weights/groundingdino_swint_ogc.pth \
  drop_after_grasp:=false
```

发送目标文本：

```bash
rostopic pub /grasp_target_text std_msgs/String "data: 'red block'" -1
```

其他例子：

```bash
rostopic pub /grasp_target_text std_msgs/String "data: 'blue cube'" -1
rostopic pub /grasp_target_text std_msgs/String "data: 'banana'" -1
rostopic pub /grasp_target_text std_msgs/String "data: 'bottle'" -1
```

## 后端冒烟测试

不连接机械臂，只测模型输出：

```bash
cd ~/sagittarius_ws
source /mnt/d/ai_models/groundingdino-venv/bin/activate
export PYTHONPATH=/mnt/d/ai_models/GroundingDINO:$PYTHONPATH

python3 src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/nodes/perception_backend_smoke_test.py \
  --image src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/test_data/sample_red_block.ppm \
  --text "red block" \
  --backend grounding_dino \
  --config /mnt/d/ai_models/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py \
  --weights /mnt/d/ai_models/GroundingDINO/weights/groundingdino_swint_ogc.pth \
  --device cpu
```

成功时会输出：

```text
source_model: grounding_dino
image_size: ...
num_boxes: ...
selected_label: ...
selected_score: ...
selected_bbox_xyxy: ...
selected_center: ...
```

## 如何增加新的感知后端

新增模型时建议只改感知层：

1. 在 `nodes/perception_framework/backends/` 新增一个文件，例如 `yolo_world.py`
2. 继承 `BasePerceptionBackend`
3. 实现 `infer(self, image_bgr, text_prompt) -> DetectionResult`
4. 在 `backend_factory.py` 注册 backend 名称
5. 启动时传入 `perception_backend:=你的后端名`

新后端不要直接控制机械臂，不要做像素到机械臂坐标映射，也不要直接调用 `sgr_ctrl`。这些都由主编排节点统一处理。

## 兼容说明

为了不打断之前已经能跑的命令，以下旧入口仍然保留：

```bash
roslaunch sagittarius_object_color_detector color_classification.launch
roslaunch sagittarius_object_color_detector color_classification_image_test.launch
rosrun sagittarius_object_color_detector color_classification.py
```

但这些只是兼容入口，推荐新命令统一使用：

```bash
roslaunch sagittarius_object_color_detector language_guided_grasp.launch
roslaunch sagittarius_object_color_detector language_guided_grasp_image_test.launch
rosrun sagittarius_object_color_detector language_guided_grasp.py
```

## 调试结论

当前代码主链路已经完成，剩余风险主要来自运行环境：

- WSL2 下机械臂串口和摄像头同时透传时可能导致底层串口通信不稳定
- 如果真实相机闭环不稳定，优先用 `language_guided_grasp_image_test.launch` 验证软件链路
- 要做正式演示，优先考虑原生 Ubuntu 或外接普通 UVC 摄像头

## 许可证

原始 Sagittarius 代码来自 NXROBO Sagittarius ROS 工作空间。本仓库在其基础上增加语言目标抓取和可插拔感知后端改造。
