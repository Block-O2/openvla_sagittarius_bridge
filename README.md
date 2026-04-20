# openvla_sagittarius_bridge

## 项目定位

这个仓库基于 Sagittarius ROS1 机械臂栈改造，当前重点是把原来的“HSV 颜色识别抓取”升级为“语言目标识别抓取”：

```text
目标文本 -> 感知后端 -> 检测框 -> 像素中心 -> vision_config.yaml 坐标映射 -> sgr_ctrl 抓取
```

当前已实现的感知后端是 GroundingDINO。架构上已经预留了可插拔后端接口，后续可以继续接入 YOLO-World、OWL-ViT、Grounded-SAM 等模型。

机械臂执行侧尽量保持原样，仍然复用 Sagittarius MoveIt、底层 SDK 和 `sgr_ctrl` action，不重写整套运动控制。

## 项目亮点 / Key Contributions

这个项目的价值不只是“换了一个检测模型”，而是把原本很固定的颜色抓取例程，推进成一个更接近真实语言引导抓取系统的原型：

- 从基于 HSV 阈值的规则式颜色抓取，升级为基于 GroundingDINO 的语言条件目标抓取
- 支持语义/类别级目标描述，例如 `banana`、`bottle`、`cube`，不再局限于红绿蓝色块
- 保留原有 Sagittarius 机械臂执行链路，低侵入复用 MoveIt、SDK 和 `sgr_ctrl` action
- 保留 `vision_config.yaml` 的像素到机械臂平面坐标映射，避免重新标定整套控制链路
- 增加安全决策层：无检测、低置信度、无效文本时不触发抓取，把“安全不执行”作为明确行为
- 增加连续帧稳定检测和 busy 保护，降低瞬时误检或重复触发导致的误抓风险
- 增加标注图和状态话题，方便演示时解释“模型看到了什么、为什么抓/为什么不抓”
- 提供静态图片可控验证模式，在 WSL 相机/串口不稳定时仍然可以复现实验链路
- 架构正在向可插拔感知后端演进，未来可以接入 YOLO-World、OWL-ViT、Grounded-SAM 等模型
- 保留未来双机部署可能性：一台机器跑大模型推理，另一台机器负责相机和机械臂控制
- 面向资源受限、硬件不稳定环境做了工程化折中，更符合学生项目真实落地过程

## 当前状态

已确认可以工作的部分：

- GroundingDINO 环境和权重可以加载
- 文本目标话题 `/grasp_target_text` 可以驱动检测
- 测试图片代替相机的静态验证链路已经跑通
- 检测结果可以继续进入原有 `sgr_ctrl` 抓取执行链路
- `vision_config.yaml` 线性回归映射仍然保留
- 标注图输出和状态话题可以用于演示和调试
- busy 期间收到的新目标会先延后保存，避免中途打断正在执行的抓取

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
│   ├── language_guided_ai_station.launch
│   ├── language_guided_robot_station.launch
│   ├── color_classification.launch
│   ├── color_classification_image_test.launch
│   └── usb_cam.launch
├── nodes/
│   ├── language_guided_grasp.py
│   ├── language_guided_perception.py
│   ├── language_guided_executor.py
│   ├── color_classification.py
│   ├── perception_backend_smoke_test.py
│   ├── publish_test_image.py
│   ├── sgr_ctrl.py
│   └── perception_framework/
│       ├── detection_types.py
│       ├── backend_factory.py
│       ├── decision.py
│       ├── observation.py
│       ├── visualization.py
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

- `language_guided_grasp.py`：ROS 编排节点，负责接收图像和文本、调用感知后端、状态流管理、稳定判断、坐标映射、触发抓取
- `language_guided_perception.py`：双机 AI Station 节点，只负责 GroundingDINO 推理并发布轻量 JSON 检测结果
- `language_guided_executor.py`：双机 Robot Station 节点，订阅 JSON 检测结果，完成稳定检测、坐标映射和 `sgr_ctrl` 抓取
- `perception_framework/detection_types.py`：统一检测结果结构 `DetectionResult` 和 `DetectionBox`
- `perception_framework/backends/base.py`：所有感知后端的抽象接口 `BasePerceptionBackend`
- `perception_framework/backends/grounding_dino.py`：GroundingDINO 后端实现
- `perception_framework/backend_factory.py`：根据 `perception_backend` 参数创建后端
- `perception_framework/selection.py`：目标选择逻辑，当前默认选择最高分框
- `perception_framework/decision.py`：抓取前安全决策，区分已选中、未检测到、低置信度、无效目标
- `perception_framework/observation.py`：双机模式下 JSON target observation / execution feedback 的序列化辅助
- `perception_framework/visualization.py`：绘制检测框、标签、分数、选中中心点和决策状态
- `perception_framework/stability.py`：连续帧中心点稳定检测
- `perception_framework/coordinate_mapping.py`：读取 `vision_config.yaml`，把像素中心映射到机械臂平面坐标
- `perception_framework/execution.py`：对原有 `SGRCtrlAction` 的轻量封装
- `publish_test_image.py`：把本地图片发布成 `/usb_cam/image_raw`，用于不接真实相机时验证链路
- `perception_backend_smoke_test.py`：不启动 ROS action，只验证感知后端推理输出

## 双机部署

当前分支增加了双机部署雏形，把大模型推理和机械臂执行拆成两个站点：

```text
AI Station 你的电脑
  GroundingDINO 推理
  /usb_cam/image_raw -> /language_guided_grasp/target_observation

Robot Station 实验室电脑
  摄像头 + Sagittarius 机械臂 + MoveIt + sgr_ctrl
  /language_guided_grasp/target_observation -> vision_config.yaml -> sgr_ctrl
```

推荐职责分配：

- 实验室电脑作为 `Robot Station`，连接机械臂和摄像头，并作为 ROS master
- 你的电脑作为 `AI Station`，运行 GroundingDINO 和语言目标检测
- 两台机器通过 ROS topic 传输图像、轻量 JSON 检测结果和执行反馈

新增入口：

- AI Station launch：`language_guided_ai_station.launch`
- Robot Station launch：`language_guided_robot_station.launch`
- AI Station 节点：`language_guided_perception.py`
- Robot Station 节点：`language_guided_executor.py`

双机说明文件：

```text
dual_machine/
├── ai_station/
│   ├── README.zh.md
│   ├── README.en.md
│   ├── env.example.sh
│   └── run_ai_station.sh
└── robot_station/
    ├── README.zh.md
    ├── README.en.md
    ├── env.example.sh
    └── run_robot_station.sh
```

最关键的中间 topic 是：

```text
/language_guided_grasp/target_observation
```

它使用 `std_msgs/String` 发送 JSON，包含目标文本、检测状态、bbox、中心点、置信度和图像尺寸。这样实验室电脑不需要安装 GroundingDINO，也不需要 GPU。

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
  evaluate_target_selection
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

## 运行状态流

主节点现在维护一个轻量状态流，用于减少重复触发，也方便演示和排查：

```text
waiting_for_target -> detecting -> target_locked -> grasping -> placing -> done
                                      └──────────── failed
```

状态含义：

- `waiting_for_target`：节点已启动，等待 `/grasp_target_text` 输入
- `detecting`：已有目标文本，正在对图像做语言条件检测
- `target_locked`：目标中心在连续帧中稳定，准备进入抓取
- `grasping`：正在调用 `sgr_ctrl` 执行抓取
- `placing`：抓取成功后正在执行可选放置动作
- `done`：一次抓取流程成功结束
- `failed`：当前请求未满足执行条件，例如无检测、低置信度、后端不可用或抓取失败

状态会发布到：

```text
/language_guided_grasp/state
```

查看状态：

```bash
rostopic echo /language_guided_grasp/state
```

如果抓取执行期间又收到新的目标文本，节点不会立即打断机械臂动作，而是先缓存为 pending target，当前动作结束后再进入下一轮检测。

## 安全决策与可视化

当前抓取前会经过一个明确的安全决策层：

- `selected`：检测到目标，最高分候选超过 `min_grasp_score`，继续稳定检测，稳定后才抓取
- `no_detection`：没有检测到与文本目标匹配的候选，不触发机械臂
- `low_confidence`：有候选但分数低于 `min_grasp_score`，不触发机械臂
- `invalid_target`：目标文本为空或过短，不触发机械臂

默认会发布一张标注图，便于演示和排查：

```text
/language_guided_grasp/annotated_image
```

标注图包含检测框、候选标签、分数、选中中心点和当前决策状态。图片测试模式默认还会把最新标注图保存到：

```text
/tmp/language_guided_grasp_latest.jpg
```

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
  min_grasp_score:=0.35 \
  publish_annotated_image:=true \
  state_topic:=/language_guided_grasp/state \
  groundingdino_config:=/mnt/d/ai_models/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py \
  groundingdino_weights:=/mnt/d/ai_models/GroundingDINO/weights/groundingdino_swint_ogc.pth \
  drop_after_grasp:=false \
  clear_target_after_success:=true
```

### 测试图片代替相机

```bash
roslaunch sagittarius_object_color_detector language_guided_grasp_image_test.launch \
  device:=cpu \
  min_grasp_score:=0.35 \
  save_annotated_image:=true \
  annotated_image_path:=/tmp/language_guided_grasp_latest.jpg \
  state_topic:=/language_guided_grasp/state \
  groundingdino_config:=/mnt/d/ai_models/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py \
  groundingdino_weights:=/mnt/d/ai_models/GroundingDINO/weights/groundingdino_swint_ogc.pth \
  drop_after_grasp:=false \
  clear_target_after_success:=true
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
  save_annotated_image:=true \
  annotated_image_path:=/tmp/language_guided_grasp_latest.jpg \
  state_topic:=/language_guided_grasp/state \
  groundingdino_config:=/mnt/d/ai_models/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py \
  groundingdino_weights:=/mnt/d/ai_models/GroundingDINO/weights/groundingdino_swint_ogc.pth \
  drop_after_grasp:=false \
  clear_target_after_success:=true
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
  --device cpu \
  --output /tmp/language_guided_grasp_smoke.jpg
```

成功时会输出：

```text
source_model: grounding_dino
image_size: ...
num_boxes: ...
decision_status: selected
decision_reason: ...
selected_label: ...
selected_score: ...
selected_bbox_xyxy: ...
selected_center: ...
annotated_output: /tmp/language_guided_grasp_smoke.jpg
```

## 推荐演示场景

1. 抓取颜色加类别目标：`red block`
2. 抓取类别目标：`banana`
3. 抓取类别目标：`bottle`
4. 多物体场景中只抓取指定目标：例如画面里同时有方块和瓶子，只发布 `bottle`
5. 请求不存在的目标：例如发布 `airplane`，确认状态为 `no_detection` 且机械臂不动作
6. 静态图片演示：运行 `language_guided_grasp_image_test.launch`，查看 `/tmp/language_guided_grasp_latest.jpg`
7. 可选 pick-and-place：设置 `drop_after_grasp:=true`，抓取成功后放到固定位置

常用目标文本：

```bash
rostopic pub /grasp_target_text std_msgs/String "data: 'red block'" -1
rostopic pub /grasp_target_text std_msgs/String "data: 'blue cube'" -1
rostopic pub /grasp_target_text std_msgs/String "data: 'banana'" -1
rostopic pub /grasp_target_text std_msgs/String "data: 'bottle'" -1
rostopic pub /grasp_target_text std_msgs/String "data: 'airplane'" -1
```

演示时建议同时打开两个观察窗口：

```bash
rostopic echo /language_guided_grasp/state
rostopic hz /language_guided_grasp/annotated_image
```

如果是静态图片模式，也可以直接查看保存的标注图：

```bash
xdg-open /tmp/language_guided_grasp_latest.jpg
```

## 如何增加新的感知后端

新增模型时建议只改感知层：

1. 在 `nodes/perception_framework/backends/` 新增一个文件，例如 `yolo_world.py`
2. 继承 `BasePerceptionBackend`
3. 实现 `infer(self, image_bgr, text_prompt) -> DetectionResult`
4. 在 `backend_factory.py` 注册 backend 名称
5. 启动时传入 `perception_backend:=你的后端名`

新后端不要直接控制机械臂，不要做像素到机械臂坐标映射，也不要直接调用 `sgr_ctrl`。这些都由主编排节点统一处理。

## 未来工作 / Planned Extensions

后续如果继续扩展，建议优先沿着这些方向推进：

- 插入更多开放词汇感知后端，例如 YOLO-World、OWL-ViT、Grounded-SAM，并通过 `perception_backend` 参数切换
- 把语言目标从单一短语扩展到更复杂指令，例如“抓左边的瓶子”“抓离机械臂最近的香蕉”
- 增加更清晰的双机通信方案：感知机器只发布轻量结果，控制机器只负责映射和执行
- 在原生 Ubuntu 或更稳定硬件环境下验证真实相机闭环，降低 WSL usbipd 带来的运行不确定性
- 改进相机标定和手眼标定，让像素到机械臂坐标的映射更稳健
- 扩展 pick-and-place 策略，例如根据类别选择不同放置区，而不是固定放置点
- 增加更完整的状态机和恢复策略，例如抓取失败后自动回退、重新检测或请求人工确认
- 在当前语言抓取基础上进一步接入 OpenVLA / VLM 策略，用视觉语言模型产生更高层任务决策

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
