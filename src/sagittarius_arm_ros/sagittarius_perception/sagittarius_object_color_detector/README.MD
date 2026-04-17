# Sagittarius 语言目标抓取说明

## 1. 项目目标

本次改造的目标是：

- 保留原有机械臂抓取链路
- 保留像素坐标到机械臂平面坐标的线性回归映射
- 把原先基于 HSV 颜色阈值的抓取逻辑，改成基于 GroundingDINO 的文本目标识别抓取

典型目标文本包括：

- `red block`
- `blue cube`
- `green object`
- `banana`
- `bottle`

当前节点名仍然保持为 `color_classification_node`，主脚本仍然是 `color_classification.py`。
这两个名字是为了兼容旧 launch 和旧脚本，内部实现已经重构为“语言目标感知后端 + 坐标映射 + 抓取执行”的模块化结构。

---

## 2. 这个包里最重要的文件

### 2.1 主逻辑

`nodes/color_classification.py`

作用：

- 订阅图像话题 `/usb_cam/image_raw`
- 订阅目标文本话题 `/grasp_target_text`
- 根据 `perception_backend` 参数创建感知后端
- 调用统一后端接口做文本条件检测
- 选最高分框
- 取 bbox 中心点
- 用 `vision_config.yaml` 做像素到机械臂平面坐标映射
- 继续复用 `sgr_ctrl` action 完成搜索位姿、抓取、可选放置
- 只负责流程编排，不再直接写死 GroundingDINO 的模型细节

### 2.1.1 模块化感知框架

`nodes/perception_framework/`

作用：

- 定义统一检测结果结构 `DetectionResult`
- 定义检测框结构 `DetectionBox`
- 定义后端配置 `BackendConfig`
- 定义后端接口 `BasePerceptionBackend`
- 提供后端工厂 `create_backend`
- 提供目标选择、稳定检测、坐标映射、抓取执行的独立小模块

当前已有后端：

- `nodes/perception_framework/backends/grounding_dino.py`

后续如果要接入 YOLO-World、OWL-ViT、Grounded-SAM 等模型，优先在 `backends/` 里新增一个 adapter，而不是继续修改 `color_classification.py`。

### 2.2 主启动文件

`launch/color_classification.launch`

作用：

- 启动 MoveIt
- 启动 `sgr_ctrl.py`
- 启动相机
- 启动新的 GroundingDINO 抓取节点

### 2.3 相机启动文件

`launch/usb_cam.launch`

作用：

- 启动 `usb_cam_node`
- 现在支持从 launch 参数传入：
  - `video_dev`
  - `image_width`
  - `image_height`
  - `framerate`
  - `pixel_format`
  - `io_method`
  - `output`
  - `stderr_log`

### 2.4 测试图片发布节点

`nodes/publish_test_image.py`

作用：

- 从本地读取一张图片
- 按固定频率重复发布到 `/usb_cam/image_raw`
- 用于在没有稳定真实相机输入时，继续验证 GroundingDINO 检测与抓取链路

### 2.5 图片测试启动文件

`launch/color_classification_image_test.launch`

作用：

- 启动 MoveIt
- 启动 `sgr_ctrl.py`
- 启动测试图片发布节点
- 启动 `color_classification.py`
- 用本地图片代替真实相机输入

### 2.6 机械臂动作链路

`nodes/sgr_ctrl.py`

作用：

- 保留原仓库里的机械臂 action 抓取逻辑
- `color_classification.py` 不重写这部分，只复用它

### 2.7 action 定义

`action/SGRCtrl.action`

作用：

- 定义 `SGRCtrlAction`
- 当前抓取仍然走它

### 2.8 标定参数

`config/vision_config.yaml`

作用：

- 保存像素坐标映射到机械臂抓取平面的线性回归参数
- 当前 GroundingDINO 版本继续复用它

---

## 3. 本次主要修改了什么

### 3.1 `color_classification.py`

原来：

- 用 HSV 阈值检测红绿蓝色块
- 检测后直接抓取

第一次改造后：

- 改成 GroundingDINO 文本条件检测
- 支持默认目标文本 `~default_target_text`
- 支持从 `/grasp_target_text` 动态更新目标文本
- 多框时选最高置信度框
- 增加稳定检测机制：
  - `stable_required`
  - `center_tolerance`
- 增加 `busy` 保护，抓取过程中不重复触发
- 抓取成功后默认清空当前目标文本
- `drop_after_grasp` 可控制是否自动放置
- 保留原 action 链路：
  - 搜索位姿
  - `ACTION_TYPE_PICK_XYZ_RPY`
  - 失败 fallback 到 `ACTION_TYPE_PICK_XYZ`
  - 可选 `ACTION_TYPE_PUT_XYZ`

额外说明：

- 为了绕开 GroundingDINO 官方 `inference.py` 在当前环境里引出的 `matplotlib/supervision` 兼容问题，代码里没有直接依赖 `groundingdino.util.inference`，而是直接调用 GroundingDINO 的底层模块来完成模型加载和预测。
- 这样做不改变功能目标，但能更稳地在当前 ROS1 + WSL 环境里运行。

当前架构重构后：

- `color_classification.py` 保留为 ROS 编排入口
- GroundingDINO 细节移动到：
  `nodes/perception_framework/backends/grounding_dino.py`
- 检测输出统一为：
  `DetectionResult`
- 目标选择移动到：
  `nodes/perception_framework/selection.py`
- 稳定检测移动到：
  `nodes/perception_framework/stability.py`
- 像素到机械臂平面坐标映射移动到：
  `nodes/perception_framework/coordinate_mapping.py`
- `sgr_ctrl` action 调用封装到：
  `nodes/perception_framework/execution.py`
- 旧类名 `GroundingDINOGraspNode` 保留为别名，避免旧说明或脚本引用时完全断掉

### 3.2 `color_classification.launch`

新增了这些参数：

- `perception_backend`
- `perception_config`
- `perception_weights`
- `groundingdino_config`
- `groundingdino_weights`
- `default_target_text`
- `target_topic`
- `device`
- `box_threshold`
- `text_threshold`
- `stable_required`
- `center_tolerance`
- `pick_z`
- `drop_after_grasp`
- `drop_x`
- `drop_y`
- `drop_z`
- `allow_start_without_groundingdino`
- `allow_start_without_backend`
- 相机参数透传：
  - `video_dev`
  - `image_width`
  - `image_height`
  - `framerate`
  - `pixel_format`
  - `io_method`
  - `usb_cam_output`
  - `usb_cam_stderr_log`

### 3.3 `usb_cam.launch`

原来是固定写死：

- `/dev/usb_cam`
- `640x480`
- `yuyv`
- `mmap`

现在改成参数化，便于在当前机器上尝试不同视频设备和格式。

### 3.4 README

本文档用于持续记录：

- 这个包的作用
- 这次改造的目标
- 修改了哪些文件
- 当前调试结果
- 下一步应该怎么继续

### 3.5 `perception_backend_smoke_test.py`

新增了一个不启动 ROS action 的模型冒烟测试脚本：

`nodes/perception_backend_smoke_test.py`

作用：

- 读取一张本地图片
- 加载指定 backend
- 输入一段文本
- 输出统一检测结果里的最高分目标

这个脚本适合在不连接机械臂、不启动 MoveIt 的情况下，只验证感知后端是否能正常推理。

---

## 4. GroundingDINO 安装方式

### 4.1 推荐目录

当前调试机器上实际使用的是：

- GroundingDINO 仓库：
  `/mnt/d/ai_models/GroundingDINO`
- Python 虚拟环境：
  `/mnt/d/ai_models/groundingdino-venv`
- 权重：
  `/mnt/d/ai_models/GroundingDINO/weights/groundingdino_swint_ogc.pth`

### 4.2 启动前环境

完整启动前建议执行：

```bash
cd ~/sagittarius_ws
source /mnt/d/ai_models/groundingdino-venv/bin/activate
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export PYTHONPATH=/mnt/d/ai_models/GroundingDINO:$PYTHONPATH
export MPLCONFIGDIR=/tmp/matplotlib-cfg
mkdir -p "$MPLCONFIGDIR"
```

### 4.3 当前额外安装过的 Python 依赖

为了兼容 ROS 和当前虚拟环境，额外安装过：

- `rospkg`
- `catkin_pkg`
- `empy`
- `netifaces`

---

## 5. 启动方式

### 5.1 只测机械臂链路

```bash
cd ~/sagittarius_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch sagittarius_moveit demo_true.launch
```

### 5.2 启动完整文本抓取

当前推荐命令模板：

```bash
roslaunch sagittarius_object_color_detector color_classification.launch \
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

说明：

- 当前机器上相机日志默认写到文件，不再刷满主终端
- 相机 stderr 默认写到：
`/home/hank/.ros/log/usb_cam_stderr.log`

### 5.3 用测试图片代替真实相机

如果当前机器上的真实摄像头在 WSL 下不稳定，可以直接使用本地图片继续测试：

```bash
roslaunch sagittarius_object_color_detector color_classification_image_test.launch \
  device:=cpu \
  groundingdino_config:=/mnt/d/ai_models/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py \
  groundingdino_weights:=/mnt/d/ai_models/GroundingDINO/weights/groundingdino_swint_ogc.pth \
  drop_after_grasp:=false
```

建议：

- 当前路线不需要 attach 任何真实摄像头，只保留机械臂串口即可
- 图片尽量清晰
- 画面里尽量只放一个主要目标
- 如果图片里目标不止一个，GroundingDINO 会按最高置信度框去抓
- 仓库内已经提供了一张默认测试图：
  `test_data/sample_red_block.ppm`

如果你要换成自己的图片，可以覆盖：

```bash
roslaunch sagittarius_object_color_detector color_classification_image_test.launch \
  test_image:=/绝对路径/你的测试图片.jpg \
  device:=cpu \
  groundingdino_config:=/mnt/d/ai_models/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py \
  groundingdino_weights:=/mnt/d/ai_models/GroundingDINO/weights/groundingdino_swint_ogc.pth \
  drop_after_grasp:=false
```

### 5.4 单独检查测试图片话题

```bash
rostopic hz /usb_cam/image_raw
```

如果看到稳定频率，就说明测试图片已经成功发布成图像话题。

### 5.5 发送抓取目标文本

```bash
rostopic pub /grasp_target_text std_msgs/String "data: 'red block'" -1
```

其他例子：

```bash
rostopic pub /grasp_target_text std_msgs/String "data: 'blue cube'" -1
rostopic pub /grasp_target_text std_msgs/String "data: 'banana'" -1
rostopic pub /grasp_target_text std_msgs/String "data: 'bottle'" -1
```

---

## 6. 当前节点实际工作流程

当前 `color_classification.py` 的执行流程是：

1. 启动节点
2. 读取 `vision_config.yaml`
3. 根据 `perception_backend` 创建感知后端
4. 等待 `sgr_ctrl` action server
5. 机械臂进入搜索位姿
6. 订阅：
   - `/usb_cam/image_raw`
   - `/grasp_target_text`
7. 收到目标文本后开始检测
8. 调用后端统一接口：
   `infer(image, text_prompt) -> DetectionResult`
9. 检测到多个框时，选最高分框
10. 连续多帧中心点稳定后，才开始抓取
11. 取 bbox 中心点
12. 映射到机械臂抓取平面坐标
13. 调 `sgr_ctrl` 抓取
14. 如果开启 `drop_after_grasp`，则执行固定点放置
15. 抓取成功后清空目标文本

---

## 6.1 模块化架构说明

当前内部结构可以理解成 5 层：

1. 输入层：
   `color_classification.py` 订阅图像和目标文本

2. 感知层：
   `BasePerceptionBackend` 统一接口，当前实现是 `GroundingDinoBackend`

3. 决策层：
   `select_highest_score` 选择最高分检测框，`CenterStabilityFilter` 判断目标中心是否稳定

4. 映射层：
   `VisionPlaneMapper` 读取 `vision_config.yaml`，把像素中心映射到机械臂平面坐标

5. 执行层：
   `SagittariusGraspExecutor` 继续复用原来的 `SGRCtrlAction`

这样拆分以后，机器人侧链路和模型侧链路被隔离开了。以后换模型时，原则上不用动机械臂执行逻辑。

---

## 6.2 统一检测结果结构

所有感知后端都应该返回：

`DetectionResult`

主要字段：

- `source_model`：后端名称，例如 `grounding_dino`
- `timestamp`：推理时间戳
- `image_size`：图像尺寸
- `boxes`：候选检测框列表
- `selected_box`：被下游选择的目标框
- `selected_center`：被选择目标框的像素中心
- `mask`：可选 mask，给 Grounded-SAM 等后端预留
- `metadata`：后端额外信息

每个候选框是：

`DetectionBox`

主要字段：

- `bbox_xyxy`：`(x1, y1, x2, y2)`
- `score`：置信度
- `label`：模型输出标签或短语
- `metadata`：额外信息

下游选择、稳定判断、映射、抓取不应该直接依赖 GroundingDINO 的原始 tensor 输出。

---

## 6.3 如何增加新的感知后端

如果以后要增加 YOLO-World、OWL-ViT、Grounded-SAM 等模型，推荐步骤是：

1. 在这里新增文件：
   `nodes/perception_framework/backends/your_backend.py`

2. 继承：
   `BasePerceptionBackend`

3. 实现：
   `infer(self, image_bgr, text_prompt) -> DetectionResult`

4. 在这里注册 backend 名称：
   `nodes/perception_framework/backend_factory.py`

5. 启动时指定：
   `perception_backend:=your_backend`

注意：

- 新后端只需要负责“输入图像和文本，输出统一 DetectionResult”
- 不要在后端里直接控制机械臂
- 不要在后端里写像素到机械臂坐标映射
- 不要在后端里触发 `sgr_ctrl`
- 这样可以保持模型层和机器人执行层低耦合

---

## 6.4 后端冒烟测试

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

如果成功，会看到类似：

```text
source_model: grounding_dino
image_size: (...)
num_boxes: ...
selected_label: ...
selected_score: ...
selected_bbox_xyxy: ...
selected_center: ...
```

---

## 7. 调试阶段更新

本节只记录当前阶段已经确认的结果，不按时间流水账记录。

### 7.1 已确认成功的部分

- `color_classification.py` 已经改成 GroundingDINO 文本检测版本
- 机械臂抓取 action 仍然复用原来的 `sgr_ctrl`
- `vision_config.yaml` 的线性回归映射链路仍然保留
- GroundingDINO 模型环境已经装好
- 权重已经就位
- `color_classification_node` 可以正常启动
- 目标文本 `/grasp_target_text` 可以被节点收到
- WSL 下机械臂串口映射已经打通：
  - `/dev/ttyACM0`
  - `/dev/sagittarius -> ttyACM0`
- `usb_cam` 包已经安装
- 当前机器上 `/dev/video0` 可枚举，并且 `/usb_cam/image_raw` 可以跑到约 `30Hz`
- 使用默认测试图 `test_data/sample_red_block.ppm` 的图片代替相机方案已经验证成功
- 在图片测试方案下，节点可以：
  - 收到 `/grasp_target_text`
  - 完成 GroundingDINO 推理
  - 继续走机械臂执行链路
  - 最终出现 `Execution completed: SUCCEEDED`
- 当前已完成模块化重构：
  - GroundingDINO 已经从主节点中拆成可插拔后端
  - 主节点保留旧入口，但内部变成通用 perception-to-action 编排器

### 7.2 已确认存在的问题

- 在当前 `WSL2 + usbipd + 机械臂串口 + 内置摄像头` 组合下，同时跑相机和机械臂时，底层串口通信不稳定
- 典型报错：
  - `GetDataGram: No full reply available for read after 1s`
  - `sdk_sagittarius_arm ... process has died`
- 机械臂会反复重连，伴随哔哔声
- 说明当前真正的瓶颈不在 GroundingDINO 逻辑，而在 WSL 的 USB 透传稳定性

### 7.3 对相机设备的结论

通过 `v4l2-ctl` 已确认：

- `/dev/video0` 才是真正的主采集设备
- `/dev/video1` 不是适合当前抓取逻辑的主视频流
- `/dev/video0` 支持：
  - `MJPG`
    - `1280x720 @ 30 FPS`
    - `640x480 @ 25 FPS`
  - `YUYV`
    - `1280x720 @ 10 FPS`
    - `640x480 @ 25 FPS`

### 7.4 对当前整体验证状态的判断

当前状态可以概括为：

- 代码改造：基本完成
- 模型环境：基本完成
- 机械臂单独运行：可以
- 相机单独运行：基本可以
- 完整闭环联调：被 WSL 下 USB 透传稳定性卡住
- 使用“图片代替相机”方案时，语言目标抓取链路已经可以跑通

### 7.5 当前新增的绕过方案

为了绕开当前机器上的 WSL 摄像头稳定性问题，已经新增“测试图片代替真实相机”的方案：

- 新增节点：
  `nodes/publish_test_image.py`
- 新增 launch：
  `launch/color_classification_image_test.launch`

这条路线的意义是：

- 不再依赖当前机器上的真实摄像头取流稳定性
- 可以继续验证：
  - 文本目标输入
  - GroundingDINO 检测
  - bbox 中心提取
  - 坐标映射
  - 抓取动作触发
- 当前该路线已经在本机验证通过

---

## 8. 下一步建议

当前最现实的继续路线有 3 条。

### 路线 A：原生 Ubuntu

最推荐。

原因：

- 避开 WSL 的 USB/IP 稳定性问题
- 最有希望一次跑通“机械臂 + 相机 + GroundingDINO”

### 路线 B：外接普通 UVC USB 摄像头

如果继续在 WSL 下调试，比内置摄像头更稳。

### 路线 C：图片代替相机

如果只想继续验证算法链路，可以用本地图片发布成 `/usb_cam/image_raw`，继续测试：

- 文本目标接收
- GroundingDINO 检测
- bbox 中心提取
- 坐标映射
- 抓取触发链路

当前仓库里已经实现了这条路线，优先推荐在当前 WSL 环境下使用。

---

## 9. 当前改过的文件

- `src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/nodes/color_classification.py`
- `src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/nodes/perception_framework/__init__.py`
- `src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/nodes/perception_framework/detection_types.py`
- `src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/nodes/perception_framework/backend_factory.py`
- `src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/nodes/perception_framework/selection.py`
- `src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/nodes/perception_framework/stability.py`
- `src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/nodes/perception_framework/coordinate_mapping.py`
- `src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/nodes/perception_framework/execution.py`
- `src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/nodes/perception_framework/backends/base.py`
- `src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/nodes/perception_framework/backends/grounding_dino.py`
- `src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/nodes/perception_framework/backends/__init__.py`
- `src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/nodes/perception_backend_smoke_test.py`
- `src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/launch/color_classification.launch`
- `src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/launch/color_classification_image_test.launch`
- `src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/launch/usb_cam.launch`
- `src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/nodes/publish_test_image.py`
- `src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/README.MD`

---

## 10. 备注

### 10.1 标定文件缺失警告

日志中的：

`Camera calibration file ... not found`

目前不是主问题。它不会导致相机完全无图，只是没有单独的相机标定文件。

### 10.2 如果只想先看节点是否收到文本

可以观察日志里是否出现：

`Updated grasp target text: 'red block'`

### 10.3 如果只想先确认图像话题是否在发布

```bash
rostopic hz /usb_cam/image_raw
```

---

## 11. 当前结论

这次仓库修改的主要任务已经完成，当前剩下的不是算法主逻辑问题，而是运行环境问题：

- GroundingDINO 语言抓取节点已经接上原机械臂抓取链路
- 当前最主要阻塞是 WSL 下机械臂串口和内置摄像头同时透传时的稳定性
- 不依赖真实摄像头时，基于测试图片的语言目标抓取链路已经验证成功
- 如果换到更稳定的相机输入环境，整套方案继续推进的概率很高
