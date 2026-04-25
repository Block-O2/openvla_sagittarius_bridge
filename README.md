# sagittarius-semantic-grasp

Sagittarius 机械臂语言引导语义抓取项目

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
- 支持多观察位扫描：可以配置 `front / left / right` 三个观察位，并为每个观察位使用独立标定文件
- 支持轻量任务级命令拆解，例如 `抓红色方框放进蓝色桶里面`，先定位抓取目标，再定位放置目标
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
- 2026-04-22 实机验证：单机模式下 `red block` 已经完成 GroundingDINO 检测、稳定锁定、坐标映射和 `sgr_ctrl` 抓取，日志结果为 `Grasp succeeded`
- 2026-04-25 代码升级：支持多观察位扫描与动态放置目标，允许先扫 `front / left / right`，再执行 `pick -> place`

当前主要限制：

- 在当前 WSL2 + usbipd 环境下，同时透传机械臂串口和摄像头时可能不稳定
- 真实相机闭环建议优先在原生 Ubuntu 或更稳定的 UVC 摄像头环境下验证
- 当前 `vision_config.yaml` 如果仍是 `k1=0.0`、`k2=0.0`，不同像素点会映射到同一个机械臂平面坐标。也就是说模型确实根据摄像头选目标，但抓取落点还不会随目标像素位置变化，正式演示前需要手动标定。
- 多观察位模式下，每个观察位都必须单独标定。不能直接复用 `front` 的 `vision_config.yaml` 去解释 `left/right` 视角下的像素坐标。

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
│   ├── vision_config.yaml
│   └── manual_calibration_points.example.csv
├── launch/
│   ├── language_guided_grasp.launch
│   ├── language_guided_grasp_image_test.launch
│   ├── language_guided_calibration_base.launch
│   ├── color_classification.launch
│   ├── color_classification_image_test.launch
│   └── usb_cam.launch
├── nodes/
│   ├── language_guided_grasp.py
│   ├── language_guided_calibration.py
│   ├── color_classification.py
│   ├── perception_backend_smoke_test.py
│   ├── publish_test_image.py
│   ├── sgr_ctrl.py
│   └── perception_framework/
│       ├── detection_types.py
│       ├── backend_factory.py
│       ├── decision.py
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
├── scripts/
│   ├── ensure_sagittarius_serial.sh
│   ├── run_single_machine_gpu_grasp.sh
│   └── run_manual_vision_calibration.sh
├── CMakeLists.txt
└── package.xml
```

核心职责：

- `language_guided_grasp.py`：ROS 编排节点，负责接收图像和文本、调用感知后端、状态流管理、稳定判断、坐标映射、触发抓取
- `perception_framework/detection_types.py`：统一检测结果结构 `DetectionResult` 和 `DetectionBox`
- `perception_framework/backends/base.py`：所有感知后端的抽象接口 `BasePerceptionBackend`
- `perception_framework/backends/grounding_dino.py`：GroundingDINO 后端实现
- `perception_framework/backend_factory.py`：根据 `perception_backend` 参数创建后端
- `perception_framework/selection.py`：目标选择逻辑，当前默认选择最高分框
- `perception_framework/decision.py`：抓取前安全决策，区分已选中、未检测到、低置信度、无效目标
- `perception_framework/visualization.py`：绘制检测框、标签、分数、选中中心点和决策状态
- `perception_framework/stability.py`：连续帧中心点稳定检测
- `perception_framework/coordinate_mapping.py`：读取 `vision_config.yaml`，把像素中心映射到机械臂平面坐标
- `perception_framework/execution.py`：对原有 `SGRCtrlAction` 的轻量封装
- `publish_test_image.py`：把本地图片发布成 `/usb_cam/image_raw`，用于不接真实相机时验证链路
- `language_guided_calibration.py`：复用原始标定思路，让机械臂移动到已知点，用户放方块，GroundingDINO 记录像素中心并写回 `vision_config.yaml`
- `perception_backend_smoke_test.py`：不启动 ROS action，只验证感知后端推理输出
- `scripts/ensure_sagittarius_serial.sh`：检查/修复 WSL 下机械臂串口 `/dev/ttyACM0` 和兼容链接 `/dev/sagittarius`
- `nodes/manual_vision_calibration.py`：根据手动采集的像素点和机械臂平面坐标拟合 `vision_config.yaml`

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

在多观察位任务模式下，执行前会先跑一个轻量扫描流程：

```text
任务文本 -> pick/place 目标拆解 -> front/left/right 依次观察
         -> 用各观察位自己的 vision_config 映射到机械臂平面
         -> pick -> place
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

## 多观察位动态放置

现在除了单目标文本，例如：

```text
red block
```

也支持轻量任务级文本，例如：

```text
抓红色方框放进蓝色桶里面
pick red block and place it into blue bucket
```

节点会先把文本拆成：

- `pick_target`
- `place_target`

然后按 `scan_view_order` 指定的顺序去观察位扫描，默认顺序为：

```text
front,left,right
```

如果某个观察位成功锁定目标，就使用该观察位对应的标定文件完成像素到机械臂平面坐标映射。这样可以支持“桶不在正前方，而是放在左边或右边”的情况。

### 多观察位参数

主 launch 和图片测试 launch 现在都支持：

- `left_view_enabled:=true`
- `left_view_vision_config:=.../vision_config_left.yaml`
- `left_view_x/y/z/roll/pitch/yaw:=...`
- `right_view_enabled:=true`
- `right_view_vision_config:=.../vision_config_right.yaml`
- `right_view_x/y/z/roll/pitch/yaw:=...`
- `scan_view_order:=front,left,right`
- `scan_attempts_per_view:=5`
- `scan_stable_required:=3`
- `scan_settle_sec:=0.8`
- `dynamic_place_z:=0.20`

说明：

- `front` 默认复用原来的 `search_pose_*` 和 `vision_config.yaml`
- `left/right` 必须显式提供各自的 `vision_config`
- 默认推荐把 `left_view_yaw` 设为正值、`right_view_yaw` 设为负值，让相机真正朝左/朝右转，而不是只做平移
- 默认推荐把 `left/right` 的观察位抬高到 `z=0.20` 左右，减少机身遮挡并扩大侧向视野
- `dynamic_place_z` 是“把物体放到桶上方并松手”的绝对 Z，高度应根据桶口高度手动设定

### 多观察位标定建议

推荐分别为三个观察位准备三个文件：

```text
config/vision_config_front.yaml
config/vision_config_left.yaml
config/vision_config_right.yaml
```

现有 `language_guided_calibration.py` 已经支持通过不同的：

- `~vision_config`
- `~observe_x/y/z/roll/pitch/yaw`

来给不同观察位单独标定。也就是说，不需要重写标定脚本，只要换观察位姿和输出路径即可。

标定点不要三套都集中在中间。推荐思路：

- `front`：覆盖桌面中间区域
- `left`：点整体偏到 `y > 0` 的左侧区域
- `right`：点整体偏到 `y < 0` 的右侧区域

一个实用的 5 点示例：

```text
front : 0.220,0.000 ; 0.220,0.030 ; 0.240,0.020 ; 0.240,-0.020 ; 0.260,0.000
left  : 0.220,0.030 ; 0.220,0.060 ; 0.240,0.050 ; 0.240,0.020 ; 0.260,0.040
right : 0.220,-0.030 ; 0.220,-0.060 ; 0.240,-0.050 ; 0.240,-0.020 ; 0.260,-0.040
```

这样比“三套视角都标中间那一小块”更适合多观察位扫描。

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

实机单机脚本默认还会保存两张调试图，方便确认“相机到底看到了什么”和“模型到底选中了哪里”：

```text
/tmp/language_guided_grasp_raw_single_gpu.jpg
/tmp/language_guided_grasp_single_gpu.jpg
```

含义：

- `language_guided_grasp_raw_single_gpu.jpg`：原始相机画面，不带任何检测框
- `language_guided_grasp_single_gpu.jpg`：带 GroundingDINO 检测框、分数、选中中心点的标注图

如果出现“模型能抓但落点偏很多”，优先检查这两张图和 ROS 日志里的 `Stable target ... pixel center`、`Target ... mapped to arm plane`。如果图里目标框是对的，但映射出的 `x/y` 明显不随像素变化，问题通常在 `vision_config.yaml` 标定，而不是模型检测。

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

### 机械臂串口初始化检查

如果启动时报：

```text
open serial failed :/dev/sagittarius fail
Failed to init device
GetDataGram: Serial is not open
```

先不要继续做标定或抓取，先确认 WSL 是否生成了机械臂串口节点。

Windows PowerShell 中确认机械臂已经 attach：

```powershell
usbipd list
```

机械臂通常显示为：

```text
2e88:4603 USB 串行设备 / HDSC CDC Device
```

如果状态不是 `Attached`，在 PowerShell 中执行：

```powershell
usbipd attach --wsl --busid <机械臂BUSID>
```

回到 Ubuntu 终端，先运行串口检查脚本：

```bash
cd ~/sagittarius_ws
bash src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/scripts/ensure_sagittarius_serial.sh
```

正常结果应该能看到：

```text
/dev/ttyACM0
/dev/sagittarius -> /dev/ttyACM0
```

如果脚本提示需要创建 `/dev/ttyACM0`，它会尝试使用 `sudo mknod`。如果你的终端要求输入密码，输入 Ubuntu 用户密码即可。

如果出现下面这种 sudo 本身的权限错误：

```text
sudo: /usr/bin/sudo must be owned by uid 0 and have the setuid bit set
```

说明当前 WSL 的 sudo 权限位异常。请在 Windows PowerShell 中执行一次：

```powershell
wsl -d Ubuntu-20.04 -u root -- bash -lc "chown root:root /usr/bin/sudo /bin/su && chmod 4755 /usr/bin/sudo /bin/su"
```

然后回到 Ubuntu 重新运行串口检查脚本。

如果 `lsusb` 能看到 `2e88:4603`，但是脚本仍然找不到 `/dev/ttyACM0`，通常说明 usbipd/WSL 这次 attach 不完整。推荐按顺序做：

```powershell
usbipd detach --busid <机械臂BUSID>
usbipd attach --wsl --busid <机械臂BUSID>
```

如果仍然不行，重启 WSL 后再 attach：

```powershell
wsl --shutdown
usbipd attach --wsl --busid <机械臂BUSID>
```

再回到 Ubuntu 运行：

```bash
lsusb
ls -l /dev/ttyACM* /dev/sagittarius
bash src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/scripts/ensure_sagittarius_serial.sh
```

只有这个检查通过后，再启动 `demo_true.launch` 或语言抓取 launch。

### 单机 GPU 实机测试推荐参数

你现在的使用方式是单机：同一台电脑同时连接机械臂、机械臂摄像头，并运行 GroundingDINO。因为有 RTX 4060，推荐使用：

```bash
device:=cuda
```

不要再用 `device:=cpu` 做正式测试，否则 GroundingDINO 会把 CPU 打满，推理也会很慢。

单机实机推荐启动参数：

```bash
device:=cuda \
video_dev:=/dev/video0 \
pixel_format:=mjpeg \
image_width:=640 \
image_height:=480 \
framerate:=10 \
search_pose_mode:=camera_down \
search_pose_x:=0.20 \
search_pose_y:=0.00 \
search_pose_z:=0.15 \
search_pose_pitch:=1.57 \
return_to_search_pose_after_grasp:=false \
pick_orientation_mode:=auto \
drop_after_grasp:=false
```

这里的含义是：

- 启动后先让机械臂进入准备/观察姿态，避免机械臂挡住摄像头导致看不到桌面目标
- 抓取结束后不强制回到硬编码姿态，避免再次突然朝上
- 抓取动作本身优先使用动态姿态，由 `sgr_ctrl` 根据目标 XYZ 计算末端朝向
- 第一次实机建议关闭放置动作，只验证“识别 -> 抓取”

已经提供一个单机 GPU 启动脚本：

```bash
src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/scripts/run_single_machine_gpu_grasp.sh
```

默认 `EXECUTE_GRASP=false`，只做检测和映射，不执行真实抓取。确认无误后再设置 `EXECUTE_GRASP=true`。

### 位姿与抓取姿态参数

为了避免机械臂在启动或抓取结束后突然移动到错误的硬编码姿态，当前默认行为保持兼容，但实机脚本默认使用 `camera_down` 桌面观察姿态：

- `search_pose_mode:=none`：launch 默认值，启动时不强制移动到固定搜索位姿
- `search_pose_mode:=stay` / `hold_current`：兼容写法，同样表示保持当前位置
- `search_pose_mode:=camera_down` / `table_view`：推荐实机模式，进入可配置 XYZ+RPY 桌面观察姿态，让相机朝向桌面
- `search_pose_mode:=define_stay`：显式启用旧的 `ACTION_TYPE_DEFINE_STAY` 预设姿态；这个姿态可能让机械臂向上折叠、相机朝天，通常不推荐
- `search_pose_mode:=xyz_rpy` / `legacy`：兼容旧的固定 XYZ+RPY 搜索位姿，当前等价于可配置桌面观察姿态
- `search_pose_x/y/z/roll/pitch/yaw`：桌面观察姿态参数，默认 `x=0.20, y=0.00, z=0.15, roll=0.0, pitch=1.57, yaw=0.0`
- `return_to_search_pose_after_grasp:=false`：默认值，抓取结束后不再强制回到搜索位姿
- `return_to_search_pose_after_grasp:=true`：抓取结束后按 `search_pose_mode` 执行回位
- `pick_orientation_mode:=auto`：默认值，抓取时优先使用 `ACTION_TYPE_PICK_XYZ`，由 `sgr_ctrl` 根据目标点动态计算末端姿态
- `pick_orientation_mode:=fixed_rpy` / `legacy`：显式使用旧的固定 `pitch=1.57` 抓取姿态

如果摄像头视野需要机械臂先抬到准备姿态，实机抓取时使用：

```bash
search_pose_mode:=camera_down \
search_pose_x:=0.20 \
search_pose_y:=0.00 \
search_pose_z:=0.15 \
search_pose_pitch:=1.57 \
return_to_search_pose_after_grasp:=false \
pick_orientation_mode:=auto \
drop_after_grasp:=false
```

这样只在启动阶段进入相机朝向桌面的观察/准备姿态，抓取结束后不会再次强制回到硬编码姿态。如果现场观察发现相机还不够朝下，优先小幅调整 `search_pose_z` 或 `search_pose_pitch`，不要再使用 `define_stay`。

### 手动标定 vision_config.yaml

当前语言抓取仍然沿用原开源包的线性映射约定：

```text
robot_x = k1 * pixel_y + b1
robot_y = k2 * pixel_x + b2
```

如果 `config/vision_config.yaml` 里 `k1=0.0` 且 `k2=0.0`，说明还没有有效标定。此时模型即使正确识别了目标，不同像素位置也都会映射到同一个机械臂平面点，抓取会明显偏。

原始开源包也提供了 HSV 蓝色块标定流程：

```bash
roslaunch sagittarius_object_color_detector camera_calibration_hsv.launch
bash src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/scripts/send_topic.sh
```

这个流程会调用：

```text
nodes/calibration.py
nodes/calibration_pose.py
scripts/send_topic.sh
```

但它依赖 HSV 蓝色方块识别，和当前 GroundingDINO 语言抓取链路不是完全同一套感知逻辑。当前项目更推荐下面的手动 CSV 标定：用语言抓取节点保存标注图，人工记录每个目标中心像素和对应机械臂平面坐标，再拟合 `vision_config.yaml`。

如果你不想手动猜 `robot_x,robot_y`，推荐使用新的语言辅助标定脚本。它复刻原包思路：机械臂先移动到 5 个已知坐标，你把红色方块放到夹爪正下方，这样方块真实坐标就等于机械臂当前已知坐标；然后机械臂移到观察姿态，GroundingDINO 识别方块中心并自动配对。

终端 1：启动机械臂、MoveIt、`sgr_ctrl` 和相机，不启动抓取节点：

```bash
cd ~/sagittarius_ws
source /mnt/d/ai_models/groundingdino-venv/bin/activate
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export PYTHONPATH=/mnt/d/ai_models/GroundingDINO:$PYTHONPATH
export MPLCONFIGDIR=/tmp/matplotlib-cfg
mkdir -p "$MPLCONFIGDIR"

roslaunch sagittarius_object_color_detector language_guided_calibration_base.launch \
  video_dev:=/dev/video0 \
  pixel_format:=mjpeg \
  image_width:=640 \
  image_height:=480 \
  framerate:=10
```

这个 launch 只启动基础服务和相机，不会主动执行标定点移动。标定移动会在终端 2 的交互脚本里按步骤触发。

终端 2：运行交互式语言标定脚本：

```bash
cd ~/sagittarius_ws
source /mnt/d/ai_models/groundingdino-venv/bin/activate
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export PYTHONPATH=/mnt/d/ai_models/GroundingDINO:$PYTHONPATH
export MPLCONFIGDIR=/tmp/matplotlib-cfg
mkdir -p "$MPLCONFIGDIR"

rosrun sagittarius_object_color_detector language_guided_calibration.py \
  _target_text:="red block" \
  _device:=cuda \
  _image_topic:=/usb_cam/image_raw \
  _groundingdino_config:=/mnt/d/ai_models/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py \
  _groundingdino_weights:=/mnt/d/ai_models/GroundingDINO/weights/groundingdino_swint_ogc.pth \
  _vision_config:=$HOME/sagittarius_ws/src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/config/vision_config.yaml \
  _output_csv:=$HOME/sagittarius_ws/src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/config/manual_calibration_points.csv \
  _image_dir:=/tmp/language_guided_calibration \
  _place_z:=0.12 \
  _place_z_fallbacks:=0.12,0.15,0.18 \
  _update_vision_config:=true
```

如果你在做 `left/right` 多观察位标定，不要只改 `observe_y`，还要同步给 `observe_yaw` 一个明显的偏航角。例如：

```bash
# left
_observe_x:=0.20 _observe_y:=0.08 _observe_z:=0.15 \
_observe_roll:=0.0 _observe_pitch:=1.57 _observe_yaw:=0.45

# right
_observe_x:=0.20 _observe_y:=-0.08 _observe_z:=0.15 \
_observe_roll:=0.0 _observe_pitch:=1.57 _observe_yaw:=-0.45
```

否则机械臂只是“平移到左边/右边”，相机朝向并没有真正左转/右转，多观察位标定意义会很弱。

运行后按终端提示操作：

1. 机械臂移动到第 1 个已知点。
2. 把红色方块放到夹爪正下方。
3. 按回车。
4. 机械臂移到观察姿态，相机拍照，GroundingDINO 记录像素中心。
5. 重复到第 5 个点。

脚本会自动生成：

```text
src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/config/manual_calibration_points.csv
/tmp/language_guided_calibration/point_01_raw.jpg
/tmp/language_guided_calibration/point_01_annotated.jpg
...
```

并写回：

```text
src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/config/vision_config.yaml
```

如果想先只采集和试算，不写回配置，把 `_update_vision_config:=true` 改成：

```bash
_update_vision_config:=false
```

如果第一个标定点仍然出现 MoveIt 自碰撞或 `result=3`，说明当前机械臂起始状态不适合直接去低位参考点。先不要硬抓，可以把标定参考高度调高：

```bash
_place_z:=0.15 \
_place_z_fallbacks:=0.15,0.18,0.22
```

标定只需要 `x/y` 已知，不要求夹爪真的贴近桌面。把红块放在夹爪垂直投影下方即可。

手动标定需要准备一个 CSV 文件，记录几组点：

```text
pixel_x,pixel_y,robot_x,robot_y
```

含义：

- `pixel_x,pixel_y`：目标在相机图像中的中心点，可以从标注图或日志里的 `center` 读取
- `robot_x,robot_y`：你希望机械臂在桌面平面上抓取的实际坐标，单位是米

仓库里提供了示例文件：

```text
src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/config/manual_calibration_points.example.csv
```

正式标定时复制一份：

```bash
cd ~/sagittarius_ws
cp src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/config/manual_calibration_points.example.csv \
   src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/config/manual_calibration_points.csv
```

把 `manual_calibration_points.csv` 里的示例数值换成你自己测到的点，建议至少 5 个点：中心、左上、右上、左下、右下。

推荐采集方式：

1. 运行单机脚本，但先关闭真实抓取，只保存原图和标注图。
2. 在桌面上放置红色方块，发送 `red block`。
3. 读取日志里的像素中心，或打开标注图查看中心点。
4. 把这个方块在桌面上对应的机械臂平面坐标记为 `robot_x,robot_y`。
5. 移动方块到下一个位置，重复采集至少 5 个点。

采集命令示例：

```bash
cd ~/sagittarius_ws
EXECUTE_GRASP=false \
DEVICE=cuda \
SAVE_RAW_IMAGE=true \
RAW_IMAGE_PATH=/tmp/language_guided_grasp_raw_calib.jpg \
SAVE_ANNOTATED_IMAGE=true \
ANNOTATED_IMAGE_PATH=/tmp/language_guided_grasp_annotated_calib.jpg \
bash src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/scripts/run_single_machine_gpu_grasp.sh
```

新开一个终端发送目标：

```bash
cd ~/sagittarius_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rostopic pub /grasp_target_text std_msgs/String "data: 'red block'" -1
```

查看保存结果：

```bash
ls -l /tmp/language_guided_grasp_raw_calib.jpg /tmp/language_guided_grasp_annotated_calib.jpg
```

如果需要从 Windows 打开图片，可以复制到桌面：

```bash
cp /tmp/language_guided_grasp_raw_calib.jpg /mnt/c/Users/HankL/Desktop/
cp /tmp/language_guided_grasp_annotated_calib.jpg /mnt/c/Users/HankL/Desktop/
```

先只试算，不写入配置：

```bash
cd ~/sagittarius_ws
python3 src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/nodes/manual_vision_calibration.py \
  --csv src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/config/manual_calibration_points.csv \
  --vision-config src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/config/vision_config.yaml \
  --dry-run
```

确认输出的平均误差合理后，写回 `vision_config.yaml`：

```bash
cd ~/sagittarius_ws
bash src/sagittarius_arm_ros/sagittarius_perception/sagittarius_object_color_detector/scripts/run_manual_vision_calibration.sh
```

脚本会自动备份旧配置，备份文件形如：

```text
vision_config.yaml.bak_YYYYMMDD_HHMMSS
```

写回后重新启动语言抓取节点，并确认启动日志不再出现：

```text
vision_config mapping is degenerate
```

### 真实相机闭环

```bash
roslaunch sagittarius_object_color_detector language_guided_grasp.launch \
  device:=cuda \
  video_dev:=/dev/video0 \
  pixel_format:=mjpeg \
  image_width:=640 \
  image_height:=480 \
  framerate:=10 \
  min_grasp_score:=0.35 \
  publish_annotated_image:=true \
  save_raw_image:=true \
  raw_image_path:=/tmp/language_guided_grasp_raw_live.jpg \
  save_annotated_image:=true \
  annotated_image_path:=/tmp/language_guided_grasp_annotated_live.jpg \
  state_topic:=/language_guided_grasp/state \
  groundingdino_config:=/mnt/d/ai_models/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py \
  groundingdino_weights:=/mnt/d/ai_models/GroundingDINO/weights/groundingdino_swint_ogc.pth \
  search_pose_mode:=camera_down \
  search_pose_x:=0.20 \
  search_pose_y:=0.00 \
  search_pose_z:=0.15 \
  search_pose_pitch:=1.57 \
  return_to_search_pose_after_grasp:=false \
  pick_orientation_mode:=auto \
  drop_after_grasp:=false \
  clear_target_after_success:=true
```

### 测试图片代替相机

```bash
roslaunch sagittarius_object_color_detector language_guided_grasp_image_test.launch \
  device:=cuda \
  execute_grasp:=false \
  min_grasp_score:=0.35 \
  save_annotated_image:=true \
  annotated_image_path:=/tmp/language_guided_grasp_latest.jpg \
  save_raw_image:=true \
  raw_image_path:=/tmp/language_guided_grasp_raw_test.jpg \
  state_topic:=/language_guided_grasp/state \
  groundingdino_config:=/mnt/d/ai_models/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py \
  groundingdino_weights:=/mnt/d/ai_models/GroundingDINO/weights/groundingdino_swint_ogc.pth \
  search_pose_mode:=none \
  return_to_search_pose_after_grasp:=false \
  pick_orientation_mode:=auto \
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
  device:=cuda \
  execute_grasp:=false \
  save_annotated_image:=true \
  annotated_image_path:=/tmp/language_guided_grasp_latest.jpg \
  save_raw_image:=true \
  raw_image_path:=/tmp/language_guided_grasp_raw_test.jpg \
  state_topic:=/language_guided_grasp/state \
  groundingdino_config:=/mnt/d/ai_models/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py \
  groundingdino_weights:=/mnt/d/ai_models/GroundingDINO/weights/groundingdino_swint_ogc.pth \
  search_pose_mode:=none \
  return_to_search_pose_after_grasp:=false \
  pick_orientation_mode:=auto \
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
  --device cuda \
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
- 在当前语言抓取基础上进一步接入更强的 VLM / 多模态策略，用视觉语言模型产生更高层任务决策

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
