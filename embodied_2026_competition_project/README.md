# 2026 具身智能机器人项目

版本v13，2026-10-07；配置schema_version仍为7。本页说明main的现场运行、参数和开发位置。详细报错见[ERROR_GUIDE.md](docs/ERROR_GUIDE.md)，全部时间阈值见[TIMEOUTS.md](docs/TIMEOUTS.md)，完整逻辑见[MAIN_FLOW_V13.md](docs/MAIN_FLOW_V13.md)。

这是已合并的完整项目文件夹，包含运行代码、现场配置、抓取与坐标换算模块、视觉模型及最新文档。解压后进入能看到 `main.py` 的目录，按下方启动顺序运行。如果你之后重新打过点，先将现场最新点位合入 `config/navigation.local.json`。

## 1. main如何完成比赛

运行 `python3 -B main.py`，默认使用config/navigation.local.json、连接真实设备，不需要--real或按钮/裁判开始topic。现场允许启动时直接运行。

启动顺序：完整校验参数/模型 → 打开场次记录 → ROS/导航 → Kinova连接 → 自动张爪/回配置home → 腕部RealSense → 头部Azure Kinect → 语音 → 开始600秒任务计时。恢复已有场次保留原时钟，重连时间也计入。

路线开始：保留当前地图定位，实际导航到initial_pose，再到entry；两点完全相同则不重复。默认不是把initial_pose发布成当前位置。--resume已完成入场时不回起点，先恢复剩余回程/收臂和负载。

搜索按detection_points列表顺序逐轮遍历：清旧动态障碍 → 导航检测点 → 头部识别/播报全部稳定物品及数量 → 腕部当前家具视角重测深度/角度 → 选一件 → 前伸/闭爪/回程/收臂 → 三指判断 → 有物或疑似持物优先回drop → 到点直接put → 更新编号/游标 → 下一点。腕部补报只发语音请求、不等播放完成，不让坐标因等语音变旧。

每轮一个待查点访问一次，一次运送一件。成功抓取不退休该点，下轮可以继续抓同家具的其他物品。只有头部无物品且全部配置腕部视角实际完成无框，连续两轮确认空才结束复查；默认访问次数不限，不需填每家具库存。

| 难度/场景 | 独立实现 |
| --- | --- |
| 一级，桌面/床面等开放家具表面 | arm/pick_surface.py + coordinates.py |
| 二级，三层开放柜 | arm/pick_shelf.py + shelf_coordinates.py；当前mid→high→low |
| 二级，无遮挡0.3m圆圈中的平躺地面物品 | arm/pick_ground.py + ground_coordinates.py；沿用jujia26实际调用catch_ty |
| 三级，柜门/抽屉内部 | open_cabinet/open_drawer接口保留；打开后分别复用shelf/ground；当前未启用 |

| 特殊情况 | 当前处理 |
| --- | --- |
| 检测点不可达（包括move_base=4/在线的9） | 有限重试后只跳过本轮，下轮清临时状态再试 |
| 识别有框但没深度/不稳定、抓空、观察没到达 | 本点有限重试，不记家具空；120秒阈值（地面180秒）后恢复换点 |
| 收臂/释放额外到位误差、三指边界/不稳定 | 有新鲜有效反馈时提示，有限恢复；疑似持物不丢状态、不虚计成功 |
| 投放点被挤占 | 停止等5秒、清图重发同drop，共3次/总120秒；耗尽且接口在线时先在当前收臂处张爪确认、再home，之后才去下一家具 |
| 到drop后holding读数不确定 | 不读取holding；直接place→张爪→确认张开→home |
| 默认已用535秒 | 取消新搜索/投放，保留负载、完成必要收臂，优先离场；持物允许离场 |
| 离场点受阻 | 失败等5秒并清图重试，直到到达或600秒TIME_UP；仅实际到达算离场完成 |
| 持续设备/通信/执行无响应 | 有限复核/恢复失败才暂停；初始/入场失败也允许暂停 |

场地封闭；当前不接人员监测、ROI、自定义末端范围。底盘避障由现有move_base、激光、costmap及TF负责，main下发目标并等待/清图/重试。失败后是否转圈取决于导航栈行为，main没有转圈示意命令。

## 2. 现场启动顺序

### 2.1 底盘导航终端

沿用你们已成功的命令，在底盘机器启动相同地图：

```bash
ssh eaibot@192.168.31.200
roslaunch dashgo_nav navigation_imu_2.launch
```

SSH用来远程启动程序；ROS通信本身依赖master和双向网络。RViz核对当前定位/激光与地图墙对齐。

### 2.2 各结点

```bash
export ROS_MASTER_URI=http://192.168.31.200:11311
export ROS_IP=192.168.31.5
```

### 2.3 main终端

```bash
cd /home/zq/embodied_2026_competition_project
source /opt/ros/noetic/setup.bash
source /home/zq/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.31.200:11311
export ROS_IP=192.168.31.5
python3 -B main.py
```

每个终端分别source，其他终端的环境不会自动传过来。使用你们可加载相机SDK/模型的Python环境。顶部Kinect和腕部RealSense由main打开，不需要额外相机ROS节点；关闭占用它们的采图程序。-B只关闭Python缓存，不改变任务。

新场次自动张爪/收臂；home为local中的 `[0.212583,-0.255877,0.506381,94.805,63.854,6.944]`。不会因为操作者未事先手动摆回home或初始三指非空阈值单独阻止启动，仍要真实驱动动作成功。保留现场填写的地图、home/place、手眼和模型数值。

若已实际摆在initial_pose、需要给AMCL初始位置提示，才运行 `python3 -B main.py --set-initial-pose`；默认用当前定位真正导航到它。机器人位于其他已定位位置时，不能发布起点坐标冒充当前位置。关闭起点导航可改localization.navigate_to_initial_pose=false。

同场设备排障后运行 `python3 -B main.py --resume`，不重新获得600秒，不重新发布固定起点；新比赛正常main。--match-id是可选场次编号。--no-log只关闭完整日志/关键图，仍保存恢复文件。

### 2.5 启动异常快速定位

| 输出 | 先检查 |
| --- | --- |
| 参数错误 | 输出的配置文件、JSON路径、行号；普通main现在自动完整检查 |
| unable to communicate with master/init_node等待 | 底盘master、ROS_MASTER_URI/ROS_IP、本机IPv4和网络；不是必须先SSH才允许ROS |
| kinova_msgs/kinova_bringup找不到 | 本终端source实际Kinova工作空间，rospack find kinova_bringup |
| arm/finger服务或话题无反馈 | 驱动终端、型号前缀，动作等待和反馈均有限时间 |
| 相机打不开 | USB/供电、占用程序、SDK、serial和流分辨率 |
| 语音无订阅 | 提前启动SummerTTS、检查topic/network；节点在线但done失配可继续 |
| No space left on device | 检查ROS日志盘空间及inode；main备用记录不代替外部ROS节点修复 |

地图pose是底盘map停靠位，detect_pose是机械臂base下的观察位。底盘“已到达”之后arm action=4属于机械臂动作失败，不能据此认定地图点不可达。查看动作步骤、目标/实际位姿、反馈年龄以及驱动日志，具体命令见ERROR_GUIDE。

## 3. 参数由谁填、填在哪里

导航、视觉、机械臂统一修改 **`config/navigation.local.json`**。下面的 Python 文件是参数读取或运算位置，通常不需要修改其中的常量。

`navigation.example.json` 是完整默认模板，地图坐标为空；不要用它覆盖已填好的 local。相对路径从配置目录计算，例如 `../models/table.pt` 和 `../runs`。

### 3.1 导航组：地图、停靠点与初始定位

底盘位姿为 `[[x,y,0],[0,0,qz,qw]]`，单位米，坐标系 `map`；四元数须归一化。平面朝向为 yaw 弧度时，`qz=sin(yaw/2)`、`qw=cos(yaw/2)`。它与机械臂六维位姿不是同一种参数。

| local 字段 | 含义/当前用法 | 读取代码 |
| --- | --- | --- |
| `initial_pose` | 起始导航目标，格式与其他map点相同；默认使用当前定位实际导航过去 | `core/workflow.py`、`core/session.py` |
| `localization.navigate_to_initial_pose` | 新任务是否先导航到initial_pose，默认true；不重置当前位置 | 导航组 / `core/configuration.py`、`core/workflow.py` |
| `localization.initial_pose_wait_s` | 发布前准备等待，默认5秒 | `navigation/initial_pose.py` |
| `locations.entry` | 入场导航目标 | `core/workflow.py` |
| `locations.drop` | 己方得分区底盘停靠目标；须与机械臂释放位配套 | `core/workflow.py`、`arm/put.py` |
| `locations.exit` | 离场导航目标 | `core/workflow.py` |
| `detection_points[].pose` | 到该家具观测/抓取时的底盘位置和朝向 | `core/workflow.py` |
| `detection_points[].reposition_points` | 同家具备用底盘点的关系记录，当前仅校验引用，不自动插队；实际换点按列表顺序执行 | `core/configuration.py` |
| `flow.skip_unreachable_detection` | 当前true支持观察位失败后看备用位；健康导航接口下的检测点失败始终只跳过本轮，不永久跳过本场 | `core/workflow.py`、`arm/observe.py` |
| `timing.nav_timeout_s` | 一次导航最大等待，当前35秒 | `core/session.py` |

`setup_network.bash` 设置底盘 master 地址和本机 IPv4。本包保留原网络参数。实际地图、costmap、机器人 footprint、激光/TF、速度和导航避障参数，放在原导航工作空间的 YAML/launch；不要填进机械臂姿态。

大桌子可设置多个观察点，例如 `table_left`、`table_right`：每个 `id` 唯一，`pose` 各自打点，`furniture_id` 都写同一个家具 ID。把多个点放进detection_points的预期路线顺序。当前reposition_points只记录/校验关系，不立即改变路线；轮到另一个点时先收臂、导航、重新识别，不复用旧点XYZ。

底盘离家具多少距离须结合机械臂可达范围和腕部可见范围在已搭好的房间确认。代码不会从地图点自动推算最佳抓取停靠距离。

### 3.2 视觉组：两台相机和模型

| 字段 | 作用与取值 | 读取代码 |
| --- | --- | --- |
| `camera.backend`、`camera.color_mode` | **指顶部相机**。当前实现 Azure Kinect、1080p彩色图；不控制腕部 RealSense | `camera/vision.py`、`core/configuration.py` |
| `model.weights` | 顶部模型文件路径，当前 `../models/table.pt` | `camera/vision.py` |
| `model.labels`、`sha256`、`metadata_file` | 精确类别顺序、权重哈希和类别资料；换模型时一起更新 | `core/configuration.py` |
| `model.confidence` | 顶部检测置信度下限，当前0.5；升高会减少低置信度候选 | `camera/vision.py` |
| `model.frames` | 顶部一次识别采集多少帧，当前3；不是摄像头 FPS | `camera/vision.py` |
| `model.imgsz` | 顶部模型推理输入尺寸，当前640；不是相机原始分辨率 | `camera/vision.py` |
| `model.device` | 顶部推理设备；auto 自动选择，cpu 用CPU，字符串0/1指定GPU | `camera/vision.py` |
| `perception.min_hits`、`iou_threshold` | 顶部跨帧至少出现次数（当前2）及框重叠阈值（当前0.4） | `camera/selection.py` |
| `actions.camera_serial` | **腕部 RealSense** 序列号。只有一台 RealSense 时可为空；多台须指定 | `camera/wrist.py` |
| `actions.wrist.width`、`height` | 腕部彩色/深度流分辨率，当前640×480 | `camera/wrist.py` |
| `actions.wrist.frames`、`min_hits` | 腕部一次扫描帧数5、稳定出现至少3次 | `camera/wrist.py`、`selection.py` |
| `actions.wrist.imgsz`、`device` | 腕部推理输入尺寸与计算设备，含义与顶部相同，但分别配置 | `camera/wrist.py` |
| `actions.wrist.iou_threshold` | 腕部同类框跨帧重叠门槛，当前0.4 | `camera/selection.py` |
| `actions.wrist.depth_min_m`、`depth_max_m` | 框中心有效测深距离，当前0.1–1.2米；这是相机深度，不是末端工作范围 | `camera/wrist.py` |
| `actions.wrist.depth_tolerance_m` | 多帧三维位置差容许值，当前0.015米 | `camera/selection.py` |
| `actions.wrist.settle_frames` | 机械臂到观察位后丢弃缓存帧，当前5帧 | `camera/wrist.py` |
| `actions.wrist.max_age_s` | 坐标允许使用的最大年龄，当前2秒；超时会重拍/放弃本次，避免使用旧位置 | `arm/grasp.py` |
| `actions.wrist.max_fresh_retries` | 位置过期时重新测量次数，当前1 | `arm/grasp.py` |
| `actions.wrist.scan_timeout_s` | 一次腕部扫描超时，当前12秒 | `arm/actions.py` |
| `actions.wrist.angle_tolerance_deg` | 地面轮廓角度的多帧差异上限，当前10度 | `camera/selection.py`、`orientation.py` |
| `actions.profiles.*.model` | surface/shelf/ground 各自腕部模型；当前都引用同一份21类权重，置信度0.2 | `camera/wrist.py` |
| `actions.profiles.*.label_map` | 顶部类别名 → 相应腕部模型类别名；当前逐字相同 | `arm/actions.py`、`camera/wrist.py` |
| `excluded_labels` | 全局不选择的类别；当前只有 tablet，身份确认后可调整 | `core/workflow.py`、`camera/wrist.py` |
| `detection_points[].target_labels` | 空列表表示搜索全部允许类别；随机摆放时保持为空 | `camera/vision.py`、`camera/wrist.py` |

`config/targets.json` 是本次完整物品集，不是家具库存或位置表。`model.labels` 必须使用权重中的英文类别；中文播报名放在 `speech.label_names`。当前顶部不测用于抓取的深度，手眼标定对应腕部相机。

### 3.3 机械臂组：驱动、观察姿态、收臂与释放

机械臂六维位姿统一为 `[x,y,z,tx,ty,tz]`：位置单位米，坐标系 `actions.tool_frame`；最后三个是来源 Kinova EulerXYZ 角，单位度。不能直接复制 `tool_pose.orientation` 的四元数四个数到后三项。

| 字段 | 含义 | 读取代码 |
| --- | --- | --- |
| `actions.mode` | 当前必须保持 kinova，真实机械臂模式 | `main.py` |
| `actions.robot_type`、`tool_frame` | 驱动型号/话题前缀与工具反馈坐标系 | `arm/driver.py` |
| `actions.home_pose` | 收臂位；新任务复位、抓取回程、放置回程使用这个值 | `arm/startup.py`、`actions.py` |
| `actions.place_pose` | 在得分区停靠后张爪释放的末端位姿，与 drop 地图点共同决定落点 | `arm/put.py` |
| `actions.finger_open`、`finger_close`、`finger_tight` | 三指0–100百分比目标；全局7/85/100，各抓取 profile 可覆盖 | `arm/startup.py`、`grasp.py`、`put.py` |
| `actions.initial_feedback_timeout_s` | 初始化等待 cartesian_command/finger_position，各默认10秒 | `arm/original_kinova.py` |
| `actions.grasp_timeout_s`、`place_timeout_s` | 一次抓取/放置上限，当前120/45秒；地面点已有grasp_timeout_s=180。到点总等待还受wait_timeout_s约束 | `core/workflow.py` |
| `actions.tool_pose_timeout_s`、`tool_pose_max_age_s` | 到位反馈等待2秒、消息最大年龄0.5秒 | `arm/driver.py` |
| `actions.pose_tolerance_m`、`orientation_tolerance_deg` | 0.015米/5度附加误差提示；动作状态3且有新鲜有效反馈时，超这项只提醒，不单独中断 | `arm/driver.py` |
| `detection_points[].detect_pose` | 桌面或地面主观察位；柜层改用 layers 内的 pose | `arm/pick_surface.py`、`pick_ground.py` |
| `detection_points[].alternate_detect_poses` | 同一底盘停靠点的备用观察姿态列表；为空则只用主位 | 各 `pick_*.py` |
| `detection_points[].lift_dz_m` | 闭爪后抬高距离，当前0.03米；二级地面源路径直接回home，不另加抬起点 | `arm/grasp.py`、`shelf_coordinates.py` |
| `detection_points[].layers[].detect_pose` | 柜子 low/mid/high 三层各自观察姿态 | `arm/pick_shelf.py` |
| `detection_points[].layers[].alternate_detect_poses` | 该层备用观察位；先用主位，再尝试已填备用位 | `arm/pick_shelf.py` |
| `detection_points[].layer_hint` | 从哪层开始，当前 mid；然后向上，再向下 | `arm/pick_shelf.py` |

观察姿态是让腕部看清该表面、且能进入抓取路径的机械臂位姿；一个家具不必只有一种姿态。底盘停靠位置或家具高度变了，需在该实际点位确认观察效果。收臂位与释放位之间不设置额外投放中间姿态。

#### 柜层：最新脚本的参数与完整动作

以下为 `task2_pick_v2.py` 的实际执行值，已写入 local：

| 层 | 观察位 `[x,y,z,tx,ty,tz]` |
| --- | --- |
| low | `[0.270429,-0.074298,0.334523,-93.770,75.993,175.232]` |
| mid | `[0.295184,-0.075805,0.711248,-93.770,75.993,175.232]` |
| high | `[0.366699,-0.085689,0.861343,-93.770,75.993,175.232]` |

每个观察位默认最多识别2次（`actions.profiles.shelf.detect_tries`）；无有效深度物品时，先看该层已配置备用位，再换下一层。若连观察位都没到达，则先收臂并重试同一姿态一次，仍失败再尝试备用位/其他层。移动重试与识别重拍是两件事；均不限制在家具上抓几件。

柜层手眼/经验参数放在 `actions.profiles.shelf`：

- `hand_eye_R` / `hand_eye_T_m`：最新脚本的旋转矩阵/平移，输入顺序保留 `[camera_x,camera_z,camera_y]`。
- `compensation_m: [0,0.085,0.12]`：配合 `[-p_y,p_x,p_z]` 重排后的额外位置补偿，单位米。它不等于重新做手眼标定。
- `detect_euler_deg: [-93.770,75.993,175.232]`：本套公式使用的固定观察朝向；改变朝向不能只改点上的后三个数。
- `align_y_factor: 0.6666666666666666`：对准中间点的 `Y=(观察Y+抓取Y)*2/3`，X、Z保持抓取点值。原脚本注释称“X中点”，与实际语句不同；本项目使用实际语句。
- `execution_mode: "full"`：默认执行对准 → 到抓取点 → 85%/100%闭爪 → 反馈 → 抬起 → XY中点回收 → 配置home → 再次持物反馈。原脚本注释掉的动作已接回。

比赛main要求 `execution_mode: "full"`。`align_only`是来源调试模式，当前会在动作前被参数校验拒绝，不能用main跑到中间点再暂停。

柜层回程的 XY中点来自新脚本 `return_home`，与投放前的中间姿态是不同动作。实现分开在 `pick_shelf.py`、`coordinates.py`、`shelf_coordinates.py` 和共有执行器 `grasp.py`。

#### 地面：jujia26 实际调用的 catch_ty

`actions.profiles.ground` 保存来源手眼、锚点与角度参数；点上 `ground: {}` 表示使用这些默认值。确实需要某点特例时，再写 `detection_points[].ground.anchor_m`、`grasp_z_m` 覆盖。

| 字段 | 来源默认/含义 |
| --- | --- |
| `reference_detect_pose` | `[0.529655,-0.073377,0.494384,174.838,7.703,-89.706]`，源观察位 |
| `observe_settle_s` | 观察位到位后等待3秒再采图，期间仍响应停止 |
| `hand_eye_R`、`hand_eye_T_m` | 沿用 catch_ty.image_to_arm；源调用的相机输入顺序为 X/Z/Y |
| `anchor_m` | `[0.633305,-0.195270,0.257235]`，分段横移/前伸的基准 |
| `grasp_z_m` | `-0.20`，机械臂基座坐标中的抓取Z；不是 map 的高度 |
| `side_clearance_m`、`forward_offset_m` | 横移侧向量0.20米、前伸偏置0.10米 |
| `grasp_euler_xy_deg`、`yaw_offset_deg` | 源固定角度 `[-173,-3.4]`，末端Z角为物品轮廓角减89.4度 |
| `finger_open`、`finger_close`、`finger_tight` | 地面采用5/75/100百分比 |

动作保留：观察/等待 → 测XYZ和角度（腕部补报不等待播放，必要时原位重测）→ 横移 → 前伸 → 下降 → 闭爪75/100 → 直接回配置home → 最后判断持物。没有再添与源地面路径不同的抬起/反向退出点。抽屉仍使用其独立内部几何和分段退出，未启用时无需填写。

已有手眼参数已沿来源接入，不要求重新提交标定结果。新柜层与地面各用自己的公式，不把同一套补偿混用到所有场景。参数有来源仍需与当前机器人安装、家具和停靠点相符。

地面轮廓角度缺失时，`actions.profiles.ground.angle_fallback_deg` 默认0度，恢复往年脚本的默认角度行为；日志会标明采用默认值，不冒充测量结果。若不希望使用默认角度，填null，则等待重拍直到本点阈值。缺少稳定深度仍不能计算抓取坐标。

#### 与原始已跑通代码的对应关系

`hangzhou2026/tongyong_25/jujia26.py` 的 `trash_room` 调用 `KinovaRobotGroud.catch_ground(self.kinova, weights_path=...)`，方法来自同目录 `catch_ty.py`；它复用已有Kinova实例。当前 `pick_ground.py` 与 `ground_coordinates.py` 对应这条路径，使用赛事物品模型和允许类别替代 `paper_ball/empty_bottle`。

地面保留观察后等待3秒、X/Z/Y输入顺序、手眼R/T、横移/前伸/下降三段坐标、5/75/100百分比和闭爪后直接收臂。`arm/original_kinova.py`沿用这几个来源的动作发送和单位换算；v13把服务/动作等待时长改为读取配置，抓取几何和单位换算保持原值。

桌面公式对应 `桌面抓取.zip/xg_table_pick.py`；柜层使用后来上传的 `task2_pick_v2.py`，保留其当前三层姿态和公式。`kinova_wn/task2_pick_final.py` 是另一套较早的柜层参数，不能混入v2。原v2仍仅执行开爪和中间点，其余动作被注释；当前full模式已按此前确认接回完整闭爪、抬起和收臂。

原脚本的 `arm_run` 主要发送目标、等待结果并打印“Cartesian pose sent”，上层不少函数随后直接返回True。当前还检查驱动结果和实际夹爪反馈，避免把“函数已执行完”直接计为抓取成功。

#### 三指反馈如何判断成功

这些参数在 `actions.gripper_feedback`，由机械臂组负责，读取代码 `arm/feedback.py`。

| 参数 | 作用/当前值 |
| --- | --- |
| `empty_closed_turn` | 空夹接近全闭的门槛6750；Kinova满量程约6800turn，不是角度或毫米 |
| `blocked_margin_turn` | 100turn仅用于接近满闭的提醒，不再作为暂停死区 |
| `min_blocked_fingers` | 至少1指在张开与满闭之间，判受阻持物；稳定才计确认抓取 |
| `open_max_turn` | 张开确认阈值800turn |
| `stable_samples`、`stable_span_s` | 至少3条新消息、跨0.12秒；不能重复计同一条消息 |
| `stable_tolerance_turn` | 多条反馈变化不超过60turn |
| `timeout_s`、`max_age_s` | 最多等3秒，只接受0.5秒内的新鲜反馈 |

全开判open、全部≥6750判empty、至少1指在800与6750之间判holding，其他组合unknown。新鲜但不稳定时用末尾样本中值保守保留疑似持物，不计确认抓取。6750附近只提示；持续无新反馈才复核连接。夹住柜沿也可能受阻，这仍是闭合程度判断，不是重量或夹持力。完整闭爪/回程/收臂后判断；到投放点不再检查holding，直接执行put。

`put`的确认交付依据是已确认抓取记录、实际drop到达、释放动作和开爪反馈。疑似抓取也释放，但不计确认交付。没有得分区落点传感器，也不能因取消到点holding复核就证明途中未掉物；现场要对齐drop/place。



### 3.4 路线、恢复、计时与记录

| 参数/记录 | 含义与当前值 | 负责/代码 |
| --- | --- | --- |
| strategy.enabled_levels | [1,2]；未标定三级不加3 | 调度/core/strategy.py |
| detection_points列表顺序 | 每轮按这个顺序，不按抓到两件切换难度；当前柜→桌→地面 | 导航/调度 |
| strategy.empty_confirmations | 连续2次完整空观察；故障/有物不判空 | 视觉/调度 |
| strategy.max_visits_per_point | 0不限；有多个物品保持0 | 调度 |
| recovery.drop_max_attempts/drop_retry_timeout_s/drop_retry_wait_s | 总3次/120秒/间隔5秒；次数和窗口任一先到则结束 | 导航/core/workflow.py |
| recovery.reset_timeout_s/reset_retries | 每次30秒、额外1次；失败复核连接，最后最小恢复也限时 | 机械臂/core/recovery.py |
| recovery.point_wait_timeout_s / point.wait_timeout_s | 默认120，当前地面180；点内总观察/播报/抓取/重试上限 | 视觉/机械臂 |
| timing.match_s及离场预算 | 600秒；当前535秒优先离场，保留持物，详见TIMEOUTS公式 | 现场维护/session.py |
| evidence_dir、recording.save_images | ../runs、true；配置副本/事件/结果和关键图 | 现场维护/evidence.py |
| physical_grasps/physical_deliveries | 新鲜稳定受阻反馈确认抓取，再实际到drop张开才计确认交付，按object_id去重 | core/workflow.py |
| exit_motion_completed | 只在离场导航实际状态3时true，不由已交付数决定 | core/workflow.py |
| abandoned_objects | 投放路线耗尽在当前处释放的物品，不计成功交付 | arm/release.py |
| stop_topic | 默认/embodied/stop，Bool true主动暂停 | session.py/safety.py |

同一家具有多个底盘点时共享furniture_id，各自id/pose唯一；全部放入路线顺序即可。当前reposition_points不立即插队，不自动计算新底盘位。主/备用观察姿态会在当前底盘位轮换，三层柜按hint向上再向下。任一视角未实际完成且没有找到物品，不把家具判空。

完整时间表集中于[TIMEOUTS.md](docs/TIMEOUTS.md)，包含JSON字段、当前值、读代码、每个屏幕阶段和失败去向。新增字段已有默认值并显式放进本版local/example，不需要另外新建配置。调整单点wait_timeout_s只影响这一点。旧逐点冷却/WAIT_RECHECK、难度切换窗口、108秒新工作预算已移除。

## 4. 文件结构与继续开发

| 文件 | 单独职责/改什么时使用 |
| --- | --- |
| main.py | 命令、动作前配置检查、最终错误提示；比赛步骤不写到这里 |
| config/navigation.local.json | 唯一现场参数入口，保留已填物理值；example是默认模板 |
| config/targets.json、models/ | 本次完整物品集、权重和精确标签/哈希资料 |
| core/configuration.py、storage_configuration.py | 合约、模型与参数验证；增加参数同时加校验 |
| core/timeouts.py、safety.py | 新默认值、有限等待/时钟/停止门控 |
| core/workflow.py、strategy.py | 场次流程及配置顺序逐轮调度 |
| core/recovery.py、faults.py | 模块有限复核/恢复与明确故障类型 |
| core/session.py | 连接和资源生命周期、同场时钟、地图导航 |
| core/checkpoint.py、evidence.py、recording.py、diagnostics.py | 状态快照、证据、有限后台落盘、源文件/键/行号提示 |
| navigation/ | 原导航接口、底盘、initialpose发布；真实避障参数仍在原导航工作空间 |
| camera/vision.py、wrist.py | 头部Kinect/腕部RealSense，独立进程采图推理 |
| camera/selection.py、orientation.py | 稳定实例/深度与地面角度 |
| arm/original_kinova.py、driver.py | 来源动作/单位函数、有限动作反馈；不放家具抓取业务 |
| arm/startup.py、actions.py、observe.py | 新场次复位、负载/回程状态、观察位有限重试 |
| arm/pick_surface.py、pick_shelf.py、pick_ground.py | 各场景观察与选一件；保持独立 |
| arm/coordinates.py、shelf_coordinates.py、ground_coordinates.py | 各场景手眼/经验补偿和路径计算；当前公式保留 |
| arm/grasp.py、feedback.py | 共有执行/保存回程、三指闭合程度分类 |
| arm/put.py | 到实际得分停靠点的释放与回执，不等holding |
| arm/release.py | 路线受阻时主动弃物，与得分区put分开 |
| arm/open_cabinet.py、open_drawer.py、storage.py | 三级接口，当前关闭 |
| speech/ | TTS发布与已有节点源代码；运行workspace外部已有 |
| docs/ | 状态、待确认参数、阈值、报错与完整流程图 |

新增家具点：复制example对应场景对象到local的detection_points，填唯一id、共享furniture_id、difficulty、grasp_profile、map pose、机械臂detect_pose/layers并启用。多个桌点按路线顺序排列。不要把地图坐标当机械臂坐标。

新模型：放models，更新model或profiles.*.model的weights/sha256/labels/metadata_file及label_map，同步targets与speech.label_names。新增姿态/超时优先改local；变换公式改对应coordinates，候选选择改对应pick，采图改camera，放置改put，恢复策略改recovery/workflow。注释应说明坐标系/单位、输入输出及失败的去向。

三级未来补storage把手/开门路径/松手/内部layers或ground几何，再启用点并把3加入enabled_levels。当前无需再补按钮、人员监控、ROI或自定义末端XYZ边界。除已知外部驱动/SDK外，主流程没有缺一个待补的本地Python模块；现场仍需确认现有姿态适配当前家具，见[MISSING_FILES.md](docs/MISSING_FILES.md)。

## 5. 自动恢复与排障后继续

正常点内错误会自动有限恢复：取消旧目标、新鲜反馈、已保存的剩余抓取回程、home；空爪张开，疑似持物保留。边界/不稳定反馈不补造分数。已释放回执先保存，收臂失败不重放同一PUT。投放导航失败耗尽则提前在当前处开爪，避免到家具才张开碰到摆好的物品。

持续硬件/通信/执行不可达、参数不合约、初始/入场失败或主动停止才需要人工排障；极端未覆盖程序损坏仍有最后异常防线。仍持物时恢复原场次，不启动会张爪的新场次：

```bash
python3 -B main.py --resume
```

同机恢复包含暂停/重连时间，原场次结束/超时/电脑重启不重新获得600秒。恢复文件损坏会报文件与字段；调度部分可重建，物理负载不能猜成空爪。三级把手中断不自动重放。现有回程是来源路径，不是任意中断姿态的全臂避碰规划。

默认项目runs下每场目录有events.jsonl、config_used.json、result.json、images；父目录有latest_match.json、checkpoint-场次.json。记录盘失败会尝试本机临时备用目录并提示具体位置，resume选择默认/备用最新文件。记录队列满或两处都不可写时只保证当前内存状态，不能保证退出后可恢复。--no-log仍保存恢复文件。

完整图下载：[PNG](docs/MAIN_FLOW_V13.png)、[SVG](docs/MAIN_FLOW_V13.svg)，说明版见[MAIN_FLOW_V13.md](docs/MAIN_FLOW_V13.md)。当前实现进度见[PROJECT_STATUS.md](docs/PROJECT_STATUS.md)。
