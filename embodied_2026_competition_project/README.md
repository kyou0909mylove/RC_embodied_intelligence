# RC 具身智能项目

更新日期：2026-10-12。进入含 `main.py` 的目录，仍运行 `python3 -B main.py`。柜层沿用上传的 `task2_pick_v4(2).py` 抓取动作；本次统一模型配置并删除重复权重，保留主函数、启动参数、已填导航点位和抓取参数。

## 原功能文件与调用关系

以下六份功能源码的动作、坐标、补偿和开合爪参数保持不变。`arm/task2_pick_v4.py` 仅修改模型路径及对应说明，其余五份沿用本次输入文件。

| 原功能文件 | 当前调用 |
| --- | --- |
| `arm/task2_pick_v4.py` | 沿用 `task2_pick_v4(2).py` 动作，柜层由 `main.pick_shelf_original` 直接调用 `pick_on_shelf`，模型路径指向集中后的 `best(1).pt` |
| `arm/xg_table_pick.py` | 家具表面：`pick_surface` 调用 `pick_once` |
| `arm/catch_ty.py` | 地面：`pick_ground` 调用原类的 `catch_ground` |
| `catch_ground/src/catch.py` | 原 Kinova 动作发送及单位换算 |
| `catch_ground/src/realsense_yolo11.py` | 原 RealSense 识别器 |
| `catch_ground/src/realsense_yolo11_desk.py` | 原 SDK 导入依赖 |

`pick_surface.py`、`pick_shelf.py`、`pick_ground.py` 保留为独立调用接口，原动作仍在对应原功能文件里。投放、收臂、初始化、开柜、开抽屉也各自保留独立文件。

最新版柜层的每层 `grasp_delta`、物品类型补偿、药盒侧面Z补偿、中高层目标偏左时的中间点全部原样保留。低/中/高层闭爪后抬升分别为原文件的0.03/0.05/0.03米。中间点最多额外等待3次，每次0.5秒，超过后原代码继续前伸和合爪，不以距离判断的布尔返回值阻断。

## 模型统一放在 models

| 文件 | 用途 |
| --- | --- |
| `models/best.pt` | 21 类，头部和地面物品识别 |
| `models/best(1).pt` | 18 类，家具表面和原 v4 柜层识别 |
| `models/TABLE_MODEL_INFO.json` | `best.pt` 的类别和哈希资料 |
| `models/DESK_MODEL_INFO.json` | `best(1).pt` 的类别和哈希资料，桌面和柜层共用 |

原 `models/table.pt` 与上传的 `best.pt` 字节完全相同，此前已删除。本次确认 `best(3).pt` 与 `best(1).pt` 字节完全相同，删除 `best(3).pt`，两个保留模型的内容不变。

顶层 `model` 和 `actions.profiles.ground.model` 使用 `../models/best.pt`；`actions.profiles.surface.model` 和 `actions.profiles.shelf.model` 使用 `../models/best(1).pt`。两份 JSON 的桌面、柜层类别和哈希资料已同步，`label_map` 沿用已明确的六类头部到腕部映射。配置读取会合并 local 与 example，因此两份文件均须保留本次修改。

`arm/driver.py::load_original` 为柜层和桌面入口指定同一个 `models/best(1).pt`。柜层原脚本自身的模型默认路径也已指向该文件，删除重复权重后独立入口仍可找到模型。桌面接口 `arm/pick_surface.py` 仍使用原有六类映射；本次没有扩展可抓类别。

## 本次目录整理

删除已不参与主流程的六份文件：`arm/original_kinova.py`、`arm/catch.py`、`arm/task2_pick.py`、`arm/ground_coordinates.py`、`arm/shelf_coordinates.py`、`arm/observe.py`。同时删除只为这些旧入口服务的重复坐标计算和观测方法。

`arm/coordinates.py` 仅保留驱动反馈比较实际使用的欧拉角转四元数。`camera/wrist.py` 仍被资源管理、错误恢复和关闭路径引用，因此保留。原 SDK 导入的两个 RealSense 模块及三级开柜、开抽屉功能也保留。

## 延续的运行行为

- RealSense 每次识别最多 5 次，识别成功立即继续；柜层按层计数，一层无目标会检查剩余未查层。同一柜子后续抓取轮换起始层。
- 地面使用当前物品模型，在有效腕部识别后播报“地面发现××，开始抓取”，播报发送不等待播放完成。
- 普通前伸按 `jujia26` 顺序等待返回后继续合爪、收紧和收臂；普通动作的非成功状态或单次等待超时记录后继续。收臂完成后才判断是否持物。
- 投放位普通返回失败或单动作等待超时后仍执行张爪，再确认释放和收臂。初始化、真实收臂失败、急停、ROS 关闭及硬截止保护继续保留。
- 地图目标发送配置中的完整 XYZ，允许打点记录的非零 Z；初始化定位消息仍沿用平面 Z=0。

## 启动与现场配置

提前启动已跑通的底盘导航、Kinova 驱动及 SummerTTS；在主函数终端加载实际 ROS/工作空间环境。`setup_network.bash` 中的网络值须与现场一致。关闭其他占用相机的采图程序。

```bash
cd /home/zq/embodied_2026_competition_project
source /opt/ros/noetic/setup.bash
source /home/zq/catkin_ws/devel/setup.bash
source setup_network.bash
python3 -B main.py
```

默认配置为 `config/navigation.local.json`。现场后来重新打过点，应将最新点位合入这份配置；`navigation.example.json` 是含未填写点位的模板，不用于覆盖现场 local。

```bash
python3 -B main.py --check-config
python3 -B main.py --list-labels
python3 -B main.py --resume
```

前两条只检查配置或显示类别，不连接设备。`--resume` 恢复同场记录并保留原截止时间；新场次正常运行 `main.py`。只有实际摆在配置初始点、需要发布定位先验时使用 `--set-initial-pose`。`--no-log` 关闭完整日志和图片，仍保留恢复文件。

柜层和表面抓取姿态以对应原功能文件为准，旧 JSON 中同名抓取几何不会覆盖这些原脚本。修改地图点、home/place、任务时间和反馈参数仍在 local；模型资料和中文名称分别对应模型配置与 `speech.label_names`。

## 核验

2026-10-09 基线记录：50 份 Python 完成静态编译、Python 3.8 语法和缩进检查，73 项离线回归通过。该记录是此前版本的检查结果。本次完成配置检查、保留模型哈希检查、模型路径检查和修改源码的语法检查；`main.py`、导航点位、抓取动作和保留权重的内容与本次输入相同。

离线回归使用 SDK、检测结果和 action 客户端替身，未连接真实机器人，也未验证 Linux SIGALRM。交付不包含测试脚本、日志、缓存或临时文件。来源及校验记录见 `SOURCE_INFO.json`，当前状态见 `docs/PROJECT_STATUS.md`。
