# RC 具身智能项目

更新日期：2026-10-13。进入含 `main.py` 的目录，仍运行 `python3 -B main.py`。本次新增点位剩余数量调度、腕部自主选物和模型复用；延续最后30秒优先离场、识别播报与抓取并行。修改内容和验证范围见 [2026-10-13 修改说明](docs/UPDATE_20261013.md)。

## 原功能文件与调用关系

原抓取动作、坐标、补偿和开合爪参数保留。RealSense 识别模块增加模型缓存，调用接口取消 Kinect 类别对腕部选物的限制。

| 原功能文件 | 当前调用 |
| --- | --- |
| `arm/task2_pick_v4.py` | 沿用 `task2_pick_v4(2).py` 动作，柜层由 `main.pick_shelf_original` 直接调用 `pick_on_shelf`，模型路径指向集中后的 `best(1).pt` |
| `arm/xg_table_pick.py` | 家具表面：`pick_surface` 调用 `pick_once` |
| `arm/catch_ty.py` | 地面：`pick_ground` 调用原类的 `catch_ground` |
| `catch_ground/src/catch.py` | 原 Kinova 动作发送及单位换算 |
| `catch_ground/src/realsense_yolo11.py` | 地面腕部识别器，复用同一权重/设备的模型 |
| `catch_ground/src/realsense_yolo11_ref.py` | 桌面和柜层共用的腕部识别器，复用同一权重/设备的模型 |
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

`arm/driver.py::load_original` 为柜层和桌面入口指定同一个 `models/best(1).pt`。柜层原脚本自身的模型默认路径也已指向该文件。桌面腕部按原脚本的18类 `TARGET_NAMES` 自行选物，不再限定为 Kinect 识别出的单一类别。已核对的六类映射只用于记录实际抓取类别；其他类别保留腕部原标签，显式 `excluded_labels` 仍生效。

设备准备阶段预加载已启用点位所需的腕部模型，同一识别模块中的同一权重和设备只加载一次。每次抓取仍创建独立相机管道、重新采集图像和深度、重新计算坐标，并按原流程预热和关闭相机；本次没有让 RealSense 常驻，也不复用上一次的抓取坐标。

## 点位数量与遍历顺序

在 `config/navigation.local.json` 的每个 `detection_points[]` 中填写 `object_count`，不填默认1，填0则跳过该点。例如原点位对象增加 `"object_count": 2`，表示这个点计划确认抓取两件。该数由现场填写，不根据 Kinect 数框自动更新。

- 每轮按配置列表顺序访问所有剩余点位，每点执行一次完整抓取。
- 沿用现有夹爪反馈判定，确认成功后剩余数量减1；同一物品ID重复登记、投放重试不重复扣减。
- 数量减到0后退出待搜索队列，但继续完成当前持物的投放。原 JSON 点位不删除。
- 抓取失败、没有目标或导航失败不扣数量，下一轮重新到该点；腕部原脚本内部的采图重试和柜层搜索仍保留。
- 所有点位数量完成则离场；否则持续遍历到离场预留时间。旧 `empty_confirmations`、`max_visits_per_point` 不再用于结束搜索。
- `--resume` 从恢复快照沿用剩余数量和成功ID，不重置数量或比赛时钟；新场次重新使用配置的 `object_count`。

## 本次目录整理

删除已不参与主流程的六份文件：`arm/original_kinova.py`、`arm/catch.py`、`arm/task2_pick.py`、`arm/ground_coordinates.py`、`arm/shelf_coordinates.py`、`arm/observe.py`。同时删除只为这些旧入口服务的重复坐标计算和观测方法。

`arm/coordinates.py` 仅保留驱动反馈比较实际使用的欧拉角转四元数。`camera/wrist.py` 仍被资源管理、错误恢复和关闭路径引用，因此保留。原 SDK 导入的两个 RealSense 模块及三级开柜、开抽屉功能也保留。

## 延续的运行行为

- RealSense 每次识别最多 5 次，识别成功立即继续；柜层按层计数，一层无目标会检查剩余未查层。同一柜子后续抓取轮换起始层。
- 地面使用当前物品模型，在有效腕部识别后播报“地面发现××，开始抓取”，播报发送不等待播放完成。
- 普通前伸按 `jujia26` 顺序等待返回后继续合爪、收紧和收臂；普通动作的非成功状态或单次等待超时记录后继续。收臂完成后才判断是否持物。
- 投放位普通返回失败或单动作等待超时后仍执行张爪，再确认释放和收臂。初始化、真实收臂失败、急停、ROS 关闭及硬截止保护继续保留。
- 地图目标发送配置中的完整 XYZ，允许打点记录的非零 Z；初始化定位消息仍沿用平面 Z=0。
- Kinect 用于识别播报，抓取目标和坐标由腕部识别决定。没有 Kinect 候选仍会进入腕部识别。
- 识别播报先排队，主流程立即继续抓取；后台每句收到 `/summer_tts_done` 后才发送下一句，避免挤满语音节点队列。播报失败只影响语音队列。
- 当前600秒比赛在第570秒优先中断普通任务，取消旧动作、必要回撤及收臂后导航离场，保留持物。离场准备按剩余时间减去 `exit_estimate_s` 限时，仍受总截止、急停和收臂条件约束。

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

在项目根目录运行离线回归，不连接 ROS、机器人或相机：

```bash
python3 -B -m unittest discover -s testing -p 'test_*.py' -v
```

测试覆盖点位数量、成功去重、失败复查、恢复数量、Kinect错误类别下的腕部选物、模型缓存与新相机管道、新图片，以及播报队列和离场优先级；包含 Linux SIGALRM 对阻塞等待的实际中断检查。SDK、YOLO推理和action传输使用替身，实际识别效果、加载耗时和实体抓取仍需现场验证。当前状态见 [PROJECT_STATUS](docs/PROJECT_STATUS.md)。
