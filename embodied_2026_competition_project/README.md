# 2026 具身智能项目

先读本页完成启动；功能状态、主流程与比赛策略集中在 [PROJECT_STATUS.md](docs/PROJECT_STATUS.md)，缺少的文件和现场信息集中在 [MISSING_FILES.md](docs/MISSING_FILES.md)。

## 目录

| 位置 | 用途 |
| --- | --- |
| `main.py` | 比赛主入口骨架；真实抓放尚未接入 |
| `core/` | 流程、初始化、计时、停止和记录 |
| `navigation/` | 底盘、导航、初始位姿；保留原导航辅助代码 |
| `camera/` | 原相机接口与真实检测进程 |
| `arm/` | 原抓取代码；当前 `actions.py` 只打印 |
| `speech/` | SummerTTS 发布/回执封装与节点源码 |
| `config/` | 地图点位及流程配置 |
| `models/` | 已上传的 `best.pt` 与标签信息 |
| `testing/` | 导航演练、打点、离线检查 |
| `docs/` | 两份详细说明 |
| `SOURCE_INFO.json` | 来源、模型哈希、原硬件接口签名 |

## 第一次部署

1. 保留已经跑通的 Ubuntu / ROS Noetic / Python 环境，不需要先升级版本。
2. 把旧工程**已经填好的** `config/navigation.local.json` 复制到本目录；不要用空模板覆盖。原六点、schema_version=4 配置仍可读。没有本地配置时，才复制模板并填写。
3. 六个导航点之外，单独填写 `initial_pose`：每次启动时机器人固定摆放位置及朝向。全部位姿格式为 `[[x,y,0],[0,0,qz,qw]]`，来自打点时同一张 map。
4. 按原来的方式启动底盘、原地图、定位和 move_base。这个工程不代替原 ROS 驱动启动包。
5. 启动原 SummerTTS 节点；没有语音节点时，可先使用下面的 `--speech-print` 命令。

`../models/best.pt` 相对配置文件所在目录解析。我们没有你电脑上的六个实测点，模板保留 `null`，未填完整会在连接设备前报出缺项。

## 一条命令测试

在项目根目录，加载原来已经使用的 ROS 环境：

```bash
source /opt/ros/noetic/setup.bash
source ./setup_network.bash
python3 -B testing/test_navigation_flow.py
```

如果原流程还要求加载你的工作空间 `devel/setup.bash`，照原流程加载。也可以进入 `testing/` 后直接运行 `python3 test_navigation_flow.py`。

测试入口自动选择本地配置、启用真实导航、设置初始位姿、生成唯一测试编号，默认不保存流程日志。机器人必须已经摆在初始位姿对应的固定位置和朝向；发布先验后，在 RViz 确认地图、姿态、激光对齐，再在终端输入 `OK`。

流程为：底盘初始化、相机/机械臂模拟初始化 → 初始定位确认 → 入场 → 检测1/模拟识别/语音/抓取打印 → 投放/放下打印 → 检测2/同流程 → 检测3/同流程 → 离场。三点正常完成共八个导航目标。检测点列表可继续添加。

暂时没有真实语音：

```bash
python3 -B testing/test_navigation_flow.py --speech-print
```

此选项只打印，不出声。若本次已经手动完成定位且无需重新发布先验，使用 `--localization-confirmed`；它与 `--set-initial-pose` 互斥。

新模板 `speech.wait_done=true` 会等待上传新版节点的 `/summer_tts_done`。旧本地配置没有这个字段时，继续按异步播报运行。C++ 节点源码已加入，完整编译包/合成模型仍需确认，见缺件说明。

## 主入口、检查、停止

```bash
python3 -B main.py --real --match-id main_001 --set-initial-pose
```

主入口与测试共用流程，但主入口会启动真实 Kinect 检测；机械臂仍只打印。真实检测为空会跳过抓取和投放。**具体规则要求至少一件完成识别、抓取并放入己方得分区后才可自主离场。当前没有真实交付反馈，main 最终会停车并返回 STOPPED_WITHOUT_DELIVERY，不会因打印抓放而发送离场目标。**测试可完整演练离场，不能计作比赛成功。

主入口实机模式要求本地配置 `start.mode="topic"`，使用原有开始接口在裁判开门时发布 `start.topic` 配置的非锁存 String 消息。本工程不新增门/按钮驱动，也不能把新 match-id 当作重置正式比赛时间的办法。导航测试会自动使用延迟开始，不改本地配置。

识别以语音为主要判定；有争议时规则要求带时间戳、物品框及名称的图片。新模板 `recording.save_images=true`，每个检测点只保存选中目标的一张关键 JPG；旧本地配置若为 false，请显式改 true。主入口保留证据时省略 `--no-log`；测试继续默认不落图/日志。

离线检查不连接设备：

```bash
python3 -S -B testing/check_project.py
python3 -S -B testing/check_flow_logic.py
python3 -S -B main.py --config config/navigation.local.json --check-config --set-initial-pose
```

连接 ROS 时不要使用 `-S`。Ctrl+C 或向配置的停止话题 `/embodied/stop` 发布 `std_msgs/Bool` 的 `true`，会取消导航并请求零速。

`match-id` 用于记录和防重复启动，不影响地图坐标。测试默认不存图或流程日志，仍保留少量场次启动标记；外部 ROS 日志不由本程序管理。主入口省略 `--no-log` 才会保存本程序的流程记录。
