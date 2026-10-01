# 2026 具身智能机器人主函数：中文注释版

版本：2026-10-01中文注释版。入口：`main_2026.py`。配置：schema 3。

**首次阅读请打开`READ_CODE.md`。全部13个Python文件均有中文说明，涵盖文件职责、类／函数用途、变量、坐标单位与关键流程。**

基于已维护的统一工程继续完善，以《往年代码.zip》《往年代码2.zip》为机器人接口依据。保留搜索、识别播报、靠近、抓取、持物反馈、运回、放置复核、任务配额和自主离场流程。生产主文件只定义 `main()`；检查脚本是独立开发工具。

**本次只完成静态代码检查、接口一致性核对和打包，没有运行真实机器人或模拟回归。** 本包是待现场标定和联调的工程，配置模板保留空值，不代表可以直接参加比赛。

## 本轮改进

- 机器人运行相关的12个Python文件只补充中文注释，可执行AST与上一版静态交付包一致。检查工具另调整注释版的原始AST校验分支。
- 保持10个旧模块的95个函数／方法声明和原签名；6个模块原可执行AST保留，4个模块沿用上一版对8个已有方法体的维护。
- Kinova构造器在首次开爪／回home前包装两个action client。开始初始化前注入同一截止门控，保持原构造器签名。
- 原导航构造器及两类机械臂初始化反馈在门控启用时使用有限等待；等待后核对停止消息和截止时间。
- 松爪使用配置规定的多个反馈样本，要求三根手指均处于打开范围且稳定；交付仍须独立视觉复核。
- 彩色与对齐深度尺寸须一致；轨迹必须在本次扫描最后一帧出现，早期出现后消失的物品不计成功。
- action等待拒绝零、负值、非有限值和无有限截止的等待；取消异常保留原停止原因。
- 提前保留底盘停止发布器，恢复SIGINT/SIGTERM/SIGALRM处理器；运行记录或ROS关闭失败返回非零。
- 当前入口移除了调用测试替身的`--simulate`模式。上轮验证原文仅保留在历史文档中，未作为本轮结果。

## 文件

| 文件 | 用途 |
|---|---|
| `main_2026.py` | 统一主流程、配置校验、阶段截止与运行记录 |
| `config/competition.example.json` | 18条任务、能力、模型及现场参数模板 |
| `legacy/` | 往年机器人模块；签名与原动作数值保留 |
| `support/safety.py` | 从融合工程沿用的截止门控与action包装 |
| `tools/static_check.py` | 标准库静态检查器，无ROS／SDK／NumPy依赖 |
| `FUNCTIONS.md` | 32项主流程复用接口及接线来源 |
| `READ_CODE.md` | 初学者阅读顺序、全部13个Python路径、变量及坐标单位说明 |
| `CHANGES.md` / `MERGE_NOTES.md` | 本轮维护范围及历史融合取舍 |
| `VALIDATION.md` | 本次执行的检查、结果及边界 |
| `validation/STATIC_REPORT.json` | 逐项静态检查结果 |
| `validation/legacy_contracts.json` | 从原始压缩包抽取的接口／结构基线 |
| `validation/CLI_CHECKS.json` | 不连接设备的命令行检查记录 |
| `legacy_manifest.json` / `LEGACY_FIXES.patch` | 当前／原始哈希，及上一版从往年源码引入的方法体差异 |
| `ANNOTATIONS.patch` / `validation/ANNOTATION_REPORT.json` | 本版相对上一版的Python差异、注释覆盖与AST比对 |
| `MANIFEST.sha256` | 包内文件校验清单 |

## 不连接设备的检查

进入解压后的`embodied_2026_commented_project`：

```bash
python3 tools/static_check.py
python3 main_2026.py --list-tasks
python3 main_2026.py --check-config
sha256sum -c MANIFEST.sha256
```

检查器仅读取源码和JSON、构建AST与编译代码对象，不导入机器人模块，不执行代码对象。接口核对覆盖显式旧接口调用、抓取命令列表及本地导入；不证明第三方ROS／SDK运行行为。

模板`--check-config`返回2是预期结果。`--list-tasks`只展示并检查清单，返回0不表示现场参数就绪。静态检查器返回0表示源码／接口通过，返回1表示有失败项。

若需另存报告，将路径放在运行目录中：

```bash
python3 tools/static_check.py --output runs/static_report.json
```

## 现场填写与实际运行入口

沿用机器人现有Linux／ROS1的Python3环境；生产源码按Python3.8语法核对，依赖的具体兼容版本仍须以原机器人环境为准。

```bash
cp config/competition.example.json config/competition.local.json
python3 main_2026.py --config config/competition.local.json --check-config
```

需要填写`locations`、场地／得分区／来源区域、投放位、相机内参、物品包络、夹爪阈值和实测耗时。模型键为`search_weights`、`table_weights`、`table_short_weights`和可选地面能力的`ground_weights`；不随包提供模型。相对路径相对于JSON所在目录。

标定完成后，填写对应`calibrated`和任务／能力`ready`状态。默认18条是清单模板，其中三级柜体操作明确为`unsupported`；没有通过标志补造开柜、抽屉技能。

实际机器人已经启动地图、定位、move_base、TF、Kinova驱动和TTS后，保留如下入口供现场使用；本次未执行：

```bash
python3 main_2026.py --config config/competition.local.json --real --match-id round01
```

`--real`准备阶段会执行原开爪和回home动作。正式计时从现场实时开始消息起算600秒；同一局保持同一个match-id与记录目录。原有场次锁拒绝已经开始的同局重启，本版没有断点续赛。

开始接口默认`/start_signal`的`std_msgs/String("start")`；停止接口默认`/embodied/stop`的`std_msgs/Bool(true)`。现场须提供正式开门对应的信号；本工程没有新建开门检测节点。

## 保留的能力边界

原抓取高度、手眼矩阵、home路径、桌面／地面动作和导航配置需要实机确认。位置反馈只是持物辅助证据，不能证明真实抓稳或全程未掉落。投放确认依赖可见物品、类别、尺寸包络与槽位标定；它是本地记录，不是裁判计分。

没有开柜／拉抽屉、两相机严格实例对应、连续掉落监测或对手意图识别。TTS没有播放完成回执。未恢复可信的完整依赖版本锁，不自动安装或升级ROS、SDK和模型库。

门控只在统一入口注入后生效；直接运行旧模块演示入口仍会沿用其历史行为。SIGALRM、轮询与取消请求属于软件控制，C扩展、网络和驱动故障仍可能延迟响应，不能据静态检查宣称硬实时停止。
