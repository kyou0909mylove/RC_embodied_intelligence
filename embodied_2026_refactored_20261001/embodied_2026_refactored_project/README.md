# 2026 具身智能机器人工程：模块拆分版

日期：2026-10-01。入口：main_2026.py，51 行；配置格式继续使用 schema 3。

本版按用户要求拆分长主函数，保留任务条件、原机器人接口、动作顺序、时间预算和交付判断。验证范围是静态源码、接口一致性、只读启动检查与工程打包；未运行机器人或模拟回归任务。

第一次阅读请打开 [READ_CODE.md](READ_CODE.md)。先测试导航请看 [START_NAV_TEST.md](START_NAV_TEST.md)。

## 文件职责

| 文件 | 内容 |
|---|---|
| main_2026.py | 命令行、配置检查、启动完整任务 |
| app/configuration.py | 配置读取与分段校验 |
| app/context.py | 状态名称、运行进度、600 秒常量 |
| app/session.py | 设备准备、正式开始信号、场次锁、异常与清理 |
| app/workflow.py | 状态循环和各动作阶段 |
| app/vision.py | 多帧搜索、靠近复核、投放前后视觉核对 |
| app/evidence.py | 检查点、事件日志和最终记录 |
| legacy/ | 10 个旧机器人模块，与上一中文注释版字节一致 |
| support/safety.py | 已有时间/停止门控，与上一中文注释版字节一致 |
| config/competition.example.json | 待填写的现场模板 |
| tools/static_check.py | 全工程静态检查，覆盖所有新流程模块 |
| tools/refactor_audit.py | 与拆分前入口比较迁移片段的业务 AST |
| tools/test_navigation.py | 已有独立导航检查脚本，本轮按原文件收录 |
| validation/ | 接口基线、迁移依据和本版结果 |

## 不连接机器人的检查

在工程目录运行：

```bash
python3 -S tools/static_check.py
python3 -S main_2026.py --help
python3 -S main_2026.py --list-tasks
python3 -S main_2026.py --check-config
```

默认模板没有填完整，--check-config 返回 2 是预期结果。--list-tasks 返回 0 只表示清单可读取。检查器仅读源码与 JSON、构建 AST 和编译代码对象，不执行机器人代码。

普通 import app.session 等操作不会初始化设备；ROS、模型和 SDK 导入位于实机准备方法内。

## 部署到你们的 Ubuntu20.04 电脑

完整复制整个 embodied_2026_refactored_project 目录。入口依赖 app/、legacy/、support/ 和配置文件，不能只复制入口。

沿用现场已有 ROS1 与工作空间环境。底盘启动、ROS 网络与导航步骤见 START_NAV_TEST.md。setup_network.bash 设置底盘 master 和本机路由 IP，原 ROS/catkin 环境仍须由现场加载。

完整任务还需要机械臂驱动、相机/TF、语音节点及对应模型。tongyong.sh 正文尚未取得，作用以现场文件为准。

## 填写配置

将 config/competition.example.json 复制为 config/competition.local.json，填写：

- 入场、离场、观察、搜索和投放停车位及场地边界。
- 来源区域、得分区与放置槽位边界、机械臂 drop_poses。
- 模型路径、精确搜索/抓取标签与相机内参。
- 物品半径、夹爪阈值、耗时估计及标定状态。

模型相对路径以 JSON 文件所在目录为基准。导航位姿为 [[x,y,z],[qx,qy,qz,qw]]；机械臂 drop_poses 为六个 mdeg 数值，格式不同。

```bash
python3 main_2026.py --config config/competition.local.json --check-config
```

三级柜门/抽屉能力仍为 unsupported，JSON 状态不能创造未实现的动作。

## 完整实机入口

完成现场标定和联调后，完整命令保持：

```bash
python3 main_2026.py --config config/competition.local.json --real --match-id round01
```

--real 的准备阶段就会执行原开爪和回 home。准备完成后等待正式开始：默认 /start_signal 的 std_msgs/String("start")，本次有效非锁存信号到达后固定 600 秒截止。停止接口默认 /embodied/stop 的 std_msgs/Bool(true)。

同一记录目录使用进程锁，已经正式开始的同一 match-id 拒绝重启。检查点用于分析，不提供续赛恢复。

动作成功、位置反馈支持持物、视觉确认交付分别记录。只有投放后新增物品判据通过才增加交付数量；本地台账不等于裁判计分。

## 本版验证

205 项静态检查通过，0 项失败；22 个 Python 文件按 Python3.8语法解析并编译；95 个原接口声明与 41 处完整任务明确调用匹配。8 项只读 CLI/导入隔离检查通过。

33 项迁移结构核对包括基线、30 个迁移片段、阶段预算与阶段结束检查。还原变量表示和明确方法出口后，业务 AST 一致；这不证明真实设备行为或所有集成语义完全等价。

详见 [VALIDATION.md](VALIDATION.md)。REFACTOR.patch 记录本轮代码差异；LEGACY_FIXES.patch 与 ANNOTATIONS.patch 是此前版本的来源记录。历史报告在 docs/history_before_refactor/，不计入本版结果。
