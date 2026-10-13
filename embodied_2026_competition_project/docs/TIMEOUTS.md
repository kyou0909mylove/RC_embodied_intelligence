# 等待与恢复规则

更新：2026-10-13。现场覆盖填写 `config/navigation.local.json`；默认值还在 `core/timeouts.py` 和 `core/configuration.py`。本页说明当前原功能文件调用方式，旧整合抓取的预算和观测门禁不参与当前抓取动作。

## 整场与软离场时刻

`timing.match_s=600`，新场次设备准备完成后开始计时；同场恢复保留原开始时间。`exit_reserve_s=30`，第570秒由统一 `DeadlineGate.check()` 和阻塞计时器优先触发 `ExitRequested`，中断普通导航、识别、抓取或投放并进入离场。

离场先取消旧任务，根据实际姿态做必要回撤和收臂，保留持物，不再张爪或去投放点。离场准备上限为剩余整场时间减去 `exit_estimate_s`，当前参数在570秒触发时通常余10秒用于准备。离场阶段暂时屏蔽软离场触发，整场硬截止、急停、ROS关闭和导航前收臂条件继续生效；无法满足收臂条件时不会发送底盘离场目标。

`DeadlineGate.limit` 保留实际阶段硬截止，取外层截止、整场截止和当前时间加阶段上限中的最早值；`check()` 同时返回至软离场时刻的最短剩余时间。阶段超时后的处理由调用处决定，不按剩余软预算预测抓取是否成功。

## 普通动作与严格动作

| 参数 | 当前local值 | 行为 |
| --- | --- | --- |
| `actions.arm_action_timeout_s` | 20秒 | 普通机械臂单动作等待；超时取消本次目标并返回空结果，原函数继续下一动作 |
| `actions.finger_action_timeout_s` | 10秒 | 普通夹爪单动作等待；超时记录后继续原顺序 |
| `actions.home_timeout_s` | 25秒 | 配置收臂总上限；真实失败交给有限恢复，持续不能收臂停止 |
| `actions.startup_open_timeout_s` | 10秒 | 初始化张爪保持真实成功检查 |
| `actions.startup_home_timeout_s` | 20秒 | 初始化收臂保持真实成功检查 |
| `actions.place_timeout_s` | 60秒 | 一次完整投放阶段硬上限；释放位普通失败状态不单独阻止张爪 |
| `actions.server_timeout_s` | 5秒 | action server连接等待 |
| `actions.initial_feedback_timeout_s` | 10秒 | 初始化cartesian_command/finger_position反馈等待 |
| `actions.cancel_timeout_s` | 3秒 | 取消旧动作及状态复核等待 |
| `actions.tool_pose_timeout_s` | 2秒 | 工具位姿反馈等待 |
| `actions.gripper_feedback.timeout_s` | 以local为准 | 抓取结束或张爪后读取新鲜三指反馈 |

原Kinova文件写死的10/5秒请求由客户端适配为上述20/10秒，原文件不改。普通ABORTED、REJECTED等状态仍记录，不按成功状态阻止下一条合爪/张爪。原功能文件的home、配置home和初始化保持新目标真实成功检查；导航也保持成功才算到达。

当前原抓取不使用聚合 `actions.grasp_timeout_s` 或旧位置TTL来截断前伸到合爪这段顺序。配置中仍兼容这些旧字段；它们不是当前原功能文件的抓取门禁。`actions.observe_retries` 对应的旧discover路径已经删除。

## 识别与恢复

| 项目 | 当前行为 | 实现 |
| --- | --- | --- |
| 柜层RealSense | 每层识别最多5次；成功立即继续；空层检查本次未查余下层 | `arm/pick_shelf.py` 临时设置原v4常量，结束后恢复 |
| 家具表面、地面RealSense | 每次识别最多5次；找到目标立即返回 | 原识别器默认及地面调用接口 |
| 原识别器预热 | 30帧预热不计入识别次数；采图/推理耗时另计 | `catch_ground/src/realsense_yolo11.py` |
| 头部扫描 | `timing.scan_timeout_s=12`秒 | `core/workflow.py`、`camera/vision.py` |
| 初始化/相机重启 | `timing.prepare_timeout_s=60`秒每模块 | `core/session.py`、`core/recovery.py` |
| 普通机械臂恢复 | 每次30秒，首次失败可额外重试1次；间隔2秒，另有最小恢复路径 | `recovery.reset_timeout_s/reset_retries/reset_retry_wait_s` |
| 导航 | 每次35秒，普通检测点首次失败可额外重试1次，耗尽跳过本轮 | `timing.nav_timeout_s`、`recovery.nav_retries` |
| 投放点受阻 | 最多3次，总窗口120秒；间隔5秒并清图 | `recovery.drop_max_attempts/drop_retry_timeout_s/drop_retry_wait_s` |
| 离场点受阻 | 失败间隔5秒重试，仍受600秒截止 | `recovery.exit_retry_wait_s` |
| 地面抓取播报 | 有效腕部识别后发送，不等待播放、节点重连或重试 | `arm/pick_ground.py`、`core/workflow.py` |

上述等待均受实际外层硬截止约束。`camera/wrist.py` 仍用于资源关闭和异常恢复，但当前三个原抓取函数直接使用上传的RealSense识别器，旧腕部稳定采样参数不替代原识别逻辑。

每点每轮执行一次完整抓取，失败不减少 `object_count`，下一轮重新导航到位。原脚本内部的采图重试和柜层搜索保留。腕部模型在准备阶段加载并复用，每次相机启动、预热、采图和关闭仍按原流程执行。

主流程识别播报排队后立即继续抓取；后台按 `/summer_tts_done` 顺序发送，确认超时只停止后续语音派发，不中断抓取。离场清除尚未发送的播报，当前播放不因机械臂取消动作而停止。

本次43项离线回归通过，包括真实Linux SIGALRM阻塞中断、播报和离场检查；SDK推理与action传输使用替身，未连接真实机器人。启动与模型位置见[README](../README.md)。
