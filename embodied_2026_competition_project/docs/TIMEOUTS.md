# 主流程时间阈值与修改位置（v13）

更新：2026-10-07。数值对应本次提供的 `config/navigation.local.json`，不是远程读取的机器人参数。现场统一改这个文件；Python列用于找到读取位置。默认模板是 `config/navigation.example.json`，新增默认项集中在 `core/timeouts.py`，点等待等流程默认项在 `core/configuration.py::_flow_defaults`。

秒数字段须为有限正数，重试次数须为整数。`reset_retries=1` 表示首次失败后再试一次，共两次；`drop_max_attempts=3` 则表示包括首次的总共三次。不要在JSON中添加第二个同名顶层对象，应改已有对象里的字段。修改后可运行 `python3 -B main.py --check-config`，它不连接设备、不发动作。

## 1. 三层计时关系

| 层级 | 当前值与实际含义 | 读取位置 |
| --- | --- | --- |
| 比赛总截止 | `timing.match_s=600`，新场次设备准备完成后开始。不是裁判时钟；`--resume`保留原开始时间，暂停、重连时间也占这600秒 | `core/session.py::_start_task_clock` |
| 优先离场时刻 | `600-max(30,45+20)=535`秒；30来自exit_reserve_s，45来自place_timeout_s，20来自exit_estimate_s。剩65秒时停止搜索/投放，保留可能持物，先收臂再离场 | `core/session.py`、`core/safety.py`、`core/workflow.py::run/_exit` |
| 单阶段截止 | 本阶段配置秒数、外层阶段剩余秒数、535秒软截止及600秒硬截止取最早值。进入离场后只屏蔽535秒，仍受600秒与主动停止限制 | `DeadlineGate.limit/check` |

旧的108秒“开始新一轮预算”、逐点冷却和难度时间窗已经移除。不会在第462秒因为旧预算停止寻找。离场导航成功才记 `exit_motion_completed=true`；600秒到而未到达记 `TIME_UP`，不是离场成功，也不要求用resume继续已超时场次。

## 2. 启动、记录与通用状态

| local中的参数路径 | 当前值 | 作用与超时处理 | Python读取位置 |
| --- | --- | --- | --- |
| `timing.config_check_timeout_s` | 10秒 | 配置/元数据/权重校验；超时在设备动作前拒绝，并输出文件与相关键 | `main.py` |
| `timing.prepare_timeout_s` | 60秒 | 运行记录准备及每个设备模块初始化/相机重启的上限；不是全部初始化加起来60秒。准备失败发生在开始流程前 | `core/session.py`、`core/recovery.py`、`arm/actions.py` |
| `timing.state_timeout_s` | 5秒 | 一次调度调用及恢复快照构造上限；软件错误重建调度或警告，不因快照错误抢断任务 | `core/workflow.py::_schedule_call`、`core/session.py::checkpoint` |
| `timing.record_timeout_s` | 2秒 | 主线程等一条JSON记录落盘的最大时长；超时后台继续写，不保证此时已经落盘 | `core/recording.py`、`core/evidence.py` |
| `timing.cleanup_timeout_s` | 每项3秒 | 每个关闭/取消/保存资源动作；失败记cleanup_errors，继续关闭其他资源 | `core/session.py::close` |
| `camera.process_join_timeout_s` | 每次0.2秒 | 相机正常关闭/terminate/kill后的等待；不等待堵塞的采图进程无限返回 | `camera/vision.py::close` |
| `localization.initial_pose_wait_s` | 5秒 | 仅显式--set-initial-pose时，发布定位先验前等待 | `navigation/initial_pose.py` |
| `localization.publish_interval_s` | 0.5秒 | 两次/initialpose发布的间隔；不是底盘导航等待 | `navigation/initial_pose.py` |

## 3. 导航与动态障碍

| local中的参数路径 | 当前值 | 作用与失败处理 | Python读取位置 |
| --- | --- | --- | --- |
| `timing.nav_timeout_s` | 单次35秒 | 一个move_base目标等待上限，只认状态3成功。初始/入场失败可暂停；检测点失败本轮跳过；投放进入专门窗口；离场持续限次分段重试到总截止 | `core/session.py::navigate` |
| `recovery.nav_retries` | 1次额外重试 | 初始/入场/检测目标首次+1次。投放和离场有独立策略，不叠加这项 | `core/workflow.py::_navigate` |
| `recovery.retry_delay_s` | 1秒 | 一般导航、当前点观察/抓空、语音软件失败之间的间隔 | `core/workflow.py` |
| `recovery.nav_server_timeout_s` | 5秒 | 初始化move_base连接及清图服务发现等待；清图服务缺失只提示，move_base持续无响应可暂停 | `navigation/navigator.py` |
| `recovery.communication_timeout_s` | 10秒 | 一次导航/机械臂/语音通信复核；持续无服务/新反馈才升级设备故障 | `core/recovery.py::probe`、`arm/driver.py::check_connection` |
| `recovery.drop_max_attempts` | 共3次 | 投放点导航总尝试数，包括首次；达到次数就结束窗口，不一定等满120秒 | `core/workflow.py::_deliver_pending` |
| `recovery.drop_retry_timeout_s` | 总共120秒 | 从本次DROP_NAV开始的总窗口，包含每次导航、失败后等待、清图和传感器更新；任一次数/时间限制先到就结束 | 同上 |
| `recovery.drop_retry_wait_s` | 5秒 | 投放失败后原地停止、保留持物，等待动态障碍可能离开，再清图重规划 | 同上 |
| `recovery.costmap_timeout_s` | 3秒 | 清理旧代价服务调用上限；失败只提示，仍可按当前地图导航 | `core/session.py::refresh_navigation` |
| `recovery.costmap_settle_s` | 1秒 | 清图后让激光重新填入真实障碍；没有关闭避障 | `core/workflow.py::_refresh_navigation` |
| `strategy.round_wait_s` | 3秒 | 一轮结束后短等，开始下一轮并清除旧临时受阻状态 | `core/workflow.py::_search` |
| `recovery.exit_retry_wait_s` | 5秒 | 离场失败后停止、等待、清图，再尝试离场；总次数由600秒截止限制 | `core/workflow.py::_exit` |
| `timing.exit_estimate_s` | 20秒估计 | 只参与535秒离场触发计算；实际离场单次导航仍是35秒 | `core/session.py::_start_task_clock` |
| `timing.exit_reserve_s` | 30秒预留 | 与place_timeout_s+exit_estimate_s取最大值；不是离场成功保证 | 同上 |

**120秒例子：** 第一次导航失败→等5秒→清图/更新→第二次失败→再等/清图→第三次尝试。每次35秒加两轮等待/清图可能超过120秒，所以总窗口可截断第三次。若三次都很快返回4，则提前结束，无需站满120秒。

窗口结束、且导航通信正常时，在当前停止且已收臂的位置张爪、确认张开、再收臂，记“主动放弃负载”，然后才允许去下个检测点。不计成功交付。窗口期间如果先到535秒，则切换持物离场，不为时间不足主动丢弃。没有“导航失败必须转圈示意”的代码，底盘旋转原因看[报错说明](ERROR_GUIDE.md)。

## 4. 识别、抓取与恢复

| local中的参数路径 | 当前值 | 作用与超时处理 | Python读取位置 |
| --- | --- | --- | --- |
| `recovery.point_wait_timeout_s` | 120秒 | 到家具后头部识别、播报、腕部、动作及本点重试的总上限，导航不算在内 | `core/workflow.py::_one_point` |
| `detection_points[].wait_timeout_s` | 未填写 | 若填，直接覆盖该点总上限；不添加则取point_wait与该点grasp_timeout较大值 | 同上 |
| `actions.grasp_timeout_s` | 120秒 | 一次腕部观察+抓取阶段，不是每段运动都120秒 | `core/workflow.py::_inspect_point` |
| `detection_points[].grasp_timeout_s` | 地面点180秒，其余未填 | 单点覆盖抓取上限；当前地面点总等待默认也为180秒。仍受535/600秒约束 | 同上 |
| `timing.scan_timeout_s` | 12秒 | 单次头部扫描；处理软件超时可换点，持续采图/连接失败有限重启后设备暂停 | `core/workflow.py::_head_scan` |
| `actions.wrist.scan_timeout_s` | 12秒 | 腕部单次扫描；不使用过期/前一点回包 | `arm/actions.py::_scan` |
| `actions.wrist.frame_timeout_s` | 每帧1秒 | RealSense wait_for_frames SDK等待上限；捕获不到帧走设备重启检查 | `camera/wrist.py::_worker` |
| `actions.wrist.max_age_s` | 2秒 | 抓取前允许使用坐标的最大年龄；不是识别总等待 | `arm/grasp.py::execute_grasp` |
| `actions.wrist.max_fresh_retries` | 1次 | 坐标过期后在原观察位重测，仍失败则本点有限恢复；不沿用旧XYZ | 同上 |
| `actions.profiles.ground.observe_settle_s` | 3秒 | 到地面观察位后等稳定再采图，沿用catch_ty | `arm/pick_ground.py` |
| `recovery.camera_restarts` | 1次额外重启 | 扫描硬件错误首次+1次重试；每次重启准备最多60秒。持续采图/通信失败暂停；推理/回包代码错误换点，不累计判空 | `core/workflow.py`、`core/recovery.py` |
| `actions.observe_retries` | 1次额外重试 | 观察位驱动4/5且空爪时，收臂后再试同一位，随后尝试已填备用位/其他层 | `arm/observe.py` |
| `actions.profiles.shelf.detect_tries` | 每姿态2次采图 | 柜层每个实际到达的观察位重拍次数；已有候选就立即进入抓取 | `arm/pick_shelf.py` |
| `recovery.reset_timeout_s` | 每次30秒 | 取消旧动作→实际反馈→已保存的剩余回程→收臂；空爪张开，疑似持物保留，不在过期点计时器内复位 | `core/recovery.py::arm`、`arm/actions.py` |
| `recovery.reset_retries` | 1次额外重试 | 普通复位最多两次；失败后复核接口，最后还有独立的最小恢复路径，仍最多30秒 | 同上 |
| `recovery.reset_retry_wait_s` | 2秒 | 两次普通复位之间的等待，仍受整场/离场触发约束 | 同上 |
| `actions.abandon_timeout_s` | 每次30秒 | 投放受阻耗尽后在去下一点前张爪、确认、收臂。失败最多按reset_retries重试，不能拿未释放物品直接去家具 | `core/workflow.py::_abandon_before_next_point`、`arm/release.py` |
| `actions.place_timeout_s` | 每次45秒 | 到drop后直接移动place、张爪、确认张开、收臂。无投放前holding读数；软件失败恢复后有限重试 | `core/workflow.py::_deliver_pending`、`arm/put.py` |

复位恢复不是无限“等它好”：通常两次各30秒，中间/失败后各有最多10秒通信复核、2秒间隔，最后最小恢复30秒。全部仍受离场触发或比赛截止约束；持续不能取消/执行收臂才暂停，单次15毫米/5度容差失败只提示。

## 5. 驱动动作与反馈

| local中的参数路径 | 当前值 | 实际用途 | Python读取位置 |
| --- | --- | --- | --- |
| `actions.server_timeout_s` | 每服务5秒 | Kinova arm/finger action server连接 | `arm/original_kinova.py` |
| `actions.initial_feedback_timeout_s` | 每话题10秒 | 初始化cartesian_command及finger_position等待，仍受设备准备上限约束 | 同上 |
| `actions.arm_action_timeout_s` | 单动作20秒 | 原arm_run实际等待；必须驱动状态3，不把动作失败伪装成功 | 同上、`core/safety.py::GuardedClient` |
| `actions.finger_action_timeout_s` | 单动作10秒 | 原finger_run实际等待 | 同上 |
| `actions.cancel_timeout_s` | 每客户端3秒 | 当前GoalID取消后等终态；2/3/4/5/8可结束，实际目标9不能证明停止。恢复新客户端无GoalHandle时改查服务器新鲜status无活动目标 | `arm/driver.py::cancel_motion/_confirm_server_idle` |
| `actions.home_timeout_s` | 20秒 | 任务收臂动作整体上限，包含快照、动作及附加反馈检查 | `arm/actions.py::_home` |
| `actions.startup_open_timeout_s` | 10秒 | 新场次自动张爪整体上限 | `arm/startup.py` |
| `actions.startup_home_timeout_s` | 20秒 | 新场次自动回配置home上限；不要求操作者先复位 | 同上 |
| `actions.tool_pose_timeout_s` | 2秒 | 读取当前工具位姿/附加到位提示等待 | `arm/driver.py::tool_pose` |
| `actions.tool_pose_max_age_s` | 0.5秒 | 工具反馈新鲜度；过期读数不能参与坐标换算 | 同上 |
| `actions.gripper_feedback.timeout_s` | 3秒 | 尝试获取稳定新三指消息；不稳定但仍有新消息则保守处理，持续无新消息才复核设备 | `arm/feedback.py::_stable` |
| `actions.gripper_feedback.max_age_s` | 0.5秒 | 可用三指反馈最大年龄 | 同上、`arm/driver.py::finger_sample` |
| `actions.gripper_feedback.stable_span_s` | 0.12秒 | 稳定样本最小时间跨度，需3条真实新消息 | `arm/feedback.py` |
| `actions.reach_estimate_s` | 每前伸段5秒估计 | 是否足够完成本次路径的预测，不是驱动执行上限 | `arm/grasp.py` |
| `actions.return_estimate_s` | 每回程/home段5秒估计 | 同上，实际动作仍受arm_action/home上限约束 | 同上 |
| `actions.close_estimate_s` | 5秒估计 | 闭爪预测，还加反馈timeout_s参与预算 | 同上 |

软件到位容差保留参数 `pose_tolerance_m=0.015`、`orientation_tolerance_deg=5`，但动作已状态3且有新鲜有效反馈时，只打印误差提示，不单独暂停。新鲜位姿缺失或frame不符另行处理，不能把没有数据等同于“只是差几毫米”。

三指稳定性参数：`stable_samples=3`、`stable_tolerance_turn=60`。满闭阈值 `empty_closed_turn=6750`，张开阈值 `open_max_turn=800`，`min_blocked_fingers=1`。100turn的 `blocked_margin_turn` 只提示接近边界，不再制造6650–6750致命死区。分类规则和疑似持物的计数见[ERROR_GUIDE.md](ERROR_GUIDE.md)。

## 6. 语音

| local中的参数路径 | 当前值 | 作用 | Python读取位置 |
| --- | --- | --- | --- |
| `speech.subscriber_timeout_s` | 5秒 | 初始等待summer_tts_topic订阅 | `speech/summer_tts_speaker.py` |
| `speech.timeout_s` | 基础12秒 | 阻塞播报完成等待基础上限，长文本按下式延长 | `core/workflow.py::_speak` |
| `speech.max_timeout_s` | 60秒 | 单条文本最大完成等待 | 同上 |
| `speech.completion_slack_s` | 2秒 | 包围语音调用的阶段余量，不增加已过期的点/整场截止 | 同上 |
| `speech.pause_s` | 2秒 | 阻塞播报但不等done时的间隔；腕部补报不用这项 | 同上 |
| `recovery.speech_retries` | 1次额外重试 | 节点在线但done不匹配或播放软件失败，有限重试后继续抓取；持续无订阅复核10秒后暂停 | 同上、`core/recovery.py::probe` |

单条完成等待=`min(60,max(12,5+0.3*文本长度))`秒，外层另加2秒，仍取本点/比赛的更早截止。头部每段最多4个名称。腕部测量后补报只发请求、不等待播放完，以免播报让2秒坐标变旧。

## 7. 屏幕阶段与计时对应

| 阶段 | 上限/结束条件 | 常规失败去向 |
| --- | --- | --- |
| 配置检查、运行记录、设备初始化 | 10秒 / 60秒 / 每模块60秒 | 启动拒绝或设备暂停，动作前标出参数 |
| INITIAL_NAV / ENTRY | 每次35秒，共2次，中间1秒和清图 | 允许暂停，属于开始阶段 |
| REFRESH_OBSTACLES | 3秒清图+1秒更新 | 辅助清图失败提示，保留真实障碍 |
| DETECT_NAV / WAIT_NAV_RETRY | 每次35秒，共2次，间隔1秒 | 接口在线则跳过本轮，下轮复查 |
| HEAD_DETECT / 头部播报 | 扫描12秒、每条动态12–60秒+2秒 | 通信有限重连，软件错误本轮换点 |
| WRIST_AND_GRASP / RETRY_POINT | 点总120/180秒，腕部每次12秒、重试间隔1秒 | 超时退出旧计时器，RESET_POINT |
| RESET_POINT / WAIT_RESET_RETRY / FALLBACK_RESET | 每次30秒、间隔2秒、最多2次普通+1次最小恢复 | 有物收臂续投，空爪张开收臂换点；持续执行不完成才暂停 |
| CHECK_DEVICE | 每次10秒 | 持续不可达暂停；正常就继续局部恢复 |
| RESTART_CAMERA | 每次最多60秒，次数受camera_restarts限制 | 持续采图/连接失败设备暂停 |
| DROP_NAV / WAIT_DROP_RETRY | 共3次 / 总120秒 / 间隔5秒+清图 | ABANDON_LOAD；先到535秒则保留负载去离场 |
| PUT | 每次45秒，最多2次，恢复另计 | 已释放不重放；可恢复则续放，耗尽才主动弃物 |
| ABANDON_LOAD | 每次30秒，最多2次 | 确认开爪/收臂后才允许下个家具 |
| NEXT_ROUND | 等3秒，然后清图3秒/更新1秒 | 按配置顺序重查待查点 |
| RESUME_DELIVERY | 采用同样DROP/PUT阈值、原600秒 | 不重置时间，不回已经完成的初始/入场点 |
| EXIT / WAIT_EXIT_RETRY | 单次35秒 / 失败等5秒再清图 | 一直重试到实际成功或600秒TIME_UP |
| 结束记录/关闭 | 记录等2秒、每资源3秒 | 提示保存/清理失败，不反改已完成计数 |
| OPEN_STORAGE（未启用） | 未来每个点的storage.timeout_s，无当前生效值 | 一级/二级不会进入；不把未知开门重放成抓物恢复 |

所有循环有本点、次数或比赛截止约束。Linux SIGALRM是主线程软中断；某些C扩展可能延迟处理信号，相机进程可单独关闭。无法保证断电/OOM/硬杀后的保存或任意卡死驱动的物理动作完成，这些是当前实现边界，不应把日志中的取消请求当成实测急停。
