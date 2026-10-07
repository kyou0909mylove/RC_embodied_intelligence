# v13修改说明与完整项目文件夹
更新：2026-10-07。本项目文件夹已整合全部运行代码、现场配置、抓取与坐标换算模块、视觉模型及最新文档。本次更新涉及26份Python和2份JSON配置；完整目录还包含未改动的必要文件。
下表列出本次修改。完整目录中这些修改已经合并，无需逐个替换；文件链接均指向项目内部。
本版local保留的是已收到的现场值。如果现场后来又填写过点位/姿态/夹爪阈值，以现场当前值为准，合入新等待字段。模型、抓取几何及标定未重做；三种pick和coordinates源文件本次未改，已随完整项目保留。
正常启动仍是 `python3 -B main.py`；连接持续故障修好后同场 `python3 -B main.py --resume`。默认535秒优先离场，600秒总截止；启动驱动及完整说明见README。
## Python文件逐项说明

| 文件 / 项目相对路径 | 修改内容 |
| --- | --- |
| [arm/actions.py](../arm/actions.py) | 取消/反馈/剩余已保存回程/home统一恢复；保留疑似负载；最小恢复；保存put_stage；已释放回执不重放；提供abandon入口。 |
| [arm/driver.py](../arm/driver.py) | arm/finger/server/cancel读取配置；驱动成功后的附加到位误差仅提示；持续反馈缺失类型明确；resume无本地GoalHandle时查新鲜server status空闲，避免默认LOST误判。 |
| [arm/feedback.py](../arm/feedback.py) | 取消6750边界致命死区；100turn只提醒；新鲜不稳定按中值保守携带，不计确认；持续无新反馈才设备诊断。 |
| [arm/grasp.py](../arm/grasp.py) | 保存grasp_phase/剩余retreat_index以恢复；按稳定确认计真实抓取，疑似携带不虚计；路径预算和TTL有界。 |
| [arm/observe.py](../arm/observe.py) | 同观察位失败重试次数读配置；再试已填备用位/其他层，未完整观察不判空。 |
| [arm/original_kinova.py](../arm/original_kinova.py) | 来源Kinova底层等待改为配置的server5秒/arm20秒/finger10秒/初始反馈10秒；单位换算和物理目标保持来源。 |
| [arm/put.py](../arm/put.py) | 实际drop到达后不复核holding，直接释放；保留open确认；回执先保存后home，已释放仅补记录不重复放。 |
| [arm/release.py](../arm/release.py) | 新增独立主动弃物函数：当前停止处张爪、确认、home；下一家具导航前完成；记录abandoned而不是成功交付。 |
| [arm/startup.py](../arm/startup.py) | 新任务自动张爪/home读取有限阈值；恢复复用保存回程并保留负载，不再要求手动空爪/附加home容差。 |
| [camera/vision.py](../camera/vision.py) | 采图/推理/模型/连接阶段区分；进度及超时带相机来源；TimeoutError不误包成断连；有限关闭进程。 |
| [camera/wrist.py](../camera/wrist.py) | RealSense每帧有限等待；采图/推理阶段和连接故障区分，保留原测深度、类别及坐标。 |
| [core/checkpoint.py](../core/checkpoint.py) | 恢复文件在动作前验证结构、时钟、计数、负载与保存回程；错误指明文件及字段，物理负载不可猜测。 |
| [core/configuration.py](../core/configuration.py) | 完整合约成为main默认；新阈值默认/校验；比赛要求shelf=full；删除已废弃冷却/难度窗/108秒预算项；错误尽量带完整键。 |
| [core/diagnostics.py](../core/diagnostics.py) | 新增JSON路径/实际行号定位及完整软件异常堆栈记录；重复层姿态按索引区分。 |
| [core/evidence.py](../core/evidence.py) | 记录与关键图失败可提示；只认已确认落盘的result路径；交付统计与离场资格解耦。 |
| [core/faults.py](../core/faults.py) | 新增明确类型：设备不可达、软件状态待恢复、附加到位误差、参数错误。 |
| [core/recording.py](../core/recording.py) | 新增FIFO后台写入，有限等待/队列，磁盘失败备用路径；不承诺未落盘状态重启后可恢复。 |
| [core/recovery.py](../core/recovery.py) | 新增独立模块；导航/机械臂/语音通信复核；普通复位两次/最小恢复，退出过期计时器；相机有限重连。 |
| [core/safety.py](../core/safety.py) | 新增离场软截止及ExitRequested；嵌套计时不延长外层；按GoalID取消，LOST不作为已结束证据；硬截止/主动停止始终有效。 |
| [core/session.py](../core/session.py) | 每模块准备有限；同场时钟及drop到达/回执保存；快照异常不抢断；恢复优先最新默认/备用文件；逐资源限时关闭。 |
| [core/timeouts.py](../core/timeouts.py) | 新增集中默认等待值/计数校验，现场覆盖仍在local，不改物理数据。 |
| [core/workflow.py](../core/workflow.py) | 按阶段有限恢复；检测失败本轮跳过；drop最多3次/120秒耗尽先开爪；到点直接PUT；535秒持物离场；exit受阻重试到600秒；调度软件状态可重建。 |
| [main.py](../main.py) | 正常启动也完整校验配置；参数报文件/键/行号；设备故障与普通恢复区分；result只打印确认已保存的路径。 |
| [navigation/initial_pose.py](../navigation/initial_pose.py) | 发布间隔读配置；保留jujia26来源函数，不替换地图初始点。 |
| [navigation/navigator.py](../navigation/navigator.py) | 服务等待读取配置；move_base断连明确设备故障；清图服务缺失提示继续，避免误打印已连接。 |
| [speech/summer_tts_speaker.py](../speech/summer_tts_speaker.py) | 初始订阅等待读配置；运行中由workflow区分持续断连和在线done不匹配，腕部补报不阻塞。 |

## 配置与文档

| 文件 / 相对路径 | 内容 |
| --- | --- |
| [README.md](../README.md) | 只说明main现场启动、三组参数、主流程和后续代码/资料放置，包含Kinova驱动指令。 |
| [SOURCE_INFO.json](../SOURCE_INFO.json) | 记录v13采用行为、来源与81项离线验证范围；保留历史资料并注明等待改动。 |
| [config/navigation.example.json](../config/navigation.example.json) | 同步完整默认模板，与新字段/校验一致；不是已经填好点位的local。 |
| [config/navigation.local.json](../config/navigation.local.json) | 新增各阶段等待/重试字段，移除无效旧预算；已收到的map、观察位、home/place、手眼/几何、模型/标签参数原值保留。 |
| [docs/ERROR_GUIDE.md](ERROR_GUIDE.md) | 按启动、导航、机械臂、PUT、视觉语音、恢复/系统整理每项原因、处理及排查。 |
| [docs/MAIN_FLOW_V13.md](MAIN_FLOW_V13.md) | 主函数、抓取、投放离场、故障四张分支图及准确流程说明。 |
| [docs/MAIN_FLOW_V13.png](MAIN_FLOW_V13.png) | 完整竖向逻辑图，蓝/黄/绿/紫/红分区，包含主循环、三种抓取和特殊情况。 |
| [docs/MAIN_FLOW_V13.svg](MAIN_FLOW_V13.svg) | 与PNG相同内容的矢量图，文字转路径便于放大/分享。 |
| [docs/MISSING_FILES.md](MISSING_FILES.md) | 外部运行环境及除打点外需确认的已有参数；没有新增未提供本地模块。 |
| [docs/PROJECT_STATUS.md](PROJECT_STATUS.md) | 实现项、仍会暂停的边界、现场不足与离线核验范围。 |
| [docs/TIMEOUTS.md](TIMEOUTS.md) | 各时间参数、当前值、配置路径、读取代码、阶段上限和失败去向集中表。 |

## 验证与限制

81项离线行为核验、Python3.8语法解析、已填local完整配置检查通过。地图点、所有机械臂观察位、home/place、手眼/几何数据及模型字节与工作基线比较保持原值。未连接机器人，不能据此宣称当前房间实机全流程已验收。
导航健康但不可达、单点超时、夹爪边界/不稳定、普通软件问题已有有限恢复。持续不能取消/执行home/open、实际设备通信/采图不可达、开场导航失败、参数/恢复文件损坏或主动停止仍可能暂停；异常表详述这些边界。
本项目保留MAIN_FLOW_V13说明及PNG/SVG逻辑图作为当前运行依据。
