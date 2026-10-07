# 报错原因、自动处理与现场排查（v13）

更新：2026-10-07。按本次main实际控制流整理，默认 `actions.mode=kinova`、难度 `[1,2]`。参数目录为项目下 `config/navigation.local.json`，默认模板 `config/navigation.example.json`。完整时间表见[TIMEOUTS.md](TIMEOUTS.md)，流程见[MAIN_FLOW_V13.md](MAIN_FLOW_V13.md)。

## 1. 先分清“换点”“离场”“暂停”

| 终端/结果 | 含义 | 接下来做什么 |
| --- | --- | --- |
| 本轮跳过、RETRY_POINT、RESET_POINT、WAIT_DROP_RETRY | 自动恢复，不是退出Python | 等该阶段阈值，不重复启动main |
| 夹爪提醒、到位误差提醒、记录提醒 | 有数据但不理想，或记录保存受影响 | 程序继续；按对应表检查原因 |
| priority_exit_requested / EXIT | 停止新抓放、收臂，允许持物离场 | 不需要为0交付或仍持物人工干预 |
| FLOW_COMPLETED，exit_motion_completed=true | 实际离场导航到达 | 返回码0；计数不等于裁判分数 |
| TIME_UP，exit_motion_completed=false | 已到600秒但没有确认离场到达 | 正常按截止收尾，返回码0；没有伪报离场成功 |
| `[参数错误]` | 文件/参数不合约，动作前检查；运行时frame等也可报 | 按输出文件、JSON路径、行号修改；返回码2 |
| `设备故障暂停` | 持续通信/采图不可达，或真实取消/收臂/张爪执行有限恢复后仍不能完成 | 排障，再用同场--resume；返回码1 |
| `流程暂停` | 初始/入场失败、主动停止，或超出已覆盖边界的异常 | 看最后阶段和具体异常，不当作普通判空 |
| `无法读取运行记录` | 同时运行、恢复文件损坏/不存在、已完成/已超时等 | 见恢复表，返回码2 |

单次导航4、一次动作超时、三指边界、暂时不稳定、附加home/place容差、普通推理/回包异常、KeyError等点内软件错误，不再直接让整场退出。若最小恢复仍无法确认收臂/取消或开爪，不能在机械臂仍可能活动时继续驾驶底盘；保留最后恢复记录并打印执行失败。这也意味着未知的持续程序损坏不能保证自动修好。

## 2. 启动与参数

所有普通main启动都做完整参数、姿态、模型资料检查，再连接设备。无需必须另运行check-config。参数报错会尽量输出如 `actions.home_pose：.../config/navigation.local.json，第...行`；缺失键没有实际行号，只报应补的键。重复detect_pose按检测点和层索引定位，不能把别的层的行号当答案。

| 编号 | 报错/现象与可能原因 | 程序处理 | 排查/修改位置 |
| --- | --- | --- | --- |
| A01 | 文件不存在、无读取权限、JSON逗号/括号/引号错误 | 启动参数拒绝 | 查看--config实际路径；JSON报错行列；local及example |
| A02 | schema_version不支持，对象写成列表/字符串 | 动作前拒绝 | schema_version目前7；actions、wrist、profiles、timing等应是对象 |
| A03 | 初始/entry/drop/exit或检测点pose为空、格式错误、四元数不归一 | 动作前拒绝 | initial_pose、locations.*、detection_points[].pose；map格式两层数组 |
| A04 | 检测点ID重复/非法、没有可启用点、enabled不是布尔值 | 动作前拒绝 | detection_points[].id/enabled、strategy.enabled_levels |
| A05 | 抓取profile与难度不一致、柜层不是三层/层ID重复/hint不对应 | 动作前拒绝 | difficulty、grasp_profile、layers、layer_hint；一级surface、二级shelf/ground |
| A06 | 机械臂姿态长度/数值错误；把四元数放入后三项 | 动作前拒绝 | home_pose/place_pose、各detect_pose；六数米/度，非四元数 |
| A07 | 柜层或地面观察朝向与经验公式不符 | 动作前拒绝，报相关点/字段 | layers[].detect_pose、profiles.shelf.detect_euler_deg；ground.reference_detect_pose；不是15mm实测容差 |
| A08 | R矩阵/T/补偿/地面锚点/抓取Z格式无效 | 动作前拒绝 | actions.profiles.surface/shelf/ground；点上的ground覆盖；沿用来源，不混公式 |
| A09 | 模型不存在、SHA/标签/元数据不匹配、label_map含未知类别 | 动作前拒绝 | model及profiles.*.model、models/TABLE_MODEL_INFO.json、label_map |
| A10 | robot_type/tool_frame/camera.backend/分辨率不匹配 | 动作前拒绝或连接失败 | 当前j2n6s300、j2n6s300_link_base、Azure Kinect；腕部serial/流尺寸 |
| A11 | 超时为0/负数/NaN，采样次数/阈值顺序错误 | 动作前拒绝 | 报错完整键；TIMEOUTS表；turn不是百分比 |
| A12 | 柜层execution_mode=align_only | 动作前拒绝 | actions.profiles.shelf.execution_mode比赛须full；避免跑到调试中间位后暂停 |
| A13 | 初始定位发布误把目标当当前位置 | 代码无法自行证明摆放正确 | 只有已经实际摆在initial_pose才加--set-initial-pose；默认用当前定位导航到它 |
| A14 | 同一项目另一个main仍在运行 | 场次锁拒绝第二实例 | 关闭重复实例，不能靠删除锁文件绕开；锁只在本机，不约束其他控制节点 |
| A15 | import rospy/kinova_msgs/SDK/Ultralytics失败，Python环境不对 | 设备准备失败，动作前或准备阶段暂停 | source当前终端ROS及实际Kinova工作空间，使用已验证视觉环境 |
| A16 | init_node停留，master或本机回连地址不通 | 单ROS初始化最多60秒，不再无限等待 | ROS_MASTER_URI/ROS_IP、底盘master、主机网络；不以是否SSH作为唯一判断 |
| A17 | 启动自动张爪或home动作失败 | 启动设备暂停，可检查驱动后resume | 动作结果和驱动终端；startup_open10秒/home20秒，不要求手动初始空爪阈值 |

## 3. 导航：状态4、转圈、投放被挤占

| 编号 | 情況/可能原因 | 本版处理 | 排查 |
| --- | --- | --- | --- |
| N01 | INITIAL_NAV重试后失败 | 允许暂停 | current定位、initial_pose、当前map/TF/激光；尚未完成开场 |
| N02 | ENTRY重试后失败 | 允许暂停 | locations.entry与门口通行；初始点同entry时只导航一次 |
| N03 | DETECT_NAV状态4，临时障碍/规划或控制失败 | 有限重试，接口正常则仅跳过本轮，下轮清旧障碍重试 | /move_base/result的status.text；点位是否在障碍格，动态机器人是否挡路 |
| N04 | 状态2或8，目标被取消/抢占 | 按目标阶段有限重试；检测点下轮重查 | 是否另开导航程序/RViz目标；本版取消只按当前GoalID，避免迟到cancel_all误杀新目标 |
| N05 | 状态5，被服务器拒绝 | 有限重试后按目标阶段处理 | 地图目标、frame、驱动/导航日志；参数格式可先校验 |
| N06 | 状态9 LOST，未取得有效结果 | 先复核server；在线可本轮跳过/重试，不单因9暂停 | 若server持续不可达则设备暂停；检查节点/network，不能把9当到达或刹停证明 |
| N07 | 一直ACTIVE/PENDING/取消中，无终态 | 35秒后取消本目标，分阶段恢复 | /move_base/status、TF、局部规划器和速度输出；不是无限等 |
| N08 | move_base action server持续不可用 | 通信复核10秒失败设备暂停 | navigation launch是否退出、master可达、ROS网络双向回连 |
| N09 | DROP_NAV失败，对方挤占投放位/通道 | 等5秒、清图后重发同drop；总3次/120秒 | 看status.text、激光和local costmap；wait不是随机换家具 |
| N10 | 投放3次/120秒耗尽，但接口正常 | 当前停止处先张爪确认、收臂，记弃物，再去下一家具；不算交付 | ABANDON_LOAD记录；确保开爪发生在离开当前处之前 |
| N11 | 投放窗口里先到离场触发时刻 | 优先离场，保留持物并收臂 | priority_exit_requested；不会只因时间不足先丢弃 |
| N12 | EXIT仍失败/被动态障碍阻挡 | 停止等5秒、清图、重试至实际到达或600秒 | 不能因一轮4就退出，也不伪报成功；硬件失联仍暂停 |
| N13 | clear_costmaps服务缺失/调用超时 | 提醒后使用现有地图继续，清图最多3秒 | 服务名是否正确，导航服务终端；不代表全部导航失联 |
| N14 | 清图后障碍很快出现 | 正常传感器重建；仍会避障 | 真实机器人仍在附近；不会永久删除仍存在的障碍 |
| N15 | 正在导航时转圈 | main没有“转圈示意失败”动作；可能是move_base恢复、局部规划转向或目标朝向调整 | 看Rotate recovery/oscillation日志、状态text、cmd_vel.angular.z；只凭照片不能确认原因 |
| N16 | 定位飘移、激光与墙不齐、机器人走错地图点 | main不会自动校准地图定位 | RViz核对AMCL/TF、同地图、初始实际位置；不要通过发布错误当前位置修正目标 |

状态对照：0等待、1执行中、2被抢占/取消、3成功、4执行终止、5拒绝、6抢占中、7撤销中、8已撤销、9结果丢失。主流程只认3为到达；状态本身不足以给出具体原因。

ROS Noetic move_base的恢复/终止条件可对照[官方move_base.cpp](https://github.com/ros-planning/navigation/blob/noetic-devel/move_base/src/move_base.cpp)；恢复相关配置项见[官方MoveBase.cfg](https://github.com/ros-planning/navigation/blob/noetic-devel/move_base/cfg/MoveBase.cfg)。实际是否启用旋转恢复、耐心阈值等，以你们导航工作空间和rosparam为准，本版不擅自更改它们。

## 4. 机械臂、三指反馈和软件状态

| 编号 | 情況/可能原因 | 本版处理 | 排查/参数 |
| --- | --- | --- | --- |
| M01 | 初始cartesian_command/finger_position未发布 | 每项10秒，失败准备暂停 | /j2n6s300_driver/out对应话题、型号、驱动、网络 |
| M02 | arm/finger action server失联或一直没新反馈 | 复核接口及新消息最多10秒；持续失败设备暂停 | Kinova供电、连接、驱动终端；机械臂故障不是换导航点 |
| M03 | 观察位arm action=4/5，空爪 | home后重试一次，再尝试已填备用位/其他层 | action命名和具体步骤；detect_pose是否实际可到达，不直接归因手眼 |
| M04 | 柜层只停很短时间就走 | 本版头部为空也执行腕部三层；动作/软件失败会打印原因，不算家具空 | HEAD_DETECT、arm_observation_failed、每层detect_pose；确认现场替换完整v13相关文件 |
| M05 | 有一层/一个配置视角没完成观察 | arm_failed，不累计判空；点内重试/超时恢复，下轮再来 | 对应层动作结果；不要把其他空层当整柜无物 |
| M06 | 前伸/闭爪/回程动作失败或阶段超时 | 取消旧目标→反馈复核→完成已记录剩余回程→home；疑似持物优先投放 | grasp_phase、retreat_index、retreat_poses和arm_action_failed |
| M07 | 整个点等待阈值到达 | 空爪张开/收臂换点；疑似持物保留收臂续投 | point_wait120秒，地面180秒，可由wait_timeout覆盖 |
| M08 | home/place驱动状态3，但软件15mm/5°误差超过阈值 | 新鲜有效反馈只提示，不单因该门槛暂停 | home/place标定及目标/实际误差；不是取消驱动成功检查 |
| M09 | 工具位姿短暂缺失/过期 | 当前读最多2秒，失败后复核/恢复；持续无新消息才设备暂停 | tool_pose_max_age0.5秒、frame_id、话题频率 |
| M10 | tool_pose的frame_id不匹配 | ParameterFault，表明actions.tool_frame和文件位置 | 必须与驱动输出相同，不把map/相机frame当机械臂base |
| M11 | 工具坐标/四元数含非有限数或无效归一 | 反馈故障复核，持续无效暂停 | 驱动消息质量；不能用无效位姿计算抓取 |
| M12 | 三指如[6714,6774,6744]接近6750 | 判受阻持物并提示边界，不再抛边界致命异常 | 6750满闭、800张开，100仅提示；观察实际空夹/持物读数 |
| M13 | 三指新消息存在，但3秒内不稳定 | 用末尾新样本中值保守保留可能持物；grasp_confirmed=false，不增加真实抓取/交付数 | stable_samples3、span0.12秒、每指变化60turn；不是通信断连 |
| M14 | 三指全闭且稳定 | 判空夹，收臂完成后本点再试；不虚报抓取 | 全部>=6750；物品过薄/滑落或阈值不适配也可能造成空夹判断 |
| M15 | 三指全张开 | <=800判open；抓取后空，释放后可确认张开 | 对应动作上下文，0 0 0通常是张开而非疑似持物 |
| M16 | 部分全开部分全闭，无法可靠分类 | 未知但有新反馈→保守持物继续，计数不确认 | 夹持对象、指接触情况；框架不把“未知”改成成功 |
| M17 | 持续没有新的finger_position | 3秒本次读取失败后复核10秒；持续缺失设备暂停 | 忽略锁存旧消息、超0.5秒反馈；确认驱动频率/网络 |
| M18 | 实际取消目标后仍ACTIVE/LOST，无法确认结束 | 有界重试，仍不能确认取消则执行故障暂停；新客户端无GoalHandle的默认9不直接误判 | 当前GoalID各3秒；新客户端查新鲜action status数组为空闲。仍有旧/其他活动目标不伪造已停止，不cancel_all其他程序 |
| M19 | load_state=unknown、travel_ready=false导致导航前阻挡 | StateMismatch进入恢复，保留疑似负载并收臂后再发导航 | 是软件记录与当前动作需要核对，不等于自动判硬件坏 |
| M20 | 持物但缺少完整抓取上下文 | 建恢复用未知实例，保留负载，不补造抓取分数；收臂后投放 | 恢复文件和pending_pick；损坏的文件结构在启动前拒绝 |
| M21 | RESET_POINT第一次失败 | 独立计时器重试/通信复核，普通复位最多2次，最后最小恢复 | 每次30秒，间隔2秒；不会无限恢复等待 |
| M22 | 常规及最小恢复都无法实际取消/收臂 | 暂停；未确认收臂不发下一家具导航 | 看最后执行错误、驱动/目标位姿；持续程序错误也可能需修复后resume |
| M23 | 坐标计算/候选回包RuntimeError/KeyError/TypeError | 本点记录traceback、恢复，随后换点；不把异常当“无物品” | software_recovery的stage和堆栈；已确认配置错误单独拒绝 |
| M24 | 调度pending/next_point/observe等软件状态损坏 | 在5秒内捕获，重建调度；真实抓放编号/计数独立保存 | scheduler_rebuilt；可能重新检查已查点，不重复计同一object_id |

### 三个夹爪分类条件，具体在哪里改

在 `config/navigation.local.json → actions.gripper_feedback`，读取函数 `arm/feedback.py::classify_fingers`：

1. 三指都 `<=open_max_turn`，当前800：张开。
2. 三指都 `>=empty_closed_turn`，当前6750：空夹接近全闭。
3. 至少 `min_blocked_fingers` 指位于 `(open_max_turn,empty_closed_turn)`，当前至少1指：受阻持物；稳定才计确认抓取。

其他组合是未知，仍有新消息就保守处理。`blocked_margin_turn=100`不是角度/厘米，也不再把6750附近读数直接判为暂停。闭合程度不能区分夹住物品和夹住家具边缘，实际抓取路径仍需符合当前家具。

## 5. PUT与主动放弃负载

| 编号 | 情況/可能原因 | 本版处理 | 排查 |
| --- | --- | --- | --- |
| P01 | 到drop后夹爪持物读数不确定 | 不做holding读取；直接place→张爪→确认开爪→home | 用户要求已采用；仍保留释放动作结果和张开确认 |
| P02 | 缺少实际drop到达结果、误调用put | StateMismatch→有限恢复/重新投放；不能补造到达 | drop_arrived必须来自move_base=3，不是发送目标 |
| P03 | place动作失败/PUT超时 | 取消、保留上下文、恢复home后有限重试PUT | place45秒、单arm20秒；home/place误差本身只提示 |
| P04 | 张爪动作失败或反馈没有确认open | 恢复后最多两次PUT；耗尽仍需主动释放确认才去下一点 | finger_open目标、三指读数；持续不开爪属于执行故障，可暂停 |
| P05 | 已张开释放，随后home失败 | 立即保存last_put_receipt，恢复只收臂并登记一次，不重放同次释放 | put_stage=released/opening，drop_arrived，回执ID |
| P06 | 恢复回执有ID但抓取确认记录缺失 | 警告，不加真实交付数，空爪仍继续 | confirmed_object_ids、delivered_object_ids；不凭回执补造抓取 |
| P07 | 疑似持物完成投放 | 执行释放/收臂，但status=released_unconfirmed_pick，不计确认交付 | grasp_confirmed=false；机器人继续比赛 |
| P08 | 途中物品滑落，但到点不复核holding | 仍执行PUT；现有闭合确认+动作记录无法证明途中未滑落 | v13按要求取消到点复核，不能自动声明lost_in_transit或区内得分 |
| P09 | 投放路线失败后弃物 | arm/release.py在下一检测导航前开爪、确认、home，记abandoned_objects | 不是得分区put，不增加physical_deliveries |
| P10 | 弃物已开爪、随后home失败 | 释放状态先保存，恢复只完成必要home；开爪未完成不去家具 | load_abandoned与恢复快照；持续执行不完成才暂停 |

记录中的“确认交付”依据是已确认抓取、实际drop到达、实际张爪完成；没有得分区落点传感器。`zone_sensor_verified_deliveries`保持0，不能把返回码0或导航成功当裁判得分。取消到点holding复核后，软件也不能识别所有途中滑落。

## 6. 相机、识别、语音

| 编号 | 情況/可能原因 | 本版处理 | 排查/配置 |
| --- | --- | --- | --- |
| V01 | RealSense打不开/多台未选serial/被viewer占用/流格式不支持 | 启动失败暂停；运行中采图错误有限重启后仍失败才暂停 | actions.camera_serial、wrist640×480、关闭占用程序 |
| V02 | Kinect打不开、SDK缺失、USB/供电断开 | 同样按设备故障处理 | Azure Kinect SDK/pykinect_azure；main直接打开，不需相机ROS节点 |
| V03 | SDK采图卡住/连续无有效帧 | 带capture阶段标记，最多有限重启，持续无响应设备暂停 | scan12秒、frame1秒、USB/SDK；不是无物品 |
| V04 | 相机进程退出、Pipe断开 | 有界关闭、重启连接；重复失败设备暂停 | camera_process_restarted、phase=connection |
| V05 | 模型初始化/权重解析/依赖导入错误 | 动作前参数/环境错误拒绝或设备准备暂停 | model.weights/元数据、当前Python环境；不是空家具 |
| V06 | 推理算子/候选格式/回包点号错误 | 软件恢复、丢弃回包，本轮换点，不记空 | phase=inference/software、software_recovery堆栈；核对YOLO输出 |
| V07 | 超时后收到前一个点的旧结果 | 关闭忙进程，下一次重连重新采图，不使用旧结果 | wrist_restart_needed/head_restart_needed，当前point_id |
| V08 | 框不稳定/中心深度0/多帧差异大 | 本点有限重拍、备用姿态；阈值到恢复换点，不累计判空 | confidence、frames/min_hits、depth_min/max/tolerance，物品反光/遮挡 |
| V09 | 测量超过2秒，播报后反复重拍不抓 | 腕部补报不等语音完成；坐标过期原位重拍一次，仍失败局部恢复 | max_age_s2、max_fresh_retries1；时间戳与关键图 |
| V10 | 地面角度取不到 | 默认旧脚本0度fallback并明确记录；填null则等待重测到本点上限 | profiles.ground.angle_fallback_deg；深度仍必须有效 |
| V11 | 头部未识别到柜层/地面 | 仍执行腕部所有配置观察位/柜层，找到一件就抓 | 头部和腕部视角不相同；不是仅头部空就跳过 |
| V12 | 多物品只报一件/识别乐事却不抓 | 本版先报全部稳定实例/同类数量，腕部补报新项；有框不等于有稳定深度 | head_scan_completed、grasp_result.reason；查看深度、可到达观察位、本点阈值 |
| V13 | 一家具连续完整空观察 | 头部无框且全部腕部配置视角均实际完成无框，连续两次才退休该点 | empty_confirmations2；失败/有物清空空观察计数 |
| S01 | summer_tts_topic无订阅 | 初始化等5秒，复核最多10秒；持续断连设备暂停 | SummerTTS节点是否已启动、ROS网络、topic名称 |
| S02 | 发布显示publishing但没有声音 | 发消息不等于音频播放 | 节点终端、扬声器输出、音量、模型/播放后端；rostopic info只证实订阅 |
| S03 | 节点在线但done没来/文本不匹配/播放软件错误 | 有限重试后记录继续抓取，不因匹配回执一直反复扫描 | speech.wait_done、done回包内容；requests与done数量 |
| S04 | torch/torchvision警告 | 警告本身不退出；真实导入/推理错误按对应阶段处理 | 在当前环境查看版本、pip check，使用你们已验证可运行的组合，不盲目升级 |

## 7. 恢复、记录与外部停止

| 编号 | 情況/可能原因 | 本版处理 | 排查 |
| --- | --- | --- | --- |
| R01 | --resume找不到latest或场次文件 | 启动拒绝 | runs/latest_match.json、checkpoint-场次.json；可--match-id指定已有场次 |
| R02 | 恢复JSON/schema/started/负载字段格式损坏 | 发动作前拒绝，输出恢复文件及字段 | 保留原文件排查，不手工把疑似持物改成empty以绕过 |
| R03 | 电脑重启boot_id变化、原场已600秒/finished | 拒绝原场resume，不重置600秒 | 新比赛正常main；原比赛无法凭重启延长时钟 |
| R04 | resume原holding/pending_pick停在回程途中 | 先取消旧目标、核对新反馈、执行保存的剩余回程、home；持物优先续投 | grasp_phase/retreat_index；不为当前家具编造任意碰撞退出路径 |
| R05 | 旧调度永久禁用/冷却记录、内部调度字段损坏 | 旧标志清除重查；调度错误重建，物理编号去重保留 | scheduler_version；重建可能重复观察，不重复计同ID |
| R06 | 默认记录盘满/权限问题/保存慢 | FIFO后台有限写；失败尝试本机临时备用目录，提醒后继续动作 | 终端[记录提醒]、evidence_dir；2秒等待不代表必已落盘 |
| R07 | 备用目录也不可写、队列满 | 当前内存继续，不保证重启后可恢复；明确提示 | 释放空间或权限；退出后没有文件就无法凭空恢复 |
| R08 | 快照构造KeyError/序列化错误 | 最多5秒，提醒继续任务；不覆盖旧快照为伪造状态 | 本次快照可能未保存，查记录提醒和旧checkpoint时间 |
| R09 | 关键图编码/写盘失败 | 只记录提醒，不当相机坏/无物品 | recording.save_images、磁盘空间；正常识别继续 |
| R10 | ROS roslaunch日志No space left on device | 外部节点可能无法启动，继而通信失败；main备用日志不能救外部ROS节点 | df -h、df -i、rosclean check；先处理实际日志盘，再启动节点 |
| R11 | Ctrl+C / SIGINT / SIGTERM、stop_topic=true | 主动暂停、取消当前目标、请求零速、保存最后状态 | /embodied/stop是Bool；false不解除已锁存停止，恢复需新进程 |
| R12 | ROS shutdown | 设备/系统暂停 | 节点关闭、launch退出、名称冲突；单master网络断未必直接shutdown，会由通信复核发现 |
| R13 | 535秒预算触发 | 收臂保留可能持物、优先离场；不是反馈故障 | 默认600-max(30,45+20)，详见TIMEOUTS |
| R14 | 600秒硬截止 | TIME_UP收尾，不继续新动作、不要求resume | exit_motion_completed如未到达仍false |
| R15 | 关闭资源/写result失败 | 每项3秒，继续关闭其他资源，提示cleanup_errors | 不反改之前成功的实际动作结果；最终仅打印已确认保存的result路径 |
| R16 | 硬杀/断电/OOM/系统崩溃 | 可能没有保存/清理，只有最后已落盘记录可用 | 系统日志/最后checkpoint；自动恢复不等于补造丢失状态 |
| R17 | 主流程以外仍出现未覆盖异常/持续软件损坏 | 最后异常防线会保存并暂停，输出类型/目录 | 交回最后阶段及traceback；不能保证任意程序损坏都可继续物理动作 |
| R18 | 三级把手/开门中断（当前关闭） | 不把拉门等同普通物品回程，不自动重放不明开度 | 未启用3不进入；未来需独立storage实测与人工恢复确认 |

## 8. 现场最短排查顺序

先看最后阶段、错误类型、模块名。导航失败先看 `/move_base/result` 的 `status.text`；机械臂失败先看 `[机械臂动作失败]` 的步骤/目标/实测反馈和驱动终端；识别却不抓先看 `grasp_result.reason` 和深度。下面均是读取状态，不发布运动目标：

```bash
rosnode list
timeout 5s rostopic echo -n 1 /move_base/result
timeout 5s rostopic echo -n 1 /j2n6s300_driver/out/tool_pose
timeout 5s rostopic echo -n 1 /j2n6s300_driver/out/finger_position
rostopic info /summer_tts_topic
rostopic info /summer_tts_done
rospack find kinova_bringup
df -h
df -i
rosclean check
```

`timeout 5s ...`没有输出只说明该5秒未收到，不等于独立证明硬件坏。导航尚无新终态时result也可能没有消息。

需重启驱动的已知命令：`roslaunch kinova_bringup kinova_robot.launch`。需启动语音：`rosrun summer_tts summer_tts_node`。各终端先source ROS、对应工作空间和项目setup_network.bash；机械臂/相机参数来源不能替代驱动节点。

默认证据在 `runs/<场次目录>/events.jsonl`、`config_used.json`、`result.json` 和 `images/`，checkpoint在runs父目录。默认盘写失败后会提示本机临时备用位置；`--resume`从默认与备用位置选最新有效文件。记录已落盘、原场次尚未截止、连接修复后运行 `python3 -B main.py --resume`。仍持物时用新场次启动会执行张爪初始化，应恢复原场次。

此表覆盖项目已定义的故障分支和常见外部故障；不声称穷尽操作系统、第三方SDK或未来代码的所有异常。本次离线验证不等于这些故障已在机器人上逐项注入验收。
