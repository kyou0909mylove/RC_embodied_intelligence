# 运行main还需补什么（v13）

更新：2026-10-07。已保留现场local、来源标定和模型；当前没有新增一个“必须另外提供”才能导入main的本地Python模块。设备驱动/SDK仍是外部运行环境，物理姿态有来源不代表适合当前房间。

## 外部环境

| 内容 | 放哪里/怎样启动 | 负责 |
| --- | --- | --- |
| 同一地图、底盘/激光/TF与move_base | 原导航工作空间；沿用navigation_imu_2.launch | 导航 |
| Kinova驱动/消息包 | 本机实际catkin工作空间，提前roslaunch kinova_bringup kinova_robot.launch；main终端也source | 机械臂 |
| Azure Kinect SDK/pykinect_azure | 当前运行main的Python/系统环境；main打开头部设备 | 视觉 |
| pyrealsense2及USB设备 | 同环境；main打开腕部，多台时填actions.camera_serial | 视觉/机械臂 |
| PyTorch/Ultralytics可用环境 | 已能加载当前21类权重的环境；无需另补模型 | 视觉 |
| SummerTTS | 已有summer_tts_ws，提前rosrun summer_tts summer_tts_node | 语音 |
| 双向ROS网络 | 每终端source项目setup_network.bash；master/IP匹配实际网络 | 导航 |
| ROS日志可写空间 | 外部ROS日志盘；main备用记录不代替节点启动日志 | 现场维护 |

## 除打点外仍需现场确认的已有参数

全部写 `config/navigation.local.json`；表内是现有资料的验收位置，不是要求从头重新标定。

| 参数/信息 | 已有内容 | 需确认/补充位置 |
| --- | --- | --- |
| home/place | home实测值与现有place保留 | actions.home_pose适合行驶；drop停靠时actions.place_pose能把物品放区内 |
| 桌观察/备用姿态 | 来源主姿态已填 | 该家具detect_pose/alternate_detect_poses；是否可到达且物品全可见 |
| 三层柜观察/完整抓取 | v2 low/mid/high、几何及full已接入 | layers[].detect_pose及备用位、profiles.shelf；上游仍在调试需实机验证 |
| 地面几何 | catch_ty R/T、锚点、-0.20基座Z和角度保留 | profiles.ground；点上ground可覆盖anchor_m/grasp_z_m，确认当前安装/物品高度 |
| 手眼/经验补偿 | surface/shelf/ground分别沿用来源 | 安装未变先沿用；换安装或公式才改对应profiles/coordinates，不混用补偿 |
| 三指阈值 | 6750/800/至少1指，100边界仅提示 | actions.gripper_feedback；实物空夹/薄物/持物读数是否合适 |
| 相机/模型 | 头Kinect、腕RealSense、21类权重已有 | confidence、frames/min_hits、深度/角度质量；多RealSense补serial |
| 物品身份 | targets完整物品集与中文名 | tablet资料矛盾保持现有排除，现场核对后改excluded_labels/label_names，不猜身份 |
| 家具多视角 | 单桌/柜/地面3点已有 | 大桌多个map点共享furniture_id，按列表排路线；不需要每家具预填物品数 |
| 耗时 | 本版默认阈值已全部写明 | timing/actions/recovery和单点wait_timeout；按TIMEOUTS调整实测耗时 |

## 不需为当前一级/二级补的文件

不需按钮/裁判开始topic、人员监控、ROI、末端XYZ包围范围、新的中间释放姿态或每家具库存表。腕部手眼资料已采用来源，不要求再提交同一结果。三级未启用不需把手/开门路径或抽屉内部几何。

## 后续代码/资料放置

现场值写local。模型与元数据放models并同步model/label_map；物品资料补targets和speech.label_names。三种抓取分别改pick_surface/shelf/ground与对应coordinates；得分区放置改put，导航失败耗尽主动弃物改release；模块复核和计时恢复改core/recovery，不把相机/语音另复制进每个抓取脚本。

三级未来从example复制柜/抽屉点，补storage实际把手位、开门路径、松手和内部观察/几何，再启用并把3加enabled_levels。当前未测开度不能宣称“门已完全打开”。

所有启动错误应先看具体参数文件/键/行号，运行错误看ERROR_GUIDE，阈值见TIMEOUTS；只在设备排障后恢复原场次，不重置600秒。软件连接成功与当前房间全流程实机跑通仍是两件验收工作。
