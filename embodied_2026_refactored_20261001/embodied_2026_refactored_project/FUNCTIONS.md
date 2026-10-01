# 实际复用的机器人接口

本轮 main_2026.py 只保留启动入口。配置、会话、流程、视觉与证据辅助代码在 app/ 下；机器人技能依旧来自 legacy/。新增类/方法负责组织现有接口调用，没有增加柜门等未实现的技能。

10 个旧模块和 support/safety.py 与上一中文注释版字节一致。95 个原函数/方法声明保持，完整任务流程 41 处明确调用参数匹配。下表 32 项复用接口保持。

A 表示往年代码.zip 中的 tongyong_25；B 表示往年代码2.zip。精确附件路径与原始/交付哈希见 legacy_manifest.json；历史功能修改见 LEGACY_FIXES.patch。

| 序号 | 原函数/方法 | 来源 → 工程路径 | 本版用途 |
|---:|---|---|---|
| 1 | `Navigator.__init__(location)` | B/navigator2.py → legacy/common/navigator2.py | 创建导航客户端；保留部分初始化对象以便停止 |
| 2 | `Navigator.set_goal(frame_id, position, orientation)` | 同上 | 创建 map 导航目标，并由主函数补时间戳 |
| 3 | `Navigator.stop()` | 同上 | 取消导航目标 |
| 4 | `SmartGoalFinder.__init__()` | B/position_last_second.py → legacy/common/position_last_second.py | 初始化定位/路径查询 |
| 5 | `SmartGoalFinder.get_robot_pose()` | 同上 | 获取原接口返回的 `PoseStamped` |
| 6 | `SmartGoalFinder.validate_goal(start_pose, goal_pose)` | 同上 | 检查导航目标是否可规划 |
| 7 | `CoordinateConverter.__init__()` | B/camera_to_map.py → legacy/common/camera_to_map.py | 初始化坐标转换 |
| 8 | `CoordinateConverter.get_map_coords(camera_point)` | 同上 | 将原视觉三维点转换到 map，只使用 XY |
| 9 | `calculate_facing_goal(robot_pose, target_point_map, distance=...)` | B/goal_calculator.py → legacy/common/goal_calculator.py | 计算面向目标的底盘停车位 |
| 10 | `SummerTTSSpeaker.__init__()` | B/summer_tts_speaker.py → legacy/common/summer_tts_speaker.py | 创建语音发布者 |
| 11 | `SummerTTSSpeaker.speak(text)` | 同上 | 用中文名称播报识别和交付结果 |
| 12 | `Base.__init__()` | A/base_controller.py → legacy/tongyong_25/base_controller.py | 创建原底盘控制对象 |
| 13 | `Base.stop()` | 同上 | 发布停止速度 |
| 14 | `KinectCamera.__init__()` | A/catch_ground/src/detector_items_c.py → legacy/tongyong_25/catch_ground/src/detector_items_c.py | 创建原 Kinect 相机对象 |
| 15 | `KinectCamera.open_camera()` | 同上 | 打开 Kinect |
| 16 | `KinectCamera.release()` | 同上 | 释放 Kinect |
| 17 | `ItemsDetector.__init__(model_path=...)` | 同上 | 加载实际指定的搜索/复核模型 |
| 18 | `ItemsDetector.pred()` | 同上 | 检测全部框并在 `color_frame` 上绘制结果 |
| 19 | `ItemsDetector.world(camera, depth_image, yolors)` | 同上 | 由框中心深度求相机坐标；内部调用原 `get_calibration()` |
| 20 | `RealSenseYolo11Detector.__init__(weights=..., conf_thres=...)` | A/catch_ground/src/realsense_yolo11.py → legacy/tongyong_25/catch_ground/src/realsense_yolo11.py | 启用地面任务时加载原模型 |
| 21 | `RealSenseYolo11Detector.detect_targets(target_items=..., max_retry=3, show_window=False)` | 同上 | 地面目标复检；现有方法体已加有限数值/多候选检查，保留原 YoloResult 对象 |
| 22 | `RealSenseYolo11DetectorDesk.__init__(weights=...)` | A/catch_ground/src/realsense_yolo11_desk.py → legacy/tongyong_25/catch_ground/src/realsense_yolo11_desk.py | 准备时检查桌面模型实际标签；抓取内部仍由原 catch.py 创建 |
| 23 | `KinovaRobot.__init__("j2n6s300")` | A/catch_ground/src/catch.py → legacy/tongyong_25/catch_ground/src/catch.py | 初始化机械臂；原构造器包含开爪和回 home |
| 24 | `KinovaRobot.goto_detect()` | 同上 | 移动到原地面观察姿态 |
| 25 | `KinovaRobot.goto_home()` | 同上 | 使用原收拢姿态，检查最后 action 状态 |
| 26 | `KinovaRobot.catch_table(target=[grasp_label])` | 同上 | 原桌面抓取；配置权重，记录检测结果，保持隐式 None 返回 |
| 27 | `KinovaRobot.catch_table_short(target=[grasp_label])` | 同上 | 原矮桌面抓取；配置权重，记录检测结果，保持隐式 None 返回 |
| 28 | `KinovaRobot.catch_ground(result=ground_result)` | 同上 | 原地面抓取；传入带 name/x/y/z/angle 的原对象 |
| 29 | `KinovaRobot.close_finger()` | 同上 | 发送原闭爪动作，之后读取新反馈 |
| 30 | `KinovaRobot.setCurrentFingerPosition(feedback)` | 同上 | 用新收到的 FingerPosition 更新原对象 |
| 31 | `KinovaRobot.arm_run(unit="mdeg", pose_target=..., relative=False)` | 同上 | 在标定放置停车位执行配置的原笛卡尔姿态动作 |
| 32 | `KinovaRobot.finger_run(unit="percent", finger_target=[5,5,5], relative=False)` | 同上 | 使用原夹爪动作放下物品 |

`ItemsDetector.detect()` 本版不直接使用，因为它通常只给出单个匹配目标；本版通过原 `pred()` 和 `world()` 读取多个检测结果，按不同帧进行结果关联。`get_object_classes_sorted()` 不直接使用，避免其中的标签大小写变换影响精确类别匹配。

没有调用原 `Navigator.goto()/go_to_location()`，因为其循环可能无限等待并重复清图，也不返回可靠布尔成功值。流程模块仍使用原 Navigator 创建的 client，直接调用原模块已经使用的 `send_goal`、`wait_for_result`、`get_state`、`cancel_all_goals` 等 actionlib 基础接口，并限制等待时间。流程模块没有重新实现导航规划器。

其他基础设施为 `rospy.init_node/Subscriber/wait_for_message/Time/Duration/is_shutdown/signal_shutdown`、ROS 消息类型、`tts_pub.get_num_connections()`、OpenCV 证据保存以及 Python 标准库。这些负责接线、等待、记录和时间控制，不是额外编造的机器人技能。`__new__` 仅用于保存部分初始化对象，后续仍执行原类的 `__init__`。

**返回值约定**：三个 `catch_*`、`arm_run`、`finger_run` 等原方法没有可靠的成功布尔返回值。程序不写 `if robot.catch_table(...):`；使用 action 状态、夹爪反馈和放置后视觉证据逐步判断，且保留 `UNKNOWN`/`RELEASE_UNVERIFIED` 状态。比赛执行阶段使用 GuardedClient 检查内部各 action 的返回状态；但 action 成功仍不等同于物理抓稳。2026-10-01起，原构造器在首次开爪／回home前即包装client；其签名和动作顺序保持。

## 同 capture 使用的原有 SDK 方法

流程模块不新增 `get_rgbd()`；把往年 `KinectCamera.get_frame/get_depth` 内已经调用的 SDK 接口放到同一次采集下顺序使用：

| 接口 | 来源与用途 |
|---|---|
| `camera.device.update()` | 原相机方法已经使用；每帧只调用一次，取得 capture |
| `capture.get_color_image()` | 原 get_frame 内已有；取该 capture 的彩色图 |
| `capture.get_transformed_depth_image()` | 原 get_depth 内已有；取同一 capture 的对齐深度 |

相机内参通过 `camera.K` 配置覆盖；坐标计算仍调用原 `ItemsDetector.world()`，它内部读取原 `KinectCamera.get_calibration()`。

## 从新增工程复用的基础设施

| 接口 | 来源 | 用途 |
|---|---|---|
| `DeadlineGate.__init__()` / `check()` | 新附件 safety.py → support/safety.py | 统一阶段截止时间；原 check 内补充停止队列处理 |
| `GuardedClient.__init__()` | 同上 | 包装原 Navigator/Kinova 创建的 action client，不改原机器人构造器签名 |
| `GuardedClient.send_goal()` | 同上 | 原技能内部每次发送动作前经过门控 |
| `GuardedClient.wait_for_result()` | 同上 | 有界轮询、检查终态，中间失败中止旧动作序列 |
| `GuardedClient.__getattr__()` | 同上 | 转发原 get_state/get_result/cancel_all_goals 等接口 |

同文件的 `limit()` 和 `interruptible()` 保留为来源代码，但完整任务流程不调用它们；流程模块自己设置 SIGALRM，避免叠加两套定时器。开始/停止消息以标准库 `SimpleQueue.put` 作为 ROS Subscriber 回调，没有另写机器人回调函数。

原 `catch_table/catch_table_short` 内部还调用 `RealSenseYolo11DetectorDesk.detect_targets(target_items=...)`。它沿用上一版既有检测方法维护：无目标、多候选或无效数值时返回 None，不再构造图像中心的 Cola 结果。`arm_run/finger_run` 内的原 `cartesian_pose_client/gripper_client` 等底层方法保持原样，实际 action client 在首次初始化动作前由注入门控的原构造器包装。


## 本轮调用位置

| 内容 | 位置 |
|---|---|
| 原类初始化与精确模型标签核对 | RobotSession.prepare |
| Navigator、规划与靠近计算 | Workflow.handle_navigate / handle_approach |
| catch_* 与夹爪反馈 | Workflow.handle_grasp / handle_grip |
| goto_home、arm_run、finger_run | Workflow.handle_stow / handle_place |
| 原 pred/world/TF 与同 capture 采集 | VisionStage._collect_observation |
| 截止与阶段执行 | Workflow.run；门控仍为 support/safety.py |
| 取消、底盘停止、相机关闭 | RobotSession._stop_resources |

流程层 True/False 只区分原正常结束与 continue；原技能的 None 返回约定保持。只有 VisionStage._after_delivery 在判据通过后增加交付数量。

配置检查不导入机器人模块。实机准备在原构造器调用前设置同一个 deadline_gate；原两类 action client 的包装仍先于首次初始化动作。
