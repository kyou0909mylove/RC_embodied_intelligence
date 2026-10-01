# 实际复用的功能函数清单

本表列出主函数直接调用、或作为已有方法引用交给命令列表执行的接口。A 表示“往年代码.zip”中的 `shijiazhuang_2025/shijiazhuang_2025/tongyong_25`；B 表示“往年代码2.zip”的内层目录。精确附件相对路径及哈希见 `legacy_manifest.json`。

主文件仅定义 `main()`；没有新造机器人技能。截止门控来自新增附件的已有基础设施，单独列于文末。旧检测及桌面抓取方法的有限修改见 `LEGACY_FIXES.patch`；机器人动作顺序、运动学常量和函数签名保持原样。本次只做静态源码与接口检查，未运行模拟替身。全部13个Python文件的中文注释和阅读顺序见READ_CODE.md；本版Python差异见ANNOTATIONS.patch。

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

没有调用原 `Navigator.goto()/go_to_location()`，因为其循环可能无限等待并重复清图，也不返回可靠布尔成功值。主函数仍使用原 Navigator 创建的 client，直接调用原模块已经使用的 `send_goal`、`wait_for_result`、`get_state`、`cancel_all_goals` 等 actionlib 基础接口，并限制等待时间。主函数没有重新实现导航规划器。

其他基础设施为 `rospy.init_node/Subscriber/wait_for_message/Time/Duration/is_shutdown/signal_shutdown`、ROS 消息类型、`tts_pub.get_num_connections()`、OpenCV 证据保存以及 Python 标准库。这些负责接线、等待、记录和时间控制，不是额外编造的机器人技能。`__new__` 仅用于保存部分初始化对象，后续仍执行原类的 `__init__`。

**返回值约定**：三个 `catch_*`、`arm_run`、`finger_run` 等原方法没有可靠的成功布尔返回值。程序不写 `if robot.catch_table(...):`；使用 action 状态、夹爪反馈和放置后视觉证据逐步判断，且保留 `UNKNOWN`/`RELEASE_UNVERIFIED` 状态。比赛执行阶段使用 GuardedClient 检查内部各 action 的返回状态；但 action 成功仍不等同于物理抓稳。2026-10-01起，原构造器在首次开爪／回home前即包装client；其签名和动作顺序保持。

## 同 capture 使用的原有 SDK 方法

主函数不新增 `get_rgbd()`；把往年 `KinectCamera.get_frame/get_depth` 内已经调用的 SDK 接口放到同一次采集下顺序使用：

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

同文件的 `limit()` 和 `interruptible()` 保留为来源代码，但本主函数不调用它们；主函数自己设置 SIGALRM，避免叠加两套定时器。开始/停止消息以标准库 `SimpleQueue.put` 作为 ROS Subscriber 回调，没有另写机器人回调函数。

原 `catch_table/catch_table_short` 内部还调用 `RealSenseYolo11DetectorDesk.detect_targets(target_items=...)`。它沿用上一版既有检测方法维护：无目标、多候选或无效数值时返回 None，不再构造图像中心的 Cola 结果。`arm_run/finger_run` 内的原 `cartesian_pose_client/gripper_client` 等底层方法保持原样，实际 action client 在首次初始化动作前由注入门控的原构造器包装。

## 2026-09-30 现有接口维护

`catch_table` 与 `catch_table_short` 仍使用原签名及动作顺序，新增的只有两个实例权重属性和 `last_grasp_detection` 结果记录，没有新增方法。主函数通过 `table_weights` / `table_short_weights` 配置模型，准备检查和实际抓取使用同一路径。独立使用原类且不设置属性时仍保留往年默认路径。只有该结果明确为 None、且原方法未进入抓取分支时，主函数才恢复此前的 EMPTY 状态，调用原 goto_home 后换任务。抓取正常返回 None 仍不表示失败或成功。

清单、收益估计、配额、失败预算、物品包络边线检查、JSONL 日志和原子检查点均在 main() 内完成，没有新增机器人技能函数。主流程直接复用的 32 项接口清单不变；对既有方法体的全部修改见 LEGACY_FIXES.patch。

## 2026-10-01接口维护说明

95个原函数／方法签名从两个原始压缩包重新抽取核对。旧模块中6个文件字节原样，4个文件的8个既有方法体有明确补丁。未新增机器人技能方法。

| 既有方法 | 本轮维护 | 签名与返回约定 |
|---|---|---|
| Navigator.__init__ | 注入deadline_gate时，action／服务等待有限且等待后检查 | 原`(self, location)`保持 |
| KinovaRobot.__init__ | 两类client在初始化运动前门控 | 原`(self, kinova_robotType)`保持 |
| KinovaRobot.getcurrentCartesianCommand | 有限等待并显式使用返回反馈 | 原签名、None返回保持 |
| KinovaRobot.getCurrentFingerPosition | 有限等待并显式使用返回反馈 | 原签名、None返回保持 |
| catch_table／catch_table_short | 沿用配置权重和检测记录维护 | 不改为成功布尔值 |
| 两个detect_targets | 沿用无目标／歧义／非有限数值拒绝 | 原签名和对象／None约定保持 |

deadline_gate是由main在原构造器调用前设置的实例属性，不是新参数或新技能。原运动学与位姿的受保护赋值另作AST核对。
