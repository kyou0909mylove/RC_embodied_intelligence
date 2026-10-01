# 中文代码阅读指南

本工程的13个Python文件都已补充中文注释。文件开头说明职责，类和函数前说明用途，关键步骤解释变量、判断条件、返回值和单位。建议先按下面顺序阅读，不必一次读完所有源码。

## 从主函数开始

1. 打开`main_2026.py`，先看文件顶部和`def main():`前的说明。
2. 搜索`【阶段`，看每一步做什么，再搜索`【扫描模式`，看四种视觉检查。
3. 遇到`kinova.xxx(...)`，到`catch.py`中搜索`def xxx`；遇到`detector.xxx(...)`，到`detector_items_c.py`中查找。
4. 最后阅读时间门控与异常清理，再对照`config/competition.example.json`理解配置。

`main`安排动作的顺序；`legacy`里的类实际提供导航、相机、识别和抓取接口。同一个文件可以包含几十个方法，例如`catch_ground`、`goto_home`和`close_finger`都在`catch.py`，无需各自建立一个同名Python文件。

## 全部Python文件在哪里

下面路径均相对于解压后的工程目录。

| 文件 | 主要职责 | 阅读重点 |
|---|---|---|
| `main_2026.py` | 配置、任务调度和整体流程 | `state`、`holding`、阶段与扫描模式 |
| `legacy/common/navigator2.py` | 创建导航目标、访问导航客户端 | `set_goal`只创建消息，发送由客户端完成 |
| `legacy/common/position_last_second.py` | 当前定位与规划预检查 | `get_robot_pose`、`validate_goal` |
| `legacy/common/goal_calculator.py` | 计算面向物品的停车点 | 输入地图点，输出位置与四元数 |
| `legacy/common/camera_to_map.py` | 相机点转地图点 | 轴重排、TF转换，main只用地图x/y |
| `legacy/common/summer_tts_speaker.py` | 发送播报文本 | `speak`发送请求，没有播放完成回执 |
| `legacy/tongyong_25/base_controller.py` | 底盘速度与位姿 | main使用`stop`发送零速度 |
| `legacy/tongyong_25/catch_ground/src/catch.py` | 机械臂、夹爪、既有抓取方法 | `arm_run`、`finger_run`与三个`catch_*` |
| `legacy/tongyong_25/catch_ground/src/detector_items_c.py` | Kinect搜索与二维识别 | 同帧彩色/深度、`pred`、`world` |
| `legacy/tongyong_25/catch_ground/src/realsense_yolo11.py` | 地面抓取检测 | 唯一有效对象、三维坐标、物体角度 |
| `legacy/tongyong_25/catch_ground/src/realsense_yolo11_desk.py` | 桌面抓取检测 | 由原桌面抓取方法调用，角度字段固定为0 |
| `support/safety.py` | 时间和停止检查、动作客户端包装 | `DeadlineGate`与`GuardedClient` |
| `tools/static_check.py` | 开发时核对源码和接口 | AST、签名、哈希；无需连接机器人 |

## 主流程中的几个变量

| 名称 | 可以怎样理解 |
|---|---|
| `cfg` | JSON配置读入后的字典，存现场位置、模型和阈值 |
| `state` | 下一轮应该执行哪一步，例如`"SCAN"` |
| `stage` | 本轮正在执行哪一步，供超时与记录使用 |
| `scan_mode` | 同样是扫描，但这次要查来源物品、靠近目标还是投放变化 |
| `holding` | 流程对夹爪是否可能携物的判断 |
| `task` / `attempt` | 当前目标配置／这次尝试的记录 |
| `inventory` | 每类任务的配额、已确认交付数及失败次数 |
| `slots` | 放置槽位；每槽有位置、放置姿态和使用状态 |
| `deadline` | 整场截止时刻：正式开始时刻加600秒 |
| `operation_deadline` | 当前阶段截止时刻，不能晚于整场截止 |
| `record` | 本地运行记录，包括观察、尝试和退出状态 |

`state = "GRASP"`只是保存一个字符串；下一轮`while`进入对应`elif`分支时才执行抓取。

| 阶段 | 做什么 |
|---|---|
| `PREPARE` / `WAIT_START` | 准备接口、核对模型、等待本局有效开始信号 |
| `SELECT` | 按配额、失败次数、时间和预计收益选任务 |
| `NAVIGATE` | 前往来源区、靠近点、观察位、投放位或出口 |
| `SCAN` | 读取同帧图像与深度，关联多帧观察，再按扫描模式判断 |
| `APPROACH` | 计算靠近位姿，随后交给导航执行 |
| `GRASP` | 调用对应的原桌面、低桌或地面抓取方法 |
| `GRIP` | 采集新的夹爪反馈，判断是否有稳定持物辅助证据 |
| `STOW` | 调用原机械臂收拢动作，再安排返回与复核 |
| `PLACE` | 到配置放置姿态、松爪、检查打开反馈、回收拢位 |
| `STOP` | 离开循环，取消动作、发零速度并保存记录 |

`SCAN`的四个模式分别是：`SEARCH`找来源区物品，`RECHECK`靠近后复核，`SLOT_BEFORE`记录投放前基线，`SLOT_AFTER`确认投放后出现的新增物品。一次运输会多次进入`NAVIGATE`、`SCAN`和`GRIP`。

| `holding`值 | 含义 |
|---|---|
| `EMPTY` | 流程认为当前空闲，可在其它条件满足时选择下一件 |
| `UNKNOWN` | 可能已拿物，证据不足；异常时不能当作空爪继续抓 |
| `HOLD_INDICATED` | 夹爪反馈满足辅助条件，物理抓稳仍需设备验证 |
| `RELEASE_UNVERIFIED` | 已请求松爪，尚未确认物品落入本次槽位 |

确认交付须通过投放前后视觉、槽位、时间等条件，才增加`inventory`的`delivered`。`catch_*`返回`None`和动作服务器报告成功，都不能单独替代交付确认。

## 坐标与单位最容易混淆

| 数据 | 坐标或单位 | 使用处 |
|---|---|---|
| 搜索检测的`YoloResult.x/y` | 图像像素 | `depth[y,x]`读深度，数组先行后列 |
| `ItemsDetector.world(...)`输出 | 相机三维点，米 | 名字叫world，但还没有转成地图坐标 |
| `CoordinateConverter.get_map_coords(...)`输出 | map三维点，米 | main使用x/y作导航与区域判断 |
| RealSense检测的`YoloResult.x/y/z` | 相机三维点，米 | 同名结果类在另一个文件，字段含义不同 |
| 导航位姿`[[x,y,z],[qx,qy,qz,qw]]` | map位置（米）＋方向四元数 | 不是六个机械臂欧拉角分量 |
| `arm_run(unit="mdeg", pose_target=...)` | 机械臂目标：前三个米，后三个角度 | mdeg在此指米＋度，不是毫度 |
| `arm_run(unit="mrad", ...)` | 米＋欧拉角弧度 | 与mdeg的角度单位不同 |
| `arm_run(unit="mq", ...)` | 米＋四元数 | 方向使用x/y/z/w顺序 |
| `finger_run(unit="percent", ...)` | 闭合比例0～100 | 比例越大越接近闭合，不是张开比例 |
| 夹爪反馈`currentFingerPosition` | 驱动turn读数 | 与`gripper`配置阈值使用同一尺度 |

`K`是3×3相机内参，用于“像素＋深度→相机点”；TF和手眼矩阵是坐标系之间的关系。导航使用相机到地图的转换，抓取使用旧机械臂手眼转换，两条链路须分别理解。旧代码有轴交换、符号和固定偏移，注释已标明实际使用的位置；所有标定值仍需按现场确认。

## 读这些Python写法时

| 写法 | 含义 |
|---|---|
| `class KinovaRobot:` | 定义机械臂对象的结构和方法 |
| `def catch_table(self, target):` | 定义该对象的方法，`self`是当前实例，`target`是传入参数 |
| `kinova.catch_table(target=[label])` | 对已有实例调用方法，用参数名明确传参 |
| `cfg["limits"]["scan_s"]` | 先取字典limits，再取其中scan_s |
| `None` | 没有可用对象或没有显式返回值；含义取决于对应接口 |
| `is not None` | 判断确实得到了对象；不能拿来证明抓取成功 |
| `command(**kwargs)` | 把字典展开为命名参数，调用保存的方法对象 |
| `continue` / `return` | 开始下一轮循环／退出整个函数 |
| `raise ...` | 抛异常，转到对应错误处理与收尾 |
| `try / except / finally` | 尝试执行、处理错误、最后执行收尾；finally在continue时也会执行 |
| `if __name__ == "__main__":` | 直接运行文件时执行；被import时不执行这段入口 |

## 本次注释版的验证

机器人运行相关的12个Python文件与上一版静态交付包逐个比较，可执行AST保持一致，函数签名、模型路径规则和动作数值保持。第13个文件`tools/static_check.py`增加注释，并调整原始文件校验分支：新增注释改变字节时比较原始AST，继续保留原始哈希和当前哈希。

`validation/ANNOTATION_REPORT.json`记录注释覆盖和前后比对；`ANNOTATIONS.patch`记录本版相对上一版的完整Python差异；`LEGACY_FIXES.patch`记录上一版对往年源码的功能维护。这两个补丁按上述顺序描述源码来源。

可在工程目录运行`python3 tools/static_check.py`查看静态结果，运行`python3 main_2026.py --list-tasks`查看清单。示例配置仍缺现场参数，`--check-config`按预期返回2。本次没有执行真实机器人或模拟回归；`--real`是会初始化并驱动设备的现场入口。
