# 本次静态检查与工程交付记录

日期：2026-10-01。模式：STATIC_ONLY。未接入真实机器人，未运行模拟／回归任务。

本版在`embodied_2026_static_20261001.zip`基础上给全部13个Python文件添加中文注释，另附`READ_CODE.md`。12个机器人运行文件的可执行AST与该包一致；检查器仅调整原始文件校验分支，允许有来源记录的注释版以完整原始AST证明逻辑保持。

## 已执行结果

| 检查 | 结果 | 能证明的范围 |
|---|---|---|
| Python源码 | 13个文件通过 | Python3.8语法解析和当前解释器代码对象编译；没有执行代码对象 |
| 原接口声明 | 95个，全部一致 | 原参数、默认值、注解及装饰器与两个原始压缩包一致 |
| 主流程明确调用 | 41处通过 | 构造器、函数、实例方法及抓取命令列表的参数绑定 |
| 本地导入 | 16处通过 | 目标模块在包内，所引用类／函数实际存在 |
| 旧代码维护范围 | 10模块核对通过 | 6个原可执行AST保留，4个沿用上一版对8个已有方法体的维护；本版均补中文注释 |
| 注释前后逻辑 | 12个运行文件AST一致 | 相对上一版静态交付包；检查器校验分支单独记录 |
| 注释覆盖 | 13个文件，135处定义 | 文件说明、类／函数／方法说明、关键步骤与单位，见ANNOTATION_REPORT.json |
| 抓取与标定常量 | 受保护赋值一致 | 原home、手眼矩阵、夹爪尺度及抓取位置常量未改 |
| 构造器门控接线 | 通过 | main调用前注入gate；两个client包装位置先于原初始化运动 |
| 停止清理源码 | 通过 | 清理路径未发现回home、松爪或发送新目标调用 |
| 当前文件哈希 | 通过 | 旧模块及门控文件与交付清单一致 |
| 静态检查总计 | 134项通过，0项失败 | 逐项见validation/STATIC_REPORT.json |

检查器只使用标准库；不导入rospy、相机SDK、模型或机械臂模块。参数绑定通过AST与inspect.Signature完成，不调用任何机器人函数。

## 不连接设备的CLI

| 命令 | 实际退出码 | 解释 |
|---|---|---|
| python3 -S main_2026.py --help | 0 | 入口参数可解析 |
| python3 -S main_2026.py --list-tasks | 0 | 18条任务模板，4条三级unsupported |
| python3 -S main_2026.py --check-config | 2 | 默认现场配置未填，按预期拒绝启动 |

-S禁用site加载。完整命令记录、解释器版本和模板错误清单随包保存。静态检查通过与模板配置拒绝并不矛盾：前者检查工程源码，后者检查现场参数。

## 原始来源与维护边界

上一版从往年两个压缩包抽取原始源文件并核对原始哈希。本注释版继续使用其接口和AST基线，原始哈希保持，当前交付哈希随注释更新。输入压缩包的SHA256见validation/INPUTS.json；本版直接注释的基准包见validation/ANNOTATION_REPORT.json。

| 修改模块 | 允许改变的既有方法体 |
|---|---|
| navigator2.py | Navigator.__init__ |
| catch.py | KinovaRobot.__init__、getcurrentCartesianCommand、getCurrentFingerPosition、catch_table、catch_table_short |
| realsense_yolo11.py | RealSenseYolo11Detector.detect_targets |
| realsense_yolo11_desk.py | RealSenseYolo11DetectorDesk.detect_targets |

AST基线将上述方法体遮蔽后比较完整模块，另核对全部声明、嵌套函数清单及受保护的几何赋值。上一版功能差异见LEGACY_FIXES.patch；本版中文注释及检查器校验分支差异见ANNOTATIONS.patch。未维护方法体的6个模块以完整原始AST验证，新增注释不再宣称逐字节一致。

## 当前结论的边界

没有实际验证ROS服务、SDK、模型标签、推理准确率、真实耗时、动力学、可达性、碰撞、抓稳、运输掉落、实物完整入区、TTS播放或紧急制动。第三方库及设备返回类型的运行一致性仍需原机器人环境验证。静态检查不能证明新改动所有运行分支正确。

默认配置仍有现场空值和未完成标定。旧抓取None返回不代表成功；交付仍要求反馈、视觉、时间和槽位条件。没有增加柜门／抽屉、对手意图、断点恢复或新机器人技能。

历史53项测试及模拟叙述只保留在docs/HISTORICAL_VALIDATION_20260930.md，未重跑，不计入上述结果。
