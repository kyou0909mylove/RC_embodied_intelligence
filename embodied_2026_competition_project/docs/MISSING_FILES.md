# 运行环境与现场参数

更新：2026-10-09。原功能文件、SDK功能源码和两份模型已经包含在完整工程中，无需另塞 `task2_pick_v4.py` 或 RealSense 识别器。

| 外部环境 | 准备位置 |
| --- | --- |
| 同一地图、底盘/激光/TF与move_base | 沿用现场导航工作空间及 `navigation_imu_2.launch` |
| Kinova驱动及消息包 | 提前启动 `kinova_bringup`，main终端加载实际catkin环境 |
| Azure Kinect SDK/pykinect_azure | 当前Python及系统环境 |
| pyrealsense2、USB设备 | 当前Python环境，关闭其他相机占用程序 |
| PyTorch/Ultralytics | 能加载当前上传权重的环境 |
| SummerTTS | 提前启动已有 `summer_tts_node` |
| 双向ROS网络 | 每终端配置正确ROS_MASTER_URI/ROS_IP |

已填现场值在 `config/navigation.local.json`，模板在 `navigation.example.json`。重新打点时同步最新地图停靠点；home/place、超时和三指反馈阈值也由local读取。柜层和家具表面原抓取坐标以原功能文件为准。

模型统一在 `models/`：`best.pt` 用于头部、家具表面、地面，`best(3).pt` 用于柜层。新模型需同步相应权重、类别、哈希、元数据和label_map；中文播报名在 `speech.label_names`。

后续替换原功能文件时保留文件名和当前函数入口：柜层 `pick_on_shelf(kinova,target_name,hint_layer)`，家具表面 `pick_once(kinova,target_name)`，地面原类 `catch_ground`。模型位置由项目调用层适配，原文件可单独替换。

三级开柜/开抽屉接口仍保留，现场启用前填写实际storage路径和把手参数。当前local启用难度1、2；删除未使用的旧兼容文件不会删掉三级功能。

启动方式与核验范围见[README](../README.md)。
