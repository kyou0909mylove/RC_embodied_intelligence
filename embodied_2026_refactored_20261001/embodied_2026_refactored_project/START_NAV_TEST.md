# 你们机器人Ubuntu20.04电脑的导航单测步骤

本包根据两张启动流程图整理。底盘主机地址为`192.168.31.200`，SSH用户为`eaibot`；导航节点在底盘主机运行，RViz与本测试脚本在专用Ubuntu20.04电脑运行。本完整工程随包保留导航独立检查入口，可以先测试导航，再联调完整任务。

| 执行位置 | 启动内容 |
|---|---|
| SSH登录底盘后的终端 | `roslaunch dashgo_nav navigation_imu_2.launch` |
| Ubuntu20.04电脑的本地终端 | `roslaunch dashgo_rviz view_navigation.launch` |
| Ubuntu20.04电脑另一个本地终端 | `python3 tools/test_navigation.py --check/--plan/--execute` |

这些launch名称来自用户提供的流程图。此处尚未取得launch正文或`tongyong.sh`正文，具体地图路径、节点组成和`tongyong.sh`的作用需以现场文件为准。

## 先用现有RViz验证底盘导航

已有可用地图时，使用流程图第三部分“导航”。`gmapping_imu.launch`属于建图阶段，导航测试加载现有地图。确认`navigation_imu_2.launch`实际加载的是当前场地地图。

在Ubuntu20.04电脑打开第一个终端，沿用已有ROS与工作空间环境：

```bash
ssh eaibot@192.168.31.200
roslaunch dashgo_nav navigation_imu_2.launch
```

导航已在运行时保留原来的终端。这个终端是底盘主机的终端，保持运行。

在Ubuntu20.04电脑打开第二个本地终端，使用现场电脑自己的IP设置ROS网络，再启动RViz：

```text
export ROS_MASTER_URI=http://192.168.31.200:11311
export ROS_IP=这台Ubuntu20.04电脑连接底盘时的IPv4
roslaunch dashgo_rviz view_navigation.launch
```

上面的第二行是待填写说明；实际运行时替换为IP地址。可用`ip -4 route get 192.168.31.200`查看，其中`src`后面的地址即本机访问底盘所用的IPv4。若同时设置了`ROS_HOSTNAME`，它会覆盖`ROS_IP`；本包的网络脚本会在当前终端改用IPv4方式。

RViz里将Fixed Frame设为`map`。按流程图使用`2D Pose Estimate`设置实际位置和朝向，使雷达与地图轮廓基本重合；定位正确后，用`2D Nav Goal`选择空旷处约0.5～1米的近距离目标。点击后会发送实际导航目标。确认机器人到达目标附近并停稳，实体急停保持可用。

这个阶段完全使用你们现有的导航工程。如果此处失败，先检查地图、定位、TF、底盘与导航参数。

## 把本工程放到Ubuntu20.04电脑

将`embodied_2026_refactored_20261001.zip`拷到这台电脑，用文件管理器解压到主目录，得到`~/embodied_2026_refactored_project`。本次完整包已包含导航脚本及所需导航/定位模块；导航检查不读取完整任务配置，不初始化机械臂和相机。

使用这台电脑原来能启动ROS1程序的终端环境，进入包目录：

```bash
cd ~/embodied_2026_refactored_project
source ./setup_network.bash
```

脚本自动设置底盘master地址，读取这台电脑的路由源IP，并打印`ROS_MASTER_URI`和`ROS_IP`。它必须用`source`运行，变量才留在当前终端。每个新终端分别加载ROS环境与本网络配置。

如果使用该包之后启动RViz，第二个本地终端可以直接这样运行：

```bash
cd ~/embodied_2026_refactored_project
source ./setup_network.bash
roslaunch dashgo_rviz view_navigation.launch
```

`setup_network.bash`只设置网络变量，原ROS与catkin环境仍由现场现有配置加载。在确实使用Noetic时，ROS基础环境可用`source /opt/ros/noetic/setup.bash`；工作空间路径以现场为准。

## 第三个本地终端：先检查项目接口

```bash
cd ~/embodied_2026_refactored_project
source ./setup_network.bash
printenv ROS_DISTRO
python3 --version
python3 -c 'import rospy, actionlib, tf2_ros, tf_conversions, numpy; print("ROS1 Python3依赖导入通过")'
rosnode list
rosservice list
python3 tools/test_navigation.py --check --report runs/navigation_check.json
```

这与第二张图中运行`jujia26.py`的位置相同：在Ubuntu20.04电脑本地运行。使用Python3。所需Python依赖为该电脑现有ROS1的`rospy`、`actionlib`、消息包、`tf2_ros`、`tf_conversions`以及NumPy；没有模型、相机或Kinova依赖。

本次导航单测只启动已明确的导航launch、RViz与测试脚本。`tongyong.sh`的内容目前未知，可在之后接入完整具身流程前查看：

```text
在tongyong.sh所在目录执行：
sed -n '1,240p' tongyong.sh
```

测试脚本通过原Navigator连接move_base action与clear_costmaps服务，通过原SmartGoalFinder连接make_plan并读取`map -> base_link`定位。成功时最后的JSON包含：

```json
{"status": "CONNECTIVITY_OK", "goal_sent": false, "exit_code": 0}
```

该模式只检查连接与定位。若失败，保留完整输出和`runs/navigation_check.json`。

## 指定目标，检查路径并执行一次导航

先结束其它正在执行的导航目标。RViz与本脚本分别发送目标，不同时控制同一次测试。

在第三个终端等待RViz选点：

```bash
rostopic echo -n 1 /clicked_point
```

在RViz使用`Publish Point`点击空旷目标。确认消息`header.frame_id`为`map`，记录`point.x`与`point.y`。没有Publish Point时从Tools面板添加；话题名称以其Tool Properties为准。Publish Point是选点工具。

在同一个终端输入这两个实际坐标，先检查路径：

```bash
read -r -p "输入目标map坐标x和y，用空格分隔：" NAV_TEST_X NAV_TEST_Y
python3 tools/test_navigation.py --plan --goal "$NAV_TEST_X" "$NAV_TEST_Y"
```

出现`PATH_AVAILABLE`且退出码为0后，用项目接口执行一次导航：

```bash
python3 tools/test_navigation.py --execute --goal "$NAV_TEST_X" "$NAV_TEST_Y" --timeout 30 --report runs/navigation_test.json
```

省略朝向参数时保持连接检查时读到的底盘朝向。要指定最终朝向可加`--yaw-deg`，单位度，map正x方向为0度。目标坐标以米为单位，填写目标在地图中的位置。

`--execute`先检查定位和路径，再发送一次目标，不做无限重试。连接与规划各有默认20秒的软件限时，实际导航默认30秒。超时、Ctrl+C或停止消息触发取消请求。本脚本默认监听`/embodied/stop`的Bool消息，可用`--stop-topic`对齐现场停止话题。

成功输出为`NAVIGATION_SUCCEEDED`且`exit_code=0`。还需观察实际到达位置与停稳情况，动作服务器成功状态不单独证明物理表现。

## 按失败位置排查

| 结果 | 优先查看 |
|---|---|
| `rosnode list`无法连接master | 底盘网络、master是否运行、ROS_MASTER_URI |
| master可见但TF/话题/服务收不到数据 | 两端网络可达性、本机ROS_IP、ROS_HOSTNAME覆盖与主机名解析 |
| 导航action或服务等待超时 | `navigation_imu_2.launch`是否已启动，节点/服务名称是否重映射 |
| make_plan或clear_costmaps服务缺失 | 原导航配置；当前接口固定使用move_base这些服务名 |
| map到base_link的TF失败 | 初始定位、TF链、实际底盘坐标系名称及ROS时间 |
| 路径不可用 | 地图是否正确，起终点是否在可通行区域，定位与代价地图 |
| RViz能导航而脚本失败 | 项目接口名、Python3依赖、原make_plan预检查返回值 |

`rosrun tf tf_echo map base_link`可查看本项目实际使用的定位链，看完按Ctrl+C。若现场只有base_footprint，需要先核对与base_link的TF连接，不能直接把不同坐标系的结果混用。

原底盘ROS master地址保持为流程图中的`192.168.31.200`。如果现场地址改变，更新`setup_network.bash`中的两个对应地址。机器人原有速度、避障和地图参数由现有导航工程负责。

## 验证与来源

本包的4个Python文件逐字节复用已交付代码：测试脚本来自导航补充包，两个导航模块与support/safety.py来自中文注释版工程。新增内容为网络配置脚本与本文档。当前环境只做源码、接口及参数解析检查，没有访问机器人或执行模拟导航。包内STATIC_CHECK.json和MANIFEST.sha256记录核对结果。

ROS环境变量说明（ROS Wiki镜像）：
https://ftp.osuosl.org/pub/ros/ros_wiki_mirror/ROS%282f%29EnvironmentVariables.html
