#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
任务2：桌面物品抓取
=====================================================

用于【单张桌面】的物品抓取。
相比学姐的三层货架版，本文件去掉了 layer 结构，只保留一个桌面检测位。

沿用学姐已验证的关键设定（不要改）：
  1) 识别器 RealSenseYolo11Detector（地面版）：开白平衡预热，【不】用框内扫深度
     （框内扫深度会扫到偏离中心的点，导致前伸横向跑偏）；
  2) 手眼标定 camera_to_ee + camera_point_to_tool_pose

位姿格式：[x, y, z, tx, ty, tz]，单位：米 + 度。

用法：
    python3 table_pick.py --target lays_chip_tube
"""

from __future__ import print_function

import argparse
import sys
import time
from pathlib import Path

import numpy as np

# 依赖文件（catch.py / realsense_yolo11.py / best.pt 等）都在本目录，用局部导入
import rospy
import geometry_msgs.msg

from catch_ground.src.catch import KinovaRobot
from catch_ground.src.realsense_yolo11_ref import RealSenseYolo11Detector


# =============================================================================
# 调参区：所有位姿均为 [x, y, z, tx, ty, tz]，角度单位是度
# =============================================================================

# 新模型(best(1).pt)可识别的物品名（共 18 类，与模型 names 一一对应）
TARGET_NAMES = [
    "oreo_box",
    "sprite_bottle",
    "green_shampoo_bottle",
    "white_pump_bottle",
    "franzzi_box",
    "mirinda_bottle",
    "blue_bowl",
    "water_bottle",
    "blue_cube",
    "pink_cube",
    "blue_ball",
    "power_adapter",
    "white_cup",
    "nasal_spray_box",
    "iodine_swabs_box",
    "yellow_toy",
    "lays_chip_tube",
    "whitecat_detergent",
]

# 抓取朝向（检测位和抓取位共用，一般不用改）
EULER_GRASP = [81.040, 83.972, 11.606]
EULER_DETECT = [81.040, 83.972, 5.606]


def _pose(x, y, z, tx, ty, tz):
    """把 6 个数拼成一个位姿列表，全部转成 float。"""
    return [float(x), float(y), float(z), float(tx), float(ty), float(tz)]


# 收回位 home
HOME_POSE = _pose(
    0.2104809731245041,
    -0.25873029232025146,
    0.5095799565315247,
    *EULER_GRASP
)

# 桌面检测位
# ★ 现场打点后整行替换。z 就是「桌面高度」，你的桌面多高就把 z 改成多少。
DETECT_POSE = _pose(0.319639, -0.023217, 0.301487, *EULER_DETECT)

# 抬起量：抓取完成后，在抓取位 z 上再加的高度（米）
LIFT_DZ = 0.3

# 碗（blue_bowl）特殊处理：夹爪不够大，先到正上方 40cm 再垂直下降抓取
BOWL_NAME = "blue_bowl"
BOWL_APPROACH_DZ = 0.40   # 碗正上方高度（米）
BOWL_ABOVE_EULER = [-175.258, 19.271, 179.499]   # 碗抓取时夹爪竖直朝下的姿态角 [tx,ty,tz]（度）

# 腕部 RealSense 手眼标定（相机 -> 末端；标准坐标系 [x右,y下,z前]，无轴交换）
HAND_EYE_T = np.array([
    0.02359225,
    0.08173328,
    -0.14197627,
], dtype=np.float64)
HAND_EYE_R = np.array([
    [-0.999283591, 0.02314491, -0.02461251],
    [-0.023260283, -0.999191706, -0.03734953],
    [-0.02564872, -0.03450989, 0.99907178],
], dtype=np.float64)

# 夹爪开合百分比
FINGER_OPEN = [7, 7, 7]
FINGER_CLOSE = [85, 85, 85]
FINGER_TIGHT = [100, 100, 100]

# 模型权重路径（新模型是 best(1).pt）
DESK_MODEL_CANDIDATES = [
    str(Path(__file__).resolve().parents[1] / "models" / "best(1).pt"),
]

# 置信度门槛：低于该值的检测框直接不抓
CONF_THRES = 0.4


def _first_existing(paths):
    """返回第一个真实存在的文件路径；都不存在时返回第一个（触发加载报错便于排查）。"""
    for p in paths:
        if Path(p).is_file():
            return p
    return paths[0]


def _assert_mdeg(pose, name):
    """校验位姿必须是 6 个数 [x,y,z,tx,ty,tz]。"""
    if len(pose) != 6:
        raise ValueError("{} 必须是 6 个数 [x,y,z,tx,ty,tz]，当前长度={}".format(name, len(pose)))


def arm_mdeg(kinova, pose, tag=""):
    """以米 + 度下发位姿，带 tag 时打印。"""
    _assert_mdeg(pose, tag or "pose")
    if tag:
        print("{} {}".format(tag, pose))
    kinova.arm_run(unit="mdeg", pose_target=list(pose))


def get_tool2base_R(robot):
    """读取末端实际姿态四元数（与手眼标定同源），返回 工具系 -> 基座系 旋转矩阵。"""
    topic = '/' + robot.prefix + 'driver/out/tool_pose'
    msg = rospy.wait_for_message(topic, geometry_msgs.msg.PoseStamped, timeout=5.0)
    q = msg.pose.orientation
    return robot.quaternion_to_rotation_matrix(q.w, q.x, q.y, q.z)


# 夹爪中心相对相机偏移（默认值，单位米）：每个物体可在下面单独覆盖
DEFAULT_TCP_GRIP_CAM = np.array([-0.07, 0.067, 0.065], dtype=np.float64)

# ★ 每个物体单独微调：键=模型类名（与 TARGET_NAMES 一致），值=[x,y,z]（米）。
#    没在下面列出的物体，用 DEFAULT_TCP_GRIP_CAM。
TCP_GRIP_CAM_PER_OBJECT = {
    "oreo_box":             np.array([-0.09, 0.137, 0.095], dtype=np.float64),
    "sprite_bottle":        np.array([-0.07, 0.137, 0.095], dtype=np.float64),
    "green_shampoo_bottle": np.array([-0.07, 0.137, 0.095], dtype=np.float64),
    "white_pump_bottle":    np.array([-0.04, 0.177, 0.075], dtype=np.float64),
    "franzzi_box":          np.array([-0.07, 0.137, 0.095], dtype=np.float64),
    "mirinda_bottle":       np.array([-0.057, 0.177, 0.095], dtype=np.float64),
    "blue_bowl":            np.array([-0.07, 0.137, 0.095], dtype=np.float64),
    "water_bottle":         np.array([-0.07, 0.137, 0.095], dtype=np.float64),
    "blue_cube":            np.array([-0.07, 0.197, 0.095], dtype=np.float64),
    "pink_cube":            np.array([-0.07, 0.207, 0.085], dtype=np.float64),
    "blue_ball":            np.array([-0.11, 0.19, 0.125], dtype=np.float64),
    "power_adapter":        np.array([-0.07, 0.207, 0.095], dtype=np.float64),
    "white_cup":            np.array([-0.07, 0.177, 0.095], dtype=np.float64),
    "nasal_spray_box":      np.array([-0.03, 0.137, 0.065], dtype=np.float64),
    "iodine_swabs_box":     np.array([-0.01, 0.137, 0.115], dtype=np.float64),
    "yellow_toy":           np.array([-0.07, 0.177, 0.095], dtype=np.float64),
    "lays_chip_tube":       np.array([-0.07, 0.137, 0.095], dtype=np.float64),
    "whitecat_detergent":   np.array([-0.07, 0.177, 0.075], dtype=np.float64),
}


def _tcp_grip_cam_for(name):
    """按物体名取 TCP_GRIP_CAM；没单独配置的用默认值。"""
    return TCP_GRIP_CAM_PER_OBJECT.get(name, DEFAULT_TCP_GRIP_CAM)


def camera_to_ee(cam_x, cam_y, cam_z, R_tool2base, tcp_grip_cam):
    """相机坐标点 -> 相对夹爪中心的末端系点，再转到基座系（得到基座系偏移向量）。

    物体相对夹爪 = 相机坐标减去 tcp_grip_cam，再乘手眼旋转 R 转成末端系
    （基准是夹爪中心、不再是法兰，所以不加平移 T）。
    """
    p_cam = np.array([cam_x, cam_y, cam_z], dtype=np.float64) - tcp_grip_cam
    p_ee_tool = HAND_EYE_R.dot(p_cam)   # 相对夹爪中心（工具系）
    return R_tool2base.dot(p_ee_tool)   # 基座系


def camera_point_to_tool_pose(cam_x, cam_y, cam_z, detect_pose, R_tool2base, tcp_grip_cam):
    """把相机点（已转基座系）加到检测位上，得到末端目标位姿 [x,y,z,tx,ty,tz]。

    p_ee 是基座系下的偏移向量，直接加到检测位上（三个角度保持检测位不动）。
    """
    p_ee = camera_to_ee(cam_x, cam_y, cam_z, R_tool2base, tcp_grip_cam)
    pose = [
        float(detect_pose[0]) + float(p_ee[0]),
        float(detect_pose[1]) + float(p_ee[1]),
        float(detect_pose[2]) + float(p_ee[2]),
        float(detect_pose[3]),
        float(detect_pose[4]),
        float(detect_pose[5]),
    ]
    return p_ee.tolist(), pose


def build_grasp_pose_from_camera(result, detect_pose, R_tool2base, tcp_grip_cam):
    """由识别结果算出抓取位姿和抬起位姿（抬起只在抓取位 z 上加 LIFT_DZ）。"""
    p_ee, grasp = camera_point_to_tool_pose(result.x, result.y, result.z, detect_pose, R_tool2base, tcp_grip_cam)
    print("基座系偏移 = {}".format(p_ee))
    print("前伸位姿 = {}".format(grasp))
    lift = list(grasp)
    lift[2] = grasp[2] + LIFT_DZ
    return grasp, lift


def pick_once(kinova, target_items):
    """单次抓取：检测位 → 识别 → 开爪 → 前伸 → 合爪 → 抬起 → 回 home。

    target_items：待识别物品名列表；若同时检测到多个，抓取离相机最近（深度最小）的那个。
    """
    detect_pose = list(DETECT_POSE)
    _assert_mdeg(detect_pose, "DETECT_POSE")
    _assert_mdeg(HOME_POSE, "HOME_POSE")

    print("桌面抓取，目标={}，检测位 z={:.3f}".format(target_items, detect_pose[2]))

    print("1. 到检测位")
    arm_mdeg(kinova, detect_pose)
    R_tool2base = get_tool2base_R(kinova)   # 检测位实际姿态 -> 工具系转基座系

    weights = _first_existing(DESK_MODEL_CANDIDATES)
    print("RealSense 目标={}, 模型={}".format(target_items, weights))
    detector = RealSenseYolo11Detector(weights=Path(weights), conf_thres=CONF_THRES)
    result = detector.detect_targets(target_items=list(target_items), show_window=False)
    if result is None:
        print("未检测到 {} 中任意物品，回 home".format(target_items))
        arm_mdeg(kinova, HOME_POSE)
        return False

    print("检测到 {}（离相机最近，置信度 {:.2f}） 相机=({:.3f},{:.3f},{:.3f})".format(
        result.name, result.conf, result.x, result.y, result.z))
    kinova.selected_target_name = result.name
    tcp_grip_cam = _tcp_grip_cam_for(result.name)
    print("使用 TCP_GRIP_CAM = {}".format(tcp_grip_cam.tolist()))
    grasp_pose, lift_pose = build_grasp_pose_from_camera(result, detect_pose, R_tool2base, tcp_grip_cam)

    print("2. 开爪")
    kinova.finger_run(finger_target=FINGER_OPEN)
    time.sleep(0.5)

    if result.name == BOWL_NAME:
        # 碗：夹爪不够大，先到正上方 40cm，夹爪朝下，再垂直下降抓取
        grasp_pose[3:6] = list(BOWL_ABOVE_EULER)   # 夹爪竖直朝下
        lift_pose[3:6] = list(BOWL_ABOVE_EULER)    # 抬起时也保持朝下，避免把碗甩翻
        pre_grasp = list(grasp_pose)
        pre_grasp[2] = grasp_pose[2] + BOWL_APPROACH_DZ
        print("3. 碗：先到正上方 40cm")
        arm_mdeg(kinova, pre_grasp, tag="正上方")
        time.sleep(0.5)
        print("   碗：下降抓取")
        arm_mdeg(kinova, grasp_pose, tag="下降")
        time.sleep(0.5)
    else:
        print("3. 前伸")
        arm_mdeg(kinova, grasp_pose, tag="前伸")
        time.sleep(0.5)

    print("4. 合爪")
    kinova.finger_run(finger_target=FINGER_CLOSE)
    kinova.finger_run(finger_target=FINGER_TIGHT)
    time.sleep(0.5)

    print("5. 抬起")
    arm_mdeg(kinova, lift_pose, tag="抬起")

    print("6. 收回 home")
    arm_mdeg(kinova, HOME_POSE)
    return True


def main():
    """解析参数，连接机械臂并执行一次抓取。"""
    parser = argparse.ArgumentParser(description="任务2：桌面物品抓取")
    parser.add_argument("--target", type=str, default=None,
                        help="可选：只抓某一个物品名（见 TARGET_NAMES）；不传则自动识别全部物品")
    opt = parser.parse_args()

    # 不指定 --target 时，自动识别 TARGET_NAMES 里的全部物品
    target_items = [opt.target] if opt.target else TARGET_NAMES

    print("=" * 60)
    print("任务2 桌面抓取")
    print("目标={}  检测位={}".format(target_items, DETECT_POSE))
    print("=" * 60)

    rospy.init_node("table_pick", anonymous=True)
    kinova = KinovaRobot("j2n6s300")
    if not hasattr(kinova, "client_arm") or kinova.client_arm is None:
        print("机械臂未连接，请先启动 kinova_bringup")
        return

    ok = pick_once(kinova, target_items)
    print("完成" if ok else "失败")


if __name__ == "__main__":
    main()
