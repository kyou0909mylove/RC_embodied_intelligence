#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
按物品思路抓取。柱体直接去抓取位；碗/盘子可先转腕，再斜上方，再下落合爪。

每一层在 SHELF_LAYERS 里各自有一组 grasp_delta = (dx, dy, dz)：
    X = detect_x + (-p_ee[1] + dx)
    Y = detect_y + ( p_ee[0] + dy)
    Z = detect_z + ( p_ee[2] + dz)
改某一层时只改该层的 grasp_delta，不要改 camera_point_to_tool_pose。

换抓取思路时改 GRASP_PROFILES 和 GRASP_PROFILE_BY_NAME。
碗、盘子的 euler 打点后填入 [tx, ty, tz]（度）。None 表示用检测位角度。

换模型时改 MODEL_CANDIDATES 和 SHELF_TARGETS，名字必须和 best(1).pt 的 names 一致。

用法：
    python3 task2_pick_v4.py --layer mid --target sprite_bottle
    python3 task2_pick_v4.py --layer mid
    python3 task2_pick_v4.py --layer mid --photo
返回 True 已抓取，False 没抓到。
"""

from __future__ import print_function

import argparse
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

sys.path.append(r"/home/zq/catkin_ws/src/cmoon/src")

import rospy

from catch_ground.src.catch import KinovaRobot
from catch_ground.src.realsense_yolo11 import RealSenseYolo11Detector


# =============================================================================
# 调参区：人手改的位姿均为 [x, y, z, tx, ty, tz]，角度单位是度
# =============================================================================

DEFAULT_TARGET = None
DEFAULT_LAYER = "mid"
SHELF_TARGETS = [
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
DETECT_TRIES = 2

EULER_GRASP = [81.040, 83.972, 11.606]
EULER_DETECT = [81.040, 83.972, 5.606]

LAYER_ORDER = ["low", "mid", "high"]

# 画面偏左时先到中间点再抓。layers 以后可改成别的层。
LEFT_WAYPOINT = {
    "image_ratio": 0.40, # 画面偏左40%时加中间点
    "layers": ["mid", "high"], # 中层和高层会加中间点，可以根据高度更改
    "dx": -0.2,
    "dy": 0.0,
    "dz": 0.0,
    # 距离中间点超过0.03就sleep 0.5秒，最多判断3次
    "arrive_m": 0.03, 
    "sleep_s": 0.5,
    "max_wait": 3,
}

# 药盒：正面框接近方形，侧面又高又窄。侧面时抓取 Z 再减。
# min_hw 是框的高/宽，超过就算侧面。现场可改 names、阈值和补偿。
BOX_SIDE = {
    "names": ["nasal_spray_box", "iodine_swabs_box"],
    "min_hw": 1.6,
    "grasp_dz": -0.05,
}

# euler：抓取姿态角 [tx, ty, tz]，None 用检测位角度。
# use_hover：True 先到斜上方再下落；False 直接去抓取位。
# rotate_first：True 在斜上方之前先转腕（euler 不是 None 时才转）。
# grasp_dx/dy/dz：在该层 grasp_delta 之后再加到抓取位。bowl/small 在这里调。
GRASP_PROFILES = {
    "cylinder": {
        "euler": None,
        "hover_dx": 0.0,
        "hover_dz": 0.0,
        "use_hover": False,
        "rotate_first": False,
        "grasp_dx": 0.0,
        "grasp_dy": 0.0,
        "grasp_dz": 0.0,
    },
    "bowl": {
        "euler": [-125.186, 84.959, -178.574],  # 现场打点后填 [tx, ty, tz]
        "hover_dx": -0.08,
        "hover_dz": 0.05,
        "use_hover": True,
        "rotate_first": True,
        "grasp_dx": 0.055 - 0.02,  # 相对该层 grasp_delta 再加
        "grasp_dy": 0.02,
        "grasp_dz": -0.07 - 0.036,
    },
    # 174.248, 67.833, -103.716
    "ball": {
        "euler": [174.248, 67.833, -103.716],  # 现场打点后填 [tx, ty, tz]
        "hover_dx": -0.08,
        "hover_dz": 0.05,
        "use_hover": True,
        "rotate_first": True,
        "grasp_dx": 0.055 - 0.08 ,  # 相对该层 grasp_delta 再加
        "grasp_dy": 0.02 - 0.005,
        "grasp_dz": -0.07 - 0.036 - 0.053 + 0.05,
        # 0.609578, -0.283210, 0.357502
        # 0.627998, -0.288113, 0.302053
    },
    "cup": {
        "euler": [-125.186, 84.959, -178.574],  # 现场打点后填 [tx, ty, tz]
        "hover_dx": -0.08,
        "hover_dz": 0.05,
        "use_hover": True,
        "rotate_first": True,
        "grasp_dx": 0.055 - 0.02,  # 相对该层 grasp_delta 再加
        "grasp_dy": 0.02 + 0.057,
        "grasp_dz": -0.07 - 0.036 - 0.1,
    },
    "small": {
        "euler": None,  # 现场打点后填 [tx, ty, tz]
        "hover_dx": -0.03,
        "hover_dz": 0.06,
        "use_hover": False,
        "rotate_first": False,
        "grasp_dx": -0.03,
        "grasp_dy": 0.0,
        "grasp_dz": -0.07,
    },

    "cube": {
        "euler": None,  # 现场打点后填 [tx, ty, tz]
        "hover_dx": -0.03,
        "hover_dz": 0.06,
        "use_hover": False,
        "rotate_first": False,
        "grasp_dx": -0.05,
        "grasp_dy": 0.0,
        "grasp_dz": -0.07 - 0.03,
    },
}

# 必须覆盖 SHELF_TARGETS 全部类名。抓取时按名字取思路。
GRASP_PROFILE_BY_NAME = {
    "oreo_box": "cylinder",
    "sprite_bottle": "cylinder",
    "green_shampoo_bottle": "cylinder",
    "white_pump_bottle": "cylinder",
    "franzzi_box": "cylinder",
    "mirinda_bottle": "cylinder",
    "blue_bowl": "bowl",
    "water_bottle": "cylinder",
    "blue_cube": "cube",
    "pink_cube": "cube",
    "blue_ball": "ball",
    "power_adapter": "small",
    "white_cup": "cylinder",
    "nasal_spray_box": "cylinder",
    "iodine_swabs_box": "small",
    "yellow_toy": "small",
    "lays_chip_tube": "cylinder",
    "whitecat_detergent": "cylinder",
}


def _pose(x, y, z, tx, ty, tz):
    return [float(x), float(y), float(z), float(tx), float(ty), float(tz)]


HOME_POSE = _pose(
    0.2104809731245041,
    -0.25873029232025146,
    0.5095799565315247,
    *EULER_GRASP,
)

SHELF_LAYERS = {
    "low": {
        "name": "低层",
        "detect_pose": _pose(0.264462, -0.235462, 0.370439, -172.764, 88.643, -102.561),
        # 抓取补偿：X=-p_ee[1]+dx，Y=p_ee[0]+dy，Z=p_ee[2]+dz
        "grasp_delta": (-0.04 + 0.03, -0.055, 0.05),
        "lift_dz": 0.03,
    },
    "mid": {
        "name": "中层",
        "detect_pose": _pose(0.264462, -0.235462, 0.705521, -172.764, 88.643, -102.561),
        "grasp_delta": (-0.01, -0.045, 0.17 + 0.02 - 0.02),
        "lift_dz": 0.05,
    },
    "high": {
        "name": "高层",
        "detect_pose": _pose(0.366699, -0.085689, 0.861343, -93.770, 75.993, 175.232),
        "grasp_delta": (-0.07, -0.055, 0.1),
        "lift_dz": 0.03,
    },
}

HAND_EYE_R = np.array([
    [-0.999283591,  0.02314491, -0.02461251],
    [-0.023260283, -0.999191706, -0.03734953],
    [-0.02564872, -0.03450989,  0.99907178]
], dtype=np.float64)

HAND_EYE_T = np.array([0.02359225, 0.08173328, -0.14197627], dtype=np.float64)

FINGER_OPEN = [7, 7, 7]
FINGER_CLOSE = [85, 85, 85]
FINGER_TIGHT = [100, 100, 100]

MODEL_CANDIDATES = [
    str(Path(__file__).resolve().parents[1] / "models" / "best(1).pt"),
]


def _first_existing(paths):
    for p in paths:
        if Path(p).is_file():
            return p
    return paths[0]


def get_layer_cfg(layer_key):
    if layer_key not in SHELF_LAYERS:
        raise ValueError(
            "未知层 '{}', 可选: {}".format(layer_key, list(SHELF_LAYERS.keys()))
        )
    return SHELF_LAYERS[layer_key]


def grasp_profile_for(name):
    key = GRASP_PROFILE_BY_NAME.get(name, "cylinder")
    return GRASP_PROFILES[key]


def search_order(hint_layer):
    get_layer_cfg(hint_layer)
    idx = LAYER_ORDER.index(hint_layer)
    order = []
    for i in range(idx, len(LAYER_ORDER)):
        order.append(LAYER_ORDER[i])
    for i in range(idx - 1, -1, -1):
        order.append(LAYER_ORDER[i])
    return order


def _assert_mdeg(pose, name):
    if len(pose) != 6:
        raise ValueError("{} 必须是 6 个数 [x,y,z,tx,ty,tz]，当前长度={}".format(name, len(pose)))


def arm_mdeg(kinova, pose, tag=""):
    _assert_mdeg(pose, tag or "pose")
    if tag:
        print("{} {}".format(tag, pose))
    kinova.arm_run(unit="mdeg", pose_target=list(pose))


def camera_to_ee(cam_x, cam_y, cam_z):
    p_cam = np.array([cam_x, cam_z, cam_y], dtype=np.float64)
    return HAND_EYE_R.dot(p_cam) + HAND_EYE_T


def camera_point_to_tool_pose(cam_x, cam_y, cam_z, detect_pose, grasp_delta, euler=None):
    """相机点加到该层检测位上。grasp_delta 是该层的 (dx, dy, dz)。euler 为 None 时用检测位角度。"""
    dx, dy, dz = grasp_delta
    p_ee = camera_to_ee(cam_x, cam_y, cam_z)
    if euler is None:
        euler = detect_pose[3:6]
    pose = [
        float(detect_pose[0]) + float(-p_ee[1] + dx),
        float(detect_pose[1]) + float(p_ee[0] + dy),
        float(detect_pose[2]) + float(p_ee[2] + dz),
        float(euler[0]),
        float(euler[1]),
        float(euler[2]),
    ]
    return p_ee.tolist(), pose


def build_grasp_pose_from_camera(result, detect_pose, grasp_delta, euler=None):
    p_ee, grasp = camera_point_to_tool_pose(
        result.x, result.y, result.z, detect_pose, grasp_delta, euler=euler
    )
    print("相对末端 = {}".format(p_ee))
    print("抓取补偿 = {}".format(grasp_delta))
    print("前伸位姿 = {}".format(grasp))
    return grasp


def _fresh_pipeline(detector):
    detector.pipeline = rs.pipeline()
    detector.config = rs.config()


def _detect_items(detector):
    _fresh_pipeline(detector)
    return detector.detect_all_targets(
        target_items=list(SHELF_TARGETS),
        max_retry=DETECT_TRIES,
        show_window=False,
    )


def _center_dist(item):
    x1, y1, x2, y2 = item.box
    width, height = item.image_wh
    cx = (float(x1) + float(x2)) / 2.0 - float(width) / 2.0
    cy = (float(y1) + float(y2)) / 2.0 - float(height) / 2.0
    return cx * cx + cy * cy


def _view_decision(found):
    usable = [item for item in found if item.box and item.image_wh and item.has_pose]
    if not usable:
        return None
    return min(usable, key=_center_dist)


def detect_at_pose(kinova, detector, detect_pose, tag):
    print("到{} y={:.3f} z={:.3f}".format(tag, detect_pose[1], detect_pose[2]))
    arm_mdeg(kinova, detect_pose, tag=tag)
    found = _detect_items(detector)
    if not found:
        print("{} 未识别到列表中的物品".format(tag))
    return found, list(detect_pose)


def detect_on_layer(kinova, detector, layer_key):
    cfg = get_layer_cfg(layer_key)
    return detect_at_pose(kinova, detector, list(cfg["detect_pose"]), cfg["name"])


def photo_detect_layer(kinova, layer_key, save_dir=None):
    if save_dir is None:
        save_dir = Path(__file__).resolve().parent / "detect_vis"
    save_dir = Path(save_dir)
    save_dir.mkdir(parents=True, exist_ok=True)

    cfg = get_layer_cfg(layer_key)
    detect_pose = list(cfg["detect_pose"])
    arm_mdeg(kinova, detect_pose, tag=cfg["name"])
    time.sleep(0.5)

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    try:
        for _ in range(30):
            pipeline.wait_for_frames()
        frames = pipeline.wait_for_frames()
        color = frames.get_color_frame()
        img = np.asanyarray(color.get_data())
    finally:
        pipeline.stop()

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = save_dir / "{}_{}.jpg".format(layer_key, stamp)
    cv2.imwrite(str(path), img)
    print("已保存 {}".format(path))
    return str(path)


def _box_cx_ratio(item):
    x1, _, x2, _ = item.box
    width, _ = item.image_wh
    if not width:
        return 0.5
    return ((float(x1) + float(x2)) / 2.0) / float(width)


def box_hw_ratio(item):
    x1, y1, x2, y2 = item.box
    w = max(1.0, float(x2) - float(x1))
    h = max(1.0, float(y2) - float(y1))
    return h / w


def need_box_side_drop(item):
    if item.name not in BOX_SIDE["names"]:
        return False
    if item.box is None:
        return False
    ratio = box_hw_ratio(item)
    print("药盒 {} 框高宽比={:.2f} 阈值={}".format(
        item.name, ratio, BOX_SIDE["min_hw"]
    ))
    return ratio >= float(BOX_SIDE["min_hw"])


def need_left_waypoint(item, layer_key):
    if layer_key not in LEFT_WAYPOINT["layers"]:
        return False
    if item.box is None or item.image_wh is None:
        return False
    return _box_cx_ratio(item) < float(LEFT_WAYPOINT["image_ratio"])


def xyz_dist(a, b):
    dx = float(a[0]) - float(b[0])
    dy = float(a[1]) - float(b[1])
    dz = float(a[2]) - float(b[2])
    return (dx * dx + dy * dy + dz * dz) ** 0.5


def wait_until_near(kinova, pose, dist_m, sleep_s, max_wait):
    """距目标超过 dist_m 就再等 sleep_s，最多 max_wait 次，然后继续下一条。"""
    n = 0
    while True:
        kinova.getcurrentCartesianCommand()
        d = xyz_dist(kinova.currentCartesianCommand, pose)
        print("距中间点 {:.3f} m".format(d))
        if d <= float(dist_m):
            return True
        n += 1
        if n > int(max_wait):
            print("中间点等待超过 {} 次，继续下一条".format(max_wait))
            return False
        time.sleep(float(sleep_s))


def return_home(kinova, from_pose):
    retreat = list(from_pose)
    retreat[0] = (float(from_pose[0]) + HOME_POSE[0]) / 2.0
    retreat[1] = (float(from_pose[1]) + HOME_POSE[1]) / 2.0
    print("收回 home：先移动 XY")
    arm_mdeg(kinova, retreat, tag="收回xy")
    print("收回 home：再移动 Z")
    arm_mdeg(kinova, HOME_POSE, tag="收回z")


def grasp_detected(kinova, result, detect_pose, layer_key):
    """按物品思路：柱体直接抓；碗/盘子可先转腕，再斜上方，再下落合爪。"""
    cfg = grasp_profile_for(result.name)
    layer_cfg = get_layer_cfg(layer_key)
    grasp_delta = layer_cfg["grasp_delta"]
    lift_dz = float(layer_cfg.get("lift_dz") or 0.03)
    print("抓取思路={} 物品={} 层={}".format(
        GRASP_PROFILE_BY_NAME.get(result.name, "cylinder"), result.name, layer_key
    ))
    grasp_pose = build_grasp_pose_from_camera(
        result, detect_pose, grasp_delta, euler=cfg["euler"]
    )
    grasp_pose[0] += float(cfg.get("grasp_dx") or 0.0)
    grasp_pose[1] += float(cfg.get("grasp_dy") or 0.0)
    grasp_pose[2] += float(cfg.get("grasp_dz") or 0.0)
    if need_box_side_drop(result):
        grasp_pose[2] += float(BOX_SIDE.get("grasp_dz") or 0.0)
        print("药盒侧面，抓取Z再加 {}".format(BOX_SIDE.get("grasp_dz")))
    lift_pose = list(grasp_pose)
    lift_pose[2] = grasp_pose[2] + lift_dz
    print("类型补偿 = ({}, {}, {})".format(
        cfg.get("grasp_dx") or 0.0,
        cfg.get("grasp_dy") or 0.0,
        cfg.get("grasp_dz") or 0.0,
    ))
    print("抓取位姿 = {}".format(grasp_pose))
    print("抬起高度 lift_dz={}".format(lift_dz))

    print("开爪")
    kinova.finger_run(finger_target=FINGER_OPEN)
    time.sleep(0.3)

    if need_left_waypoint(result, layer_key):
        mid = list(grasp_pose)
        mid[0] = float(grasp_pose[0]) + float(LEFT_WAYPOINT.get("dx") or 0.0)
        mid[1] = float(grasp_pose[1]) + float(LEFT_WAYPOINT.get("dy") or 0.0)
        mid[2] = float(grasp_pose[2]) + float(LEFT_WAYPOINT.get("dz") or 0.0)
        print(
            "画面偏左 cx={:.2f} 层={}，先到中间点".format(
                _box_cx_ratio(result), layer_key
            )
        )
        arm_mdeg(kinova, mid, tag="中间点")
        wait_until_near(
            kinova,
            mid,
            LEFT_WAYPOINT["arrive_m"],
            LEFT_WAYPOINT["sleep_s"],
            LEFT_WAYPOINT["max_wait"],
        )

    if cfg.get("use_hover"):
        hover = list(grasp_pose)
        hover[0] = float(grasp_pose[0]) + float(cfg.get("hover_dx") or 0.0)
        hover[2] = float(grasp_pose[2]) + float(cfg.get("hover_dz") or 0.0)
        if cfg.get("rotate_first") and cfg.get("euler") is not None:
            rotate = list(hover)
            rotate[3:] = grasp_pose[3:]
            print("转腕")
            arm_mdeg(kinova, rotate, tag="转腕")
            time.sleep(0.4)
        print("斜上方")
        arm_mdeg(kinova, hover, tag="斜上方")
        time.sleep(0.4)

    print("前伸")
    arm_mdeg(kinova, grasp_pose, tag="前伸")
    time.sleep(0.4)
    print("合爪")
    kinova.finger_run(finger_target=FINGER_CLOSE)
    kinova.finger_run(finger_target=FINGER_TIGHT)
    time.sleep(0.4)
    print("抬起")
    arm_mdeg(kinova, lift_pose, tag="抬起")
    time.sleep(0.2)
    return_home(kinova, lift_pose)
    return True


def pick_on_shelf(kinova, target_name, hint_layer):
    pending = search_order(hint_layer)
    print("初步层={} 尝试顺序={}".format(hint_layer, pending))
    weights = _first_existing(MODEL_CANDIDATES)
    if target_name:
        print("腕部相机优先目标={}，更靠近中心的其他物品也会抓。模型={}".format(target_name, weights))
    else:
        print("未指定目标，离画面中心最近且在中间的物品即抓取。模型={}".format(weights))
    detector = RealSenseYolo11Detector(weights=Path(weights))

    seen = set()
    last_pose = None
    while pending:
        layer_key = pending.pop(0)
        if layer_key in seen:
            continue
        seen.add(layer_key)
        found, detect_pose = detect_on_layer(kinova, detector, layer_key)
        last_pose = detect_pose
        chosen = _view_decision(found)
        if chosen is not None:
            if target_name and chosen.name != target_name:
                print("更靠近中心的是 {}，按它抓取".format(chosen.name))
            else:
                print("抓取 {}".format(chosen.name))
            print(
                "检测到 {} 相机=({:.3f},{:.3f},{:.3f})".format(
                    chosen.name, chosen.x, chosen.y, chosen.z
                )
            )
            return grasp_detected(kinova, chosen, detect_pose, layer_key)

    print("没有可抓的中心物体")
    return_home(kinova, last_pose or HOME_POSE)
    return False


def main():
    parser = argparse.ArgumentParser(description="货架分类抓取")
    parser.add_argument(
        "--layer",
        type=str,
        default=DEFAULT_LAYER,
        choices=list(SHELF_LAYERS.keys()),
    )
    parser.add_argument("--target", type=str, default=DEFAULT_TARGET)
    parser.add_argument("--photo", action="store_true")
    opt = parser.parse_args()

    rospy.init_node("task2_pick_v4", anonymous=True)
    kinova = KinovaRobot("j2n6s300")
    if not hasattr(kinova, "client_arm") or kinova.client_arm is None:
        print("机械臂未连接，请先启动 kinova_bringup")
        return

    if opt.photo:
        print("拍照层={}".format(opt.layer))
        photo_detect_layer(kinova, opt.layer)
        return

    ok = pick_on_shelf(kinova, opt.target, opt.layer)
    if ok:
        print("完成")
    else:
        print("失败")


if __name__ == "__main__":
    main()
