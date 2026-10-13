#!/usr/bin/env python
# coding: UTF-8
"""
识别模块（参考版）：YOLOv11 + RealSense
=====================================================

忠实复刻学姐的识别逻辑，只做「识别」这一件事，不碰机械臂 / 手眼 / 抓取流程。

核心三点（防止一个物品被识别成多个物体，靠的是 NMS）：
  1) 白平衡预热：开自动白平衡 + 丢前 30 帧，保证颜色/特征稳定；
  2) NMS 去重：推理时传 iou=0.45，重叠框(同一个物体)合并成一个；
  3) 置信度过滤 + 框内最近点取深度：conf 过滤弱框，取框内离相机最近的表面得相机坐标。

用法（pick 脚本里只改 import 一行，其余不变）：
    from realsense_yolo11_ref import RealSenseYolo11Detector
    detector = RealSenseYolo11Detector(weights=Path("best.pt"))
    result = detector.detect_targets(target_items=["pink_cube"], show_window=False)

    # 一帧里列出所有目标（带像素框，用于判断边框/多物体）：
    items = detector.detect_all_targets(target_items=["..."], show_window=False)

注意：detect_targets / detect_all_targets 每次都会 enable_stream 并在结束 stop，
     同一实例连续多次调用前，先 detector._fresh_pipeline() 换一套管道。
"""

from __future__ import print_function

import time
from pathlib import Path
from typing import Union


_MODEL_CACHE = {}


def get_cached_model(weights, device=None):
    """同一权重和设备只加载一次；图像、深度、坐标及相机管道不缓存。"""
    device = device or ('cuda' if torch.cuda.is_available() else 'cpu')
    key = (str(Path(weights).expanduser().resolve()), device)
    if key not in _MODEL_CACHE:
        started = time.monotonic()
        model = YOLO(key[0]).to(device)
        _MODEL_CACHE[key] = model
        print('[腕部模型] 已加载 {}，耗时 {:.3f}s'.format(key[0],time.monotonic()-started))
    return _MODEL_CACHE[key]

import numpy as np
import cv2
import torch
import pyrealsense2 as rs
from ultralytics import YOLO


class YoloResult:
    """识别结果：物品名、相机坐标、角度、像素框、置信度。"""
    def __init__(self, name, x, y, z, angle, box=None, conf=None, image_wh=None):
        self.name = name
        self.x = x              # 相机坐标系 X（右）
        self.y = y              # 相机坐标系 Y（下）
        self.z = z              # 相机坐标系 Z（前/距离）
        self.angle = angle      # 物品角度
        self.box = box          # (x1, y1, x2, y2) 像素
        self.conf = conf        # 置信度
        self.image_wh = image_wh  # (宽, 高)
        self.has_pose = x is not None and y is not None and z is not None

    def __str__(self):
        return "物品:{} | 坐标(X,Y,Z):({:.3f},{:.3f},{:.3f}) | 角度:{:.1f}°".format(
            self.name,
            self.x if self.x is not None else 0.0,
            self.y if self.y is not None else 0.0,
            self.z if self.z is not None else 0.0,
            self.angle,
        )


class RealSenseYolo11Detector:
    # 竖直瓶类：取最近点时高度会偏向瓶口，抓取高度固定到框下方 1/3 处（框顶往下 2/3）
    BOTTLE_NAMES = {
        "sprite_bottle", "mirinda_bottle", "water_bottle",
        "green_shampoo_bottle", "white_pump_bottle", "whitecat_detergent",
    }
    BOTTLE_GRAB_HEIGHT_FRAC = 2.0 / 3.0   # 抓取点高度比例（从框顶算；2/3 = 下方 1/3 处）

    def __init__(self, weights: Path = Path("weights"),
                 imgsz: int = 640, conf_thres: float = 0.2, iou_thres: float = 0.45):
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model = get_cached_model(weights,self.device)

        # 调试：打印模型类别
        print("=== 调试:model.names 信息 ===")
        print("类型：{}".format(type(self.model.names)))
        print("内容：{}".format(self.model.names))
        print("=============================")

        self.imgsz = imgsz
        self.conf_thres = conf_thres      # 置信度门槛：过滤弱框
        self.iou_thres = iou_thres        # NMS IoU 门槛：同一物体重叠框合并

        # RealSense
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.align = rs.align(rs.stream.color)  # 深度对齐彩色

    # ------------------------------------------------------------------
    def _fresh_pipeline(self):
        """detect 系列每次都会 enable_stream 并在结束 stop，重复调用前换一套管道。"""
        self.pipeline = rs.pipeline()
        self.config = rs.config()

    def _calc_item_angle(self, item_img: np.ndarray) -> float:
        """物品最小外接矩形角度。"""
        if item_img.size == 0:
            return 0.0
        gray = cv2.cvtColor(item_img, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid = [c for c in contours if cv2.contourArea(c) > 100]
        if not valid:
            return 0.0
        rect = cv2.minAreaRect(np.vstack(valid))
        angle = rect[2]
        if rect[1][0] < rect[1][1]:
            angle += 90
        return round(90 - angle, 1)

    def _get_realsense_data(self):
        """对齐后的彩色图、深度帧、深度内参。"""
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not depth_frame or not color_frame:
            return None, None, None
        color_img = np.asanyarray(color_frame.get_data())
        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        return color_img, depth_frame, depth_intrin

    def _closest_depth_in_box(self, depth_frame, x1, y1, x2, y2):
        """取框内离相机最近的表面（鲁棒 min），返回该点 (cx, cy, depth) 米。

        抓取面通常是离相机最近的表面（碗沿、瓶身前面、盒面），比框中心更能代表
        实际抓取点；用 5% 分位数避免单个异常近的噪点拉偏。全无效时退回框中心。
        """
        x1 = max(0, int(x1))
        y1 = max(0, int(y1))
        x2 = max(x1 + 1, int(x2))
        y2 = max(y1 + 1, int(y2))
        depth_img = np.asanyarray(depth_frame.get_data())  # uint16，毫米
        H, W = depth_img.shape
        x2 = min(W, x2)
        y2 = min(H, y2)

        region = depth_img[y1:y2, x1:x2]
        valid_mask = region > 100  # 只保留 >0.1m 的有效深度，排除噪点
        valid = region[valid_mask]
        if valid.size == 0:
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            return cx, cy, depth_frame.get_distance(cx, cy)

        depth_mm = float(np.percentile(valid, 5))  # 最近表面的鲁棒深度
        depth = depth_mm / 1000.0

        near_mask = (region > 100) & (region <= depth_mm)
        ys, xs = np.nonzero(near_mask)
        cx = x1 + int(round(float(xs.mean())))
        cy = y1 + int(round(float(ys.mean())))
        return cx, cy, depth

    def _fix_bottle_grasp_height(self, cls_name, cy, y1, y2):
        """瓶类：高度固定到框下方 1/3 处（框顶往下 BOTTLE_GRAB_HEIGHT_FRAC）。

        竖直瓶取最近点时深度会落在瓶口，把抓取点抬得太高；
        这里只钳制高度 cy，深度仍用最近点（保证抓瓶身前面）。
        """
        if cls_name in self.BOTTLE_NAMES:
            return int(round(y1 + (y2 - y1) * self.BOTTLE_GRAB_HEIGHT_FRAC))
        return cy

    def _white_balance_warmup(self):
        """开自动白平衡 + 丢前 30 帧，让颜色/曝光稳定。"""
        color_sensor = self.pipeline.get_active_profile().get_device().first_color_sensor()
        color_sensor.set_option(rs.option.enable_auto_white_balance, 1)
        print("白平衡预热，丢掉前30帧")
        for _ in range(30):
            self.pipeline.wait_for_frames()

    # ------------------------------------------------------------------
    def detect_targets(self, target_items: list, max_retry: int = 5,
                       show_window: bool = True) -> Union[YoloResult, None]:
        """识别指定物品集，返回离相机最近（深度最小）的那个；多次重试失败返回 None。

        NMS 去重：模型推理时传 iou=self.iou_thres，同一物品的重叠框已被合并。
        """
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(self.config)

        retry_count = 0
        try:
            self._white_balance_warmup()

            while retry_count < max_retry:
                retry_count += 1
                print("第{}次识别...".format(retry_count))

                color_img, depth_frame, depth_intrin = self._get_realsense_data()
                if color_img is None:
                    time.sleep(0.5)
                    continue

                display_img = color_img.copy()

                # 目标类别名 -> 索引
                name_to_idx = {name: idx for idx, name in self.model.names.items()}
                target_idxs = [name_to_idx[item] for item in target_items if item in name_to_idx]
                # 推理：iou 即 NMS 阈值，合并同一物品的重叠框
                results = self.model(
                    color_img, imgsz=self.imgsz, conf=self.conf_thres,
                    iou=self.iou_thres,
                    classes=target_idxs if target_idxs else None,
                    device=self.device, verbose=False,
                )

                found_target = None
                for result in results:
                    boxes = result.boxes.cpu().numpy()
                    for box in boxes:
                        xyxy = box.xyxy[0].tolist()
                        cls_name = self.model.names[int(box.cls[0])]
                        conf = box.conf[0]
                        print("检测到物品：{} | 置信度：{:.2f} | 边界框：{}".format(
                            cls_name, conf, [round(v) for v in xyxy]))

                        x1, y1, x2, y2 = map(int, xyxy)
                        cv2.rectangle(display_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(display_img, "{} {:.2f}".format(cls_name, conf),
                                    (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                        if cls_name not in target_items or conf < self.conf_thres:
                            continue

                        # 物品中心
                        if cls_name in ["Handwash", "Shampoo"]:
                            center_x = int((x2 - x1) / 2 + x1)
                            center_y = int((y2 - y1) * 2 / 3 + y1)
                        else:
                            center_x = int((x1 + x2) / 2)
                            center_y = int((y1 + y2) / 2)
                        cv2.circle(display_img, (center_x, center_y), 5, [0, 0, 255], -1)

                        # 框内最近点取深度（抓取面通常是离相机最近的表面）
                        cx, cy, depth = self._closest_depth_in_box(depth_frame, x1, y1, x2, y2)
                        cy = self._fix_bottle_grasp_height(cls_name, cy, y1, y2)
                        if depth < 0.1 or depth > 5.0:
                            continue
                        cam_x, cam_y, cam_z = rs.rs2_deproject_pixel_to_point(
                            depth_intrin, [cx, cy], depth)

                        item_img = color_img[y1:y2, x1:x2]
                        item_angle = self._calc_item_angle(item_img)

                        candidate = YoloResult(cls_name, cam_x, cam_y, cam_z, item_angle,
                                               box=(x1, y1, x2, y2), conf=float(conf),
                                               image_wh=(color_img.shape[1], color_img.shape[0]))
                        # 同一帧多个目标时，保留离相机最近（深度最小）的那个
                        if found_target is None or candidate.z < found_target.z:
                            found_target = candidate

                if show_window:
                    cv2.putText(display_img, "Retry: {}/{}".format(retry_count, max_retry),
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, [0, 0, 255], 2)
                    cv2.imshow("YOLO Detection (Press 'q' to quit)", display_img)
                    key = cv2.waitKey(1)
                    if key == ord('q'):
                        return None

                if found_target:
                    return found_target
                time.sleep(0.8)

            print("已重试{}次，未识别到指定物品".format(max_retry))
            return None
        finally:
            self.pipeline.stop()
            if show_window:
                cv2.destroyAllWindows()

    # ------------------------------------------------------------------
    def detect_all_targets(self, target_items: list, max_retry: int = 5,
                           show_window: bool = False):
        """同一帧里列出所有指定物品（带像素框），没有则返回空列表。

        用于判断边框物体 / 多物体同时出现；NMS 已保证一个物品只有一个框。
        """
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(self.config)

        retry_count = 0
        try:
            self._white_balance_warmup()

            while retry_count < max_retry:
                retry_count += 1
                print("第{}次识别...".format(retry_count))

                color_img, depth_frame, depth_intrin = self._get_realsense_data()
                if color_img is None:
                    time.sleep(0.5)
                    continue

                height, width = color_img.shape[:2]
                name_to_idx = {name: idx for idx, name in self.model.names.items()}
                target_idxs = [name_to_idx[item] for item in target_items if item in name_to_idx]
                results = self.model(
                    color_img, imgsz=self.imgsz, conf=self.conf_thres,
                    iou=self.iou_thres,
                    classes=target_idxs if target_idxs else None,
                    device=self.device, verbose=False,
                )

                found = []
                for result in results:
                    boxes = result.boxes.cpu().numpy()
                    for box in boxes:
                        xyxy = box.xyxy[0].tolist()
                        cls_name = self.model.names[int(box.cls[0])]
                        conf = float(box.conf[0])
                        print("检测到物品：{} | 置信度：{:.2f} | 边界框：{}".format(
                            cls_name, conf, [round(v) for v in xyxy]))
                        if cls_name not in target_items or conf < self.conf_thres:
                            continue

                        x1, y1, x2, y2 = map(int, xyxy)
                        cx, cy, depth = self._closest_depth_in_box(depth_frame, x1, y1, x2, y2)
                        cy = self._fix_bottle_grasp_height(cls_name, cy, y1, y2)

                        cam_x = cam_y = cam_z = None
                        angle = 0.0
                        if 0.1 <= depth <= 5.0:
                            cam_x, cam_y, cam_z = rs.rs2_deproject_pixel_to_point(
                                depth_intrin, [cx, cy], depth)
                            item_img = color_img[y1:y2, x1:x2]
                            angle = self._calc_item_angle(item_img)

                        found.append(YoloResult(
                            cls_name, cam_x, cam_y, cam_z, angle,
                            box=(x1, y1, x2, y2), conf=conf, image_wh=(width, height),
                        ))

                if found:
                    return found
                time.sleep(0.8)

            print("已重试{}次，未识别到指定物品".format(max_retry))
            return []
        finally:
            self.pipeline.stop()
            if show_window:
                cv2.destroyAllWindows()


if __name__ == "__main__":
    detector = RealSenseYolo11Detector(weights=Path("best.pt"))
    result = detector.detect_targets(target_items=["bottle", "spoon", "orange"])
    if result:
        print("坐标:X={:.3f}, Y={:.3f}, Z={:.3f}".format(result.x, result.y, result.z))
        print("角度：{:.1f}°".format(result.angle))
    else:
        print("未检测到目标物品")
