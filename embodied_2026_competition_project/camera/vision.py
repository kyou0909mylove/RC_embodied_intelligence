# -*- coding: utf-8 -*-
"""真实 Kinect + best.pt 检测。相机/推理独立进程，主进程始终能转入离场。"""
import multiprocessing
from datetime import datetime, timezone
from pathlib import Path
import time


def _worker(connection, model_config, model_path):
    """子进程只操作相机和模型，不导入 ROS 或发送机器人动作。"""
    device = None
    try:
        import cv2
        import torch
        import pykinect_azure as kinect
        from ultralytics import YOLO
        torch.set_num_threads(2)
        model = YOLO(model_path)
        names = model.names
        ordered = [names[i] for i in sorted(names)] if isinstance(names, dict) else list(names)
        if ordered != model_config["labels"]:
            raise RuntimeError("实际加载的模型标签与配置不一致：" + str(ordered))
        kinect.initialize_libraries()
        camera_config = kinect.default_configuration
        camera_config.color_format = kinect.K4A_IMAGE_FORMAT_COLOR_MJPG
        camera_config.color_resolution = kinect.K4A_COLOR_RESOLUTION_1080P
        camera_config.depth_mode = kinect.K4A_DEPTH_MODE_WFOV_2X2BINNED
        camera_config.camera_fps = kinect.K4A_FRAMES_PER_SECOND_30
        camera_config.wired_sync_mode = kinect.K4A_WIRED_SYNC_MODE_STANDALONE
        camera_config.synchronized_images_only = True
        device = kinect.start_device(config=camera_config)
        # 准备阶段验证采图、模型执行和版本兼容；此时还没有发送导航目标。
        ok, image = device.update().get_color_image()
        if not ok or image is None:
            raise RuntimeError("Kinect 启动后未取得有效彩色图像")
        model.predict(source=image[:, :, :3], device=model_config["device"],
                      imgsz=model_config["imgsz"], conf=model_config["confidence"], verbose=False)
        connection.send({"kind": "ready", "labels": ordered})
        while True:
            request = connection.recv()
            if request["kind"] == "close":
                break
            if request["kind"] != "scan":
                raise RuntimeError("未知视觉请求")
            detections, valid_frames, attempts, image_path = [], 0, 0, None
            best, best_image = None, None
            target_labels = request["labels"] or ordered
            while valid_frames < model_config["frames"] and attempts < model_config["frames"] * 3:
                if time.monotonic() >= request["deadline"]:
                    break
                attempts += 1
                ok, image = device.update().get_color_image()
                if not ok or image is None:
                    continue
                capture_wall_utc = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
                result = model.predict(source=image[:, :, :3], device=model_config["device"],
                                       imgsz=model_config["imgsz"], conf=model_config["confidence"], verbose=False)[0]
                valid_frames += 1
                frame_detections = []
                if result.boxes is not None:
                    for box in result.boxes:
                        class_id, confidence = int(box.cls.item()), float(box.conf.item())
                        label = result.names[class_id]
                        if label in target_labels:
                            detection = {"class_id": class_id, "label": label,
                                         "confidence": confidence,
                                         "box_xyxy": [float(x) for x in box.xyxy[0].cpu().tolist()],
                                         "frame": valid_frames, "capture_wall_utc": capture_wall_utc}
                            detections.append(detection)
                            frame_detections.append(detection)
                frame_best = max(frame_detections, key=lambda x: x["confidence"]) if frame_detections else None
                if frame_best is not None and (best is None or frame_best["confidence"] > best["confidence"]):
                    best = frame_best
                    if request["image_path"] is not None:
                        # 复用已有 result.plot/cv2，不保存末帧代替真正选中的候选帧。
                        # 英文精确模型标签与物品框由 plot 绘制，中文播报名另存流程记录。
                        best_image = result.plot()
                        cv2.rectangle(best_image, (8, 6), (630, 44), (0, 0, 0), -1)
                        cv2.putText(best_image, "UTC " + capture_wall_utc, (16, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
            if valid_frames == 0:
                raise RuntimeError("扫描期间未取得有效图像；这与未识别到物品不同")
            if best_image is not None:
                image_path = request["image_path"]
                if not cv2.imwrite(image_path, best_image):
                    raise RuntimeError("无法保存检测图像：" + image_path)
            if best is not None:
                best["image_path"] = image_path
            connection.send({"kind": "scan_result", "point_id": request["point_id"],
                             "valid_frames": valid_frames, "detections": detections,
                             "target": best, "image_path": image_path})
    except (EOFError, BrokenPipeError):
        pass
    except Exception as exc:
        try:
            connection.send({"kind": "error", "message": type(exc).__name__ + ": " + str(exc)})
        except (OSError, EOFError):
            pass
    finally:
        if device is not None:
            try:
                device.stop_cameras()
                device.close()
            except Exception:
                pass
        connection.close()


class Vision:
    def __init__(self, config, evidence):
        self.config, self.evidence = config, evidence
        self.process, self.connection, self.busy = None, None, False

    def _receive(self, gate):
        while True:
            gate.check()
            if self.connection.poll(0.05):
                message = self.connection.recv()
                gate.check()
                if message["kind"] == "error":
                    raise RuntimeError("视觉设备/模型失败：" + message["message"])
                return message
            if not self.process.is_alive():
                raise RuntimeError("视觉进程退出，无法完成真实检测")

    def start(self, gate):
        context = multiprocessing.get_context("spawn")
        self.connection, child = context.Pipe()
        self.process = context.Process(target=_worker,
                                       args=(child, self.config.values["model"], str(self.config.model_path)), daemon=True)
        self.process.start()
        child.close()
        message = self._receive(gate)
        if message["kind"] != "ready":
            raise RuntimeError("视觉初始化结果无效")
        self.evidence.event("vision_ready", labels=message["labels"])

    def scan(self, point, gate):
        gate.check()
        image_path = (self.evidence.run_dir / "images" / (point["id"] + ".jpg")
                      if self.evidence.save_images else None)
        self.busy = True
        self.connection.send({"kind": "scan", "point_id": point["id"],
                              "labels": point.get("target_labels", []), "deadline": gate.deadline,
                              "image_path": str(image_path) if image_path is not None else None})
        message = self._receive(gate)
        self.busy = False
        if message["kind"] != "scan_result" or message["point_id"] != point["id"]:
            raise RuntimeError("视觉扫描结果与当前检测点不一致")
        return message

    def close(self):
        """阻塞采图/推理不拖延离场；空闲时优先让 SDK 正常关闭。"""
        if self.process is not None:
            if self.process.is_alive() and not self.busy:
                try:
                    self.connection.send({"kind": "close"})
                except (OSError, EOFError):
                    pass
                self.process.join(0.2)
            if self.process.is_alive():
                self.process.terminate()
                self.process.join(0.2)
            if self.process.is_alive():
                self.process.kill()
                self.process.join(0.2)
        if self.connection is not None:
            self.connection.close()
