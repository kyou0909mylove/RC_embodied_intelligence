# -*- coding: utf-8 -*-
"""头部Kinect与配置模型检测。相机/推理独立进程，主进程按截止限制等待。"""
# 当前只把彩色图像中的像素框、标签和置信度交给主流程；没有生成机械臂抓取坐标。
# 补深度/标定时，可从 _worker 的 detection 构造处扩展观测，但须保留已有字段契约。
# Vision 是主进程接口；_worker 是子进程实现，两者只通过 Pipe 传递请求/结果。
import multiprocessing
from datetime import datetime, timezone
from pathlib import Path
import time
from .selection import stable_candidates
from core.safety import OperationStopped


class CameraFailed(RuntimeError):
    """采图/推理进程异常；与没有检测目标不同，可有限重启设备进程。"""
    def __init__(self, message, phase='software'):
        self.phase = phase
        self.device_related = phase in ('device','capture','connection')
        super().__init__(message)


def save_key_image(writer, path, image):
    """关键图失败只返回警告；不能把磁盘/编码问题当成相机或识别失败。"""
    try:
        if writer(path,image):
            return path,None
        return None,'关键图保存失败：'+str(path)
    except Exception as exc:
        return None,'关键图保存失败：{}: {}'.format(type(exc).__name__,exc)


def _worker(connection, model_config, model_path):
    """子进程只操作相机和模型，不导入 ROS 或发送机器人动作。"""
    device = None
    phase = 'model'
    try:
        import cv2
        import torch
        import pykinect_azure as kinect
        from ultralytics import YOLO
        torch.set_num_threads(2)
        model_config = dict(model_config)
        if model_config["device"] == "auto":
            model_config["device"] = "0" if torch.cuda.is_available() else "cpu"
        model = YOLO(model_path)
        names = model.names
        ordered = [names[i] for i in sorted(names)] if isinstance(names, dict) else list(names)
        model_config['labels'] = ordered
        phase = 'device'
        kinect.initialize_libraries()
        camera_config = kinect.default_configuration
        camera_config.color_format = kinect.K4A_IMAGE_FORMAT_COLOR_MJPG
        camera_config.color_resolution = kinect.K4A_COLOR_RESOLUTION_1080P
        camera_config.depth_mode = kinect.K4A_DEPTH_MODE_WFOV_2X2BINNED
        camera_config.camera_fps = kinect.K4A_FRAMES_PER_SECOND_30
        camera_config.wired_sync_mode = kinect.K4A_WIRED_SYNC_MODE_STANDALONE
        camera_config.synchronized_images_only = True
        device = kinect.start_device(config=camera_config)
        # 相机打开、模型加载后即完成初始化；采图和推理在实际扫描时进行。
        connection.send({"kind": "ready", "labels": ordered})
        while True:
            phase = 'software'
            request = connection.recv()
            if request["kind"] == "close":
                break
            if request["kind"] != "scan":
                raise RuntimeError("未知视觉请求")
            detections, valid_frames, attempts, image_path = [], 0, 0, None
            image_warning = None
            best, best_image = None, None
            frames = []
            selection = request.get("selection", {"min_hits": 1, "iou_threshold": 0.3, "depth_tolerance_m": 0.02})
            target_labels = request["labels"]
            while valid_frames < model_config["frames"] and attempts < model_config["frames"] * 3:
                if time.monotonic() >= request["deadline"]:
                    break
                attempts += 1
                phase = 'capture'
                connection.send({'kind':'phase','phase':'capture'})
                ok, image = device.update().get_color_image()
                if not ok or image is None:
                    continue
                capture_wall_utc = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
                phase = 'inference'
                connection.send({'kind':'phase','phase':'inference'})
                result = model.predict(source=image[:, :, :3], device=model_config["device"],
                                       imgsz=model_config["imgsz"], conf=model_config["confidence"], verbose=False)[0]
                phase = 'software'
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
                                         "frame": valid_frames, "capture_wall_utc": capture_wall_utc,
                                         "image_wh": [image.shape[1], image.shape[0]]}
                            detections.append(detection)
                            frame_detections.append(detection)
                frames.append(frame_detections)
                candidates = stable_candidates(frames, selection)
                # 每次只保留最新帧仍稳定存在的实例，不能沿用之前已消失物品的旧框。
                best = candidates[0] if candidates else None
                best_image = None
                if best is not None and request["image_path"] is not None:
                    best_image = result.plot()
                    cv2.rectangle(best_image, (8, 6), (630, 44), (0, 0, 0), -1)
                    cv2.putText(best_image, "UTC " + capture_wall_utc, (16, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
            if valid_frames == 0:
                phase = 'capture'
                raise RuntimeError("扫描期间未取得有效图像；这与未识别到物品不同")
            if best_image is not None:
                image_path,image_warning = save_key_image(cv2.imwrite,request['image_path'],best_image)
            if best is not None:
                best["image_path"] = image_path
            connection.send({"kind": "scan_result", "point_id": request["point_id"],
                             "valid_frames": valid_frames, "detections": detections,
                             "target": best, "candidates": candidates, "image_path": image_path,
                             "image_save_warning":image_warning})
    except (EOFError, BrokenPipeError):
        pass
    except Exception as exc:
        try:
            connection.send({"kind": "error", "phase": phase, "message": type(exc).__name__ + ": " + str(exc)})
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
        self.sequence = 0
        self.worker_phase = 'software'

    def _receive(self, gate):
        # SIGALRM也可能在Pipe.poll/recv内部抛出，而不只在gate.check抛出。
        # 在整个接收操作外记录阶段，SDK无响应才能进入有限重连诊断。
        try:
            return self._receive_message(gate)
        except OperationStopped as exc:
            exc.camera_phase = self.worker_phase
            exc.camera_source = type(self).__name__
            raise

    def _receive_message(self, gate):
        # 小段轮询使主流程能持续检查停止和截止；相机错误不会伪装成“没有物品”。
        while True:
            gate.check()
            try:
                if self.connection.poll(0.05):
                    message = self.connection.recv()
                else:
                    message = None
            except OperationStopped:
                raise  # TimeoutError继承OSError；截止不是相机连接断开。
            except (OSError, EOFError) as exc:
                raise CameraFailed("视觉进程连接断开",phase="connection") from exc
            if message is not None:
                gate.check()
                if not isinstance(message, dict) or 'kind' not in message:
                    raise CameraFailed('视觉进程返回无效消息')
                if message['kind']=='phase':
                    self.worker_phase = message.get('phase','software')
                    continue
                if message["kind"] == "error":
                    raise CameraFailed("视觉设备/模型失败："+str(message.get("message",'未知错误')),phase=message.get('phase','software'))
                return message
            if not self.process.is_alive():
                raise CameraFailed("视觉进程退出，无法完成真实检测",phase="connection")

    def _send(self, request):
        # Pipe 可能在 send 时就断开，须与 recv 一样进入有限相机恢复流程。
        try:
            self.connection.send(request)
        except OperationStopped:
            raise
        except (OSError, EOFError) as exc:
            raise CameraFailed('视觉请求发送失败，设备进程连接断开',phase='connection') from exc

    def _record_image_warning(self, result):
        warning = result.get('image_save_warning')
        if warning:
            self.evidence.record['recording_warnings'].append(warning)

    def start(self, gate):
        # spawn 避免复制已建立的 ROS 连接；收到 ready 后才允许主流程继续准备。
        context = multiprocessing.get_context("spawn")
        self.connection, child = context.Pipe()
        self.process = context.Process(target=_worker,
                                       args=(child, self.config.values["model"], str(self.config.model_path)), daemon=True)
        self.process.start()
        child.close()
        message = self._receive(gate)
        if message["kind"] != "ready":
            raise CameraFailed("视觉初始化结果无效")
        self.config.labels = list(message['labels'])
        self.config.values['model']['labels'] = list(message['labels'])
        self.evidence.event("vision_ready", labels=message["labels"])

    def scan(self, point, gate):
        # 一次扫描只保存最终选择的候选图；target_labels 为空时使用模型全部类别。
        gate.check()
        self.sequence += 1
        image_path = (self.evidence.run_dir / "images" / ("head-{}-{:03d}.jpg".format(point["id"], self.sequence))
                      if self.evidence.save_images else None)
        self.busy = True
        self._send({"kind": "scan", "point_id": point["id"],
                              "labels": [s for s in (point.get("target_labels", []) or self.config.labels)
                                         if s not in self.config.values["excluded_labels"]], "deadline": gate.deadline,
                              "selection": self.config.values["perception"],
                              "image_path": str(image_path) if image_path is not None else None})
        message = self._receive(gate)
        self.busy = False
        if message["kind"] != "scan_result" or message["point_id"] != point["id"]:
            raise CameraFailed("视觉扫描结果与当前检测点不一致")
        self._record_image_warning(message)
        return message

    def restart(self, gate):
        self.close()
        self.process = self.connection = None
        self.busy = False
        self.start(gate)
        self.evidence.event("camera_process_restarted", camera=type(self).__name__)

    def close(self):
        """阻塞采图/推理不拖延离场；空闲时优先让 SDK 正常关闭。"""
        if self.process is not None:
            if self.process.is_alive() and not self.busy:
                try:
                    self.connection.send({"kind": "close"})
                except OperationStopped:
                    raise
                except (OSError, EOFError):
                    pass
                self.process.join(self.config.values['camera']['process_join_timeout_s'])
            if self.process.is_alive():
                self.process.terminate()
                self.process.join(self.config.values['camera']['process_join_timeout_s'])
            if self.process.is_alive():
                self.process.kill()
                self.process.join(self.config.values['camera']['process_join_timeout_s'])
        if self.connection is not None:
            self.connection.close()
