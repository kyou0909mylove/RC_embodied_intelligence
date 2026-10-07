# -*- coding: utf-8 -*-
"""腕部 RealSense 独立采图进程：沿用上传工程的对齐深度和框中心取点。

不在进程里导入 ROS 或控制机械臂；阻塞 SDK/推理可由主进程关闭。
只增加多帧/深度一致性过滤，不在框内扫其他深度点以免横向跑偏。
"""
import multiprocessing
import time
from datetime import datetime, timezone
from .selection import stable_candidates
from .vision import Vision, CameraFailed, save_key_image
from .orientation import item_angle


def _worker(connection, profiles, serial, policy, device_name):
    pipeline = None
    phase = 'model'
    try:
        import cv2
        import numpy as np
        import torch
        import pyrealsense2 as rs
        from ultralytics import YOLO
        torch.set_num_threads(2)
        if device_name == "auto":
            device_name = "0" if torch.cuda.is_available() else "cpu"
        width,height = policy["width"],policy["height"]
        models = {}
        for key, profile in profiles.items():
            model = YOLO(profile['weights'])
            names = ([model.names[i] for i in sorted(model.names)]
                     if isinstance(model.names, dict) else list(model.names))
            profile['labels'] = names
            models[key] = model
        phase = 'device'
        pipeline, cfg = rs.pipeline(), rs.config()
        if serial:
            cfg.enable_device(serial)
        elif len(rs.context().query_devices()) != 1:
            raise RuntimeError("自动选择要求恰好一个RealSense；多个设备请填camera_serial")
        cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, 30)
        cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, 30)
        active = pipeline.start(cfg)
        sensor = active.get_device().first_color_sensor()
        if sensor.supports(rs.option.enable_auto_white_balance):
            sensor.set_option(rs.option.enable_auto_white_balance, 1)
        align = rs.align(rs.stream.color)
        connection.send({'kind': 'ready', 'labels':{k:p['labels'] for k,p in profiles.items()}})
        while True:
            phase = 'software'
            request = connection.recv()
            if request['kind'] == 'close':
                break
            if request['kind'] != 'scan':
                raise RuntimeError('未知腕部相机请求')
            key, labels = request['profile'], request['labels']
            profile, model = profiles[key], models[key]
            if not labels or any(label not in profile['labels'] for label in labels):
                raise RuntimeError('腕部模型没有所请求的精确类别')
            frame_list, valid, last_plot = [], 0, None
            raw_detection_count = 0
            # 移动结束后丢弃缓存旧帧，不能使用检测位变化前的深度。
            phase = 'capture'
            connection.send({'kind':'phase','phase':'capture'})
            for _ in range(policy['settle_frames']):
                if time.monotonic() >= request['deadline']:
                    break
                pipeline.wait_for_frames(max(1,int(policy['frame_timeout_s']*1000)))
            for _ in range(policy['frames']):
                if time.monotonic() >= request['deadline']:
                    break
                phase = 'capture'
                connection.send({'kind':'phase','phase':'capture'})
                frames = align.process(pipeline.wait_for_frames(max(1,int(policy['frame_timeout_s']*1000))))
                depth, color = frames.get_depth_frame(), frames.get_color_frame()
                if not depth or not color:
                    frame_list.append([])
                    continue
                image = np.asanyarray(color.get_data())
                stamp = datetime.now(timezone.utc).isoformat(timespec='milliseconds')
                capture_mono = time.monotonic()
                phase = 'inference'
                connection.send({'kind':'phase','phase':'inference'})
                prediction = model.predict(source=image, imgsz=policy['imgsz'], conf=profile['confidence'],
                                           classes=[profile['labels'].index(label) for label in labels],
                                           device=device_name, verbose=False)[0]
                phase = 'software'
                valid += 1
                items = []
                for box in prediction.boxes if prediction.boxes is not None else []:
                    label = profile['labels'][int(box.cls.item())]
                    xyxy = [float(v) for v in box.xyxy[0].cpu().tolist()]
                    x = int((xyxy[0]+xyxy[2])/2)
                    y = int((xyxy[1]+xyxy[3])/2)
                    xyz = None
                    if 0 <= x < width and 0 <= y < height:
                        distance = depth.get_distance(x, y)
                        if policy['depth_min_m'] <= distance <= policy['depth_max_m']:
                            xyz = rs.rs2_deproject_pixel_to_point(
                                depth.profile.as_video_stream_profile().intrinsics, [x, y], distance)
                    items.append({'label': label, 'confidence': float(box.conf.item()),
                                  'box_xyxy': xyxy, 'image_wh': [width, height],
                                  'camera_xyz_m': xyz, 'capture_wall_utc': stamp, 'capture_monotonic': capture_mono})
                    if key=='ground':
                        x1,y1,x2,y2 = [int(v) for v in xyxy]
                        crop = image[max(0,y1):min(height,y2),max(0,x1):min(width,x2)]
                        items[-1]['angle_deg'] = item_angle(crop)
                raw_detection_count = max(raw_detection_count, len(items))
                frame_list.append(items)
                if request['image_path']:
                    last_plot = prediction.plot()
                    cv2.putText(last_plot, 'UTC ' + stamp, (8, 25), cv2.FONT_HERSHEY_SIMPLEX,
                                0.5, (255, 255, 255), 1, cv2.LINE_AA)
            if valid == 0:
                phase = 'capture'
                raise RuntimeError('腕部相机没有有效彩色/深度帧，不能当成空家具')
            candidates = stable_candidates(frame_list, policy)
            image_path = None
            image_warning = None
            if candidates and last_plot is not None:
                image_path,image_warning = save_key_image(cv2.imwrite,request['image_path'],last_plot)
            for item in candidates:
                item['image_path'] = image_path
            connection.send({'kind': 'scan_result', 'point_id': request['point_id'],
                             'candidates': candidates, 'valid_frames': valid,
                             'raw_detection_count': raw_detection_count,
                             'image_path': image_path,'image_save_warning':image_warning})
    except (EOFError, BrokenPipeError):
        pass
    except Exception as exc:
        try:
            connection.send({'kind': 'error', 'phase':phase, 'message': type(exc).__name__ + ': ' + str(exc)})
        except (OSError, EOFError):
            pass
    finally:
        if pipeline is not None:
            try:
                pipeline.stop()
            except Exception:
                pass
        connection.close()


class WristVision(Vision):
    """复用头部视觉的有限接收和关闭机制，不把机械臂放进可强杀的进程。"""
    def start(self, gate):
        actions = self.config.values['actions']
        used = {p['grasp_profile'] for p in self.config.values['detection_points']
                if p.get('enabled',True) and p['difficulty'] in self.config.values['strategy']['enabled_levels']}
        profiles = {key: actions['profiles'][key]['model'] for key in used}
        ctx = multiprocessing.get_context('spawn')
        self.connection, child = ctx.Pipe()
        self.process = ctx.Process(target=_worker, args=(child, profiles, actions['camera_serial'],
                                   actions['wrist'], actions['wrist']['device']), daemon=True)
        self.process.start()
        child.close()
        message = self._receive(gate)
        if message['kind'] != 'ready':
            raise CameraFailed('腕部相机初始化结果无效')
        for key, labels in message['labels'].items():
            actions['profiles'][key]['model']['labels'] = labels
        self.sequence = 0
        self.evidence.event('wrist_camera_ready', profiles=sorted(used))

    def scan(self, point, label, gate, layer=None):
        gate.check()
        if label is None:
            cfg = self.config.values
            mapping = cfg['actions']['profiles'][point['grasp_profile']]['label_map']
            allowed = point.get('target_labels') or cfg['model']['labels']
            labels = sorted({mapping[name] for name in allowed
                             if name not in cfg['excluded_labels'] and name in mapping})
            if not labels:
                return []
        else:
            labels = [label]
        self.sequence += 1
        image_path = (self.evidence.run_dir / 'images' /
                      ('wrist-{}-{:03d}.jpg'.format(point['id'], self.sequence))
                      if self.evidence.save_images else None)
        self.busy = True
        self._send({'kind': 'scan', 'point_id': point['id'],
                              'profile': point['grasp_profile'], 'labels': labels,
                              'deadline': gate.deadline,
                              'image_path': str(image_path) if image_path else None})
        result = self._receive(gate)
        self.busy = False
        if result['kind'] != 'scan_result' or result['point_id'] != point['id']:
            raise CameraFailed('腕部识别结果与当前家具不一致')
        self._record_image_warning(result)
        self.last_scan = result
        self.evidence.event('wrist_scan_completed', layer=None if layer is None else layer['id'], **result)
        return result['candidates']
