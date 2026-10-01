# -*- coding: utf-8 -*-
"""搜索相机的多帧观察与四种复核；原检测接口和坐标转换保持。"""
import math
import time
from datetime import datetime, timezone


class VisionStage:
    """与流程共享 context/session；只在 SCAN 阶段调用。"""

    def __init__(self, context, session):
        """保存运行进度和已初始化设备，本构造器不启动相机。"""
        self.context = context
        self.session = session

    def run(self):
        """采集后按目的核对；False 对应旧主循环的 continue。"""
        observation = self._collect_observation()
        valid = self._check_frames(observation)
        if not valid:
            return False
        handlers = {
            "SEARCH": self._search,
            "RECHECK": self._recheck,
            "SLOT_BEFORE": self._before_delivery,
            "SLOT_AFTER": self._after_delivery,
        }
        return handlers[self.context.scan_mode](observation)

    def _collect_observation(self):
        """同 capture 采图、检测、转换、关联并保存图像；每一步仍检查 gate。"""
        ctx = self.context
        base = self.session.base
        camera = self.session.camera
        converter = self.session.converter
        cv2 = self.session.cv2
        detector = self.session.detector
        gate = self.session.gate
        run_dir = self.session.evidence.run_dir
        vision = self.session.config.values["vision"]
        base.stop()
        # tracks是跨帧轨迹；seen_points保存所有有效几何观察；image_paths保存本次图像证据文件名。
        tracks, seen_points, image_paths = [], [], []
        valid_frames, invalid_points, last_valid_frame = 0, 0, None
        for frame_id in range(vision["frames"]):
            gate.check()
            if time.monotonic() >= ctx.operation_deadline:
                raise TimeoutError("视觉阶段预算耗尽")
            # 合入新增附件的同 capture 思路；直接组合往年已用的 SDK 方法，
            # 不新增 get_rgbd 功能函数，也不改原 KinectCamera 源文件。
            # 彩色与深度从同一个capture取得；depth[y,x]才对应这张彩色图的像素。
            capture = camera.device.update()
            gate.check()
            ok, rgb = capture.get_color_image()
            gate.check()
            if time.monotonic() >= ctx.operation_deadline:
                raise TimeoutError("彩色图像返回迟到")
            ok_depth, depth = capture.get_transformed_depth_image()
            gate.check()
            if time.monotonic() >= ctx.operation_deadline:
                raise TimeoutError("深度图像返回迟到")
            if not ok or not ok_depth or rgb is None or depth is None:
                continue
            if (rgb.ndim != 3 or depth.ndim != 2
                    or rgb.shape[:2] != depth.shape):
                continue  # 对齐深度与彩色尺寸不一致不能参与几何判断。
            valid_frames += 1
            last_valid_frame = frame_id
            detector.color_frame = rgb.copy()
            detections = detector.pred()
            gate.check()
            if time.monotonic() >= ctx.operation_deadline:
                raise TimeoutError("模型推理返回迟到")
            points = []
            for detection in detections:
                confidence = float(detection.conf)
                if not math.isfinite(confidence) or confidence < vision["confidence"]:
                    continue
                px, py = int(detection.x), int(detection.y)
                if not (0 <= py < depth.shape[0] and 0 <= px < depth.shape[1]):
                    invalid_points += 1
                    continue
                z = float(depth[py, px]) * 0.001
                if not math.isfinite(z) or not 0 < z <= vision["depth_max_m"]:
                    invalid_points += 1
                    continue
                # 第一步：像素+深度+内参得到相机点；下一步TF再得到地图点。world()名字不能理解成已经在map中。
                point_camera = detector.world(camera, depth, detection)
                if not all(math.isfinite(float(x)) for x in point_camera):
                    invalid_points += 1
                    continue
                point = converter.get_map_coords([float(x) for x in point_camera])
                gate.check()
                if time.monotonic() >= ctx.operation_deadline:
                    raise TimeoutError("坐标转换返回迟到")
                if point is None or not all(math.isfinite(float(x)) for x in point):
                    invalid_points += 1
                    continue
                p = {"label": detection.name, "xy": [float(point[0]), float(point[1])]}
                # 同帧相邻重叠框只保留一次；绝不把框的数量当作连续命中帧数。
                if any(old["label"] == p["label"] and math.dist(old["xy"], p["xy"]) <=
                       vision["same_frame_radius_m"] for old in points):
                    continue
                points.append(p)
            seen_points.extend(points)
            pairs = []
            for ti, track in enumerate(tracks):
                for pi, p in enumerate(points):
                    distance = math.dist(track["xy"], p["xy"])
                    if track["label"] == p["label"] and distance <= vision["track_radius_m"]:
                        pairs.append((distance, ti, pi))
            # 一对一关联：一个框本帧最多归入一条轨迹，一条轨迹本帧最多加一次命中。
            used_tracks, used_points = set(), set()
            for _, ti, pi in sorted(pairs):
                if ti in used_tracks or pi in used_points:
                    continue
                track, p = tracks[ti], points[pi]
                hits = len(track["frames"])
                track["xy"] = [(track["xy"][j] * hits + p["xy"][j]) / (hits + 1) for j in range(2)]
                track["frames"].append(frame_id)
                used_tracks.add(ti)
                used_points.add(pi)
            for pi, p in enumerate(points):
                if pi not in used_points:
                    tracks.append(dict(p, frames=[frame_id]))
            stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
            image_path = run_dir / (stamp + "_" + ctx.scan_mode.lower() + "_" + str(frame_id) + ".jpg")
            cv2.putText(detector.color_frame, stamp, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 255, 255), 2)
            cv2.putText(detector.color_frame, ctx.scan_mode + " elapsed=" + str(round(time.monotonic() - ctx.started, 2)),
                        (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            if not cv2.imwrite(str(image_path), detector.color_frame):
                raise OSError("证据图像无法保存")
            image_paths.append(image_path.name)
            gate.check()
            if time.monotonic() >= ctx.operation_deadline:
                raise TimeoutError("证据保存迟到")
            if frame_id + 1 < vision["frames"]:
                time.sleep(min(vision["frame_pause_s"], max(0, ctx.operation_deadline - time.monotonic())))
        # 早期看见后消失的轨迹不用于靠近或交付确认。
        # 必须在不同帧累计足够命中，且最后一帧仍出现；同帧多个框不能冒充多帧。
        stable = [t for t in tracks if len(set(t["frames"])) >= vision["min_hits"]
                  and vision["frames"] - 1 in t["frames"]]
        observation = {"mode": ctx.scan_mode, "tracks": tracks, "images": image_paths,
                       "valid_frames": valid_frames, "last_valid_frame": last_valid_frame,
                       "invalid_points": invalid_points,
                       "elapsed_s": round(time.monotonic() - ctx.started, 3)}
        ctx.record["observations"].append(observation)
        gate.check()
        if time.monotonic() >= ctx.operation_deadline:
            raise TimeoutError("视觉复核迟到")
        return dict(stable=stable, seen_points=seen_points, image_paths=image_paths,
                    valid_frames=valid_frames, invalid_points=invalid_points,
                    last_valid_frame=last_valid_frame)

    def _check_frames(self, observation):
        """不足帧时空爪可切换或有限复扫；携物观察失败仍抛异常。"""
        ctx = self.context
        strategy = self.session.config.strategy
        vision = self.session.config.values["vision"]
        valid_frames = observation["valid_frames"]
        last_valid_frame = observation["last_valid_frame"]
        if valid_frames < vision["min_hits"] or last_valid_frame != vision["frames"] - 1:
            if ctx.holding == "EMPTY" and ctx.scan_mode == "SEARCH":
                ctx.record["events"].append({"event": "search_frames_insufficient", "area": ctx.area["id"]})
                ctx.state = "SELECT"
                return False
            if (ctx.holding == "EMPTY" and ctx.scan_mode == "RECHECK"
                    and ctx.attempt["recheck_retries"] < strategy["recheck_retries"]):
                ctx.attempt["recheck_retries"] += 1
                ctx.record["events"].append({"event": "recheck_retry", "reason": "insufficient_frames"})
                ctx.state = "SCAN"
                return False
            if ctx.holding == "EMPTY" and ctx.scan_mode == "RECHECK":
                ctx.inventory[ctx.task["catalog_id"]]["pregrasp_failures"] += 1
                ctx.attempt["status"] = "RECHECK_FRAMES_INSUFFICIENT"
                ctx.state = "SELECT"
                return False
            raise RuntimeError("有效相机帧不足，不能把观察失败当作区域为空")
        return True

    def _search(self, observation):
        """选来源区内符合标签、预算和配额的稳定目标。"""
        ctx = self.context
        est = self.session.config.values["estimates"]
        gate = self.session.gate
        speaker = self.session.speaker
        strategy = self.session.config.strategy
        stable = observation["stable"]
        seen_points = observation["seen_points"]
        image_paths = observation["image_paths"]
        invalid_points = observation["invalid_points"]
        candidates = []
        x0, x1, y0, y1 = ctx.area["source_bounds"]
        for ti, t in enumerate(ctx.area["targets"]):
            item = ctx.inventory[t["catalog_id"]]
            if (item["delivered"] >= item["expected"]
                    or item["pregrasp_failures"] >= strategy["max_pregrasp_failures"]
                    or item["grasp_attempts"] >= strategy["max_grasp_attempts"]):
                continue
            needed = (est["nav_s"] + est["scan_s"] + t["pickup_estimate_s"]
                      + ctx.return_cost + ctx.exit_cost)
            if ctx.deadline - time.monotonic() < needed:
                continue
            for pi, p in enumerate(stable):
                x, y = p["xy"]
                if p["label"] == t["search_label"] and x0 <= x <= x1 and y0 <= y <= y1:
                    # 使用同一成功先验与台账数据，比较本次可见候选的预计收益；它是调度估计值。
                    probability = (2 * t["success_prior"] + item["delivered"]) / (
                        2 + item["grasp_attempts"] + item["pregrasp_failures"])
                    bias = strategy["level1_weight"] if t["level"] == 1 else 1.0
                    if not ctx.deliveries and t["level"] == 1:
                        bias *= strategy["bootstrap_level1_weight"]
                    value = (30 if t["level"] == 1 else 40) * probability * bias / needed
                    candidates.append((value, -ti, -pi, ti, pi))
        if not candidates:
            ctx.state = "SELECT"
            return False
        # Python解包：前面不需要的排名字段丢给_，ti是目标配置索引，pi是视觉轨迹索引。
        *_, ti, pi = max(candidates)
        ctx.task = ctx.area["targets"][ti]
        ctx.target_map = stable[pi]["xy"] + [0.0]  # 原转换的 z 不用于机械臂
        ctx.attempt = {"id": len(ctx.record["attempts"]) + 1, "area": ctx.area["id"],
                   "catalog_id": ctx.task["catalog_id"], "recheck_retries": 0,
                   "label": ctx.task["search_label"], "name_cn": ctx.task["name_cn"],
                   "level": ctx.task["level"], "source_xy": ctx.target_map[:2], "status": "RECOGNIZED",
                   "recognition_images": image_paths}
        ctx.record["attempts"].append(ctx.attempt)
        if speaker.tts_pub.get_num_connections() == 0:
            raise RuntimeError("识别时 TTS 连接已断开")
        gate.check()
        speaker.speak("识别到{}，{}级难度。".format(ctx.task["name_cn"], ctx.task["level"]))
        ctx.attempt["announcement"] = "REQUEST_PUBLISHED_NO_PLAYBACK_ACK"
        ctx.state = "APPROACH"
        return True

    def _recheck(self, observation):
        """靠近后核对目标；同类歧义保持原拒绝与有限重试条件。"""
        ctx = self.context
        strategy = self.session.config.strategy
        vision = self.session.config.values["vision"]
        stable = observation["stable"]
        seen_points = observation["seen_points"]
        image_paths = observation["image_paths"]
        invalid_points = observation["invalid_points"]
        visible = [p for p in stable if p["label"] == ctx.task["search_label"]]
        # 原桌面接口只能按类别抓取，不能传入实例 ID；同类歧义时不冒充实例抓取。
        other_seen = any(p["label"] == ctx.task["search_label"] and
                         math.dist(p["xy"], ctx.target_map[:2]) > vision["recheck_radius_m"]
                         for p in seen_points)
        if (invalid_points or other_seen or len(visible) != 1 or
                math.dist(visible[0]["xy"], ctx.target_map[:2]) > vision["recheck_radius_m"]):
            # 同类歧义不原地重抓。只有暂时丢失/几何失效时，空爪允许有限次复扫。
            if (not other_seen and len(visible) <= 1
                    and ctx.attempt["recheck_retries"] < strategy["recheck_retries"]):
                ctx.attempt["recheck_retries"] += 1
                ctx.attempt["status"] = "RECHECK_RETRY"
                ctx.record["events"].append({"event": "recheck_retry", "reason": "lost_or_invalid_geometry"})
                ctx.state = "SCAN"
            else:
                ctx.inventory[ctx.task["catalog_id"]]["pregrasp_failures"] += 1
                ctx.attempt["status"] = "RECHECK_AMBIGUOUS_OR_LOST"
                ctx.state = "SELECT"
        else:
            ctx.target_map[:2] = visible[0]["xy"]
            ctx.state = "GRASP"
        return True

    def _before_delivery(self, observation):
        """记录原同类物品并预留可用槽位，保持包络与间距判据。"""
        ctx = self.context
        cfg = self.session.config.values
        slots = self.session.config.slots
        targets = self.session.config.targets
        vision = self.session.config.values["vision"]
        stable = observation["stable"]
        seen_points = observation["seen_points"]
        image_paths = observation["image_paths"]
        invalid_points = observation["invalid_points"]
        if invalid_points:
            raise RuntimeError("放置前有检测框缺少可靠深度/TF，不能确认槽位无已识别占用")
        sx0, sx1, sy0, sy1 = cfg["score_bounds"]
        ctx.before_zone = [p for p in stable if p["label"] == ctx.task["search_label"] and
                       sx0 <= p["xy"][0] <= sx1 and sy0 <= p["xy"][1] <= sy1]
        uncertain_old = any(
            p["label"] == ctx.task["search_label"] and
            sx0 <= p["xy"][0] <= sx1 and sy0 <= p["xy"][1] <= sy1 and
            not any(math.dist(p["xy"], old["xy"]) <= vision["track_radius_m"] for old in ctx.before_zone)
            for p in seen_points)
        if uncertain_old:
            raise RuntimeError("得分区基线中有未稳定关联的同类检测，不能视为不存在")
        ctx.slot = None
        for s in slots:
            if s["status"] != "AVAILABLE":
                continue
            x0, x1, y0, y1 = s["bounds"]
            clearance = ctx.task["object_radius_m"] + cfg["score_margin_m"]
            usable = [max(x0, sx0 + clearance), min(x1, sx1 - clearance),
                      max(y0, sy0 + clearance), min(y1, sy1 - clearance)]
            if usable[0] >= usable[1] or usable[2] >= usable[3]:
                continue
            # 对已识别物品以计划内最大包络半径保守估算，留出双方半径和关联误差。
            # 未识别/遮挡物体仍无法由此证明不存在。
            # 间距保留两件物品的包络半径和关联误差；这是保守几何判据，不是完整障碍物检测。
            spacing = (ctx.task["object_radius_m"] + max(t["object_radius_m"] for t in targets)
                       + vision["track_radius_m"])
            occupied = any(math.hypot(
                max(usable[0] - p["xy"][0], p["xy"][0] - usable[1], 0),
                max(usable[2] - p["xy"][1], p["xy"][1] - usable[3], 0)) <= spacing
                for p in seen_points)
            if occupied:
                s["status"] = "BLOCKED"
            elif ctx.slot is None:
                ctx.slot = s
                ctx.slot["usable_bounds"] = usable
        if ctx.slot is None:
            raise RuntimeError("无可用放置位；保留夹持并停止")
        ctx.slot["status"] = "RESERVED"
        ctx.attempt["slot_id"] = ctx.slot["id"]
        ctx.attempt["before_images"] = image_paths
        ctx.attempt["before_zone"] = ctx.before_zone
        ctx.destination = ctx.slot["drop_location"]
        ctx.grip_after = "PLACE"
        ctx.nav_kind, ctx.nav_after, ctx.state = "drop", "GRIP", "NAVIGATE"
        return True

    def _after_delivery(self, observation):
        """一对一匹配原物品，确认唯一新增；交付计数只在此增加。"""
        ctx = self.context
        cfg = self.session.config.values
        gate = self.session.gate
        speaker = self.session.speaker
        vision = self.session.config.values["vision"]
        stable = observation["stable"]
        seen_points = observation["seen_points"]
        image_paths = observation["image_paths"]
        invalid_points = observation["invalid_points"]
        sx0, sx1, sy0, sy1 = cfg["score_bounds"]
        after_zone = [p for p in stable if p["label"] == ctx.task["search_label"] and
                      sx0 <= p["xy"][0] <= sx1 and sy0 <= p["xy"][1] <= sy1]
        pairs = []
        for bi, old in enumerate(ctx.before_zone):
            for ai, new in enumerate(after_zone):
                distance = math.dist(old["xy"], new["xy"])
                if distance <= vision["track_radius_m"]:
                    pairs.append((distance, bi, ai))
        old_matched, new_matched = set(), set()
        for _, bi, ai in sorted(pairs):
            if bi not in old_matched and ai not in new_matched:
                old_matched.add(bi)
                new_matched.add(ai)
        additions = [p for i, p in enumerate(after_zone) if i not in new_matched]
        x0, x1, y0, y1 = ctx.slot["usable_bounds"]
        matches = [p for p in additions if
                   x0 <= p["xy"][0] <= x1 and y0 <= p["xy"][1] <= y1]
        uncertain_new = any(
            p["label"] == ctx.task["search_label"] and
            sx0 <= p["xy"][0] <= sx1 and sy0 <= p["xy"][1] <= sy1 and
            not any(math.dist(p["xy"], new["xy"]) <= vision["track_radius_m"] for new in after_zone)
            for p in seen_points)
        ctx.attempt["after_zone"] = after_zone
        if (invalid_points or uncertain_new or len(after_zone) != len(ctx.before_zone) + 1
                or len(old_matched) != len(ctx.before_zone) or len(matches) != 1):
            ctx.slot["status"] = "UNVERIFIED"
            ctx.attempt["status"] = "DELIVERY_UNVERIFIED"
            raise RuntimeError("放置后未得到唯一的新物品视觉证据，不增加交付数量")
        gate.check()
        if time.monotonic() >= ctx.operation_deadline:
            raise TimeoutError("交付证据迟到，不登记成功")
        ctx.slot["status"] = "OCCUPIED"
        ctx.attempt["status"] = "DELIVERED_CONFIRMED"
        # 在全部判据通过且没迟到后才追加交付记录；后面的库存+1也只在这里发生。
        ctx.deliveries.append({"attempt_id": ctx.attempt["id"], "slot_id": ctx.slot["id"],
                           "catalog_id": ctx.task["catalog_id"],
                           "label": ctx.task["search_label"], "level": ctx.task["level"],
                           "map_xy": matches[0]["xy"], "images": image_paths,
                           "elapsed_s": round(time.monotonic() - ctx.started, 3)})
        item = ctx.inventory[ctx.task["catalog_id"]]
        item["delivered"] += 1
        item["status"] = "COMPLETE" if item["delivered"] >= item["expected"] else "PENDING"
        ctx.holding = "EMPTY"
        elapsed = time.monotonic() - ctx.cycle_started
        old_mean = ctx.area["cycle_mean_s"]
        ctx.area["cycle_mean_s"] = elapsed if old_mean is None else 0.5 * (old_mean + elapsed)
        gate.check()
        speaker.speak("已将{}运回己方得分区。".format(ctx.task["name_cn"]))
        ctx.state = "SELECT"
        return True
