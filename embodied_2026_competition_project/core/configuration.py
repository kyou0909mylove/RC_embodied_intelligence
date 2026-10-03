# -*- coding: utf-8 -*-
"""导航版配置：标准库校验，不连接 ROS、相机或机械臂。"""
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import re


class ConfigurationRejected(ValueError):
    pass


@dataclass
class Configuration:
    root: Path
    path: Path
    values: dict
    model_path: Path
    evidence_path: Path
    labels: list


def _number(value):
    return type(value) in (int, float) and math.isfinite(value)


def _pose_error(pose):
    if (not isinstance(pose, list) or len(pose) != 2
            or not all(isinstance(p, list) for p in pose)
            or len(pose[0]) != 3 or len(pose[1]) != 4
            or not all(_number(x) for row in pose for x in row)):
        return "须为 [[x,y,z],[qx,qy,qz,qw]]，长度单位米"
    if abs(sum(x * x for x in pose[1]) - 1.0) > 0.02:
        return "四元数未归一化"
    if abs(pose[0][2]) > 0.01 or abs(pose[1][0]) > 0.01 or abs(pose[1][1]) > 0.01:
        return "地面导航请使用 z=0、qx=qy=0，只保留平面朝向"
    return None


def load_configuration(path, root, require_poses=True):
    """require_poses=False 仅用于查看标签；实机与配置检查都要求点位完整。"""
    path, root = Path(path).expanduser().resolve(), Path(root).resolve()
    try:
        cfg = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise ConfigurationRejected("配置读取失败：" + str(exc)) from exc
    if (not isinstance(cfg, dict) or type(cfg.get("schema_version")) is not int
            or cfg["schema_version"] != 4 or cfg.get("profile") != "navigation_flow"):
        raise ConfigurationRejected("本版使用 schema_version=4、profile=navigation_flow；请使用 navigation.local.json")
    errors = []
    for key in ("locations", "model", "camera", "timing", "start", "actions", "flow"):
        if not isinstance(cfg.get(key), dict):
            errors.append(key + " 须为对象")
    if errors:
        raise ConfigurationRejected("\n- ".join(errors))
    # 兼容用户现有 schema=4 六点配置，不覆盖原文件。
    cfg.setdefault("initial_pose", None)
    cfg.setdefault("localization", {})
    cfg.setdefault("recording", {"save_images": False})
    cfg.setdefault("speech", {"pause_s": 2.0})
    cfg["flow"].setdefault("skip_unreachable_detection", True)
    if not isinstance(cfg["localization"], dict):
        raise ConfigurationRejected("localization 须为对象")
    cfg["localization"].setdefault("initial_pose_wait_s", 5.0)
    cfg["localization"].setdefault("confirmation_timeout_s", 45.0)
    if not isinstance(cfg["speech"], dict):
        raise ConfigurationRejected("speech 须为对象")
    cfg["speech"].setdefault("announce_stages", True)
    # 旧六点配置默认沿用异步播报；新模板启用上传新版节点的完成回执。
    cfg["speech"].setdefault("wait_done", False)
    cfg["speech"].setdefault("timeout_s", 12.0)
    cfg["speech"].setdefault("label_names", {
        "cola_bottle": "可乐瓶", "sprite_bottle": "雪碧瓶",
        "orange_fanta_bottle": "橙味芬达瓶", "baima_dish_soap": "洗洁精",
        "lays_stax_can": "薯片罐", "safeguard_pump_bottle": "按压瓶",
        "blue_bowl": "蓝色碗", "spoon": "勺子", "black_green_box": "黑绿盒",
        "oreo_long_box": "奥利奥长盒", "green_shampoo_bottle": "绿色洗发水瓶"})
    locations = cfg["locations"]
    poses = [("locations." + key, locations.get(key)) for key in ("entry", "exit", "drop")]
    if cfg["initial_pose"] is not None:
        poses.append(("initial_pose", cfg["initial_pose"]))
    points = cfg.get("detection_points")
    if not isinstance(points, list) or not points or not all(isinstance(p, dict) for p in points):
        errors.append("detection_points 须为至少一个检测点组成的列表")
        points = []
    ids = []
    for index, point in enumerate(points):
        pid = point.get("id")
        if not isinstance(pid, str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,64}", pid):
            errors.append("检测点 id 无效：" + str(index + 1))
        else:
            ids.append(pid)
        poses.append(("detection_points[" + str(index) + "].pose", point.get("pose")))
        for key in ("name", "region"):
            if not isinstance(point.get(key), str) or not point[key].strip():
                errors.append(str(pid) + "." + key + " 须为非空文字")
    if len(set(ids)) != len(ids):
        errors.append("检测点 id 重复")
    for name, pose in poses:
        if pose is None:
            if require_poses:
                errors.append("待填写地图位姿：" + name)
        else:
            error = _pose_error(pose)
            if error:
                errors.append(name + "：" + error)
    model = cfg["model"]
    labels = model.get("labels")
    if (not isinstance(labels, list) or not labels
            or not all(isinstance(x, str) and x.strip() for x in labels)
            or len(set(labels)) != len(labels)):
        errors.append("model.labels 须为模型的精确类别名列表")
        labels = []
    for point in points:
        selected = point.get("target_labels", [])
        if not isinstance(selected, list) or any(x not in labels for x in selected):
            errors.append(str(point.get("id")) + ".target_labels 包含模型没有的标签")
        if "test_label" in point and point["test_label"] not in labels:
            errors.append(str(point.get("id")) + ".test_label 须为模型标签，只供模拟测试使用")
    if not _number(model.get("confidence")) or not 0 < model["confidence"] <= 1:
        errors.append("model.confidence 须在 (0,1] 内")
    if type(model.get("frames")) is not int or not 1 <= model["frames"] <= 20:
        errors.append("model.frames 须在 1–20 内")
    if type(model.get("imgsz")) is not int or not 32 <= model["imgsz"] <= 1920:
        errors.append("model.imgsz 无效")
    if model.get("device") != "cpu" and not (isinstance(model.get("device"), str) and model["device"].isdigit()):
        errors.append("model.device 使用 cpu 或 GPU 编号字符串")
    weights = model.get("weights")
    model_path = path.parent / "__missing_model__"
    if not isinstance(weights, str) or not weights.strip():
        errors.append("model.weights 未填写")
    else:
        p = Path(weights).expanduser()
        model_path = (p if p.is_absolute() else path.parent / p).resolve()
        if not model_path.is_file():
            errors.append("模型不存在：" + str(model_path))
        elif hashlib.sha256(model_path.read_bytes()).hexdigest() != model.get("sha256"):
            errors.append("模型 SHA256 与本次上传的 best.pt 不一致")
    try:
        info = json.loads((root / "models/MODEL_INFO.json").read_text(encoding="utf-8"))
        actual = [name for _, name in sorted(info["names"].items(), key=lambda x: int(x[0]))]
        if labels != actual or model.get("sha256") != info["sha256"]:
            errors.append("模型配置与 MODEL_INFO.json 中提取的标签/哈希不一致")
    except (OSError, ValueError, KeyError, TypeError) as exc:
        errors.append("模型元数据读取失败：" + str(exc))
    timing = cfg["timing"]
    time_keys = ("match_s", "exit_reserve_s", "prepare_timeout_s", "nav_timeout_s", "scan_timeout_s",
                 "nav_estimate_s", "scan_estimate_s", "grasp_estimate_s", "place_estimate_s", "exit_estimate_s")
    for key in time_keys:
        if not _number(timing.get(key)) or timing[key] <= 0:
            errors.append("timing." + key + " 须为有限正数")
    if all(_number(timing.get(k)) and timing[k] > 0 for k in time_keys):
        if timing["match_s"] != 600 or timing["exit_reserve_s"] != 30:
            errors.append("本版固定 600 秒，最后 30 秒停止工作；满足真实交付条件后才执行离场")
        if timing["prepare_timeout_s"] > 60 or timing["nav_timeout_s"] > 120 or timing["scan_timeout_s"] > 60:
            errors.append("准备/导航/扫描超时上限超出允许范围")
        if timing["nav_estimate_s"] > timing["nav_timeout_s"] or timing["scan_estimate_s"] > timing["scan_timeout_s"]:
            errors.append("预计耗时不能大于对应阶段超时")
        if timing["exit_estimate_s"] > timing["exit_reserve_s"]:
            errors.append("离场估计不能超过 30 秒预留")
    if cfg["camera"] != {"backend": "azure_kinect", "color_mode": "1080p"}:
        errors.append("本版沿用 Azure Kinect 的 1080P 彩色相机")
    if cfg["actions"].get("mode") != "print":
        errors.append("当前抓取/投放接口只实现 print 模式")
    localization = cfg["localization"]
    wait_s = localization.get("initial_pose_wait_s")
    confirm_s = localization.get("confirmation_timeout_s")
    if not _number(wait_s) or not 0 <= wait_s <= 10:
        errors.append("localization.initial_pose_wait_s 须在 0–10 秒内")
    if not _number(confirm_s) or not 0 < confirm_s <= 45:
        errors.append("localization.confirmation_timeout_s 须在 (0,45] 秒内")
    speech = cfg.get("speech")
    if not isinstance(speech, dict):
        errors.append("speech 须为对象")
    else:
        if not _number(speech.get("pause_s")) or not 0 <= speech["pause_s"] <= 10:
            errors.append("speech.pause_s 须在 0–10 秒内；它只是等待，不是播报完成反馈")
        if type(speech.get("announce_stages")) is not bool:
            errors.append("speech.announce_stages 须为布尔值")
        if type(speech.get("wait_done")) is not bool:
            errors.append("speech.wait_done 须为布尔值")
        if not _number(speech.get("timeout_s")) or not 0 < speech["timeout_s"] <= 30:
            errors.append("speech.timeout_s 须在 (0,30] 秒内")
        names = speech.get("label_names")
        if (not isinstance(names, dict) or any(k not in labels for k in names)
                or not all(isinstance(v, str) and v.strip() for v in names.values())):
            errors.append("speech.label_names 须为模型标签到中文播报名的对象")
    recording = cfg.get("recording")
    if not isinstance(recording, dict) or type(recording.get("save_images")) is not bool:
        errors.append("recording.save_images 须为布尔值")
    if cfg["flow"].get("empty_detection") not in ("print_actions", "skip"):
        errors.append("flow.empty_detection 使用 print_actions 或 skip")
    if type(cfg["flow"].get("exit_when_done")) is not bool:
        errors.append("flow.exit_when_done 须为布尔值")
    if type(cfg["flow"].get("skip_unreachable_detection")) is not bool:
        errors.append("flow.skip_unreachable_detection 须为布尔值")
    start = cfg["start"]
    if start.get("mode") not in ("immediate", "topic"):
        errors.append("start.mode 使用 immediate 或 topic")
    if not _number(start.get("delay_s")) or not 0 <= start["delay_s"] <= 30:
        errors.append("start.delay_s 须在 0–30 秒内")
    for name, topic in (("start.topic", start.get("topic")), ("stop_topic", cfg.get("stop_topic"))):
        if not isinstance(topic, str) or not topic.startswith("/"):
            errors.append(name + " 须为 ROS 绝对话题名")
    if start.get("topic") == cfg.get("stop_topic"):
        errors.append("开始/停止话题不能相同")
    if not isinstance(start.get("value"), str) or not start["value"].strip():
        errors.append("start.value 须为非空 String 内容")
    evidence = cfg.get("evidence_dir")
    evidence_path = root / "runs"
    if not isinstance(evidence, str) or not evidence.strip():
        errors.append("evidence_dir 须为非空路径")
    else:
        p = Path(evidence).expanduser()
        evidence_path = (p if p.is_absolute() else path.parent / p).resolve()
    if errors:
        raise ConfigurationRejected("配置未完成，未连接机器人：\n- " + "\n- ".join(errors))
    return Configuration(root, path, cfg, model_path, evidence_path, labels)
