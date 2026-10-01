# -*- coding: utf-8 -*-
"""配置读取与静态校验。只用标准库，不导入 ROS、模型或设备模块。"""
import hashlib
import json
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any
from .context import MATCH_SECONDS


class ConfigurationRejected(Exception):
    """已输出具体配置错误，入口应以退出码 2 结束。"""


@dataclass
class Configuration:
    """经过校验的数据；areas/slots 包含本次运行的初始计数与槽位状态。"""
    root: Path
    path: Path
    values: dict
    args: Any
    areas: list
    slots: list
    catalog: list
    capabilities: dict
    strategy: dict
    intrinsics: list
    start: dict
    targets: list
    bound_catalog_ids: list


class ConfigurationLoader:
    """按原顺序分段校验，收集错误；通过后才返回 Configuration。"""

    def __init__(self, args, root):
        """保存路径和启动选项；构造本对象不会连接任何设备。"""
        self.args = args
        self.root = root

    def load(self):
        """清单模式提前返回；其他模式检查完整现场配置。"""
        self._read()
        catalog_result = self._check_catalog()
        if catalog_result is not None:
            return catalog_result
        self._check_structure()
        self._collect_areas()
        self._check_shapes()
        self._check_parameters()
        self._check_relations()
        self._check_files()
        return Configuration(
            self.root, self.config_path, self.cfg, self.args,
            self.areas, self.slots, self.catalog, self.capabilities,
            self.strategy, self.intrinsics, self.start,
            self.targets, self.bound_catalog_ids)

    def _read(self):
        """读取 JSON、检查版本，原错误文本与退出码保持。"""
        self.config_path = self.args.config.expanduser().resolve()
        try:
            self.cfg = json.loads(self.config_path.read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            print("配置读取失败：" + str(exc), file=sys.stderr)
            raise ConfigurationRejected()

        # 所有现场坐标、耗时估计和夹爪阈值显式校验；bool 不能冒充数值。
        # errors收集所有错误，便于一次看到多个待填项，而不是每次只修一个。
        self.errors = []
        if (not isinstance(self.cfg, dict) or type(self.cfg.get("schema_version")) is not int
                or self.cfg["schema_version"] != 3):
            print("需要 schema_version=3 的任务清单配置；迁移说明见 MERGE_NOTES.md", file=sys.stderr)
            raise ConfigurationRejected()

    def _check_catalog(self):
        """检查清单与能力声明；--list-tasks 在此输出并结束。"""
        self.catalog_errors = []
        self.catalog, self.capabilities = self.cfg.get("task_catalog"), self.cfg.get("capabilities")
        self.strategy = self.cfg.get("strategy")
        methods = {"table": "catch_table", "table_short": "catch_table_short", "ground": "catch_ground"}
        if not isinstance(self.catalog, list) or not self.catalog or not all(isinstance(t, dict) for t in self.catalog):
            self.catalog_errors.append("task_catalog 必须是非空对象数组")
            self.catalog = []
        ids = [t.get("id") for t in self.catalog]
        if (any(not isinstance(i, str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,64}", i) for i in ids)
                or len(set(i for i in ids if isinstance(i, str))) != len(ids)):
            self.catalog_errors.append("任务清单 id 无效或重复")
        self.catalog_by_id = {t["id"]: t for t in self.catalog if isinstance(t.get("id"), str)}
        # 逐条检查难度、摆放方式、配额、名称/标签和尺寸；三级容器操作只能标为unsupported。
        for t in self.catalog:
            if type(t.get("level")) is not int or t["level"] not in (1, 2, 3):
                self.catalog_errors.append("任务清单 level 必须为 1/2/3")
            if type(t.get("expected_count")) is not int or not 1 <= t["expected_count"] <= 18:
                self.catalog_errors.append("expected_count 必须为 1–18 的整数")
            if t.get("status") not in ("ready", "uncalibrated", "unsupported"):
                self.catalog_errors.append("任务状态必须为 ready/uncalibrated/unsupported")
            if t.get("surface") not in (*methods, "container"):
                self.catalog_errors.append("任务清单 surface 无效")
            if t.get("level") == 3 and (t.get("status") != "unsupported" or t.get("surface") != "container"):
                self.catalog_errors.append("往年接口不具备柜门/抽屉技能，三级任务只能标为 unsupported")
            if t.get("surface") == "container" and t.get("status") != "unsupported":
                self.catalog_errors.append("container 能力未实现，不能标为可运行")
            if t.get("surface") == "ground" and t.get("level") != 2:
                self.catalog_errors.append("地面目标须为二级")
            if t.get("status") == "ready":
                if any(not isinstance(t.get(k), str) or not t[k].strip()
                       for k in ("name_cn", "search_label", "grasp_label")):
                    self.catalog_errors.append("ready 任务必须填写中文名称和精确模型标签")
                r = t.get("object_radius_m")
                if type(r) not in (int, float) or not math.isfinite(r) or not 0 < r < 0.5:
                    self.catalog_errors.append("ready 任务须填写有限、正且小于 0.5 米的物品包络半径")
        if not self.catalog_errors:
            for level, maximum in ((1, 8), (2, 6), (3, 4)):
                if sum(t["expected_count"] for t in self.catalog if t["level"] == level) > maximum:
                    self.catalog_errors.append("任务清单配额超过该等级比赛物品总数：" + str(level))
        if not isinstance(self.capabilities, dict):
            self.catalog_errors.append("capabilities 必须是对象")
            self.capabilities = {}
        # 【能力接线】surface决定调用哪个既有catch方法；JSON不能凭一个ready标志创造新技能。
        for surface, method in {**methods, "container": None}.items():
            c = self.capabilities.get(surface)
            if (not isinstance(c, dict) or c.get("legacy_method") != method
                    or c.get("status") not in ("ready", "uncalibrated", "unsupported")):
                self.catalog_errors.append("能力声明与既有接口不一致：" + surface)
            elif surface == "container" and c["status"] != "unsupported":
                self.catalog_errors.append("禁止通过能力标志启用未实现的柜体操作")
        # 只显示任务清单时在这里返回；后面实际位姿、模型文件和ROS初始化都不会执行。
        if self.args.list_tasks:
            print(json.dumps({"task_catalog": self.catalog, "capabilities": self.capabilities,
                              "note": "清单是任务配额；ready 仍需模型、坐标及标定校验，不代表实机验证通过。"},
                             ensure_ascii=False, indent=2))
            if self.catalog_errors:
                print("\n".join(self.catalog_errors), file=sys.stderr)
            return 2 if self.catalog_errors else 0

    def _check_structure(self):
        """检查策略及顶层数据结构，避免后续访问错误类型。"""
        self.errors.extend(self.catalog_errors)
        # 【04 策略参数】一级偏好、尝试次数和复扫次数是调度规则，不是机械臂位姿。
        if not isinstance(self.strategy, dict):
            self.errors.append("strategy 必须是对象")
        else:
            for key in ("level1_weight", "bootstrap_level1_weight"):
                value = self.strategy.get(key)
                if type(value) not in (int, float) or not math.isfinite(value) or not 1 <= value <= 2:
                    self.errors.append("strategy." + key + " 须为 1–2 的有限数")
            for key in ("max_pregrasp_failures", "max_grasp_attempts", "recheck_retries"):
                value = self.strategy.get(key)
                if type(value) is not int or not (0 if key == "recheck_retries" else 1) <= value <= 10:
                    self.errors.append("strategy." + key + " 超出允许整数范围")
        for key in ("locations", "start", "limits", "estimates", "vision", "gripper", "camera"):
            if not isinstance(self.cfg.get(key), dict):
                self.errors.append(key + " 必须是对象")
        for key in ("areas", "slots"):
            if not isinstance(self.cfg.get(key), list) or not self.cfg[key]:
                self.errors.append(key + " 必须是非空数组")
        if self.errors:
            print("\n".join(self.errors), file=sys.stderr)
            raise ConfigurationRejected()

    def _collect_areas(self):
        """收集启用区域、初始化槽位并检查标定声明。"""
        self.areas = []
        for area in self.cfg["areas"]:
            if not isinstance(area, dict) or type(area.get("enabled")) is not bool:
                self.errors.append("每个区域须明确 enabled: true/false")
                continue
            if not area["enabled"]:
                continue
            if not isinstance(area.get("targets"), list) or not area["targets"]:
                self.errors.append("启用区域必须提供 targets")
                continue
            if not all(isinstance(t, dict) for t in area["targets"]):
                self.errors.append("targets 成员必须是对象")
                continue
            self.areas.append(dict(area, visits=0, cycle_mean_s=None))
        # 每个放置位另加运行状态：AVAILABLE可用，RESERVED预留，OCCUPIED已投放，BLOCKED/UNVERIFIED不可再用。
        self.slots = [dict(s, status="AVAILABLE") for s in self.cfg["slots"] if isinstance(s, dict)]
        if len(self.slots) != len(self.cfg["slots"]) or not self.slots or not self.areas:
            self.errors.append("至少一个有效启用区域和放置位")
        # calibrated是现场测试完成的声明；--real要求相机、目标和槽位的标定声明都完成。
        calibrations = [("camera", self.cfg["camera"].get("calibrated"))]
        calibrations += [("slot " + str(s.get("id")), s.get("calibrated")) for s in self.slots]
        calibrations += [("target " + str(t.get("name_cn")), t.get("calibrated"))
                         for a in self.areas for t in a["targets"]]
        for name, calibrated in calibrations:
            if type(calibrated) is not bool:
                self.errors.append(name + ".calibrated 必须为布尔值")
            elif self.args.real and not calibrated:
                self.errors.append(name + " 尚未完成实机标定，不启动实机")

    def _check_shapes(self):
        """检查相机内参、导航位姿、区域边界及 ID。"""
        self.intrinsics = self.cfg["camera"].get("intrinsics")
        if (not isinstance(self.intrinsics, list) or len(self.intrinsics) != 9 or
                not all(type(x) in (int, float) and math.isfinite(x) for x in self.intrinsics)):
            self.errors.append("camera.intrinsics 需要现场相机的 3×3 内参矩阵，按行展开为 9 个数")
        elif (self.intrinsics[0] <= 0 or self.intrinsics[4] <= 0 or self.intrinsics[1] != 0
              or self.intrinsics[3] != 0 or self.intrinsics[6:] != [0, 0, 1]):
            self.errors.append("camera.intrinsics 须为 fx/fy 为正、零 skew、末行为 [0,0,1] 的针孔矩阵")
        if not isinstance(self.cfg.get("stop_topic"), str) or not self.cfg["stop_topic"].startswith("/"):
            self.errors.append("stop_topic 必须是 ROS 绝对话题名")
        elif self.cfg["stop_topic"] == self.cfg["start"].get("topic"):
            self.errors.append("停止话题不能与开始话题相同")
        if type(self.cfg["gripper"].get("samples")) is not int or not 2 <= self.cfg["gripper"]["samples"] <= 10:
            self.errors.append("gripper.samples 须为 2–10 的整数")
        for entries, kind in ((self.areas, "area"), (self.slots, "slot")):
            names = [a.get("id") for a in entries]
            if any(not isinstance(n, str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,64}", n) for n in names):
                self.errors.append(kind + " id 只允许 1–64 位字母、数字、下划线、连字符")
            elif len(set(names)) != len(names):
                self.errors.append(kind + " id 重复")
        # 【位姿格式】map位姿是[[x,y,z],[qx,qy,qz,qw]]；位置用米，后四个值是方向四元数。
        self.poses = [("locations." + n, self.cfg["locations"].get(n)) for n in ("entry", "exit", "score_view")]
        self.poses += [("area " + str(a.get("id")), a.get("search_location")) for a in self.areas]
        self.poses += [("slot " + str(s.get("id")), s.get("drop_location")) for s in self.slots]
        for name, pose in self.poses:
            if (not isinstance(pose, list) or len(pose) != 2
                    or not isinstance(pose[0], list) or len(pose[0]) != 3
                    or not isinstance(pose[1], list) or len(pose[1]) != 4
                    or not all(type(self.v) in (int, float) and math.isfinite(self.v) for p in pose for self.v in p)):
                self.errors.append("待填写有效位姿：" + name)
            elif abs(sum(self.v * self.v for self.v in pose[1]) - 1) > 0.02:
                self.errors.append("四元数未归一化：" + name)
        # 【矩形格式】[xmin,xmax,ymin,ymax]；来源区用于过滤物品，槽位用于判断投放。
        self.bounds_list = [(k, self.cfg.get(k)) for k in ("field_bounds", "score_bounds")]
        self.bounds_list += [("area " + str(a.get("id")), a.get("source_bounds")) for a in self.areas]
        self.bounds_list += [("slot " + str(s.get("id")), s.get("bounds")) for s in self.slots]
        self.bounds_ok = True
        for name, b in self.bounds_list:
            if (not isinstance(b, list) or len(b) != 4
                    or not all(type(self.v) in (int, float) and math.isfinite(self.v) for self.v in b)
                    or b[0] >= b[1] or b[2] >= b[3]):
                self.errors.append("待填写 [xmin,xmax,ymin,ymax]：" + name)
                self.bounds_ok = False

    def _check_parameters(self):
        """检查目标绑定、数值、投放姿态和开始信号。"""
        self.positive = [("score_margin_m", self.cfg.get("score_margin_m"))]
        for group, keys in {
            "limits": ("prepare_s", "nav_s", "scan_s", "grasp_s", "arm_s", "grip_s", "local_s"),
            "estimates": ("nav_s", "scan_s", "place_s", "grip_s", "exit_s", "margin_s", "exit_trigger_s"),
            "vision": ("confidence", "track_radius_m", "same_frame_radius_m", "recheck_radius_m", "depth_max_m", "frame_pause_s"),
            "gripper": ("open_max", "stability_turn"),
        }.items():
            self.positive += [(group + "." + k, self.cfg[group].get(k)) for k in keys]
        # 区域目标须引用清单ID，并保持标签、难度、摆放和物品半径一致。
        self.bound_catalog_ids = []
        for area in self.areas:
            if type(area.get("visit_limit")) is not int or not 1 <= area["visit_limit"] <= 100:
                self.errors.append("visit_limit 必须为 1–100 的整数")
            labels = []
            for t in area["targets"]:
                cid = t.get("catalog_id")
                entry = self.catalog_by_id.get(cid) if isinstance(cid, str) else None
                if entry is None:
                    self.errors.append("启用目标必须引用已有 catalog_id")
                else:
                    self.bound_catalog_ids.append(cid)
                    for key in ("name_cn", "search_label", "grasp_label", "level", "surface", "object_radius_m"):
                        if t.get(key) != entry.get(key):
                            self.errors.append("目标与任务清单不一致：" + cid + "." + key)
                    if entry.get("status") == "unsupported":
                        self.errors.append("不能启用 unsupported 任务：" + cid)
                    elif self.args.real and entry.get("status") != "ready":
                        self.errors.append("目标清单尚未标为 ready：" + cid)
                capability = self.capabilities.get(t.get("surface")) if isinstance(t.get("surface"), str) else None
                if not isinstance(capability, dict) or capability.get("status") == "unsupported":
                    self.errors.append("目标没有可用的既有抓取能力")
                elif self.args.real and capability.get("status") != "ready":
                    self.errors.append("抓取能力尚未标定 ready：" + str(t.get("surface")))
                for key in ("name_cn", "search_label", "grasp_label"):
                    if not isinstance(t.get(key), str) or not t[key].strip():
                        self.errors.append("目标缺少字符串 " + key)
                labels.append(t.get("search_label"))
                if type(t.get("level")) is not int or t["level"] not in (1, 2):
                    self.errors.append("仅支持已标定的一级/二级任务；原代码没有开柜技能")
                if t.get("surface") not in ("table", "table_short", "ground"):
                    self.errors.append("surface 必须为 table/table_short/ground")
                if t.get("surface") == "ground" and t.get("level") != 2:
                    self.errors.append("地面目标按该规则配置为二级")
                self.positive += [("target." + k, t.get(k)) for k in
                             ("standoff_m", "success_prior", "pickup_estimate_s", "hold_min", "hold_max", "object_radius_m")]
            if all(isinstance(label, str) for label in labels) and len(set(labels)) != len(labels):
                self.errors.append("一个区域内同一搜索标签只能对应一种等级/表面/抓取标定")
        if len(set(self.bound_catalog_ids)) != len(self.bound_catalog_ids):
            self.errors.append("一个清单 ID 只能绑定一个启用区域；多件同类物品使用 expected_count")
        for name, value in self.positive:
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                self.errors.append("待填写有限正数：" + name)
        for s in self.slots:
            p = s.get("drop_poses")
            if (not isinstance(p, list) or not p or any(
                    not isinstance(self.v, list) or len(self.v) != 6 or not all(
                        type(x) in (int, float) and math.isfinite(x) for x in self.v) for self.v in p)):
                self.errors.append("放置位需要至少一个现场标定的 mdeg drop_poses")
        self.v = self.cfg["vision"]
        if (type(self.v.get("frames")) is not int or type(self.v.get("min_hits")) is not int
                or not 2 <= self.v["min_hits"] <= self.v["frames"] <= 20):
            self.errors.append("视觉帧数须满足 2 <= min_hits <= frames <= 20")
        self.start = self.cfg["start"]
        if not isinstance(self.start.get("topic"), str) or not self.start["topic"].startswith("/"):
            self.errors.append("start.topic 必须为 ROS 绝对话题名")
        if self.start.get("type") not in ("String", "Bool"):
            self.errors.append("start.type 仅支持 String/Bool，须与现场发布者一致")
        elif ((self.start["type"] == "String" and (not isinstance(self.start.get("value"), str) or not self.start["value"].strip()))
              or (self.start["type"] == "Bool" and self.start.get("value") is not True)):
            self.errors.append("启动内容应为非空 String 或 Bool true")
        if not isinstance(self.cfg.get("evidence_dir"), str) or not self.cfg["evidence_dir"].strip():
            self.errors.append("evidence_dir 必须是非空路径")
        if self.args.real and (not self.args.match_id or not re.fullmatch(r"[A-Za-z0-9_-]{1,64}", self.args.match_id)):
            self.errors.append("--real 需要有效 --match-id，每局唯一，同局保持不变")

    def _check_relations(self):
        """仅在类型有效时检查时间、空间和夹爪阈值关系。"""
        if not self.errors:
            # 【06 参数之间的关系】est是预计耗时，用于选任务；lim是硬性的阶段软件超时，两者用途不同。
            est, lim = self.cfg["estimates"], self.cfg["limits"]
            if lim["prepare_s"] > 60:
                self.errors.append("准备阶段软件上限不得超过 60 秒；实际以裁判计时为准")
            if not 0 < self.cfg["score_margin_m"] < 0.5:
                self.errors.append("score_margin_m 必须小于 0.5")
            if self.v["confidence"] > 1 or self.v["same_frame_radius_m"] >= self.v["track_radius_m"]:
                self.errors.append("confidence <= 1，且同帧合并距离须小于跨帧关联距离")
            if self.v["recheck_radius_m"] < self.v["track_radius_m"]:
                self.errors.append("recheck_radius_m 不得小于 track_radius_m")
            if (est["exit_trigger_s"] < est["exit_s"] + est["margin_s"]
                    or est["exit_trigger_s"] >= MATCH_SECONDS):
                self.errors.append("exit_trigger_s 须覆盖离场估计+余量，且小于 600")
            for e, l in (("nav_s", "nav_s"), ("scan_s", "scan_s"), ("place_s", "arm_s"),
                         ("grip_s", "grip_s"), ("exit_s", "nav_s")):
                if est[e] > lim[l]:
                    self.errors.append("估计耗时超过对应超时上限：" + e)
            if self.cfg["gripper"]["open_max"] >= 6800 or self.cfg["gripper"]["stability_turn"] >= 6800:
                self.errors.append("夹爪阈值应小于原驱动的 6800 turn")
            for a in self.areas:
                for t in a["targets"]:
                    if not self.cfg["gripper"]["open_max"] < t["hold_min"] < t["hold_max"] < 6800:
                        self.errors.append("需要 open_max < hold_min < hold_max < 6800")
                    if t["success_prior"] > 1:
                        self.errors.append("success_prior 不得大于 1")
                    if t["object_radius_m"] + self.cfg["score_margin_m"] >= 0.5:
                        self.errors.append("物品包络半径加边线余量过大，无法置于 1m 得分区")
            if self.bounds_ok:
                fx0, fx1, fy0, fy1 = self.cfg["field_bounds"]
                sx0, sx1, sy0, sy1 = self.cfg["score_bounds"]
                margin = self.cfg["score_margin_m"]
                if abs(sx1 - sx0 - 1) > 0.02 or abs(sy1 - sy0 - 1) > 0.02:
                    self.errors.append("score_bounds 须对应己方 1m × 1m 得分区")
                if not (fx0 <= sx0 < sx1 <= fx1 and fy0 <= sy0 < sy1 <= fy1):
                    self.errors.append("得分区不在场地范围内")
                for a in self.areas:
                    x0, x1, y0, y1 = a["source_bounds"]
                    if not (fx0 <= x0 < x1 <= fx1 and fy0 <= y0 < y1 <= fy1):
                        self.errors.append("source_bounds 超出场地")
                    if x0 < sx1 and x1 > sx0 and y0 < sy1 and y1 > sy0:
                        self.errors.append("搜索物品区域不能与得分区重叠")
                    x, y = a["search_location"][0][:2]
                    if not (fx0 <= x <= fx1 and fy0 <= y <= fy1):
                        self.errors.append("搜索停车位超出场地")
                for i, a in enumerate(self.areas):
                    x0, x1, y0, y1 = a["source_bounds"]
                    labels = {t["search_label"] for t in a["targets"]}
                    for other in self.areas[:i]:
                        a0, a1, b0, b1 = other["source_bounds"]
                        if (x0 < a1 and x1 > a0 and y0 < b1 and y1 > b0
                                and labels & {t["search_label"] for t in other["targets"]}):
                            self.errors.append("同类目标的搜索区域不能重叠，无法仅凭 map XY 区分任务归属")
                for pose in [self.cfg["locations"]["score_view"]] + [s["drop_location"] for s in self.slots]:
                    x, y = pose[0][:2]
                    if not (fx0 <= x <= fx1 and fy0 <= y <= fy1):
                        self.errors.append("得分区观察/放置停车位超出场地")
                # 还要确认槽位不越过己方边界、槽位之间有间距，并容得下物品包络。
                for i, s in enumerate(self.slots):
                    x0, x1, y0, y1 = s["bounds"]
                    if not (sx0 + margin <= x0 < x1 <= sx1 - margin
                            and sy0 + margin <= y0 < y1 <= sy1 - margin):
                        self.errors.append("放置位须位于己方得分区内并保留边线余量")
                    for other in self.slots[:i]:
                        a0, a1, b0, b1 = other["bounds"]
                        gap_x, gap_y = max(a0 - x1, x0 - a1, 0), max(b0 - y1, y0 - b1, 0)
                        if math.hypot(gap_x, gap_y) <= 2 * self.v["track_radius_m"]:
                            self.errors.append("放置位之间须留出大于两倍关联误差的间距")
                for a in self.areas:
                    for t in a["targets"]:
                        clearance = t["object_radius_m"] + margin
                        if not any(max(s["bounds"][0], sx0 + clearance) < min(s["bounds"][1], sx1 - clearance)
                                   and max(s["bounds"][2], sy0 + clearance) < min(s["bounds"][3], sy1 - clearance)
                                   for s in self.slots):
                            self.errors.append("没有容纳物品包络及边线余量的槽位：" + t["catalog_id"])

    def _check_files(self):
        """检查旧模块来源与模型路径；相对模型路径以 JSON 目录为基准。"""
        try:
            # 【07 来源检查】哈希相当于文件指纹：校验实际旧模块与本次交付清单一致。
            manifest = json.loads((self.root / "legacy_manifest.json").read_text(encoding="utf-8"))
            for entry in manifest:
                path = self.root / entry["path"]
                if hashlib.sha256(path.read_bytes()).hexdigest() != entry["sha256"]:
                    self.errors.append("原模块与交付版本不一致：" + entry["path"])
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.errors.append("原代码清单检查失败：" + str(exc))
        try:
            support_source = json.loads((self.root / "support/SOURCE.json").read_text(encoding="utf-8"))
            if hashlib.sha256((self.root / "support/safety.py").read_bytes()).hexdigest() != support_source["local_sha256"]:
                self.errors.append("截止门控与交付版本不一致：support/safety.py")
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.errors.append("截止门控来源检查失败：" + str(exc))
        # 【模型路径】按启用能力检查对应权重；有一个空/不存在的路径就不能开始实机流程。
        self.weights = [self.cfg.get("search_weights")]
        self.targets = [t for a in self.areas for t in a["targets"]]
        if any(t.get("surface") == "ground" for t in self.targets):
            self.weights.append(self.cfg.get("ground_weights"))
        self.weights += [self.cfg.get(surface + "_weights") for surface in ("table", "table_short")
                    if any(t.get("surface") == surface for t in self.targets)]
        for w in self.weights:
            if not isinstance(w, str) or not w.strip():
                self.errors.append("需要实际模型权重路径；不能把通用 YOLO 当作比赛模型")
                continue
            path = Path(w).expanduser()
            if not path.is_absolute():
                path = self.config_path.parent / path
            if not path.is_file():
                self.errors.append("缺少权重：" + str(path))
        for key in ("search_weights", "ground_weights", "table_weights", "table_short_weights"):
            if isinstance(self.cfg.get(key), str) and self.cfg[key]:
                p = Path(self.cfg[key]).expanduser()
                self.cfg[key] = str((p if p.is_absolute() else self.config_path.parent / p).resolve())
        if self.errors:
            print("配置未完成，未连接机器人：\n- " + "\n- ".join(self.errors), file=sys.stderr)
            raise ConfigurationRejected()
