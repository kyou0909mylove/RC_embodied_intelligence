#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线回归：所有设备均为测试替身，不连接 ROS，不发送机器人动作。"""
from contextlib import contextmanager
import copy
import io
import json
from pathlib import Path
from queue import SimpleQueue
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from core.configuration import ConfigurationRejected, load_configuration
from core.evidence import Evidence
from core.session import RobotSession, SessionRejected, NavigationFailed
from core.workflow import Workflow
from core.safety import DeadlineGate, OperationStopped
from camera.vision import Vision, _worker


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


class FakeSession:
    """仅用于离线测试，不是可部署的机器人能力实现。"""
    def __init__(self, values, test_mode=True, behavior="target"):
        self.config = SimpleNamespace(values=copy.deepcopy(values))
        self.test_mode, self.speech_print = test_mode, False
        self.clock = Clock()
        self.gate = DeadlineGate(self.clock)
        self.gate.deadline = self.work_deadline = 570.0
        self.match_deadline = 600.0
        self.rospy = SimpleNamespace(is_shutdown=lambda: False)
        self.evidence = Evidence(Path("/unused"), "offline", self.config.values,
                                 self.clock, test_mode=test_mode, save_logs=False)
        self.evidence.started = 0.0
        self.vision = SimpleNamespace(scan=self.scan)
        self.texts, self.route = [], []
        self.speaker = SimpleNamespace(speak=self.texts.append,
                                       tts_pub=SimpleNamespace(get_num_connections=lambda: 1))
        self.behavior, self.exit_reason = behavior, None
        self.status, self.exit_code = "ERROR", 1
        self.stop_requests = 0

    def check(self):
        self.gate.check()

    @contextmanager
    def operation(self, timeout):
        with self.gate.limit(self.gate.deadline, timeout):
            yield

    def navigate(self, name, pose):
        self.check()
        self.route.append(name)
        self.evidence.event("navigation_started", name=name)
        self.clock.advance(1)

    def scan(self, point, gate):
        if self.behavior in ("deadline", "stop", "stage_timeout"):
            self.clock.now = 570.05 if self.behavior != "stage_timeout" else self.clock() + 13
            if self.behavior == "stop":
                gate.stopped = True
            gate.check()
        if self.behavior == "camera_error":
            raise RuntimeError("没有有效相机帧")
        target = {"label": "cola_bottle", "confidence": 0.9} if self.behavior == "target" else None
        return {"point_id": point["id"], "target": target, "valid_frames": 3,
                "detections": [target] if target else [], "simulated": False}

    def begin_exit(self, reason):
        self.exit_reason = reason
        self.gate.deadline = self.match_deadline
        self.gate.check()

    def stop_navigation(self):
        self.stop_requests += 1


class FlowChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.pose = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
        cls.cfg = load_configuration(ROOT / "config/navigation.example.json", ROOT,
                                     require_poses=False).values
        cls.cfg["speech"]["wait_done"] = False  # 旧语音节点兼容路径；同步回执另有专门检查。

    def run_flow(self, cfg=None, test_mode=True, behavior="target"):
        session = FakeSession(cfg or self.cfg, test_mode, behavior)
        with patch("time.sleep", side_effect=session.clock.advance), patch("sys.stdout", new=io.StringIO()):
            Workflow(session).run()
        return session

    def filled_cfg(self):
        cfg = copy.deepcopy(self.cfg)
        cfg["model"]["weights"] = str(ROOT / "models/best.pt")
        cfg["locations"] = {key: copy.deepcopy(self.pose) for key in ("entry", "exit", "drop")}
        for point in cfg["detection_points"]:
            point["pose"] = copy.deepcopy(self.pose)
        return cfg

    def load_temp(self, cfg):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "config.json"
            path.write_text(json.dumps(cfg), encoding="utf-8")
            return load_configuration(path, ROOT)

    def test_blank_template_only_requires_six_navigation_poses(self):
        with self.assertRaises(ConfigurationRejected) as caught:
            load_configuration(ROOT / "config/navigation.example.json", ROOT)
        self.assertEqual(str(caught.exception).count("待填写地图位姿"), 6)

    def test_legacy_six_point_config_is_accepted_without_new_keys(self):
        cfg = self.filled_cfg()
        for key in ("initial_pose", "localization", "speech", "recording"):
            cfg.pop(key)
        for point in cfg["detection_points"]:
            point.pop("test_label")
        loaded = self.load_temp(cfg)
        self.assertIsNone(loaded.values["initial_pose"])
        self.assertFalse(loaded.values["recording"]["save_images"])
        self.assertEqual(loaded.values["speech"]["pause_s"], 2.0)
        self.assertFalse(loaded.values["speech"]["wait_done"])
        self.assertTrue(loaded.values["speech"]["announce_stages"])
        self.assertEqual(loaded.values["localization"]["initial_pose_wait_s"], 5.0)
        self.assertEqual(loaded.values["localization"]["confirmation_timeout_s"], 45.0)

    def test_invalid_quaternion_is_rejected(self):
        cfg = self.filled_cfg()
        cfg["locations"]["entry"][1] = [0, 0, 0, 0]
        with self.assertRaisesRegex(ConfigurationRejected, "四元数未归一化"):
            self.load_temp(cfg)

    def test_duplicate_point_id_is_rejected(self):
        cfg = self.filled_cfg()
        cfg["detection_points"][1]["id"] = cfg["detection_points"][0]["id"]
        with self.assertRaisesRegex(ConfigurationRejected, "id 重复"):
            self.load_temp(cfg)

    def test_initial_pose_is_validated_but_not_copied_from_entry(self):
        cfg = self.filled_cfg()
        self.assertIsNone(self.load_temp(cfg).values["initial_pose"])
        cfg["initial_pose"] = [[1, 2, 0], [0, 0, 0, 0]]
        with self.assertRaisesRegex(ConfigurationRejected, "initial_pose"):
            self.load_temp(cfg)

    def test_bad_optional_objects_report_configuration_errors(self):
        for key, value in (("speech", None), ("recording", None), ("localization", None)):
            cfg = self.filled_cfg()
            cfg[key] = value
            with self.subTest(key=key), self.assertRaises(ConfigurationRejected):
                self.load_temp(cfg)

    def test_localization_wait_and_confirmation_bounds_are_validated(self):
        for key, value in (("initial_pose_wait_s", -1), ("initial_pose_wait_s", 11),
                           ("initial_pose_wait_s", True), ("initial_pose_wait_s", float("nan")),
                           ("confirmation_timeout_s", 0), ("confirmation_timeout_s", 46)):
            cfg = self.filled_cfg()
            cfg["localization"][key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ConfigurationRejected):
                self.load_temp(cfg)

    def test_stage_speech_option_requires_boolean(self):
        cfg = self.filled_cfg()
        cfg["speech"]["announce_stages"] = "false"
        with self.assertRaisesRegex(ConfigurationRejected, "announce_stages"):
            self.load_temp(cfg)

    def test_new_template_enables_supplied_done_interface(self):
        cfg = load_configuration(ROOT / "config/navigation.example.json", ROOT, require_poses=False).values
        self.assertTrue(cfg["speech"]["wait_done"])

    def test_synchronous_speech_records_only_actual_matching_backend_done(self):
        cfg = copy.deepcopy(self.cfg)
        cfg["speech"]["wait_done"] = True
        session = FakeSession(cfg)
        requests = []
        session.speaker.speak = lambda text, **kw: requests.append((text, kw)) or True
        with patch("time.sleep", side_effect=session.clock.advance), patch("sys.stdout", new=io.StringIO()):
            Workflow(session).run()
        self.assertEqual(len(requests), 6)
        self.assertTrue(all(kw["wait_done"] and kw["timeout"] == 12.0 for text, kw in requests))
        self.assertEqual(session.evidence.record["speech_done_requests"], 6)
        self.assertTrue(session.evidence.record["speech_completed_confirmed"])

    def test_missing_speech_done_stops_without_grasping(self):
        cfg = copy.deepcopy(self.cfg)
        cfg["speech"]["wait_done"] = True
        session = FakeSession(cfg)
        session.speaker.speak = lambda text, **kw: False
        with patch("sys.stdout", new=io.StringIO()), self.assertRaisesRegex(RuntimeError, "summer_tts_done"):
            Workflow(session).run()
        self.assertEqual(session.evidence.record["speech_done_requests"], 0)
        self.assertFalse(session.evidence.record["speech_completed_confirmed"])
        self.assertEqual(session.evidence.record["grasp_interface_calls"], 0)
        self.assertEqual(session.route, ["入场点"])

    def test_one_command_test_entry_supplies_real_pose_unique_id_and_no_log(self):
        from testing import test_navigation_flow as entry
        with patch.object(entry, "run_main", return_value=0) as run, patch("sys.stdout", new=io.StringIO()):
            self.assertEqual(entry.main([]), 0)
            first = run.call_args.args[0]
            self.assertEqual(entry.main([]), 0)
            second = run.call_args.args[0]
        for args in (first, second):
            self.assertIn("--real", args)
            self.assertIn("--set-initial-pose", args)
            self.assertIn("--no-log", args)
        self.assertNotEqual(first[first.index("--match-id") + 1], second[second.index("--match-id") + 1])
        self.assertTrue(run.call_args.kwargs["test_mode"])

    def test_one_command_readonly_does_not_authorize_device_motion(self):
        from testing import test_navigation_flow as entry
        with patch.object(entry, "run_main", return_value=0) as run:
            self.assertEqual(entry.main(["--list-labels"]), 0)
        self.assertEqual(run.call_args.args[0], ["--list-labels"])

    def test_one_command_honors_manual_localization_and_explicit_match_id(self):
        from testing import test_navigation_flow as entry
        args = ["--localization-confirmed", "--match-id=given", "--speech-print"]
        with patch.object(entry, "run_main", return_value=0) as run, patch("sys.stdout", new=io.StringIO()):
            entry.main(args)
        used = run.call_args.args[0]
        self.assertNotIn("--set-initial-pose", used)
        self.assertEqual(sum(arg.startswith("--match-id") for arg in used), 1)

    def test_model_label_mismatch_is_rejected(self):
        cfg = self.filled_cfg()
        cfg["model"]["labels"][0] = "invented_class"
        with self.assertRaises(ConfigurationRejected):
            self.load_temp(cfg)

    def test_simulation_has_exact_eight_goals_and_three_print_action_pairs(self):
        session = self.run_flow()
        self.assertEqual(session.route, ["入场点", "区域1检测点1", "投放点", "区域2检测点1",
                                         "投放点", "区域3检测点1", "投放点", "离场点"])
        self.assertEqual(session.evidence.record["grasp_interface_calls"], 3)
        self.assertEqual(session.evidence.record["place_interface_calls"], 3)
        self.assertEqual(session.evidence.record["physical_grasps"], 0)
        self.assertEqual(session.evidence.record["physical_deliveries"], 0)
        self.assertEqual(session.status, "FLOW_COMPLETED")

    def test_simulated_speech_is_explicit_and_after_detection_before_grasp(self):
        session = self.run_flow()
        detection_texts = [text for text in session.texts if text.startswith("模拟检测，")]
        self.assertEqual(len(session.texts), 6)
        self.assertEqual(len(detection_texts), 3)
        events = session.evidence.record["events"]
        for scan_index, event in enumerate(events):
            if event["event"] == "scan_completed":
                next_speech = next(i for i in range(scan_index + 1, len(events))
                                   if events[i]["event"] == "speech_requested")
                next_grasp = next(i for i in range(scan_index + 1, len(events))
                                  if events[i]["event"] == "grasp_interface_completed")
                self.assertLess(next_speech, next_grasp)
                self.assertEqual(events[next_speech]["kind"], "detection")
                self.assertTrue(events[next_speech]["simulated_detection"])

    def test_jujia_stage_announcements_follow_arrival_search_and_early_exit_order(self):
        session = self.run_flow()
        self.assertEqual(session.texts[:2], ["已到达入场点", "开始遍历物品检测点"])
        self.assertEqual(session.texts[-1], "导航测试，开始离场演练")
        events = session.evidence.record["events"]
        entry_index = next(i for i, e in enumerate(events)
                           if e["event"] == "navigation_started" and e["name"] == "入场点")
        first_scan = next(i for i, e in enumerate(events) if e["event"] == "scan_completed")
        stage_indices = [i for i, e in enumerate(events)
                         if e["event"] == "speech_requested" and e["kind"] == "stage"]
        last_place = max(i for i, e in enumerate(events) if e["event"] == "place_interface_completed")
        exit_index = next(i for i, e in enumerate(events)
                          if e["event"] == "navigation_started" and e["name"] == "离场点")
        self.assertLess(entry_index, stage_indices[0])
        self.assertLess(stage_indices[1], first_scan)
        self.assertLess(last_place, stage_indices[-1])
        self.assertLess(stage_indices[-1], exit_index)
        self.assertEqual(session.evidence.record["speech_requests"], 6)
        self.assertTrue(all(e["playback_confirmed"] is False for e in events
                            if e["event"] == "speech_requested"))

    def test_stage_announcements_can_be_disabled_without_disabling_detection_speech(self):
        cfg = copy.deepcopy(self.cfg)
        cfg["speech"]["announce_stages"] = False
        session = self.run_flow(cfg)
        self.assertEqual(len(session.texts), 3)
        self.assertTrue(all(text.startswith("模拟检测，") for text in session.texts))
        self.assertEqual(len(session.route), 8)

    def test_test_label_is_not_a_real_vision_claim(self):
        session = self.run_flow()
        scans = [e for e in session.evidence.record["events"] if e["event"] == "scan_completed"]
        self.assertTrue(all(e["simulated"] and e["valid_frames"] == 0 for e in scans))
        self.assertEqual([e["target"]["label"] for e in scans], ["cola_bottle", "sprite_bottle", "blue_bowl"])

    def test_same_real_detection_route_and_action_placeholders(self):
        session = self.run_flow(test_mode=False)
        self.assertEqual(len(session.route), 7)
        self.assertNotIn("离场点", session.route)
        self.assertNotIn("开始自主离场", session.texts)
        self.assertEqual(session.status, "STOPPED_WITHOUT_DELIVERY")
        self.assertEqual(session.exit_code, 1)
        self.assertGreater(session.stop_requests, 0)
        detection_speech = [e for e in session.evidence.record["events"]
                            if e["event"] == "speech_requested" and e["kind"] == "detection"]
        self.assertEqual(len(detection_speech), 3)
        self.assertTrue(all(e["text"].startswith("相机检测，") and not e["simulated_detection"]
                            for e in detection_speech))
        calls = [e for e in session.evidence.record["events"] if e["event"] == "grasp_interface_completed"]
        self.assertFalse(calls[0]["physical_action"])
        self.assertEqual(calls[0]["target"]["label"], "cola_bottle")

    def test_empty_real_detection_skips_actions_even_with_old_print_actions_setting(self):
        cfg = copy.deepcopy(self.cfg)
        cfg["flow"]["empty_detection"] = "print_actions"
        session = self.run_flow(cfg, test_mode=False, behavior="empty")
        self.assertNotIn("投放点", session.route)
        self.assertEqual(session.evidence.record["grasp_interface_calls"], 0)
        self.assertEqual(session.texts, ["已到达入场点", "开始遍历物品检测点"])
        self.assertNotIn("离场点", session.route)
        self.assertEqual(session.status, "STOPPED_WITHOUT_DELIVERY")

    def test_more_detection_points_can_be_appended_and_regions_can_repeat(self):
        cfg = copy.deepcopy(self.cfg)
        cfg["detection_points"].append({"id": "detect_04", "name": "区域1检测点2",
            "region": "region_1", "pose": self.pose, "test_label": "spoon", "target_labels": []})
        session = self.run_flow(cfg)
        self.assertEqual(session.route[-3:], ["区域1检测点2", "投放点", "离场点"])
        self.assertEqual(session.evidence.record["place_interface_calls"], 4)

    def test_only_explicit_unreachable_detection_can_be_skipped(self):
        for state in (4, 5):
            session = FakeSession(self.cfg)
            original = session.navigate
            def navigate(name, pose):
                if name == "区域1检测点1":
                    raise NavigationFailed(name, state)
                original(name, pose)
            session.navigate = navigate
            with self.subTest(state=state), patch("time.sleep", side_effect=session.clock.advance), \
                 patch("sys.stdout", new=io.StringIO()):
                Workflow(session).run()
            self.assertEqual(session.evidence.record["detection_points_skipped"], ["detect_01"])
            self.assertEqual(session.evidence.record["grasp_interface_calls"], 2)
            self.assertIn("区域2检测点1", session.route)
            self.assertEqual(session.route[-1], "离场点")
            self.assertEqual(session.status, "EXITED_EARLY")

    def test_unreachable_entry_or_drop_does_not_continue_to_other_detection_points(self):
        for failed_name in ("入场点", "投放点", "离场点"):
            session = FakeSession(self.cfg)
            original = session.navigate
            def navigate(name, pose):
                if name == failed_name:
                    raise NavigationFailed(name, 4)
                original(name, pose)
            session.navigate = navigate
            with self.subTest(name=failed_name), patch("time.sleep", side_effect=session.clock.advance), \
                 patch("sys.stdout", new=io.StringIO()), self.assertRaises(NavigationFailed):
                Workflow(session).run()
            self.assertEqual(session.evidence.record["detection_points_skipped"], [])
            if failed_name != "离场点":
                self.assertNotIn("区域2检测点1", session.route)

    def test_recall_preempt_or_lost_detection_goal_is_not_skipped(self):
        for state in (2, 8, 9):
            session = FakeSession(self.cfg)
            original = session.navigate
            def navigate(name, pose):
                if name == "区域1检测点1":
                    raise NavigationFailed(name, state)
                original(name, pose)
            session.navigate = navigate
            with self.subTest(state=state), patch("time.sleep", side_effect=session.clock.advance), \
                 patch("sys.stdout", new=io.StringIO()), self.assertRaises(NavigationFailed):
                Workflow(session).run()
            self.assertEqual(session.evidence.record["detection_points_skipped"], [])
            self.assertNotIn("区域2检测点1", session.route)

    def test_unreachable_detection_skip_can_be_disabled(self):
        cfg = copy.deepcopy(self.cfg)
        cfg["flow"]["skip_unreachable_detection"] = False
        session = FakeSession(cfg)
        session.navigate = lambda name, pose: None if name == "入场点" else (_ for _ in ()).throw(NavigationFailed(name, 4))
        with patch("time.sleep", side_effect=session.clock.advance), patch("sys.stdout", new=io.StringIO()), \
             self.assertRaises(NavigationFailed):
            Workflow(session).run()
        self.assertEqual(session.evidence.record["detection_points_skipped"], [])

    def test_work_deadline_without_real_delivery_stops_in_place(self):
        session = self.run_flow(test_mode=False, behavior="deadline")
        self.assertIsNone(session.exit_reason)
        self.assertEqual(session.evidence.record["grasp_interface_calls"], 0)
        self.assertEqual(session.status, "STOPPED_WITHOUT_DELIVERY")
        self.assertNotIn("离场点", session.route)
        self.assertNotIn("开始自主离场", session.texts)
        self.assertTrue(all(e["elapsed_s"] < 570 for e in session.evidence.record["events"]
                            if e["event"] == "speech_requested"))

    def test_confirmed_real_delivery_allows_exit_at_work_deadline(self):
        session = FakeSession(self.cfg, test_mode=False, behavior="deadline")
        # 离线替身注入一件先前已真实交付的事实；生产代码没有自动产生此值。
        session.evidence.record["physical_deliveries"] = 1
        session.evidence.record["physical_grasps"] = 1
        with patch("time.sleep", side_effect=session.clock.advance), patch("sys.stdout", new=io.StringIO()):
            Workflow(session).run()
        self.assertEqual(session.route[-1], "离场点")
        self.assertEqual(session.status, "EXITED_EARLY")
        self.assertTrue(session.evidence.record["autonomous_exit_qualified"])
        self.assertNotIn("开始自主离场", session.texts)

    def test_navigation_test_exit_is_never_qualified_competition_exit(self):
        session = self.run_flow()
        self.assertFalse(session.evidence.record["autonomous_exit_qualified"])
        arrived = next(e for e in session.evidence.record["events"] if e["event"] == "exit_arrived")
        self.assertTrue(arrived["navigation_test"])
        self.assertFalse(arrived["autonomous_exit_qualified"])

    def test_main_refuses_immediate_timer_before_importing_devices(self):
        from main import main
        cfg = self.filled_cfg()
        cfg["start"]["mode"] = "immediate"
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "local.json"
            path.write_text(json.dumps(cfg), encoding="utf-8")
            with patch("core.session.RobotSession") as session, patch("sys.stderr", new=io.StringIO()) as err:
                self.assertEqual(main(["--config", str(path), "--real", "--match-id", "rules"]), 2)
            session.assert_not_called()
            self.assertIn("裁判开门信号", err.getvalue())

    def test_external_stop_does_not_send_exit_goal(self):
        session = FakeSession(self.cfg, test_mode=False, behavior="stop")
        with patch("time.sleep", side_effect=session.clock.advance), \
             patch("sys.stdout", new=io.StringIO()), self.assertRaises(OperationStopped):
            Workflow(session).run()
        self.assertIsNone(session.exit_reason)
        self.assertNotIn("离场点", session.route)

    def test_stage_timeout_is_not_misreported_as_last_thirty_seconds(self):
        session = FakeSession(self.cfg, test_mode=False, behavior="stage_timeout")
        with patch("time.sleep", side_effect=session.clock.advance), \
             patch("sys.stdout", new=io.StringIO()), self.assertRaises(OperationStopped):
            Workflow(session).run()
        self.assertIsNone(session.exit_reason)

    def test_camera_error_is_not_empty_room_and_no_action_follows(self):
        session = FakeSession(self.cfg, test_mode=False, behavior="camera_error")
        with patch("time.sleep", side_effect=session.clock.advance), \
             patch("sys.stdout", new=io.StringIO()), self.assertRaises(RuntimeError):
            Workflow(session).run()
        self.assertEqual(session.evidence.record["grasp_interface_calls"], 0)
        self.assertIsNone(session.exit_reason)

    def test_budget_exits_before_starting_a_new_round(self):
        session = FakeSession(self.cfg)
        session.clock.now = 540
        with patch("time.sleep", side_effect=session.clock.advance), patch("sys.stdout", new=io.StringIO()):
            Workflow(session).run()
        self.assertEqual(session.route, ["入场点", "离场点"])
        self.assertEqual(session.status, "EXITED_EARLY")

    def test_wait_until_work_deadline_never_speaks_or_grasps_again(self):
        cfg = copy.deepcopy(self.cfg)
        cfg["flow"]["exit_when_done"] = False
        session = self.run_flow(cfg)
        self.assertEqual(session.exit_reason, "进入比赛最后 30 秒")
        self.assertEqual(session.evidence.record["grasp_interface_calls"], 3)
        self.assertNotIn("开始自主离场", session.texts)
        self.assertTrue(all(e["elapsed_s"] < 570 for e in session.evidence.record["events"]
                            if e["event"] == "speech_requested"))

    def test_near_deadline_skips_exit_speech_without_delaying_exit(self):
        cfg = copy.deepcopy(self.cfg)
        cfg["detection_points"] = cfg["detection_points"][:1]
        session = FakeSession(cfg)
        original_navigation = session.navigate
        def navigate(name, pose):
            original_navigation(name, pose)
            if name == "投放点":
                session.clock.now = 569.0
        session.navigate = navigate
        with patch("time.sleep", side_effect=session.clock.advance), patch("sys.stdout", new=io.StringIO()):
            Workflow(session).run()
        self.assertNotIn("开始自主离场", session.texts)
        self.assertEqual(session.route[-1], "离场点")
        self.assertTrue(any(e["event"] == "stage_speech_skipped" for e in session.evidence.record["events"]))

    def test_speech_disconnect_stops_before_grasp(self):
        session = FakeSession(self.cfg)
        session.speaker.tts_pub.get_num_connections = lambda: 0
        with patch("sys.stdout", new=io.StringIO()), self.assertRaisesRegex(RuntimeError, "语音订阅者"):
            Workflow(session).run()
        self.assertEqual(session.evidence.record["grasp_interface_calls"], 0)

    def test_speech_print_sends_no_real_speech_request(self):
        session = FakeSession(self.cfg)
        session.speech_print = True
        with patch("time.sleep", side_effect=session.clock.advance), patch("sys.stdout", new=io.StringIO()):
            Workflow(session).run()
        self.assertEqual(session.texts, [])
        self.assertEqual(session.evidence.record["speech_requests"], 0)

    def test_actual_evidence_accepts_name_metadata_and_no_log_writes_nothing(self):
        with tempfile.TemporaryDirectory() as folder:
            run_dir = Path(folder) / "not_created"
            evidence = Evidence(run_dir, "unit", self.cfg, Clock(), test_mode=True, save_logs=False)
            evidence.event("stage", name="ENTRY")
            evidence.event("navigation_started", name="入场点")
            evidence.finish("FLOW_COMPLETED", 0)
            self.assertEqual(evidence.record["events"][0]["name"], "ENTRY")
            self.assertFalse(run_dir.exists())

    def test_actual_saved_evidence_contains_no_images_in_test(self):
        with tempfile.TemporaryDirectory() as folder:
            evidence = Evidence(Path(folder), "unit", self.cfg, Clock(), test_mode=True)
            evidence.event("stage", name="ENTRY")
            evidence.finish("FLOW_COMPLETED", 0)
            self.assertFalse((Path(folder) / "images").exists())
            self.assertEqual(json.loads((Path(folder) / "result.json").read_text())["physical_deliveries"], 0)

    def navigation_session(self, state):
        session = RobotSession(SimpleNamespace(values=self.cfg), "offline")
        clock, cancelled, zeros, sent = Clock(), [], [], []
        session.clock = session.gate.clock = clock
        session.work_deadline, session.match_deadline, session.gate.deadline = 570, 600, 570
        session.evidence = Evidence(Path("/unused"), "offline", self.cfg, clock, save_logs=False)
        goal = SimpleNamespace(target_pose=SimpleNamespace(header=SimpleNamespace(stamp=None)))
        session.navigator = SimpleNamespace(set_goal=lambda *args: goal,
            stop=lambda: cancelled.append(True), client=SimpleNamespace(
                send_goal=sent.append, get_state=lambda: state))
        session.base = SimpleNamespace(pub=True, stop=lambda: zeros.append(True))
        session.GoalStatus = SimpleNamespace(SUCCEEDED=3, PENDING=0, ACTIVE=1, PREEMPTING=6, RECALLING=7)
        session.rospy = SimpleNamespace(is_shutdown=lambda: False, Time=SimpleNamespace(now=lambda: 0))
        return session, clock, cancelled, zeros, sent

    def test_navigation_timeout_cancels_and_requests_zero_speed(self):
        session, clock, cancelled, zeros, sent = self.navigation_session(1)
        with patch("time.sleep", side_effect=clock.advance), patch("sys.stdout", new=io.StringIO()):
            with self.assertRaises(OperationStopped):
                session.navigate("超时", self.pose)
        self.assertTrue(cancelled and zeros)
        self.assertEqual(len(sent), 1)
        self.assertLess(clock(), 35.1)

    def test_recalled_goal_is_error_not_arrival_and_not_retried(self):
        session, clock, cancelled, zeros, sent = self.navigation_session(8)
        with patch("sys.stdout", new=io.StringIO()), self.assertRaisesRegex(RuntimeError, "状态=8"):
            session.navigate("取消", self.pose)
        self.assertEqual(session.evidence.record["navigation_points_completed"], [])
        self.assertEqual(len(sent), 1)
        self.assertTrue(cancelled and zeros)

    def test_navigation_success_uses_existing_zero_speed_interface(self):
        session, clock, cancelled, zeros, sent = self.navigation_session(3)
        with patch("sys.stdout", new=io.StringIO()):
            session.navigate("成功", self.pose)
        self.assertEqual(zeros, [True])
        self.assertEqual(session.evidence.record["navigation_points_completed"], ["成功"])

    def test_exit_cannot_run_past_match_deadline(self):
        session, clock, cancelled, zeros, sent = self.navigation_session(1)
        session.exiting = True
        session.gate.deadline = 600
        clock.now = 599.9
        with patch("time.sleep", side_effect=clock.advance), patch("sys.stdout", new=io.StringIO()):
            with self.assertRaises(OperationStopped):
                session.navigate("离场点", self.pose)
        self.assertLess(clock(), 600.1)
        self.assertTrue(cancelled and zeros)

    def test_disk_full_abort_record_cannot_prevent_stop_or_release_lock(self):
        session, clock, cancelled, zeros, sent = self.navigation_session(1)
        released = []
        session.lock_file = SimpleNamespace(close=lambda: released.append(True))
        def disk_full(*args, **kwargs):
            raise OSError(28, "No space left on device")
        session.evidence.event = session.evidence.finish = disk_full
        with patch("sys.stderr", new=io.StringIO()):
            session.__exit__(RuntimeError, RuntimeError("unit error"), None)
        self.assertTrue(cancelled and zeros and released)

    def test_cleanup_releases_lock_even_if_camera_close_fails(self):
        session = RobotSession(SimpleNamespace(values=self.cfg), "offline")
        closed = []
        def failure():
            raise RuntimeError("unit camera close failure")
        session.vision = SimpleNamespace(close=failure)
        session.lock_file = SimpleNamespace(close=lambda: closed.append(True))
        with patch("sys.stderr", new=io.StringIO()):
            session.close()
        self.assertEqual(closed, [True])

    def test_same_match_id_is_rejected_even_without_flow_logs(self):
        with tempfile.TemporaryDirectory() as folder:
            cfg = copy.deepcopy(self.cfg)
            cfg["start"]["mode"] = "immediate"
            cfg["start"]["delay_s"] = 0
            config = SimpleNamespace(values=cfg, evidence_path=Path(folder))
            first = RobotSession(config, "same", test_mode=True, save_logs=False)
            first._open_record()
            first.rospy = SimpleNamespace(is_shutdown=lambda: False)
            with patch("sys.stdout", new=io.StringIO()):
                first._wait_start()
            first.close()
            self.assertTrue((Path(folder) / "started-same.json").exists())
            second = RobotSession(config, "same", test_mode=True, save_logs=False)
            try:
                with self.assertRaises(SessionRejected):
                    second._open_record()
            finally:
                second.close()
            self.assertFalse(any(p.is_dir() for p in Path(folder).iterdir()))

    def test_parallel_session_same_evidence_directory_is_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            config = SimpleNamespace(values=self.cfg, evidence_path=Path(folder))
            first = RobotSession(config, "first", test_mode=True, save_logs=False)
            second = RobotSession(config, "second", test_mode=True, save_logs=False)
            try:
                first._open_record()
                with self.assertRaises(SessionRejected):
                    second._open_record()
            finally:
                first.close()
                second.close()

    def test_true_stop_latches_and_false_does_not_unlock(self):
        gate = DeadlineGate()
        gate.stop_messages = SimpleQueue()
        gate.stop_messages.put(SimpleNamespace(data=True))
        gate.stop_messages.put(SimpleNamespace(data=False))
        with self.assertRaises(OperationStopped):
            gate.check()
        with self.assertRaises(OperationStopped):
            gate.check()

    def test_blocked_camera_parent_checks_monotonic_deadline(self):
        clock, gate = Clock(), DeadlineGate()
        gate.clock, gate.deadline = clock, 0.2
        vision = Vision(None, None)
        vision.process = SimpleNamespace(is_alive=lambda: True)
        def poll(timeout):
            clock.advance(timeout)
            return False
        vision.connection = SimpleNamespace(poll=poll)
        with self.assertRaises(OperationStopped):
            vision._receive(gate)
        self.assertLessEqual(clock(), 0.25)

    def test_camera_worker_selects_max_confidence_without_saving_images(self):
        requests = [{"kind": "scan", "labels": [], "point_id": "p",
                     "deadline": float("inf"), "image_path": None}, {"kind": "close"}]
        messages, writes = [], []
        class Scalar:
            def __init__(self, value):
                self.value = value
            def item(self):
                return self.value
        class Vector:
            def cpu(self):
                return self
            def tolist(self):
                return [0, 0, 10, 10]
        result = SimpleNamespace(names={0: "cola_bottle", 1: "sprite_bottle"}, boxes=[
            SimpleNamespace(cls=Scalar(0), conf=Scalar(0.7), xyxy=[Vector()]),
            SimpleNamespace(cls=Scalar(1), conf=Scalar(0.9), xyxy=[Vector()])])
        class Image:
            def __getitem__(self, index):
                return self
        device = SimpleNamespace(update=lambda: SimpleNamespace(get_color_image=lambda: (True, Image())),
                                 stop_cameras=lambda: None, close=lambda: None)
        kinect = SimpleNamespace(default_configuration=SimpleNamespace(),
            initialize_libraries=lambda: None, start_device=lambda **kwargs: device,
            K4A_IMAGE_FORMAT_COLOR_MJPG=1, K4A_COLOR_RESOLUTION_1080P=1,
            K4A_DEPTH_MODE_WFOV_2X2BINNED=1, K4A_FRAMES_PER_SECOND_30=1,
            K4A_WIRED_SYNC_MODE_STANDALONE=1)
        model = SimpleNamespace(names={0: "cola_bottle", 1: "sprite_bottle"},
                                predict=lambda **kwargs: [result])
        modules = {"cv2": SimpleNamespace(imwrite=lambda *args: writes.append(args)),
                   "torch": SimpleNamespace(set_num_threads=lambda n: None),
                   "pykinect_azure": kinect, "ultralytics": SimpleNamespace(YOLO=lambda path: model)}
        connection = SimpleNamespace(send=messages.append, recv=lambda: requests.pop(0), close=lambda: None)
        cfg = dict(self.cfg["model"], labels=["cola_bottle", "sprite_bottle"])
        with patch.dict(sys.modules, modules):
            _worker(connection, cfg, "unit-only-model")
        scan = [m for m in messages if m["kind"] == "scan_result"][0]
        self.assertEqual(scan["target"]["label"], "sprite_bottle")
        self.assertEqual(scan["valid_frames"], 3)
        self.assertEqual(writes, [])
        self.assertIsNone(scan["image_path"])

    def camera_evidence_run(self, frames, write_ok=True):
        """只有测试替身；不打开相机、不加载权重、不写真实图像。"""
        class Scalar:
            def __init__(self, value):
                self.value = value
            def item(self):
                return self.value
        class Vector:
            def cpu(self):
                return self
            def tolist(self):
                return [0, 0, 10, 10]
        class Image:
            def __getitem__(self, index):
                return self
        names = {0: "cola_bottle", 1: "sprite_bottle"}
        results = [SimpleNamespace(names=names, boxes=[])]
        for number, specs in enumerate(frames, 1):
            boxes = [SimpleNamespace(cls=Scalar(cid), conf=Scalar(conf), xyxy=[Vector()])
                     for cid, conf in specs]
            results.append(SimpleNamespace(names=names, boxes=boxes,
                           plot=lambda number=number: {"frame": number}))
        writes, messages = [], []
        def put_text(image, text, *args):
            image["timestamp"] = text
        def write_image(path, image):
            writes.append((path, image.copy()))
            return write_ok
        device = SimpleNamespace(update=lambda: SimpleNamespace(get_color_image=lambda: (True, Image())),
                                 stop_cameras=lambda: None, close=lambda: None)
        kinect = SimpleNamespace(default_configuration=SimpleNamespace(),
            initialize_libraries=lambda: None, start_device=lambda **kwargs: device,
            K4A_IMAGE_FORMAT_COLOR_MJPG=1, K4A_COLOR_RESOLUTION_1080P=1,
            K4A_DEPTH_MODE_WFOV_2X2BINNED=1, K4A_FRAMES_PER_SECOND_30=1,
            K4A_WIRED_SYNC_MODE_STANDALONE=1)
        modules = {"cv2": SimpleNamespace(imwrite=write_image, putText=put_text,
                                          rectangle=lambda *args: None,
                                          FONT_HERSHEY_SIMPLEX=0, LINE_AA=16),
                   "torch": SimpleNamespace(set_num_threads=lambda n: None),
                   "pykinect_azure": kinect,
                   "ultralytics": SimpleNamespace(YOLO=lambda path: SimpleNamespace(
                       names=names, predict=lambda **kwargs: [results.pop(0)]))}
        requests = [{"kind": "scan", "labels": [], "point_id": "p",
                     "deadline": float("inf"), "image_path": "selected.jpg"}, {"kind": "close"}]
        connection = SimpleNamespace(send=messages.append, recv=lambda: requests.pop(0), close=lambda: None)
        cfg = dict(self.cfg["model"], labels=list(names.values()), frames=len(frames))
        with patch.dict(sys.modules, modules):
            _worker(connection, cfg, "unit-only-model")
        return messages, writes

    def test_evidence_keeps_earlier_selected_frame_with_matching_timestamp(self):
        messages, writes = self.camera_evidence_run([[(0, 0.95)], [(1, 0.7)], []])
        scan = next(m for m in messages if m["kind"] == "scan_result")
        self.assertEqual(scan["target"]["label"], "cola_bottle")
        self.assertEqual(scan["target"]["frame"], 1)
        self.assertEqual(len(writes), 1)
        self.assertEqual(writes[0][1]["frame"], 1)
        self.assertEqual(writes[0][1]["timestamp"], "UTC " + scan["target"]["capture_wall_utc"])
        self.assertEqual(scan["target"]["image_path"], scan["image_path"])

    def test_empty_scan_does_not_save_unrelated_image_as_detection_evidence(self):
        messages, writes = self.camera_evidence_run([[], [], []])
        scan = next(m for m in messages if m["kind"] == "scan_result")
        self.assertIsNone(scan["target"])
        self.assertIsNone(scan["image_path"])
        self.assertEqual(writes, [])

    def test_failed_evidence_write_is_reported_as_error(self):
        messages, writes = self.camera_evidence_run([[(0, 0.95)]], write_ok=False)
        self.assertFalse(any(m["kind"] == "scan_result" for m in messages))
        self.assertEqual(messages[-1]["kind"], "error")
        self.assertIn("无法保存检测图像", messages[-1]["message"])

    def initial_pose_run(self, wait_s=5.0, connections=None, stop_at=None, deadline=60,
                         shutdown_at=None, pose=None, frozen_ros_time=False, error=None):
        """测试 jujia26 迁移后的发布方法，不连接定位节点。"""
        import importlib
        clock, gate, published = Clock(), DeadlineGate(), []
        gate.clock, gate.deadline = clock, deadline
        def message():
            return SimpleNamespace(header=SimpleNamespace(), pose=SimpleNamespace(
                pose=SimpleNamespace(position=SimpleNamespace(), orientation=SimpleNamespace()), covariance=[]))
        rospy = SimpleNamespace(Publisher=lambda *args, **kwargs: SimpleNamespace(
            get_num_connections=lambda: connections(clock()) if connections else 1,
            publish=lambda msg: published.append(copy.deepcopy(msg))),
            Time=SimpleNamespace(now=lambda: 0 if frozen_ros_time else clock()),
            is_shutdown=lambda: shutdown_at is not None and clock() >= shutdown_at,
            loginfo=lambda text: None)
        def sleep(seconds):
            clock.advance(seconds)
            if stop_at is not None and clock() >= stop_at:
                gate.stopped = True
        old = sys.modules.pop("navigation.initial_pose", None)
        try:
            with patch.dict(sys.modules, {"rospy": rospy,
                 "geometry_msgs.msg": SimpleNamespace(PoseWithCovarianceStamped=message)}):
                module = importlib.import_module("navigation.initial_pose")
                controller = module.Controller()
                controller.initial_pose, controller.deadline_gate = pose or self.pose, gate
                controller.initial_pose_wait_s = wait_s
                with patch("time.sleep", side_effect=sleep):
                    if error:
                        with self.assertRaises(error):
                            controller.publish_initial_pose()
                    else:
                        controller.publish_initial_pose()
        finally:
            sys.modules.pop("navigation.initial_pose", None)
            if old is not None:
                sys.modules["navigation.initial_pose"] = old
        return clock, published

    def test_initial_pose_method_keeps_map_covariance_and_two_publishes(self):
        clock, published = self.initial_pose_run()
        self.assertEqual(len(published), 2)
        self.assertEqual(published[0].header.frame_id, "map")
        self.assertEqual(published[0].pose.pose.position.z, 0)
        self.assertEqual(published[0].pose.covariance[0], 0.25)
        self.assertEqual(published[0].pose.covariance[7], 0.25)
        self.assertEqual(published[0].pose.covariance[35], 0.0685)
        self.assertGreaterEqual(published[0].header.stamp, 5.0)
        self.assertGreaterEqual(published[1].header.stamp - published[0].header.stamp, 0.5)

    def test_initial_pose_uses_user_map_pose_not_jujia_fixed_coordinates(self):
        pose = [[1.234, -2.345, 0], [0, 0, 0.6, 0.8]]
        clock, published = self.initial_pose_run(wait_s=0, pose=pose)
        self.assertEqual(published[0].pose.pose.position.x, pose[0][0])
        self.assertEqual(published[0].pose.pose.position.y, pose[0][1])
        self.assertEqual(published[0].pose.pose.orientation.z, pose[1][2])
        self.assertEqual(published[0].pose.pose.orientation.w, pose[1][3])
        self.assertEqual(published[0].header.stamp, 0)

    def test_stop_during_initial_pose_wait_prevents_publishing(self):
        clock, published = self.initial_pose_run(stop_at=1, error=OperationStopped)
        self.assertEqual(published, [])
        self.assertLess(clock(), 1.1)

    def test_initial_pose_wait_obeys_total_prepare_deadline(self):
        clock, published = self.initial_pose_run(deadline=0.2, error=OperationStopped)
        self.assertEqual(published, [])
        self.assertLess(clock(), 0.3)

    def test_initial_pose_subscriber_lost_during_wait_is_not_reported_sent(self):
        clock, published = self.initial_pose_run(connections=lambda now: int(now < 1), error=RuntimeError)
        self.assertEqual(published, [])

    def test_initial_pose_subscriber_lost_between_publishes_prevents_second_request(self):
        clock, published = self.initial_pose_run(connections=lambda now: int(now < 5.1), error=RuntimeError)
        self.assertEqual(len(published), 1)

    def test_initial_pose_wait_does_not_depend_on_advancing_ros_time(self):
        clock, published = self.initial_pose_run(frozen_ros_time=True)
        self.assertEqual(len(published), 2)
        self.assertGreaterEqual(clock(), 5.5)
        self.assertLess(clock(), 5.6)

    def test_initial_pose_ros_shutdown_prevents_publishing(self):
        clock, published = self.initial_pose_run(shutdown_at=1, error=RuntimeError)
        self.assertEqual(published, [])

    def test_test_prepare_orders_initialization_before_localization_without_camera_arm_imports(self):
        order, clock = [], Clock()
        cfg = copy.deepcopy(self.cfg)
        cfg["initial_pose"] = self.pose
        session = RobotSession(SimpleNamespace(values=cfg), "unit", test_mode=True, set_initial_pose=True)
        session.clock = session.gate.clock = clock
        session.evidence = Evidence(Path("/unused"), "unit", cfg, clock, test_mode=True, save_logs=False)
        class Base:
            def __init__(self):
                order.append("base")
                self.pub = True
            def stop(self):
                order.append("zero")
        class Navigator:
            def __init__(self, location):
                order.append("navigator")
            def stop(self):
                pass
        class Speaker:
            def __init__(self):
                order.append("speech")
                self.tts_pub = SimpleNamespace(get_num_connections=lambda: 1, unregister=lambda: None)
        class Controller:
            def publish_initial_pose(self):
                order.append("initial_pose")
                self.initial_pose_pub = SimpleNamespace(unregister=lambda: None)
                if self.initial_pose_wait_s != 5.0:
                    raise AssertionError("没有传入 jujia26 的默认等待")
        @contextmanager
        def bounded(gate):
            gate.check()
            yield
            gate.check()
        rospy = SimpleNamespace(init_node=lambda *args, **kwargs: None, is_shutdown=lambda: False,
                                Subscriber=lambda *args, **kwargs: SimpleNamespace(unregister=lambda: None))
        modules = {"rospy": rospy, "std_msgs.msg": SimpleNamespace(Bool=object, String=object),
                   "actionlib_msgs.msg": SimpleNamespace(GoalStatus=object),
                   "navigation.base_controller": SimpleNamespace(Base=Base),
                   "navigation.navigator": SimpleNamespace(Navigator=Navigator),
                   "speech.summer_tts_speaker": SimpleNamespace(SummerTTSSpeaker=Speaker),
                   "navigation.initial_pose": SimpleNamespace(Controller=Controller)}
        original_import = __import__
        def restricted_import(name, *args, **kwargs):
            if name in ("arm.catch", "torch", "ultralytics", "camera.vision",
                        "pyrealsense2", "pykinect_azure", "kinova_msgs", "speech_2026"):
                raise AssertionError("导航测试不能导入硬件/推理模块：" + name)
            return original_import(name, *args, **kwargs)
        terminal = io.StringIO("OK\n")
        terminal.isatty = lambda: True
        try:
            with patch.dict(sys.modules, modules), patch("core.session.interruptible", bounded), \
                 patch("builtins.__import__", side_effect=restricted_import), patch("sys.stdin", terminal), \
                 patch("select.select", return_value=([terminal], [], [])), patch("sys.stdout", new=io.StringIO()):
                session._prepare()
            self.assertEqual(order, ["base", "navigator", "speech", "zero", "initial_pose"])
            self.assertTrue(any(e["event"] == "localization_manually_confirmed"
                                for e in session.evidence.record["events"]))
            self.assertEqual(session.evidence.record["navigation_points_completed"], [])
        finally:
            with patch("sys.stderr", new=io.StringIO()):
                session.close()

    def test_public_test_entry_runs_the_real_orchestrator_with_device_stubs(self):
        from main import main
        clock, route, spoken, initialization = Clock(), [], [], []
        cfg = self.filled_cfg()
        cfg["initial_pose"] = copy.deepcopy(self.pose)
        class Base:
            def __init__(self):
                initialization.append("base")
                self.pub = True
                self.pose_sub = SimpleNamespace(unregister=lambda: None)
            def stop(self):
                pass
        class Navigator:
            def __init__(self, location):
                initialization.append("navigator")
                self.client = SimpleNamespace(send_goal=lambda goal: route.append(goal),
                    get_state=lambda: 3)
            def set_goal(self, frame_id, position, orientation):
                return SimpleNamespace(target_pose=SimpleNamespace(
                    header=SimpleNamespace(frame_id=frame_id)), position=position, orientation=orientation)
            def stop(self):
                pass
        class Speaker:
            def __init__(self):
                initialization.append("speech")
                self.tts_pub = SimpleNamespace(get_num_connections=lambda: 1, unregister=lambda: None)
            def speak(self, text):
                spoken.append(text)
        class Controller:
            def publish_initial_pose(self):
                initialization.append("initial_pose")
        status = SimpleNamespace(SUCCEEDED=3, PENDING=0, ACTIVE=1, PREEMPTING=6, RECALLING=7)
        rospy = SimpleNamespace(init_node=lambda *args, **kwargs: None, is_shutdown=lambda: False,
            Subscriber=lambda *args, **kwargs: SimpleNamespace(unregister=lambda: None),
            Time=SimpleNamespace(now=lambda: 0))
        modules = {"rospy": rospy, "std_msgs.msg": SimpleNamespace(Bool=object, String=object),
                   "actionlib_msgs.msg": SimpleNamespace(GoalStatus=status),
                   "navigation.base_controller": SimpleNamespace(Base=Base),
                   "navigation.navigator": SimpleNamespace(Navigator=Navigator),
                   "speech.summer_tts_speaker": SimpleNamespace(SummerTTSSpeaker=Speaker),
                   "navigation.initial_pose": SimpleNamespace(Controller=Controller)}
        terminal = io.StringIO("OK\n")
        terminal.isatty = lambda: True
        with tempfile.TemporaryDirectory() as folder:
            cfg["evidence_dir"] = str(Path(folder) / "runs")
            path = Path(folder) / "config.json"
            path.write_text(json.dumps(cfg), encoding="utf-8")
            with patch.dict(sys.modules, modules), patch("time.monotonic", clock), \
                 patch("time.sleep", side_effect=clock.advance), patch("sys.stdin", terminal), \
                 patch("select.select", return_value=([terminal], [], [])), \
                 patch("sys.stdout", new=io.StringIO()):
                code = main(["--config", str(path), "--real", "--match-id", "test_entry",
                             "--set-initial-pose", "--no-log"], test_mode=True)
            self.assertEqual(code, 0)
            self.assertEqual(len(route), 8)
            self.assertEqual(len(spoken), 6)
            self.assertEqual(initialization, ["base", "navigator", "speech", "initial_pose"])
            self.assertTrue((Path(folder) / "runs/started-test_entry.json").exists())
            self.assertFalse(any(p.is_dir() for p in (Path(folder) / "runs").iterdir()))
            self.assertEqual(json.loads(path.read_text())["start"]["mode"], "topic")

    def test_dry_public_entry_never_imports_hardware(self):
        from main import main
        cfg = self.filled_cfg()
        original_import = __import__
        def reject_devices(name, *args, **kwargs):
            if name in ("rospy", "core.session", "torch", "ultralytics", "arm.catch"):
                raise AssertionError("只读配置检查不能导入硬件：" + name)
            return original_import(name, *args, **kwargs)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "config.json"
            path.write_text(json.dumps(cfg), encoding="utf-8")
            with patch("builtins.__import__", side_effect=reject_devices), patch("sys.stdout", new=io.StringIO()):
                self.assertEqual(main(["--config", str(path), "--check-config"], test_mode=True), 0)

    def test_initial_pose_without_subscriber_fails_bounded_and_never_publishes(self):
        clock, published = self.initial_pose_run(connections=lambda now: 0, error=RuntimeError)
        self.assertLess(clock(), 5.2)
        self.assertEqual(published, [])

    def speaker_run(self, done_messages=(), connections=1, stop_at=None, deadline=60, error=None):
        """测试上传的语音类和真实等待循环；ROS/TTS 均为替身。"""
        import importlib
        clock, gate, published, callbacks = Clock(), DeadlineGate(), [], []
        gate.clock, gate.deadline = clock, deadline
        def publish(msg):
            published.append(msg.data)
            for text, latched in done_messages:
                callbacks[0](SimpleNamespace(data=text,
                    _connection_header={"latching": "1" if latched else "0"}))
        rospy = SimpleNamespace(Publisher=lambda *args, **kwargs: SimpleNamespace(
            get_num_connections=lambda: connections, publish=publish),
            Subscriber=lambda topic, cls, callback: callbacks.append(callback) or SimpleNamespace(unregister=lambda: None),
            is_shutdown=lambda: False, loginfo=lambda *args: None,
            logwarn=lambda *args: None, logerr=lambda *args: None)
        def sleep(seconds):
            clock.advance(seconds)
            if stop_at is not None and clock() >= stop_at:
                gate.stopped = True
        old = sys.modules.pop("speech.summer_tts_speaker", None)
        try:
            with patch.dict(sys.modules, {"rospy": rospy, "std_msgs.msg": SimpleNamespace(String=SimpleNamespace)}), \
                 patch("time.monotonic", clock), patch("time.sleep", side_effect=sleep):
                module = importlib.import_module("speech.summer_tts_speaker")
                speaker = module.SummerTTSSpeaker.__new__(module.SummerTTSSpeaker)
                speaker.deadline_gate = gate
                module.SummerTTSSpeaker.__init__(speaker)
                if error:
                    with self.assertRaises(error):
                        speaker.speak("当前播报", wait_done=True, timeout=0.2)
                    result = None
                else:
                    result = speaker.speak("当前播报", wait_done=True, timeout=0.2)
            return result, clock, published, speaker
        finally:
            sys.modules.pop("speech.summer_tts_speaker", None)
            if old is not None:
                sys.modules["speech.summer_tts_speaker"] = old

    def test_actual_speaker_accepts_matching_nonlatched_done(self):
        result, clock, published, speaker = self.speaker_run(done_messages=[("当前播报", False)])
        self.assertTrue(result)
        self.assertEqual(published, ["当前播报"])
        self.assertIsNone(speaker._waiting_text)

    def test_actual_speaker_ignores_different_text_done(self):
        result, clock, published, speaker = self.speaker_run(done_messages=[("其他播报", False)])
        self.assertFalse(result)
        self.assertLess(clock(), 0.3)
        self.assertIsNone(speaker._waiting_text)

    def test_actual_speaker_ignores_latched_historical_done(self):
        result, clock, published, speaker = self.speaker_run(done_messages=[("当前播报", True)])
        self.assertFalse(result)
        self.assertEqual(published, ["当前播报"])

    def test_actual_speaker_does_not_publish_without_tts_subscriber(self):
        result, clock, published, speaker = self.speaker_run(connections=0)
        self.assertFalse(result)
        self.assertEqual(published, [])
        self.assertLess(clock(), 5.1)

    def test_actual_speaker_stop_during_done_wait_clears_waiting_state(self):
        result, clock, published, speaker = self.speaker_run(stop_at=0.1, error=OperationStopped)
        self.assertEqual(published, ["当前播报"])
        self.assertIsNone(speaker._waiting_text)
        self.assertLess(clock(), 0.2)

    def test_actual_speaker_done_wait_cannot_overrun_work_deadline(self):
        result, clock, published, speaker = self.speaker_run(deadline=0.1, error=OperationStopped)
        self.assertEqual(published, ["当前播报"])
        self.assertLess(clock(), 0.2)
        self.assertIsNone(speaker._waiting_text)


if __name__ == "__main__":
    unittest.main(verbosity=2)
