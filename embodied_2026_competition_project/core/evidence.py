# -*- coding: utf-8 -*-
"""记录接口调用，不把模拟动作算作真实抓取、交付；支持不落盘。"""
import json
from datetime import datetime, timezone


class Evidence:
    def __init__(self, run_dir, match_id, config, clock, test_mode=False,
                 speech_print=False, save_logs=True):
        self.run_dir, self.clock, self.save_logs = run_dir, clock, save_logs
        self.save_images = bool(save_logs and not test_mode and config["recording"]["save_images"])
        self.started = None
        self.record = {"profile": "navigation_flow", "match_id": match_id, "status": "PREPARING",
                       "run_mode": "navigation_test" if test_mode else "main",
                       "hardware_navigation": True, "hardware_vision": not test_mode,
                       "speech_mode": "print" if speech_print else "topic_request",
                       "action_mode": "print", "physical_grasps": 0, "physical_deliveries": 0,
                       "autonomous_exit_qualified": False,
                       "grasp_interface_calls": 0, "place_interface_calls": 0,
                       "speech_requests": 0, "speech_done_requests": 0,
                       "speech_completed_confirmed": False,
                       "detection_points_completed": [], "detection_points_skipped": [],
                       "navigation_points_completed": [],
                       "events": []}
        self.event_path = run_dir / "events.jsonl"
        if self.save_images:
            (run_dir / "images").mkdir()
        if save_logs:
            (run_dir / "config_used.json").write_text(
                json.dumps(config, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    def event(self, event_name, **details):
        # event_name 不与 navigation/stage 的 name=... 冲突。
        event = {"event": event_name, "wall_utc": datetime.now(timezone.utc).isoformat(),
                 "elapsed_s": None if self.started is None else round(self.clock() - self.started, 3)}
        event.update(details)
        self.record["events"].append(event)
        if self.save_logs:
            with self.event_path.open("a", encoding="utf-8") as f:
                f.write(json.dumps(event, ensure_ascii=False) + "\n")

    def finish(self, status, exit_code):
        self.record["status"], self.record["exit_code"] = status, exit_code
        self.record["elapsed_s"] = None if self.started is None else round(self.clock() - self.started, 3)
        if not self.save_logs:
            return
        target = self.run_dir / "result.json"
        temp = self.run_dir / "result.json.tmp"
        temp.write_text(json.dumps(self.record, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        temp.replace(target)
