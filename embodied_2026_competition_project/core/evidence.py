# -*- coding: utf-8 -*-
"""记录接口调用，不把模拟动作算作真实抓取、交付；支持不落盘。"""
import json
from datetime import datetime, timezone


class Evidence:
    def __init__(self, run_dir, match_id, config, clock, test_mode=False,
                 speech_print=False, save_logs=True, writer=None):
        self.run_dir, self.clock, self.save_logs = run_dir, clock, save_logs
        self.save_images = bool(save_logs and not test_mode and config["recording"]["save_images"])
        self.started = None
        self.writer = writer
        self.result_path = None
        self.record_timeout_s = config["timing"]["record_timeout_s"]
        self.record = {"profile": config["profile"], "match_id": match_id, "status": "PREPARING",
                       "run_mode": "navigation_test" if test_mode else "main",
                       "hardware_navigation": True, "hardware_vision": not test_mode,
                       "speech_mode": "print" if speech_print else "topic_request",
                       "action_mode": "print" if test_mode else config["actions"]["mode"], "physical_grasps": 0, "physical_deliveries": 0,
                       "has_confirmed_delivery": False, "confirmed_object_ids": [], "delivered_object_ids": [],
                       "grasp_interface_calls": 0, "place_interface_calls": 0,
                       "speech_requests": 0, "speech_done_requests": 0,
                       "speech_completed_confirmed": False,
                       "detection_points_completed": [], "detection_points_skipped": [],
                       "navigation_points_completed": [],
                       "delivery_actions_completed": 0, "zone_sensor_verified_deliveries": 0,
                       "events": [], "recording_warnings": []}
        self.event_path = run_dir / "events.jsonl"
        try:
            if self.save_images:
                (run_dir/'images').mkdir(parents=True,exist_ok=True)
            if save_logs:
                if writer:
                    writer.write(run_dir/'config_used.json',config,self.record_timeout_s)
                else:
                    run_dir.mkdir(parents=True,exist_ok=True)
                    (run_dir/'config_used.json').write_text(json.dumps(config,ensure_ascii=False,indent=2)+'\n')
        except OSError as exc:
            self.record['recording_warnings'].append(str(exc))
            self.save_images = False

    def event(self, event_name, **details):
        # event_name 不与 navigation/stage 的 name=... 冲突。
        event = {"event": event_name, "wall_utc": datetime.now(timezone.utc).isoformat(),
                 "elapsed_s": None if self.started is None else round(self.clock() - self.started, 3)}
        event.update(details)
        self.record["events"].append(event)
        if self.save_logs:
            try:
                text = json.dumps(event,ensure_ascii=False)+'\n'
                if self.writer:
                    self.writer.append(self.event_path,text)
                else:
                    with self.event_path.open('a',encoding='utf-8') as stream:
                        stream.write(text)
            except OSError as exc:
                self.record['recording_warnings'].append(str(exc))

    def finish(self, status, exit_code):
        self.record["status"], self.record["exit_code"] = status, exit_code
        self.record["elapsed_s"] = None if self.started is None else round(self.clock() - self.started, 3)
        if not self.save_logs:
            return
        target = self.run_dir / "result.json"
        if self.writer:
            if self.writer.write(target,self.record,self.record_timeout_s):
                self.result_path = self.writer.saved_paths.get(str(target),target)
            return
        temp = self.run_dir / "result.json.tmp"
        temp.write_text(json.dumps(self.record, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        temp.replace(target)
        self.result_path = target
