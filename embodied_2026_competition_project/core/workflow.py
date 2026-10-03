# -*- coding: utf-8 -*-
"""main 与测试共用：每个检测点一轮检测/语音/抓取占位/投放，再到下一点。"""
import time
from arm import actions
from .safety import OperationStopped
from .session import NavigationFailed


class Workflow:
    def __init__(self, session):
        self.session = session
        self.cfg = session.config.values

    def _stage(self, name, **details):
        self.session.evidence.event("stage", name=name, **details)
        print("[阶段] " + name, flush=True)

    def _speak(self, text, kind, simulated_detection=None):
        """仅编排上传的 SummerTTSSpeaker；同步时等待其已有 done 接口。"""
        session = self.session
        speech = self.cfg["speech"]
        wait_s = speech["timeout_s"] if speech["wait_done"] and not session.speech_print else speech["pause_s"]
        with session.operation(wait_s + 2.0):
            if session.speech_print:
                print("[语音打印] " + text + "；未真实播报", flush=True)
            else:
                session.check()
                if session.speaker.tts_pub.get_num_connections() == 0:
                    raise RuntimeError("语音订阅者已断开；未继续后续流程")
            session.evidence.event("speech_requested", text=text, kind=kind,
                                   simulated_detection=simulated_detection,
                                   hardware_request=not session.speech_print,
                                   wait_done=speech["wait_done"], playback_confirmed=False)
            if not session.speech_print:
                session.evidence.record["speech_requests"] += 1
                session.evidence.record["speech_completed_confirmed"] = False
                if speech["wait_done"]:
                    done = session.speaker.speak(text, wait_done=True, timeout=speech["timeout_s"])
                    session.evidence.event("speech_completion", text=text, backend_done_confirmed=bool(done))
                    if not done:
                        raise RuntimeError("未收到匹配的 /summer_tts_done；停止后续流程，不伪造播报成功")
                    session.evidence.record["speech_done_requests"] += 1
                    session.evidence.record["speech_completed_confirmed"] = (
                        session.evidence.record["speech_done_requests"] == session.evidence.record["speech_requests"])
                    return
                session.speaker.speak(text)
            # 异步兼容/打印模式的等待只是演练节奏，不是播报完成反馈。
            end = session.clock() + speech["pause_s"]
            while session.clock() < end:
                session.check()
                time.sleep(min(0.05, max(0.0, end - session.clock())))

    def _one_point(self, point):
        session, timing = self.session, self.cfg["timing"]
        self._stage("DETECT_NAV", point_id=point["id"], region=point["region"])
        try:
            session.navigate(point["name"], point["pose"])
        except NavigationFailed as exc:
            # 只在检测点前、未进行抓取时，允许明确 ABORTED(4)/REJECTED(5) 跳过。
            # 导航超时、抢占/取消、丢失、外部停止均不在这里恢复。
            if not self.cfg["flow"]["skip_unreachable_detection"] or exc.state not in (4, 5):
                raise
            session.check()
            session.evidence.record["detection_points_skipped"].append(point["id"])
            session.evidence.event("detection_point_skipped", point_id=point["id"],
                                   navigation_state=exc.state, physical_action=False)
            print("[跳过检测点] {}：move_base 状态={}；没有检测、抓取或投放，继续其他点".format(
                point["name"], exc.state), flush=True)
            return
        self._stage("DETECT", point_id=point["id"])
        with session.operation(timing["scan_timeout_s"]):
            if session.test_mode:
                # 仅构造显式模拟输入，不调用/伪造新的相机技能函数。
                label = point.get("test_label") or (
                    point.get("target_labels", []) or self.cfg["model"]["labels"])[0]
                target = {"label": label, "simulated": True, "source": "configured_test_input"}
                observation = {"point_id": point["id"], "target": target, "detections": [target],
                               "valid_frames": 0, "simulated": True, "image_path": None}
                print("[相机模拟] {}：模拟发现 {}；真实相机未启动".format(point["name"], label), flush=True)
            else:
                observation = session.vision.scan(point, session.gate)
                target = observation["target"]
        session.evidence.event("scan_completed", **observation)
        if target is None:
            print("[检测] 有效帧数={}，未识别到目标；跳过抓取和投放，不空抓".format(
                observation["valid_frames"]), flush=True)
            session.evidence.record["detection_points_completed"].append(point["id"])
            return
        if not session.test_mode:
            print("[真实检测] {}，置信度 {:.3f}；只选择本次最高置信度候选".format(
                target["label"], target["confidence"]), flush=True)
        self._stage("SPEAK", point_id=point["id"])
        label_text = self.cfg["speech"]["label_names"].get(target["label"], target["label"])
        text = ("模拟检测，" if session.test_mode else "相机检测，") + point["name"] + "发现" + label_text
        self._speak(text, kind="detection", simulated_detection=session.test_mode)
        self._stage("GRASP", point_id=point["id"])
        with session.operation(timing["grasp_estimate_s"] + 1):
            result = actions.grasp(target, point)
        session.evidence.record["grasp_interface_calls"] += 1
        session.evidence.event("grasp_interface_completed", **result)
        self._stage("DROP_NAV", point_id=point["id"])
        session.navigate("投放点", self.cfg["locations"]["drop"])
        self._stage("PLACE", point_id=point["id"])
        with session.operation(timing["place_estimate_s"] + 1):
            result = actions.place(target, self.cfg["locations"]["drop"])
        session.evidence.record["place_interface_calls"] += 1
        session.evidence.event("place_interface_completed", **result)
        session.evidence.record["detection_points_completed"].append(point["id"])

    def run(self):
        session, timing = self.session, self.cfg["timing"]
        reason = "检测点已按列表遍历完成"
        try:
            self._stage("ENTRY")
            session.navigate("入场点", self.cfg["locations"]["entry"])
            if self.cfg["speech"]["announce_stages"]:
                # 借鉴 jujia26.control/find_room 的阶段顺序，仍调用已有 SummerTTSSpeaker。
                self._stage("ENTRY_SPEAK")
                self._speak("已到达入场点", kind="stage")
                self._stage("SEARCH_SPEAK")
                self._speak("开始遍历物品检测点", kind="stage")
            for point in self.cfg["detection_points"]:
                session.check()
                needed = (2 * timing["nav_estimate_s"] + timing["scan_estimate_s"]
                          + (self.cfg["speech"]["timeout_s"] if self.cfg["speech"]["wait_done"]
                             and not session.speech_print else self.cfg["speech"]["pause_s"])
                          + timing["grasp_estimate_s"] + timing["place_estimate_s"])
                if session.work_deadline - session.clock() < needed:
                    reason = "剩余工作时间不足以完成下一轮"
                    break
                self._one_point(point)
            else:
                if not self.cfg["flow"]["exit_when_done"]:
                    self._stage("WAIT_EXIT")
                    while True:
                        session.check()
                        time.sleep(0.05)
        except OperationStopped:
            # 外部停止/ROS 关闭/阶段提前超时均停车退出，不冒险自动换目标。
            # 只有比赛工作截止（570秒）才转入离场导航。
            if session.gate.stopped or session.rospy.is_shutdown() or session.clock() < session.work_deadline:
                raise
            reason = "进入比赛最后 30 秒"
        # 正式规则 PDF 第14页：至少识别、抓取并送入己方得分区一件后才可离场。
        # 现有计数只接收真实交付证据；actions.py 的 print 不更新此计数。
        # 导航演练允许走完整路线，但模拟交付不能取得比赛离场资格。
        qualified = not session.test_mode and session.evidence.record["physical_deliveries"] >= 1
        session.evidence.record["autonomous_exit_qualified"] = qualified
        if not session.test_mode and not qualified:
            session.stop_navigation()
            session.evidence.event("exit_not_allowed", reason="尚无真实物品交付确认",
                                   physical_deliveries=session.evidence.record["physical_deliveries"])
            session.status, session.exit_code = "STOPPED_WITHOUT_DELIVERY", 1
            print("[停止] 尚未确认至少一件物品完成识别、抓取并放入己方得分区；"
                  "不发送离场目标。抓放打印不能替代真实交付。", flush=True)
            return
        if self.cfg["speech"]["announce_stages"]:
            # 提前完成且满足离场条件时才播报；到工作截止则直接停止工作。
            speech_budget = (self.cfg["speech"]["timeout_s"] if self.cfg["speech"]["wait_done"]
                             and not session.speech_print else self.cfg["speech"]["pause_s"])
            if session.work_deadline - session.clock() >= speech_budget + 2.0:
                try:
                    self._stage("EXIT_SPEAK")
                    self._speak("导航测试，开始离场演练" if session.test_mode else "开始自主离场", kind="stage")
                except OperationStopped:
                    if session.gate.stopped or session.rospy.is_shutdown() or session.clock() < session.work_deadline:
                        raise
                    reason = "进入比赛最后 30 秒"
            else:
                session.evidence.event("stage_speech_skipped", reason="工作截止前时间不足")
        if session.evidence.record["detection_points_skipped"] and reason == "检测点已按列表遍历完成":
            reason = "检测点已遍历，含明确不可达并跳过的点"
        session.begin_exit(reason)
        self._stage("EXIT")
        session.navigate("离场点", self.cfg["locations"]["exit"])
        session.evidence.event("exit_arrived", autonomous_exit_qualified=qualified,
                               navigation_test=session.test_mode)
        all_points_done = len(session.evidence.record["detection_points_completed"]) == len(self.cfg["detection_points"])
        session.status = "FLOW_COMPLETED" if all_points_done else "EXITED_EARLY"
        session.exit_code = 0
        print("[完成] 导航与接口流程结束；不等于真实抓取/交付成功，真实数量均为 0。", flush=True)
