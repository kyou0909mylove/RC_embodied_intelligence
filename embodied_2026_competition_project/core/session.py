# -*- coding: utf-8 -*-
"""现有导航/语音资源管理；测试只模拟相机、机械臂，不导入其 SDK。"""
from contextlib import contextmanager
from datetime import datetime, timezone
import json
from queue import SimpleQueue
import re
import select
import signal
import sys
import time
from .evidence import Evidence
from .safety import DeadlineGate, OperationStopped, interruptible


class SessionRejected(RuntimeError):
    pass


class NavigationFailed(RuntimeError):
    """现有 move_base 的终态结果；只封装状态信息，不增加导航能力。"""
    def __init__(self, name, state):
        self.name, self.state = name, state
        super().__init__("导航失败：{}，move_base 状态={}".format(name, state))


class RobotSession:
    def __init__(self, config, match_id, test_mode=False, speech_print=False,
                 save_logs=True, set_initial_pose=False, localization_confirmed=False):
        self.config, self.match_id = config, match_id
        self.test_mode, self.speech_print, self.save_logs = test_mode, speech_print, save_logs
        self.set_initial_pose, self.localization_confirmed = set_initial_pose, localization_confirmed
        self.clock = time.monotonic
        self.gate = DeadlineGate(clock=self.clock)
        self.gate.stop_messages = SimpleQueue()
        self.base = self.navigator = self.vision = self.speaker = self.localizer = self.rospy = None
        self.stop_sub = self.start_sub = self.lock_file = self.evidence = None
        self.old_handlers = {}
        self.status, self.exit_code = "ERROR", 1
        self.work_deadline = self.match_deadline = None
        self.exiting = False

    def __enter__(self):
        try:
            self._open_record()
            self._prepare()
            self._wait_start()
            return self
        except BaseException:
            self.close()
            raise

    def _open_record(self):
        import fcntl
        if not isinstance(self.match_id, str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,64}", self.match_id):
            raise SessionRejected("--real 需要 --match-id，使用字母、数字、下划线或连字符")
        parent = self.config.evidence_path
        parent.mkdir(parents=True, exist_ok=True)
        self.lock_file = (parent / ".navigation_session.lock").open("a+")
        try:
            fcntl.flock(self.lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise SessionRejected("同一记录目录已有运行中的流程；同一底盘只允许一个控制入口") from exc
        self.marker = parent / ("started-" + self.match_id + ".json")
        if self.marker.exists():
            raise SessionRejected("这个 match-id 已启动过；下一次测试使用新编号，不重置同场计时")
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
        run_dir = parent / (self.match_id + "-" + stamp)
        if self.save_logs:
            run_dir.mkdir()
        self.evidence = Evidence(run_dir, self.match_id, self.config.values, self.clock,
                                 test_mode=self.test_mode, speech_print=self.speech_print,
                                 save_logs=self.save_logs)

    def _prepare(self):
        for sig in (signal.SIGINT, signal.SIGTERM):
            self.old_handlers[sig] = signal.signal(sig, signal.default_int_handler)
        self.gate.deadline = self.clock() + self.config.values["timing"]["prepare_timeout_s"]
        with interruptible(self.gate):
            import rospy
            from std_msgs.msg import Bool, String
            from actionlib_msgs.msg import GoalStatus
            from navigation.base_controller import Base
            from navigation.navigator import Navigator
            self.rospy, self.String, self.GoalStatus = rospy, String, GoalStatus
            rospy.init_node("embodied_2026_flow", anonymous=False, disable_signals=True)
            self.stop_sub = rospy.Subscriber(self.config.values["stop_topic"], Bool,
                                             self.gate.stop_messages.put, queue_size=10)
            self.base = Base.__new__(Base)
            self.base.deadline_gate = self.gate
            Base.__init__(self.base)
            self.navigator = Navigator.__new__(Navigator)
            self.navigator.deadline_gate = self.gate
            Navigator.__init__(self.navigator, location={})
            self.check()
            print("[初始化] 底盘：已调用原 Base + Navigator，未发送移动目标", flush=True)
            print("[初始化] 机械臂：模拟完成；未构造 KinovaRobot，未开爪或移动", flush=True)
            self.evidence.event("arm_initialization_simulated", physical_action=False)
            if self.test_mode:
                print("[初始化] 相机：模拟完成；未打开相机，未导入 torch/ultralytics/相机 SDK", flush=True)
                self.evidence.event("camera_initialization_simulated", physical_action=False)
            else:
                from camera.vision import Vision
                self.vision = Vision(self.config, self.evidence)
                self.vision.start(self.gate)
                self.check()
                print("[初始化] 相机：真实 Kinect 和上传模型初始化完成", flush=True)
            if self.speech_print:
                print("[初始化] 语音：仅打印模式，不会出声", flush=True)
            else:
                from speech.summer_tts_speaker import SummerTTSSpeaker
                self.speaker = SummerTTSSpeaker.__new__(SummerTTSSpeaker)
                self.speaker.deadline_gate = self.gate
                SummerTTSSpeaker.__init__(self.speaker)
                self.check()
                if self.speaker.tts_pub.get_num_connections() == 0:
                    raise SessionRejected("语音无订阅者；启动已有 summer_tts_node，或明确使用 --speech-print")
                print("[初始化] 语音：已连接 /summer_tts_topic；是否真的出声仍需听音验证", flush=True)
            if self.set_initial_pose:
                from navigation.initial_pose import Controller
                self.base.stop()
                self.localizer = Controller.__new__(Controller)
                self.localizer.initial_pose = self.config.values.get("initial_pose")
                self.localizer.initial_pose_wait_s = self.config.values["localization"]["initial_pose_wait_s"]
                self.localizer.deadline_gate = self.gate
                Controller.publish_initial_pose(self.localizer)
                self.evidence.event("initial_pose_requested", pose=self.localizer.initial_pose,
                                    source="jujia26.Controller.publish_initial_pose",
                                    initial_pose_wait_s=self.localizer.initial_pose_wait_s,
                                    localization_converged_confirmed=False)
        self.check()
        # /initialpose 是定位先验，不是移动指令，也没有原始收敛回执接口。
        # 未重新发布时，显式 --localization-confirmed 可使用已人工校准的定位。
        if not self.localization_confirmed:
            if not sys.stdin.isatty():
                raise SessionRejected("需要交互终端确认定位；已人工校准时可用 --localization-confirmed")
            confirm_s = self.config.values["localization"]["confirmation_timeout_s"]
            print("[定位确认] 在 RViz 确认是打点时同一张 map，机器人位置/朝向正确，"
                  "激光与墙对齐且稳定。确认后输入 OK 回车；最多 {} 秒，且不超过剩余准备时间。".format(
                      confirm_s), flush=True)
            end = min(self.gate.deadline, self.clock() + confirm_s)
            while True:
                self.check()
                if self.clock() >= end:
                    raise SessionRejected("定位尚未人工确认；未发送入场导航目标")
                if select.select([sys.stdin], [], [], 0.05)[0]:
                    answer = sys.stdin.readline()
                    if not answer:
                        raise SessionRejected("确认输入已关闭；未发送入场导航目标")
                    if answer.strip().upper() == "OK":
                        break
                    print("只有确认对齐后输入 OK 才会继续；否则 Ctrl+C 停止。", flush=True)
        self.evidence.event("localization_manually_confirmed", automated_convergence_check=False)
        self.gate.deadline = float("inf")
        print("全部准备完成：真实导航；机械臂只打印；相机{}。".format(
            "只打印" if self.test_mode else "真实检测"), flush=True)

    def _wait_start(self):
        start = self.config.values["start"]
        if start["mode"] == "topic":
            messages = SimpleQueue()
            self.start_sub = self.rospy.Subscriber(start["topic"], self.String, messages.put, queue_size=10)
            print("等待开始信号：{} = {}".format(start["topic"], start["value"]), flush=True)
            while True:
                self.check()
                if not messages.empty():
                    message = messages.get_nowait()
                    latched = str(getattr(message, "_connection_header", {}).get("latching", "0")) == "1"
                    if not latched and message.data.strip() == start["value"]:
                        break
                time.sleep(0.05)
        else:
            print("{} 秒后开始计时并执行入场导航。".format(start["delay_s"]), flush=True)
            end = self.clock() + start["delay_s"]
            while self.clock() < end:
                self.check()
                time.sleep(min(0.05, max(0.0, end - self.clock())))
        self.check()
        started = self.clock()
        self.evidence.started = started
        timing = self.config.values["timing"]
        self.match_deadline = started + timing["match_s"]
        self.work_deadline = self.match_deadline - timing["exit_reserve_s"]
        self.gate.deadline = self.work_deadline
        with self.marker.open("x", encoding="utf-8") as f:
            json.dump({"match_id": self.match_id, "match_s": 600, "exit_reserve_s": 30,
                       "run_dir": str(self.evidence.run_dir) if self.save_logs else None}, f, ensure_ascii=False)
        self.evidence.event("started", work_s=570, exit_reserve_s=30)

    def check(self):
        self.gate.check()
        if self.rospy is not None and self.rospy.is_shutdown():
            self.gate.stopped = True
            raise OperationStopped("ROS 已停止")

    @contextmanager
    def operation(self, timeout):
        deadline = self.match_deadline if self.exiting else self.work_deadline
        with self.gate.limit(deadline, timeout):
            self.check()
            yield
            self.check()

    def navigate(self, name, pose):
        timeout = self.config.values["timing"]["nav_timeout_s"]
        try:
            with self.operation(timeout):
                self.evidence.event("navigation_started", name=name, pose=pose)
                print("[导航] 前往 " + name, flush=True)
                goal = self.navigator.set_goal("map", pose[0], pose[1])
                goal.target_pose.header.stamp = self.rospy.Time.now()
                self.check()
                self.navigator.client.send_goal(goal)
                while True:
                    self.check()
                    state = self.navigator.client.get_state()
                    if state == self.GoalStatus.SUCCEEDED:
                        self.check()
                        self.base.stop()
                        self.evidence.event("navigation_succeeded", name=name)
                        self.evidence.record["navigation_points_completed"].append(name)
                        print("[导航] 已到达 " + name, flush=True)
                        return
                    if state not in (self.GoalStatus.PENDING, self.GoalStatus.ACTIVE,
                                     self.GoalStatus.PREEMPTING, self.GoalStatus.RECALLING):
                        raise NavigationFailed(name, state)
                    time.sleep(0.05)
        except BaseException:
            self.stop_navigation()
            raise

    def stop_navigation(self):
        # 复用原有取消/零速接口，不反复清除障碍物代价地图或无限重发目标。
        if self.navigator is not None and hasattr(self.navigator, "client"):
            try:
                self.navigator.stop()
            except Exception:
                pass
        if self.base is not None and hasattr(self.base, "pub"):
            try:
                self.base.stop()
            except Exception:
                pass

    def begin_exit(self, reason):
        self.stop_navigation()
        if self.vision is not None:
            self.vision.close()
            self.vision = None
        self.exiting = True
        self.gate.deadline = self.match_deadline
        self.check()
        self.evidence.event("exit_started", reason=reason)
        print("[离场] 停止搜索、语音和动作占位，前往离场点：" + reason, flush=True)

    def close(self):
        self.gate.stopped = True
        self.stop_navigation()
        cleanup_errors = []

        def finish_one(name, action):
            try:
                action()
            except Exception as exc:
                cleanup_errors.append(name + ": " + str(exc))

        if self.vision is not None:
            finish_one("关闭视觉进程", self.vision.close)
            self.vision = None
        subscribers = [self.stop_sub, self.start_sub]
        if self.base is not None:
            subscribers.append(getattr(self.base, "pose_sub", None))
        for subscriber in subscribers:
            if subscriber is not None:
                finish_one("关闭订阅", subscriber.unregister)
        if self.speaker is not None and hasattr(self.speaker, "tts_pub"):
            finish_one("关闭语音发布", self.speaker.tts_pub.unregister)
        if self.speaker is not None and hasattr(self.speaker, "tts_done_sub"):
            finish_one("关闭语音完成订阅", self.speaker.tts_done_sub.unregister)
        if self.localizer is not None and hasattr(self.localizer, "initial_pose_pub"):
            finish_one("关闭初始定位发布", self.localizer.initial_pose_pub.unregister)
        for sig, handler in self.old_handlers.items():
            finish_one("恢复信号处理", lambda sig=sig, handler=handler: signal.signal(sig, handler))
        self.old_handlers.clear()
        if self.evidence is not None:
            self.evidence.record["cleanup_errors"] = cleanup_errors
            finish_one("保存记录", lambda: self.evidence.finish(self.status, self.exit_code))
        if self.lock_file is not None:
            finish_one("释放场次锁", self.lock_file.close)
            self.lock_file = None
        if cleanup_errors:
            print("清理阶段信息：" + "; ".join(cleanup_errors), file=sys.stderr)

    def __exit__(self, exc_type, exc, traceback):
        try:
            if exc_type is not None:
                self.status = "STOPPED" if isinstance(exc, (KeyboardInterrupt, OperationStopped)) else "ERROR"
                if self.evidence is not None:
                    self.evidence.event("aborted", error=str(exc), error_type=exc_type.__name__)
        except Exception as record_error:
            print("异常记录保存失败：" + str(record_error), file=sys.stderr)
        finally:
            # 磁盘满等记录异常也不能阻止取消导航、零速和释放锁。
            self.close()
        return False
