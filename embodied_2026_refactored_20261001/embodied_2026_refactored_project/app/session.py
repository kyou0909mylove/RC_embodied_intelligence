# -*- coding: utf-8 -*-
"""设备会话、开始信号与统一清理。ROS/SDK 导入只在实机准备方法中发生。"""
import json
import os
import queue
import signal
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from .context import MATCH_SECONDS, RunContext
from .evidence import EvidenceWriter


class SessionRejected(Exception):
    """场次或平台条件不满足；设备尚未初始化，入口返回 2。"""


class SessionAborted(Exception):
    """准备阶段失败且已完成清理；入口读取会话退出码。"""


class RobotSession:
    """保存部分初始化对象，任一准备/运行错误都统一请求停止。"""

    def __init__(self, config):
        """只保存数据；构造本对象不连接设备。"""
        self.config = config
        self.args = config.args
        self.context = RunContext()
        self.session_lock = None
        self.evidence = None
        self.gate = None
        self.closed = False
        self.signals_installed = False
        self.old_handlers = {}
        self.navigator = self.base = self.kinova = self.camera = None
        self.planner = self.converter = self.detector = self.ground_detector = self.speaker = None
        self.rospy = self.cv2 = None
        self.start_sub = self.stop_sub = None
        self._initialize_record()

    def __enter__(self):
        """取得场次锁后准备设备、等待开始；准备中断也会收尾。"""
        try:
            self._open_record_directory()
            self._setup_gate()
            self._install_signals()
            self.prepare()
            self.wait_start()
        except SessionRejected:
            if self.session_lock is not None:
                self.session_lock.close()
            raise
        except BaseException as exc:
            self.finish(exc)
            if isinstance(exc, (Exception, KeyboardInterrupt)):
                raise SessionAborted() from exc
            raise
        return self

    def __exit__(self, exc_type, exc, traceback):
        """运行阶段退出后统一清理；普通错误转换为原退出码 1。"""
        self.finish(exc)
        return isinstance(exc, (Exception, KeyboardInterrupt))

    def _initialize_record(self):
        """建立原数量台账与记录，deliveries/inventory 保持共享引用。"""
        ctx = self.context
        cfg = self.config.values
        args = self.args
        catalog = self.config.catalog
        capabilities = self.config.capabilities
        strategy = self.config.strategy
        bound_catalog_ids = self.config.bound_catalog_ids
        slots = self.config.slots
        ctx.state, ctx.holding = "PREPARE", "EMPTY"
        ctx.started = ctx.deadline = None
        ctx.task = ctx.area = ctx.slot = None
        ctx.target_map = None
        ctx.result_code = 0
        ctx.deliveries = []
        # 每个清单ID独立统计：expected计划数，delivered已视觉确认数，grasp_attempts抓取次数，pregrasp_failures抓取前失败次数。
        ctx.inventory = {t["id"]: {"expected": t["expected_count"], "delivered": 0,
                     "grasp_attempts": 0, "pregrasp_failures": 0,
                     "status": "PENDING" if t["id"] in bound_catalog_ids else
                               ("UNSUPPORTED" if t["status"] == "unsupported" else "NOT_ENABLED")}
                     for t in catalog}
        # record是本地运行证据：保存观察、尝试、事件、库存和退出状态；official_score保持None，不能冒充裁判得分。
        ctx.record = {"match_id": args.match_id, "deliveries": ctx.deliveries, "observations": [],
                  "attempts": [], "events": [], "exit_completed": False,
                  "rule_source": "用户提供 PDF，第12–14页", "official_score": None,
                  "task_catalog": catalog, "capabilities": capabilities, "inventory": ctx.inventory,
                  "strategy": strategy, "slot_capacity": len(slots)}
        ctx.journal_cursor = 0

    def _open_record_directory(self):
        """平台、场次锁和旧局标记检查先于设备初始化。"""
        cfg = self.config.values
        args = self.args
        config_path = self.config.path
        if not sys.platform.startswith("linux"):
            print("实机流程需要 Linux / ROS 1。", file=sys.stderr)
            raise SessionRejected()
        if signal.getitimer(signal.ITIMER_REAL)[0] > 0:
            print("当前进程已有活动定时器，不能覆盖其截止时间。", file=sys.stderr)
            raise SessionRejected()

        # 【08 实机场次保护】从此进入实际运行路径；同目录同时只能有一个实机进程，同局不能重置计时。
        import fcntl
        evidence = Path(cfg["evidence_dir"]).expanduser()
        if not evidence.is_absolute():
            evidence = config_path.parent / evidence
        evidence = evidence.resolve()
        evidence.mkdir(parents=True, exist_ok=True)
        # 在原构造器可能开爪之前拒绝同局重启；不提供伪造的恢复剩余时间功能。
        self.marker = evidence / ("match_" + args.match_id + ".json")
        self.session_lock = (evidence / "robot_session.lock").open("a+")
        try:
            fcntl.flock(self.session_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            self.session_lock.close()
            print("该记录目录已有实机进程运行。", file=sys.stderr)
            raise SessionRejected()
        if self.marker.exists():
            self.session_lock.close()
            print("此 match-id 已启动过；禁止重新初始化夹爪或重置本局计时。", file=sys.stderr)
            raise SessionRejected()
        run_dir = evidence / (args.match_id + "_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ"))
        run_dir.mkdir()
        (run_dir / "config_used.json").write_text(json.dumps(cfg, ensure_ascii=False, indent=2), encoding="utf-8")
        self.evidence = EvidenceWriter(run_dir)

    def _setup_gate(self):
        """沿用原 DeadlineGate 与消息队列，检查实际模块来源。"""
        root = self.config.root
        sys.path[:0] = [str(root), str(root / "legacy/common"), str(root / "legacy/tongyong_25")]
        # 【截止门控】两者来自support/safety.py；统一时间和停止检查，并包装原action客户端。
        from support.safety import DeadlineGate, GuardedClient
        if Path(sys.modules["support.safety"].__file__).resolve() != (root / "support/safety.py").resolve():
            self.session_lock.close()
            print("截止门控模块来源错误", file=sys.stderr)
            raise SessionRejected()
        # gate是检查对象；stop_messages/start_messages是ROS回调放消息的队列，不直接在回调中做机器人动作。
        self.gate = DeadlineGate(clock=time.monotonic)
        self.stop_messages = queue.SimpleQueue()
        self.gate.stop_messages = self.stop_messages
        self.start_messages = queue.SimpleQueue()
        self.start_sub = self.stop_sub = None
        self.navigator = self.base = self.kinova = self.camera = None
        self.rospy = self.cv2 = None
        self.GuardedClient = GuardedClient

    def _install_signals(self):
        """准备与比赛使用同一组 Linux 主线程信号处理。"""
        # 逐个记录，安装中途被打断时也能恢复已经修改的处理器。
        for sig in (signal.SIGALRM, signal.SIGTERM, signal.SIGINT):
            self.old_handlers[sig] = signal.signal(sig, signal.default_int_handler)
        self.signals_installed = True

    def prepare(self):
        """准备阶段会执行原开爪/回 home；首次运动前仍注入同一个 gate。"""
        cfg = self.config.values
        root = self.config.root
        targets = self.config.targets
        intrinsics = self.config.intrinsics
        signal.setitimer(signal.ITIMER_REAL, cfg["limits"]["prepare_s"])
        self.gate.deadline = time.monotonic() + cfg["limits"]["prepare_s"]
        import numpy as np
        import cv2
        self.cv2 = cv2
        import rospy
        self.rospy = rospy
        from actionlib_msgs.msg import GoalStatus
        self.GoalStatus = GoalStatus
        from kinova_msgs.msg import FingerPosition
        self.FingerPosition = FingerPosition
        from std_msgs.msg import String, Bool
        self.String, self.Bool = String, Bool
        # 导入导航、底盘、TF、检测、语音和机械臂的旧接口；具体文件在legacy子目录。
        from navigator2 import Navigator
        from base_controller import Base
        from position_last_second import SmartGoalFinder
        from camera_to_map import CoordinateConverter
        from goal_calculator import calculate_facing_goal
        self.calculate_facing_goal = calculate_facing_goal
        from summer_tts_speaker import SummerTTSSpeaker
        from catch_ground.src.catch import KinovaRobot
        from catch_ground.src.detector_items_c import KinectCamera, ItemsDetector
        from catch_ground.src.realsense_yolo11 import RealSenseYolo11Detector
        from catch_ground.src.realsense_yolo11_desk import RealSenseYolo11DetectorDesk

        # 防止环境中的同名旧模块抢先导入，明确本次运行的真实来源。
        for name, relative in (
            ("navigator2", "legacy/common/navigator2.py"),
            ("base_controller", "legacy/tongyong_25/base_controller.py"),
            ("position_last_second", "legacy/common/position_last_second.py"),
            ("camera_to_map", "legacy/common/camera_to_map.py"),
            ("goal_calculator", "legacy/common/goal_calculator.py"),
            ("summer_tts_speaker", "legacy/common/summer_tts_speaker.py"),
            ("catch_ground.src.catch", "legacy/tongyong_25/catch_ground/src/catch.py"),
            ("catch_ground.src.detector_items_c", "legacy/tongyong_25/catch_ground/src/detector_items_c.py"),
            ("catch_ground.src.realsense_yolo11", "legacy/tongyong_25/catch_ground/src/realsense_yolo11.py"),
            ("catch_ground.src.realsense_yolo11_desk", "legacy/tongyong_25/catch_ground/src/realsense_yolo11_desk.py"),
        ):
            if Path(sys.modules[name].__file__).resolve() != (root / relative).resolve():
                raise RuntimeError("同名模块来源错误：" + name)
        self.rospy.init_node("embodied_2026_main", anonymous=False, disable_signals=True)
        self.stop_sub = self.rospy.Subscriber(cfg["stop_topic"], self.Bool, self.stop_messages.put, queue_size=1)
        self.gate.check()
        # 尽早保存底盘发布器；后续模型/设备准备失败也能请求零速度。
        self.base = Base.__new__(Base)
        Base.__init__(self.base)
        self.gate.check()
        # 加载真实搜索模型后核对精确标签；配置文件里写了标签不代表模型真的能识别。
        self.detector = ItemsDetector(model_path=cfg["search_weights"])
        self.gate.check()
        names = self.detector.model.names
        search_names = set(names.values() if isinstance(names, dict) else names)
        for t in targets:
            if t["search_label"] not in search_names:
                raise ValueError("搜索模型没有精确标签：" + t["search_label"])
        for surface in ("table", "table_short"):
            selected = [t for t in targets if t["surface"] == surface]
            if selected:
                weights = cfg[surface + "_weights"]
                check = RealSenseYolo11DetectorDesk(weights=Path(weights))
                self.gate.check()
                names = check.model.names
                labels = set(names.values() if isinstance(names, dict) else names)
                if any(t["grasp_label"] not in labels for t in selected):
                    raise ValueError("桌面模型标签与 grasp_label 不一致：" + surface)
                del check
        self.ground_detector = None
        if any(t["surface"] == "ground" for t in targets):
            self.ground_detector = RealSenseYolo11Detector(weights=Path(cfg["ground_weights"]), conf_thres=0.5)
            names = self.ground_detector.model.names
            labels = set(names.values() if isinstance(names, dict) else names)
            if any(t["grasp_label"] not in labels for t in targets if t["surface"] == "ground"):
                raise ValueError("地面模型标签与 grasp_label 不一致")
        self.speaker = SummerTTSSpeaker()
        if self.speaker.tts_pub.get_num_connections() == 0:
            raise RuntimeError("TTS 节点未就绪")
        # 保存部分初始化对象，构造过程中被中断也能取消已建立的 action 客户端。
        # __new__先保存未初始化对象，再执行原__init__；这样初始化中断时还能尝试取消已建立的客户端。
        self.navigator = Navigator.__new__(Navigator)
        self.navigator.deadline_gate = self.gate
        Navigator.__init__(self.navigator, cfg["locations"])
        self.planner, self.converter = SmartGoalFinder(), CoordinateConverter()
        if self.planner.get_robot_pose() is None or self.rospy.is_shutdown():
            raise RuntimeError("定位或 TF 未就绪")
        self.kinova = KinovaRobot.__new__(KinovaRobot)
        # 把同一个门控对象注入旧类实例；原__init__参数签名不变，初始化动作也要先查停止/截止。
        self.kinova.deadline_gate = self.gate
        self.gate.check()
        KinovaRobot.__init__(self.kinova, "j2n6s300")
        # 两个既有 catch_table 方法保留签名和动作顺序，通过实例属性读取部署权重。
        self.kinova.table_weights = cfg.get("table_weights")
        self.kinova.table_short_weights = cfg.get("table_short_weights")
        self.gate.check()
        if (self.rospy.is_shutdown() or self.kinova.client_arm.get_state() != self.GoalStatus.SUCCEEDED
                or self.kinova.client_finger.get_state() != self.GoalStatus.SUCCEEDED):
            raise RuntimeError("原机械臂初始化失败")
        self.camera = KinectCamera()
        self.camera.K = np.array(intrinsics, dtype=float).reshape(3, 3)
        self.camera.open_camera()
        self.gate.check()
        # Kinova 构造器从 client 建立时即门控；初始化动作与比赛动作使用同一包装。
        self.navigator.client = self.GuardedClient(self.navigator.client, self.gate, self.rospy.Duration)
        signal.setitimer(signal.ITIMER_REAL, 0)
        self.gate.deadline = float("inf")

    def wait_start(self):
        """忽略锁存开始消息；本次有效信号固定 started+600 并写标记。"""
        ctx = self.context
        start = self.config.start
        args = self.args
        ctx.state = "WAIT_START"
        self.start_sub = self.rospy.Subscriber(start["topic"], self.String if start["type"] == "String" else self.Bool,
                                     self.start_messages.put, queue_size=1)
        self.speaker.speak("准备完成，等待比赛开始。")
        while not self.rospy.is_shutdown():
            self.gate.check()
            try:
                msg = self.start_messages.get(timeout=0.1)
            except queue.Empty:
                continue
            self.gate.check()
            if str(getattr(msg, "_connection_header", {}).get("latching", "0")) == "1":
                ctx.record["events"].append({"event": "ignored_latched_start"})
                continue
            data = msg.data.strip() if isinstance(msg.data, str) else msg.data
            if data != start["value"]:
                continue
            # started为收到本次有效开始消息时的单调时钟；deadline固定为started+600。
            ctx.started = time.monotonic()
            ctx.deadline = ctx.started + MATCH_SECONDS
            self.gate.deadline = ctx.deadline
            ctx.record["start_wall_utc"] = datetime.now(timezone.utc).isoformat()
            ctx.record["events"].append({"event": "started", "wall_utc": ctx.record["start_wall_utc"]})
            with self.marker.open("x", encoding="utf-8") as f:
                json.dump({"match_id": args.match_id, "start_wall_utc": ctx.record["start_wall_utc"],
                           "run_dir": str(self.evidence.run_dir), "resume_supported": False}, f, ensure_ascii=False)
                f.flush()
                os.fsync(f.fileno())
            break
        if ctx.started is None:
            raise RuntimeError("ROS 在启动信号到达前退出")

    def _stop_resources(self):
        """独立取消各客户端、发零速度、关相机；不新增回 home 或松爪动作。"""
        ctx = self.context
        signal.setitimer(signal.ITIMER_REAL, 0)
        # 【13 最终收尾】最外层finally负责取消导航/机械臂/夹爪、发零速度、关相机、保存记录和释放锁。
        # 这里不会回home或开爪，因为停止后的新动作可能造成再次运动或掉物。
        self.gate.stopped = True
        # 独立取消三类客户端；某个停止接口异常不会跳过其余停止请求。
        stop_calls = []
        if self.navigator is not None and hasattr(self.navigator, "client"):
            stop_calls.append(self.navigator.stop)
        if self.kinova is not None:
            for client_name in ("client_arm", "client_finger"):
                if hasattr(self.kinova, client_name):
                    stop_calls.append(getattr(self.kinova, client_name).cancel_all_goals)
        if self.base is not None and hasattr(self.base, "pub") and hasattr(self.base, "twist"):
            stop_calls.extend([self.base.stop] * 5)
        for stop_call in stop_calls:
            try:
                stop_call()
            except Exception as exc:
                ctx.record["events"].append({"event": "stop_failed", "error": str(exc)})
        # 停止后不回 home、不打开夹爪；这些都是新动作，可能让手中物体掉落。
        if self.camera is not None and hasattr(self.camera, "device"):
            try:
                self.camera.release()
            except Exception as exc:
                ctx.record["events"].append({"event": "camera_release_failed", "error": str(exc)})
        if self.cv2 is not None:
            try:
                self.cv2.destroyAllWindows()
            except Exception:
                pass
        for subscriber in (self.start_sub, self.stop_sub):
            if subscriber is not None:
                try:
                    subscriber.unregister()
                except Exception:
                    pass

    def _shutdown(self):
        """关闭 ROS，恢复信号并释放场次锁。"""
        ctx = self.context
        try:
            if self.rospy is not None:
                self.rospy.signal_shutdown("2026 主流程结束")
        except Exception as exc:
            print("ROS 关闭失败：" + str(exc), file=sys.stderr)
            ctx.result_code = 1
        finally:
            try:
                for sig, handler in self.old_handlers.items():
                    try:
                        signal.signal(sig, handler)
                    except Exception as exc:
                        ctx.record["events"].append({"event": "signal_restore_failed", "error": str(exc)})
                        ctx.result_code = 1
            finally:
                if self.session_lock is not None:
                    self.session_lock.close()

    def finish(self, error=None):
        """记录原异常分类并清理一次，最终退出码与台账含义保持。"""
        if self.closed:
            return
        self.closed = True
        if self.old_handlers:
            # 进入清理后立即关闭本会话定时器，避免清理过程中再次触发 SIGALRM。
            signal.setitimer(signal.ITIMER_REAL, 0)
        ctx = self.context
        if isinstance(error, KeyboardInterrupt):
            ctx.record["events"].append({"event": "interrupt_or_deadline", "state": ctx.state})
            ctx.result_code = 1
        elif isinstance(error, Exception):
            ctx.record["events"].append({"event": "stopped", "state": ctx.state,
                                         "error": type(error).__name__ + ": " + str(error)})
            print("流程停止：" + str(error), file=sys.stderr)
            ctx.result_code = 1
        if self.gate is not None:
            self._stop_resources()
        if self.evidence is not None:
            self.evidence.finalize(ctx, self.config.slots)
        self._shutdown()
        print("已复核交付={}，自主离场={}；本地记录不等于裁判计分。".format(
            len(ctx.deliveries), ctx.record["exit_completed"]))
