# -*- coding: utf-8 -*-
"""main资源生命周期、同场恢复和计时；设备调用与日志保存均有等待上限。"""
# 资源生命周期：打开场次/恢复 -> 准备设备 -> 执行任务 -> 保存状态/关闭。
# 所有任务共用 DeadlineGate；monotonic 计时不依赖 ROS 时间是否暂停或跳变。
# kinova 模式在此准备 Actions、腕部相机和实际三指反馈，关闭时逐一取消/释放。
from contextlib import contextmanager
from pathlib import Path
from datetime import datetime, timezone
import json
from queue import SimpleQueue
import re
import signal
import sys
import time
from .evidence import Evidence
from .checkpoint import boot_id, config_digest, read_checkpoint
from .recording import RecordWriter, fallback_directory
from .faults import DeviceUnavailable, ParameterFault
from .safety import DeadlineGate, OperationStopped, interruptible, cancel_unfinished_goal


class SessionRejected(RuntimeError):
    pass


class NavigationFailed(RuntimeError):
    """现有 move_base 的终态结果；只封装状态信息，不增加导航能力。"""
    def __init__(self, name, state, reason=''):
        self.name, self.state, self.reason = name, state, reason
        super().__init__("导航失败：{}，move_base 状态={}，说明={}".format(name, state, reason or '未提供'))


class RobotSession:
    def __init__(self, config, match_id, test_mode=False, speech_print=False,
                 save_logs=True, set_initial_pose=False, localization_confirmed=False, resume=False, launched=None):
        self.config, self.match_id = config, match_id
        self.test_mode, self.speech_print, self.save_logs = test_mode, speech_print, save_logs
        self.set_initial_pose, self.localization_confirmed = set_initial_pose, localization_confirmed
        self.clock = time.monotonic
        self.resume = resume
        self.launched = self.clock() if launched is None else launched
        self.restored = None
        self.scheduler = None
        self.entry_completed = False
        self.initial_nav_completed = False
        self.last_put_receipt = None
        self.gate = DeadlineGate(clock=self.clock)
        self.gate.stop_messages = SimpleQueue()
        self.base = self.navigator = self.vision = self.speaker = self.localizer = self.rospy = None
        self.stop_sub = self.lock_file = self.evidence = None
        self.old_handlers = {}
        self.status, self.exit_code = "ERROR", 1
        self.work_deadline = self.match_deadline = float('inf')
        self.exiting = False
        self.actions = None
        self.started = None
        self.task_started = False
        self.drop_arrived = False
        self.writer = RecordWriter(fallback_directory(config.root),self._record_warning)
        self._record_warnings = set()

    def _record_warning(self, message):
        if message in self._record_warnings:
            return
        self._record_warnings.add(message)
        print('[记录提醒] '+message, file=sys.stderr,flush=True)
        if self.evidence is not None:
            self.evidence.record['recording_warnings'].append(message)

    def _write_record(self, path, data):
        return self.writer.write(path,data,self.config.values['timing']['record_timeout_s'])

    def __enter__(self):
        try:
            record_gate = DeadlineGate(clock=self.clock)
            record_gate.deadline = self.clock()+self.config.values['timing']['prepare_timeout_s']
            try:
                with interruptible(record_gate):
                    self._open_record()
            except OperationStopped as exc:
                raise SessionRejected('运行记录准备超时，目录：'+str(self.config.evidence_path)) from exc
            self.check()
            # 设备准备分模块限时；同场恢复仍受原比赛总截止约束。
            with self.gate.suspend_exit():
                self._prepare()
                self._start_task_clock()
            self.checkpoint()
            return self
        except BaseException:
            self.close()
            raise

    def _open_record(self):
        import fcntl
        self.match_id = re.sub(r'[^A-Za-z0-9_-]', '_', str(self.match_id))[:64] or 'match'
        parent = self.config.evidence_path
        # 所有同项目实例共用本机临时锁，磁盘满后换记录目录也不能启动第二个控制流程。
        lock_root = fallback_directory(self.config.root)
        lock_root.mkdir(parents=True,exist_ok=True)
        self.lock_file = (lock_root/'.navigation_session.lock').open('a+')
        try:
            fcntl.flock(self.lock_file.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise SessionRejected('同一记录目录已有运行中的控制流程') from exc
        try:
            parent.mkdir(parents=True,exist_ok=True)
        except OSError as exc:
            self._record_warning('默认记录目录不可写，使用备用目录：'+str(exc))
            parent = lock_root
            self.config.evidence_path = parent
        self.marker = parent/('started-'+self.match_id+'.json')
        self.checkpoint_path = parent/('checkpoint-'+self.match_id+'.json')
        if self.resume:
            try:
                candidates = [self.checkpoint_path,lock_root/self.checkpoint_path.name]
                readable = []
                for candidate in dict.fromkeys(candidates):
                    if candidate.is_file():
                        readable.append((candidate.stat().st_mtime_ns,candidate))
                if not readable:
                    raise OSError('未找到checkpoint-'+self.match_id+'.json')
                # 默认写入失败后备用文件可能更新；优先读最新的完整快照。
                self.checkpoint_path = max(readable,key=lambda item:item[0])[1]
                self.restored = read_checkpoint(self.checkpoint_path,self.config.values, strict=False)
            except (OSError,ValueError,KeyError,TypeError) as exc:
                raise SessionRejected('无法恢复：'+str(exc)) from exc
            self.task_started = self.restored.get('task_started', True)
            if self.task_started and self.restored.get('boot_id') != boot_id():
                raise SessionRejected('电脑已重启，不能可靠沿用本场计时；不允许用--resume重置600秒')
            self.started = self.restored.get('started') if self.task_started else None
            run_dir = Path(self.restored['run_dir'])
            self.entry_completed = self.restored['entry_completed']
            self.initial_nav_completed = self.restored.get('initial_nav_completed', self.entry_completed)
            self.last_put_receipt = self.restored.get('last_put_receipt')
            self.drop_arrived = self.restored.get('drop_arrived',False)
        else:
            stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
            run_dir = parent/(self.match_id+'-'+stamp)
        # 新任务按设备分别限时；同一已开始场次的重连继续占用原比赛时间。
        if self.started is not None:
            timing = self.config.values['timing']
            self.match_deadline = self.started + timing['match_s']
            self.work_deadline = self.match_deadline - timing['exit_reserve_s']
            self.gate.deadline = self.gate.match_deadline = self.match_deadline
            if self.clock()>=self.match_deadline:
                raise SessionRejected('该场已经到比赛截止时间；不重连设备、不重置计时')
        else:
            self.gate.deadline = float('inf')
        self.evidence = Evidence(run_dir,self.match_id,self.config.values,self.clock,
            test_mode=self.test_mode,speech_print=self.speech_print,save_logs=self.save_logs,writer=self.writer)
        self.evidence.started = self.started
        if self.restored:
            self.evidence.record.update(self.restored['record'])
            self.evidence.record['events'] = []
            self.evidence.event('resumed', task_clock_preserved=self.started is not None)
        else:
            self._write_record(self.marker,{'match_id':self.match_id,'started':self.started,'run_dir':str(run_dir)})
            self.evidence.event('preparing', timing_basis='after_device_initialization')
        self._write_record(parent/'latest_match.json',{'match_id':self.match_id})
        self.checkpoint()

    def _start_task_clock(self):
        """设备准备完成后开始新任务；已开始任务的同机恢复保留原截止时间。"""
        if self.started is None:
            self.started = self.clock()
        self.task_started = True
        timing = self.config.values['timing']
        self.match_deadline = self.started + timing['match_s']
        self.work_deadline = self.match_deadline - timing['exit_reserve_s']
        self.gate.deadline = self.gate.match_deadline = self.match_deadline
        # 默认45秒投放+20秒离场=65秒；若配置更大离场预留则取更大值。
        reserve = max(timing['exit_reserve_s'],self.config.values['actions']['place_timeout_s']
                      +timing['exit_estimate_s'])
        self.gate.exit_deadline = self.match_deadline-reserve
        self.evidence.started = self.started
        self._write_record(self.marker, {'match_id':self.match_id,'started':self.started,
                                'run_dir':str(self.evidence.run_dir)})
        self.evidence.event('task_clock_started', match_s=timing['match_s'],
                            remaining_s=max(0, self.match_deadline-self.clock()))
        self.check()

    def checkpoint(self):
        """动作前后均保存；持物未知时也保存，恢复不得把未知当成空手。"""
        if self.evidence is None:
            return
        # 快照/序列化本身的异常只影响恢复记录，不抢断正在进行的比赛。
        # 此计时独立于动作超时，超时后也能尝试保存最后状态。
        state_gate = DeadlineGate(clock=self.clock)
        state_gate.deadline = self.clock()+self.config.values['timing']['state_timeout_s']
        try:
            with interruptible(state_gate):
                return self._save_checkpoint()
        except Exception as exc:
            self._record_warning('本次恢复快照未保存，继续任务：{}: {}'.format(type(exc).__name__,exc))
            return False

    def _save_checkpoint(self):
        action_state = self.actions.snapshot() if self.actions is not None else (
            self.restored.get('actions') if self.restored else None)
        data = {'schema_version':1,'boot_id':boot_id(),'config_digest':config_digest(self.config.values),
                'match_id':self.match_id,'started':self.started,'task_started':self.task_started,
                'run_dir':str(self.evidence.run_dir),
                'entry_completed':self.entry_completed,'initial_nav_completed':self.initial_nav_completed,
                'actions':action_state,
                'last_put_receipt':self.last_put_receipt,'drop_arrived':self.drop_arrived,
                'scheduler':self.scheduler.snapshot() if self.scheduler else (
                    self.restored.get('scheduler') if self.restored else None),
                'record':{k:v for k,v in self.evidence.record.items() if k!='events'},
                'finished':self.exit_code==0 and self.status in ('FLOW_COMPLETED','EXITED_EARLY','TIME_UP')}
        data["record"]["status"] = self.status
        data["record"]["exit_code"] = self.exit_code
        data["checkpoint_wall_ns"] = time.time_ns()
        return self._write_record(self.checkpoint_path,data)

    def _prepare_call(self, module, action):
        timeout = self.config.values['timing']['prepare_timeout_s']
        try:
            with self.operation(timeout),interruptible(self.gate):
                action()
        except (OperationStopped, DeviceUnavailable, ParameterFault):
            raise
        except Exception as exc:
            from camera.vision import CameraFailed
            if isinstance(exc,CameraFailed) and not exc.device_related:
                raise ParameterFault('model / actions.profiles.*.model',
                                     '设备准备阶段模型/依赖加载失败：'+str(exc)) from exc
            raise DeviceUnavailable(module,'初始化失败：'+str(exc)) from exc

    def _prepare(self):
        """导航→自动张爪/收臂及腕部→头部→语音；每个模块都有准备上限。"""
        for sig in (signal.SIGINT,signal.SIGTERM):
            self.old_handlers[sig] = signal.signal(sig,signal.default_int_handler)
        import rospy
        from std_msgs.msg import Bool,String
        from actionlib_msgs.msg import GoalStatus
        self.rospy,self.String,self.GoalStatus = rospy,String,GoalStatus
        self._prepare_call('ROS',lambda:rospy.init_node('embodied_2026_flow',anonymous=False,disable_signals=True))
        self.stop_sub = rospy.Subscriber(self.config.values['stop_topic'],Bool,self.gate.stop_messages.put,queue_size=10)
        from navigation.base_controller import Base
        from navigation.navigator import Navigator
        def prepare_navigation():
            self.base = Base.__new__(Base)
            self.base.deadline_gate = self.gate
            Base.__init__(self.base)
            self.navigator = Navigator.__new__(Navigator)
            self.navigator.deadline_gate = self.gate
            self.navigator.server_timeout_s = self.config.values['recovery']['nav_server_timeout_s']
            Navigator.__init__(self.navigator,location={})
        self._prepare_call('导航',prepare_navigation)
        print('[初始化] 导航连接完成',flush=True)
        if not self.test_mode:
            from arm.actions import Actions
            from camera.vision import Vision
            self.actions = Actions(self)
            self._prepare_call('机械臂及腕部相机',self.actions.prepare)
            print('[初始化] 机械臂自动复位和腕部相机准备完成',flush=True)
            self.vision = Vision(self.config,self.evidence)
            self._prepare_call('头部相机',lambda:self.vision.start(self.gate))
            print('[初始化] 头部相机和模型加载完成',flush=True)
        else:
            self.evidence.event('devices_simulated',physical_action=False)
        if self.speech_print:
            print('[初始化] 语音使用打印模式',flush=True)
        else:
            from speech.summer_tts_speaker import SummerTTSSpeaker
            self.speaker = SummerTTSSpeaker.__new__(SummerTTSSpeaker)
            self.speaker.deadline_gate = self.gate
            self.speaker.subscriber_timeout_s = self.config.values['speech']['subscriber_timeout_s']
            self._prepare_call('语音',lambda:SummerTTSSpeaker.__init__(self.speaker))
            from .recovery import Recovery
            Recovery(self,lambda *args,**kwargs:None).probe('语音')
            print('[初始化] 已连接语音播报节点',flush=True)
        if self.set_initial_pose and not self.resume:
            from navigation.initial_pose import Controller
            self.localizer = Controller.__new__(Controller)
            self.localizer.initial_pose = self.config.values['initial_pose']
            self.localizer.initial_pose_wait_s = self.config.values['localization']['initial_pose_wait_s']
            self.localizer.publish_interval_s = self.config.values['localization']['publish_interval_s']
            self.localizer.deadline_gate = self.gate
            self.base.stop()
            self._prepare_call('初始定位发布',lambda:Controller.publish_initial_pose(self.localizer))
            self.evidence.event('initial_pose_requested',pose=self.localizer.initial_pose)
        self.check()
        self.evidence.event('devices_initialized',localization_confirmation_required=False)
        print('全部初始化完成，开始执行任务',flush=True)

    def check(self):
        self.gate.check()
        if self.rospy is not None and self.rospy.is_shutdown():
            self.gate.stopped = True
            raise DeviceUnavailable("ROS","节点已停止或连接已关闭")

    @contextmanager
    def operation(self, timeout):
        # 当前阶段的上限与整场截止共同约束操作，超时不会自动放宽到无限等待。
        deadline = self.match_deadline
        with self.gate.limit(deadline, timeout):
            self.check()
            yield
            self.check()

    def refresh_navigation(self, reason):
        """清除旧动态代价；服务结束后由Workflow等待激光重新更新。"""
        self.stop_navigation()
        with self.operation(self.config.values['recovery']['costmap_timeout_s']), interruptible(self.gate):
            self.navigator.clear_costmap_client()
        self.evidence.event('costmaps_refreshed', reason=reason)
        print('[导航] 已清理旧代价地图，等待传感器更新', flush=True)

    def navigate(self, name, pose):
        # pose 使用 map 坐标。只有 SUCCEEDED(3) 视为到达；终态失败交回 Workflow 决定处理。
        timeout = self.config.values["timing"]["nav_timeout_s"]
        if pose is None:
            raise ParameterFault('locations / detection_points.*.pose',name+'地图点位尚未填写')
        try:
            with self.operation(timeout),interruptible(self.gate):
                if self.actions is not None:
                    self.actions.require_travel_ready()
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
                        reason = self.navigator.client.get_goal_status_text()
                        self.evidence.event('navigation_failed', name=name, state=state, reason=reason)
                        raise NavigationFailed(name, state, reason)
                    time.sleep(0.05)
        except BaseException:
            self.stop_navigation()
            raise

    def stop_navigation(self):
        # 按当前客户端的 GoalID 取消旧目标，并复用原 Base.stop 请求零速。
        # Navigator.stop 的 cancel_all_goals 没有目标 ID：若取消消息晚于
        # 下一条 /goal 到达服务器，会误取消刚发送的离场目标（状态 2）。
        if self.navigator is not None and hasattr(self.navigator, "client"):
            try:
                cancel_unfinished_goal(self.navigator.client)
            except Exception:
                pass
        if self.base is not None and hasattr(self.base, "pub"):
            try:
                self.base.stop()
            except Exception:
                pass

    def begin_exit(self, reason):
        # 允许持物离场；收臂由Workflow先恢复，离场只受整场总截止约束。
        self.stop_navigation()
        if self.vision is not None:
            self.vision.close()
            self.vision = None
        self.exiting = True
        self.gate.deadline = self.match_deadline
        self.check()
        if self.actions is not None:
            self.actions.require_travel_ready()
            self.evidence.record['exit_with_object'] = bool(self.actions.holding)
        self.evidence.event("exit_started", reason=reason)
        print("[离场] 停止搜索与抓放，前往离场点：" + reason, flush=True)

    def close(self):
        # 成功、失败、Ctrl+C 和准备异常都走此处；取消请求/零速请求不等于实测刹停反馈。
        if self.evidence is not None:
            try:
                self.checkpoint()
            except Exception as exc:
                print("恢复状态保存失败："+str(exc),file=sys.stderr)
        self.gate.stopped = True
        cleanup_errors = []

        def finish_one(name, action):
            try:
                cleanup = DeadlineGate(clock=self.clock)
                cleanup.deadline = self.clock()+self.config.values['timing']['cleanup_timeout_s']
                with interruptible(cleanup):
                    action()
            except Exception as exc:
                cleanup_errors.append(name + ": " + str(exc))

        finish_one('取消导航及请求零速',self.stop_navigation)

        if self.actions is not None:
            if self.evidence is not None:
                self.evidence.record['holding_at_stop'] = self.actions.holding
                self.evidence.record['load_state_at_stop'] = self.actions.load_state
                self.evidence.record['arm_travel_ready_at_stop'] = self.actions.travel_ready
            finish_one('关闭抓放资源', self.actions.close)
        if self.vision is not None:
            finish_one("关闭视觉进程", self.vision.close)
            self.vision = None
        subscribers = [self.stop_sub]
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
