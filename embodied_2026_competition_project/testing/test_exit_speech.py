"""离线回归：模拟接口，验证离场中断与播报并行；不连接机器人。"""
import copy
import json
import math
from pathlib import Path
from queue import Queue, SimpleQueue
import sys
import threading
import time
from types import ModuleType, SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from core.safety import DeadlineGate, ExitRequested, OperationStopped, ActionFailed, GuardedClient, interruptible
from core.session import RobotSession
from core.workflow import Workflow
from core.evidence import Evidence
from arm.actions import Actions

CFG = json.loads((ROOT / 'config/navigation.local.json').read_text())
POINT = next(p for p in CFG['detection_points'] if p.get('enabled', True))

# 实际语音 Python 接口；ROS 仅替换成可控制完成回执的本地桩。
ros = ModuleType('rospy')
ros.is_shutdown = lambda: False
ros.loginfo = ros.logwarn = ros.logerr = lambda *a, **kw: None
sys.modules['rospy'] = ros
msgs = ModuleType('std_msgs.msg')
class String:
    def __init__(self, data=''):
        self.data = data
msgs.String = String
sys.modules['std_msgs'] = ModuleType('std_msgs')
sys.modules['std_msgs.msg'] = msgs
from speech.summer_tts_speaker import SummerTTSSpeaker


def until(predicate, timeout=2):
    end = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() >= end:
            raise AssertionError('模拟接口未在测试窗口内响应')
        time.sleep(.005)


class Publisher:
    def __init__(self):
        self.messages = []
    def get_num_connections(self):
        return 1
    def publish(self, msg):
        self.messages.append(msg.data)


def speaker():
    result = SummerTTSSpeaker.__new__(SummerTTSSpeaker)
    result._last_done_text = result._waiting_text = None
    result._done_lock = threading.Lock()
    result._speak_lock = threading.Lock()
    result._done_event = threading.Event()
    result.tts_pub = Publisher()
    return result


class SimSession(RobotSession):
    def __init__(self, now=0):
        self.now = now
        self.clock = lambda: self.now
        self.config = SimpleNamespace(values=copy.deepcopy(CFG))
        self.gate = DeadlineGate(clock=self.clock)
        self.gate.stop_messages = SimpleQueue()
        self.match_deadline = self.gate.match_deadline = self.gate.deadline = 600
        self.work_deadline = self.gate.exit_deadline = 570
        self.exiting = self.speech_print = self.test_mode = False
        self.rospy = SimpleNamespace(is_shutdown=lambda: False)
        self.restored = self.actions = self.scheduler = self.last_put_receipt = None
        self.vision = self.base = self.navigator = None
        self.speaker = None
        self.started = 0
        self.task_started = True
        self.match_id = 'simulation'
        self.marker = ROOT / 'unused-marker.json'
        self.evidence = Evidence(ROOT / 'unused-run', 'simulation', CFG, self.clock, save_logs=False)
        self.evidence.started = 0
        self._record_warnings = set()
        self._speech_queue = Queue()
        self._speech_shutdown = threading.Event()
        self._speech_thread = self._speech_gate = None
        self.trace = []
        self.snapshots = 0
    def _save_checkpoint(self):
        self.snapshots += 1
        return True
    def _write_record(self, *args):
        return True
    def navigate(self, name, pose):
        self.check()
        if self.actions:
            self.actions.require_travel_ready()
        self.trace.append(('navigate', name, self.now))
        self.now += 4
        self.check()
    def stop_voice(self):
        self._speech_shutdown.set()
        if self._speech_gate:
            self._speech_gate.stopped = True
        if self._speech_thread:
            self._speech_thread.join(timeout=.3)


class Driver:
    def __init__(self, s, home_seconds=2):
        self.s, self.home_seconds = s, home_seconds
    def cancel_motion(self):
        self.s.trace.append(('cancel_arm', self.s.now))
    def move(self, pose, verify_pose=False, stage=None):
        self.s.check()
        self.s.trace.append(('arm_move', stage, self.s.now))
        self.s.now += self.home_seconds
        self.s.check()
    def fingers(self, values):
        raise AssertionError('离场流程不应发送张爪或补抓取目标')


def arm(s, home_seconds=2, holding=False):
    a = Actions(s)
    a.driver = Driver(s, home_seconds)
    a.pending_pick = {'attempt_id': 'obj', 'object_id': 'obj', 'label': 'water',
                      'point_id': POINT['id'], 'furniture_id': POINT['furniture_id'],
                      'target': {'label': 'water'}, 'grasp_phase': 'closing',
                      'retreat_poses': [], 'retreat_index': 0, 'grasp_confirmed': False}
    a._load_unknown = True
    if holding:
        a.holding = a.pending_pick
        a.pending_pick = None
        a._load_unknown = False
    s.actions = a
    return a


class GraspActions:
    def __init__(self):
        self.load_state = 'empty'
        self.holding = None
        self.started = threading.Event()
    def grasp(self, target, point):
        self.started.set()
        self.load_state = 'holding'
        self.holding = {'grasp_confirmed': True}
        return {'status': 'grasped'}


class Verification(unittest.TestCase):
    def test_soft_hard_deadlines_and_stop(self):
        s = SimSession(569.9)
        self.assertAlmostEqual(s.gate.check(), .1)
        s.now = 570
        with self.assertRaises(ExitRequested): s.check()
        with s.gate.suspend_exit(): self.assertEqual(s.gate.check(), 30)
        s.now = 600
        with self.assertRaises(OperationStopped) as raised: s.check()
        self.assertNotIsInstance(raised.exception, ExitRequested)
        s.now = 570
        s.gate.stop_messages.put(SimpleNamespace(data=True))
        with self.assertRaises(OperationStopped) as raised: s.check()
        self.assertNotIsInstance(raised.exception, ExitRequested)

    def test_resume_keeps_original_exit_deadline(self):
        s = SimSession(580)
        with s.gate.suspend_exit(): s._start_task_clock()
        self.assertEqual(s.started, 0)
        self.assertEqual(s.gate.exit_deadline, 570)
        with self.assertRaises(ExitRequested): s.check()

    def test_resume_enters_workflow_in_exit_window(self):
        s = SimSession(580)
        s._open_record = lambda: None
        s._prepare = lambda: None
        self.assertIs(s.__enter__(), s)
        w = Workflow(s)
        w._initialize_route = s.check
        w.run()
        self.assertEqual(s.trace, [('navigate', '离场点', 580)])

    def test_real_signal_interrupts_blocking_wait(self):
        gate = DeadlineGate()
        gate.match_deadline = gate.deadline = time.monotonic()+1
        gate.exit_deadline = time.monotonic()+.05
        start = time.monotonic()
        with self.assertRaises(ExitRequested):
            with interruptible(gate): time.sleep(.5)
        self.assertLess(time.monotonic()-start, .3)

    def test_snapshot_propagates_exit(self):
        s = SimSession(570)
        with self.assertRaises(ExitRequested): s.checkpoint()
        self.assertEqual(s.snapshots, 0)
        with s.gate.suspend_exit(): self.assertTrue(s.checkpoint())

    def test_snapshot_mid_save_interrupts_at_exit(self):
        s = SimSession()
        s.clock = time.monotonic
        s.gate.clock = s.clock
        s.match_deadline = s.gate.match_deadline = s.gate.deadline = s.clock()+1
        s.work_deadline = s.gate.exit_deadline = s.clock()+.05
        s._save_checkpoint = lambda: time.sleep(.5)
        with self.assertRaises(ExitRequested): s.checkpoint()

    def test_guarded_client_cancels_inflight_goal(self):
        s = SimSession(569.9)
        class Client:
            cancelled = 0
            def get_state(self): return 1
            def wait_for_result(self, duration):
                s.now = 570
                return False
            def cancel_goal(self): self.cancelled += 1
        client = Client()
        wrapped = GuardedClient(client, s.gate, lambda value: value, require_success=False)
        with self.assertRaises(ExitRequested): wrapped.wait_for_result()
        self.assertEqual(client.cancelled, 1)

    def test_head_scan_interrupt_routes_to_exit(self):
        s = SimSession(569)
        def scan(point, gate):
            s.now = 570
            gate.check()
        s.vision = SimpleNamespace(scan=scan, close=lambda: s.trace.append(('close_camera', s.now)))
        w = Workflow(s)
        w._initialize_route = lambda: None
        s.navigate = lambda name, pose: (s.check(), s.trace.append(('navigate', name, s.now)))
        w.run()
        self.assertIn(('navigate', '离场点', 570), s.trace)
        self.assertEqual(s.evidence.record['grasp_interface_calls'], 0)
        self.assertEqual(s.evidence.record['place_interface_calls'], 0)

    def test_exit_preserves_held_object_and_reserves_navigation(self):
        s = SimSession(570)
        a = arm(s, holding=True)
        original = a.holding
        with s.gate.suspend_exit(): Workflow(s)._exit('time')
        self.assertIs(a.holding, original)
        self.assertIn(('navigate', '离场点', 572), s.trace)
        self.assertEqual(s.status, 'FLOW_COMPLETED')
        stage = next(e for e in s.evidence.record['events'] if e.get('name') == 'EXIT_PREPARE')
        self.assertEqual(stage['timeout_s'], 10)

    def test_exit_home_failure_retries_until_deadline_without_driving(self):
        s = SimSession(570)
        a = arm(s)
        def fail_home(*args, **kwargs):
            s.now += 1
            raise ActionFailed('home',4,'模拟收臂失败')
        a.driver.move = fail_home
        w = Workflow(s)
        w._initialize_route = s.check
        def wait(seconds):
            s.now += seconds
            s.check()
        w._wait = wait
        w.run()
        self.assertEqual(s.status,'TIME_UP')
        self.assertFalse(a.travel_ready)
        self.assertFalse(any(t[0]=='navigate' for t in s.trace))
        self.assertGreater(sum(t[0]=='cancel_arm' for t in s.trace),1)

    def test_hard_timeout_has_no_new_exit_goal(self):
        s = SimSession(569)
        w = Workflow(s)
        w._initialize_route = lambda: None
        def current(point):
            s.now = 600
            s.check()
        w._one_point = current
        w.run()
        self.assertEqual(s.status, 'TIME_UP')
        self.assertEqual(s.trace, [])

    def test_grasp_interruption_routes_to_exit_without_retry(self):
        s = SimSession(569)
        a = arm(s)
        context = a.pending_pick
        a.pending_pick = None
        a._load_unknown = False
        a.travel_ready = True
        s.speech_print = True
        def grasp(*args):
            a.pending_pick = context
            a._load_unknown = True
            a.travel_ready = False
            s.now = 570
            s.check()
        a.grasp = grasp
        w = Workflow(s)
        w._initialize_route = lambda: None
        w._head_scan = lambda p: {'candidates': [{'label': 'water'}]}
        original_navigation = s.navigate
        s.navigate = lambda name, pose: (s.check() if name != '离场点'
                                        else original_navigation(name, pose))
        w.run()
        self.assertIn(('navigate', '离场点', 572), s.trace)
        self.assertEqual(a.load_state, 'unknown')
        self.assertFalse(any(e.get('name') in ('RETRY_POINT', 'RESET_POINT', 'PUT')
                             for e in s.evidence.record['events']))

    def test_navigation_signal_cancels_old_goal(self):
        s = SimSession()
        s.clock = time.monotonic
        s.gate.clock = s.clock
        s.match_deadline = s.gate.match_deadline = s.gate.deadline = s.clock()+1
        s.work_deadline = s.gate.exit_deadline = s.clock()+.05
        class Client:
            sent = cancelled = 0
            def send_goal(self, goal): self.sent += 1
            def get_state(self): return 1
            def cancel_goal(self): self.cancelled += 1
        client = Client()
        s.navigator = SimpleNamespace(client=client,
            set_goal=lambda *a: SimpleNamespace(target_pose=SimpleNamespace(header=SimpleNamespace())))
        s.rospy.Time = SimpleNamespace(now=lambda: 0)
        s.GoalStatus = SimpleNamespace(SUCCEEDED=3, PENDING=0, ACTIVE=1, PREEMPTING=6, RECALLING=7)
        with self.assertRaises(ExitRequested):
            RobotSession.navigate(s, '当前检测点', CFG['locations']['entry'])
        self.assertEqual(client.sent, 1)
        self.assertEqual(client.cancelled, 1)

    def test_put_interruption_skips_remaining_delivery(self):
        s = SimSession(569)
        a = arm(s, holding=True)
        a.travel_ready = True
        s.navigate = lambda *args: None
        def put(*args):
            s.now = 570
            s.check()
        a.put = put
        with self.assertRaises(ExitRequested): Workflow(s)._deliver_pending()
        self.assertEqual(s.evidence.record['place_interface_calls'], 0)

    def test_async_grasp_and_ordered_audio_completion(self):
        s = SimSession()
        s.speaker = speaker()
        s.actions = GraspActions()
        self.addCleanup(s.stop_voice)
        w = Workflow(s)
        w._head_scan = lambda p: {'candidates': [{'label': 'water'}]}
        self.assertEqual(w._inspect_point(POINT), ('grasped', True))
        self.assertTrue(s.actions.started.is_set())
        until(lambda: len(s.speaker.tts_pub.messages) == 1)
        self.assertEqual(s.evidence.record['speech_done_requests'], 0)
        self.assertTrue(s.queue_speech('第二句', 'detection', 1))
        self.assertTrue(s.queue_speech('第三句', 'detection', 1))
        # 不匹配回执不能推进下一句，机械臂阶段超时也不影响语音自己的门控。
        s.gate.deadline = -1
        s.speaker._done_callback(String('错误回执'))
        time.sleep(.06)
        self.assertEqual(len(s.speaker.tts_pub.messages), 1)
        s.speaker._done_callback(String(s.speaker.tts_pub.messages[0]))
        until(lambda: len(s.speaker.tts_pub.messages) == 2)
        self.assertEqual(s.speaker.tts_pub.messages[1], '第二句')
        s.speaker._done_callback(String('第二句'))
        until(lambda: len(s.speaker.tts_pub.messages) == 3)
        self.assertEqual(s.speaker.tts_pub.messages[2], '第三句')
        s.speaker._done_callback(String('第三句'))
        until(lambda: s.evidence.record['speech_done_requests'] == 3)

    def test_exit_keeps_current_audio_and_discards_unsent_audio(self):
        s = SimSession()
        s.speaker = speaker()
        self.addCleanup(s.stop_voice)
        self.assertTrue(s.queue_speech('正在播放', 'detection', 1))
        self.assertTrue(s.queue_speech('尚未发送', 'detection', 1))
        until(lambda: len(s.speaker.tts_pub.messages) == 1)
        s.begin_exit('time')
        s.speaker._done_callback(String('正在播放'))
        until(lambda: s.evidence.record['speech_done_requests'] == 1)
        s._speech_thread.join(timeout=.3)
        self.assertEqual(s.speaker.tts_pub.messages, ['正在播放'])
        self.assertFalse(s.queue_speech('离场后新消息', 'detection', 1))

    def test_async_send_failure_does_not_block_grasp(self):
        s = SimSession()
        s.actions = GraspActions()
        def fail(*args): raise RuntimeError('模拟连接失败')
        s.queue_speech = fail
        w = Workflow(s)
        w._head_scan = lambda p: {'candidates': [{'label': 'water'}]}
        self.assertEqual(w._inspect_point(POINT), ('grasped', True))

    def test_missing_done_stops_dispatch_without_flooding_cpp_queue(self):
        s = SimSession()
        s.speaker = speaker()
        self.addCleanup(s.stop_voice)
        self.assertTrue(s.queue_speech('无回执', 'detection', .05))
        self.assertTrue(s.queue_speech('不能挤入节点', 'detection', .05))
        until(lambda: s._speech_shutdown.is_set())
        self.assertEqual(s.speaker.tts_pub.messages, ['无回执'])
        self.assertFalse(s.queue_speech('后续请求', 'detection', .05))

    def test_print_speech_does_not_wait_before_grasp(self):
        s = SimSession()
        s.speech_print = True
        s.actions = GraspActions()
        w = Workflow(s)
        w._head_scan = lambda p: {'candidates': [{'label': 'water'}]}
        w._wait = lambda seconds: self.fail('识别打印不应等待 pause_s')
        self.assertEqual(w._inspect_point(POINT), ('grasped', True))


if __name__ == '__main__':
    unittest.main(verbosity=2)
