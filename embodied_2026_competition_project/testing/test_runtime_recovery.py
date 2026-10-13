"""运行故障回归：保留严格启动入口，入场后恢复不退出、不重置计数。"""
from types import SimpleNamespace
from unittest.mock import Mock
import unittest

from test_exit_speech import SimSession, POINT, arm, GraspActions
from core.workflow import Workflow
from core.strategy import PointScheduler
from core.session import NavigationFailed
from core.faults import DeviceUnavailable, ParameterFault
from core.safety import ActionFailed, ExitRequested, OperationStopped
from camera.vision import CameraFailed
from arm.actions import Actions


def flow(session):
    result = Workflow(session)
    result._initialize_route = lambda: None
    def wait(seconds):
        session.now += seconds
        session.check()
    result._wait = wait
    return result


def scheduler(session, count=1, all_points=False):
    points = session.config.values['detection_points']
    for point in points:
        point['enabled'] = all_points or point['id']==POINT['id']
        point['object_count'] = count
    session.scheduler = PointScheduler(points,session.config.values['strategy'],
                                       session.clock,session.evidence.event)
    return session.scheduler


def ready_arm(session):
    result = Actions(session)
    result.travel_ready = True
    session.actions = result
    return result


def context(point=POINT):
    return {'object_id':'confirmed-one','attempt_id':'confirmed-one','label':'water',
            'target':{'label':'water'},'point_id':point['id'],
            'furniture_id':point['furniture_id'],'grasp_confirmed':True,
            'grasp_phase':'home','retreat_poses':[],'retreat_index':0}


class RuntimeRecovery(unittest.TestCase):
    def test_prepare_call_still_rejects_device_failure(self):
        s = SimSession(100)
        with self.assertRaises(DeviceUnavailable):
            s._prepare_call('相机',Mock(side_effect=RuntimeError('初始化失败')))
        self.assertEqual(s.started,0)

    def test_initial_and_entry_navigation_failure_still_propagate(self):
        for initial_completed in (False,True):
            with self.subTest(initial_completed=initial_completed):
                s = SimSession(100)
                s.test_mode = True
                s.initial_nav_completed = initial_completed
                s.entry_completed = False
                w = Workflow(s)
                w._navigate = Mock(side_effect=NavigationFailed('入口',4,'无法到达'))
                w._search = Mock()
                with self.assertRaises(NavigationFailed):
                    w.run()
                self.assertFalse(s.entry_completed)
                self.assertEqual(s.initial_nav_completed,initial_completed)
                w._search.assert_not_called()

    def test_initial_device_error_still_propagates(self):
        s = SimSession(100)
        w = flow(s)
        w._initialize_route = Mock(side_effect=DeviceUnavailable('导航','未连接'))
        with self.assertRaises(DeviceUnavailable):
            w.run()

    def test_runtime_errors_retry_same_session_without_restarting_clock(self):
        faults = [DeviceUnavailable('导航','断连'),ParameterFault('运行参数','无效'),
                  RuntimeError('软件错误'),OperationStopped('阶段超时')]
        for fault in faults:
            with self.subTest(fault=type(fault).__name__):
                s = SimSession(100)
                s.evidence.record['confirmed_object_ids'] = ['already-counted']
                s.close = Mock()
                w = flow(s)
                w._initialize_route = Mock()
                w._search = Mock(side_effect=[fault,'数量完成'])
                w._exit = Mock()
                w.run()
                self.assertEqual(w._search.call_count,2)
                self.assertEqual(s.now,102)
                self.assertEqual(s.started,0)
                self.assertEqual(s.match_deadline,600)
                self.assertEqual(s.work_deadline,570)
                self.assertEqual(s.evidence.record['confirmed_object_ids'],['already-counted'])
                w._initialize_route.assert_called_once()
                s.close.assert_not_called()

    def test_session_is_only_closed_after_runtime_retry_finishes(self):
        s = SimSession(100)
        s._open_record = lambda:None
        s._prepare = lambda:None
        s.close = Mock()
        w = flow(s)
        attempts = []
        def search():
            s.close.assert_not_called()
            attempts.append(s.now)
            if len(attempts)==1:
                raise DeviceUnavailable('导航','运行中断连')
            return '数量完成'
        w._search = search
        w._exit = Mock()
        with s:
            w.run()
            s.close.assert_not_called()
        s.close.assert_called_once()
        self.assertEqual(attempts,[100,102])
        self.assertEqual(s.started,0)

    def test_detection_navigation_device_or_parameter_failure_skips_point(self):
        for fault in (DeviceUnavailable('导航','断连'),ParameterFault('pose','无效')):
            with self.subTest(fault=type(fault).__name__):
                s = SimSession(100)
                w = flow(s)
                w._navigate = Mock(side_effect=fault)
                w._inspect_point = Mock()
                self.assertEqual(w._one_point(POINT),('unreachable',False))
                w._inspect_point.assert_not_called()
                self.assertEqual(s.evidence.record['physical_grasps'],0)

    def test_runtime_point_parameter_error_goes_to_next_point_without_deducting(self):
        s = SimSession(100)
        ready_arm(s)
        sched = scheduler(s,all_points=True)
        self.assertGreaterEqual(len(sched.points),2)
        w = flow(s)
        visited = []
        def inspect(point):
            visited.append(point['id'])
            if len(visited)==1:
                raise ParameterFault('腕部模型','本点不可用')
            s.now = 570
            s.check()
        w._one_point = inspect
        w._exit = Mock()
        w.run()
        self.assertEqual(visited,[p['id'] for p in sched.points[:2]])
        self.assertEqual(sched.states[visited[0]]['remaining_count'],1)
        self.assertEqual(s.evidence.record['physical_grasps'],0)

    def test_blocking_stage_speech_failure_only_returns_false(self):
        faults = [DeviceUnavailable('语音','无订阅'),ParameterFault('speech','无效'),
                  RuntimeError('无完成回执')]
        for fault in faults:
            with self.subTest(fault=type(fault).__name__):
                s = SimSession(100)
                s.speaker = SimpleNamespace(tts_pub=SimpleNamespace(get_num_connections=lambda:1),
                                            speak=Mock(side_effect=fault))
                self.assertFalse(flow(s)._speak('开始搜索','stage',blocking=True))
                self.assertEqual(s.evidence.record['speech_done_requests'],0)

    def test_head_model_or_device_failure_still_starts_wrist_detection(self):
        faults = [CameraFailed('头部模型失败',phase='model'),
                  DeviceUnavailable('头部相机','无数据'),ParameterFault('head_model','无效')]
        for fault in faults:
            with self.subTest(fault=type(fault).__name__):
                s = SimSession(100)
                s.speech_print = True
                s.actions = GraspActions()
                s.vision = SimpleNamespace(scan=Mock(side_effect=fault),close=Mock())
                w = flow(s)
                w.recovery.camera = Mock(side_effect=fault)
                self.assertEqual(w._inspect_point(POINT),('grasped',True))
                self.assertTrue(s.actions.started.is_set())
                degraded = [e for e in s.evidence.record['events'] if e['event']=='head_vision_degraded']
                self.assertEqual(len(degraded),1)
                self.assertFalse(degraded[0]['empty_confirmed'])

    def test_head_failure_does_not_swallow_exit_request(self):
        s = SimSession(569)
        s.actions = GraspActions()
        def scan(*args):
            s.now = 570
            s.check()
        s.vision = SimpleNamespace(scan=scan,close=Mock())
        with self.assertRaises(ExitRequested):
            flow(s)._inspect_point(POINT)
        self.assertFalse(s.actions.started.is_set())

    def test_arm_recovery_failure_waits_then_continues(self):
        s = SimSession(100)
        a = arm(s)
        w = flow(s)
        attempts = []
        def recover(reason):
            attempts.append(s.now)
            if len(attempts)==1:
                raise DeviceUnavailable('机械臂执行','收臂未完成')
            a.travel_ready = True
            a.pending_pick = None
            a._load_unknown = False
            return {'holding':False}
        w.recovery.arm = recover
        def point(*args):
            s.now = 570
            s.check()
        w._one_point = point
        w.run()
        self.assertEqual(len(attempts),2)
        self.assertEqual(s.status,'FLOW_COMPLETED')
        self.assertTrue(a.travel_ready)
        self.assertTrue(all(item[1]=='离场点' for item in s.trace if item[0]=='navigate'))

    def test_persistent_arm_failure_waits_through_exit_until_total_deadline(self):
        s = SimSession(560)
        a = arm(s)
        w = flow(s)
        w.recovery.arm = Mock(side_effect=DeviceUnavailable('机械臂执行','无法收臂'))
        def failed_move(*args,**kwargs):
            s.now += 1
            raise ActionFailed('home',4,'拒绝收臂')
        a.driver.move = failed_move
        w.run()
        self.assertEqual(s.status,'TIME_UP')
        self.assertFalse(a.travel_ready)
        self.assertFalse(any(item[0]=='navigate' for item in s.trace))
        self.assertGreater(w.recovery.arm.call_count,1)
        self.assertEqual(s.evidence.record['physical_deliveries'],0)

    def test_unconfirmed_release_preserves_load_and_blocks_new_grasp(self):
        s = SimSession(565)
        a = arm(s,holding=True)
        a.travel_ready = True
        held = a.holding
        w = flow(s)
        attempts = []
        def release():
            attempts.append(s.now)
            raise DeviceUnavailable('机械臂执行','释放未确认')
        w._deliver_pending = release
        w._one_point = Mock()
        w.run()
        self.assertGreater(len(attempts),1)
        self.assertIs(a.holding,held)
        w._one_point.assert_not_called()
        self.assertEqual(s.evidence.record['physical_deliveries'],0)
        self.assertEqual(s.status,'FLOW_COMPLETED')

    def test_delivery_fault_does_not_deduct_or_credit_twice(self):
        s = SimSession(100)
        a = ready_arm(s)
        sched = scheduler(s)
        p = sched.points[0]
        held = context(p)
        w = flow(s)
        def inspect(point):
            a.holding = held
            w._register_grasp(point,held)
            w._register_grasp(point,held)
            raise RuntimeError('抓取完成后的软件异常')
        calls = []
        def deliver():
            calls.append(s.now)
            if len(calls)==1:
                raise DeviceUnavailable('导航','返回投放区断连')
            a.holding = None
            receipt = dict(held,status='delivered',delivery_confirmed=True)
            w._register_delivery(receipt)
            w._register_delivery(receipt)
            return 'delivered'
        w._one_point = inspect
        w._deliver_pending = deliver
        w._exit = Mock()
        w.run()
        self.assertEqual(sched.states[p['id']]['remaining_count'],0)
        self.assertEqual(s.evidence.record['physical_grasps'],1)
        self.assertEqual(s.evidence.record['physical_deliveries'],1)
        self.assertEqual(s.evidence.record['delivery_actions_completed'],1)
        self.assertEqual(s.evidence.record['confirmed_object_ids'],[held['object_id']])

    def test_scheduler_parameter_fault_rebuilds_and_reapplies_pick(self):
        s = SimSession(100)
        sched = scheduler(s,2)
        p = sched.points[0]
        sched.register_pick = Mock(side_effect=ParameterFault('scheduler','故障'))
        w = flow(s)
        w._schedule_call('register_pick',p,'new-id','water',True)
        w._schedule_call('register_pick',p,'new-id','water',True)
        state = s.scheduler.states[p['id']]
        self.assertEqual(state['remaining_count'],1)
        self.assertEqual(state['counted_object_ids'],['new-id'])

    def test_scheduler_snapshot_failure_preserves_current_counts(self):
        s = SimSession(100)
        sched = scheduler(s,2)
        p = sched.points[0]
        sched.register_pick(p,'current-id','water',True)
        s.evidence.record['confirmed_object_ids'] = ['current-id']
        sched.next_point = Mock(side_effect=RuntimeError('选择失败'))
        sched.snapshot = Mock(side_effect=RuntimeError('快照函数失败'))
        flow(s)._schedule_call('next_point')
        state = s.scheduler.states[p['id']]
        self.assertEqual(state['remaining_count'],1)
        self.assertEqual(state['counted_object_ids'],['current-id'])

    def test_exit_cancel_failure_retries_without_opening_gripper(self):
        s = SimSession(570)
        a = arm(s,holding=True)
        held = a.holding
        a.driver.cancel_motion = Mock(side_effect=[DeviceUnavailable('机械臂取消接口','未空闲'),None])
        w = flow(s)
        w._initialize_route = s.check
        w.run()
        self.assertEqual(a.driver.cancel_motion.call_count,2)
        self.assertIs(a.holding,held)
        self.assertEqual(s.status,'FLOW_COMPLETED')
        self.assertTrue(a.travel_ready)

    def test_exit_navigation_device_or_parameter_failure_retries(self):
        faults = [DeviceUnavailable('导航','断连'),ParameterFault('exit','运行异常')]
        for fault in faults:
            with self.subTest(fault=type(fault).__name__):
                s = SimSession(570)
                w = flow(s)
                w._initialize_route = s.check
                s.navigate = Mock(side_effect=[fault,None])
                w.run()
                self.assertEqual(s.navigate.call_count,2)
                self.assertEqual(s.status,'FLOW_COMPLETED')
                self.assertTrue(s.evidence.record['exit_motion_completed'])

    def test_persistent_exit_navigation_failure_ends_without_fake_arrival(self):
        s = SimSession(570)
        w = flow(s)
        w._initialize_route = s.check
        attempts = []
        def failed_navigation(*args):
            attempts.append(s.now)
            raise NavigationFailed('离场点',4,'不可达')
        s.navigate = failed_navigation
        w.run()
        self.assertGreater(len(attempts),1)
        self.assertTrue(all(now<600 for now in attempts))
        self.assertEqual(s.status,'TIME_UP')
        self.assertFalse(s.evidence.record['exit_motion_completed'])

    def test_exit_deadline_interrupts_runtime_retry_wait(self):
        s = SimSession(569)
        w = flow(s)
        w._search = Mock(side_effect=RuntimeError('运行异常'))
        w.run()
        w._search.assert_called_once()
        self.assertEqual(s.status,'FLOW_COMPLETED')
        self.assertEqual(s.trace,[('navigate','离场点',571)])

    def test_active_stop_is_not_swallowed_by_runtime_recovery(self):
        s = SimSession(100)
        w = flow(s)
        def failed_search():
            s.gate.stop_messages.put(SimpleNamespace(data=True))
            raise RuntimeError('停止时发生软件异常')
        w._search = failed_search
        with self.assertRaises(OperationStopped):
            w.run()
        self.assertTrue(s.gate.stopped)
        self.assertEqual(s.trace,[])

    def test_ros_shutdown_remains_terminal(self):
        s = SimSession(100)
        w = flow(s)
        def failed_search():
            s.rospy.is_shutdown = lambda:True
            raise RuntimeError('ROS已关闭')
        w._search = failed_search
        with self.assertRaises(DeviceUnavailable):
            w.run()
        self.assertTrue(s.gate.stopped)

    def test_keyboard_interrupt_is_not_swallowed(self):
        s = SimSession(100)
        w = flow(s)
        w._search = Mock(side_effect=KeyboardInterrupt())
        with self.assertRaises(KeyboardInterrupt):
            w.run()

    def test_event_serialization_failure_only_records_warning(self):
        s = SimSession()
        s.evidence.save_logs = True
        s.evidence.event('bad_payload',value=object())
        self.assertEqual(len(s.evidence.record['recording_warnings']),1)

    def test_event_writer_failure_only_records_warning(self):
        s = SimSession()
        s.evidence.save_logs = True
        s.evidence.writer = SimpleNamespace(append=Mock(side_effect=RuntimeError('写入失败')))
        s.evidence.event('record_failure')
        self.assertEqual(len(s.evidence.record['recording_warnings']),1)

    def test_event_logging_does_not_swallow_deadline(self):
        for exc in (ExitRequested('离场'),OperationStopped('截止')):
            with self.subTest(exc=type(exc).__name__):
                s = SimSession()
                s.evidence.save_logs = True
                s.evidence.writer = SimpleNamespace(append=Mock(side_effect=exc))
                with self.assertRaises(type(exc)):
                    s.evidence.event('deadline')


if __name__=='__main__':
    unittest.main(verbosity=2)
