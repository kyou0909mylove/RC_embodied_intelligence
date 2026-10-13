"""顺序计数调度、腕部独立选物和模型缓存的离线回归；不连接实机。"""
import copy
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from core.strategy import PointScheduler
from core.configuration import load_configuration, load_runtime_configuration, ConfigurationRejected
from core.workflow import Workflow
from core.checkpoint import read_checkpoint
from arm.actions import Actions
from arm.pick_surface import catch_table
from arm.pick_ground import catch_ground
from test_exit_speech import SimSession, GraspActions, CFG, POINT


def point(pid, count=None, furniture=None):
    result = {'id':pid,'difficulty':1,'enabled':True,'furniture_id':furniture or pid}
    if count is not None:
        result['object_count'] = count
    return result


def scheduler(points, saved=None, confirmed=()):
    return PointScheduler(points,{'enabled_levels':[1,2],'round_wait_s':3},lambda:0,
                          lambda *args,**kw:None,saved,confirmed)


class CountTests(unittest.TestCase):
    def test_default_one_and_zero_skip(self):
        a,b = point('a'),point('b',0)
        s = scheduler([a,b])
        self.assertEqual(s.states['a']['remaining_count'],1)
        self.assertEqual(s.pending(),[a])
        self.assertEqual(s.next_point(),a)

    def test_order_repeats_only_remaining_points(self):
        a,b,c = point('a',2),point('b',1),point('c',1)
        s = scheduler([a,b,c])
        self.assertEqual(s.next_point(),a)
        s.register_pick(a,'a1','water',True)
        s.register_delivery(a,'a1')
        self.assertEqual(s.next_point(),b)
        s.register_pick(b,'b1','water',True)
        s.register_delivery(b,'b1')
        self.assertEqual(s.next_point(),c)
        s.observe(c,'no_target')
        self.assertIsNone(s.next_point())
        self.assertTrue(s.start_next_round())
        self.assertEqual(s.next_point(),a)
        s.register_pick(a,'a2','water',True)
        s.register_delivery(a,'a2')
        self.assertEqual(s.next_point(),c)
        s.register_pick(c,'c1','water',True)
        s.register_delivery(c,'c1')
        self.assertIsNone(s.next_point())
        self.assertEqual(s.pending(),[])
        self.assertFalse(s.start_next_round())

    def test_duplicate_pick_and_delivery_do_not_double_decrement(self):
        p = point('a',3)
        s = scheduler([p])
        s.next_point()
        s.register_pick(p,'one','water',True)
        s.register_pick(p,'one','water',True)
        s.register_delivery(p,'one')
        s.register_delivery(p,'one')
        self.assertEqual(s.states['a']['remaining_count'],2)
        self.assertEqual(s.states['a']['counted_object_ids'],['one'])

    def test_unconfirmed_load_counts_only_after_confirmation(self):
        p = point('a')
        s = scheduler([p])
        s.register_pick(p,'one','unknown',False)
        self.assertEqual(s.states['a']['remaining_count'],1)
        s.register_pick(p,'one','water',True)
        self.assertEqual(s.states['a']['remaining_count'],0)

    def test_failures_keep_point_and_ignore_old_empty_visit_limits(self):
        p = point('a')
        s = PointScheduler([p],{'enabled_levels':[1],'empty_confirmations':1,
                          'max_visits_per_point':1},lambda:0,lambda *a,**k:None)
        for outcome in ('no_target','empty','blocked','unreachable','action_failed'):
            self.assertEqual(s.next_point(),p)
            s.observe(p,outcome)
            self.assertEqual(s.states['a']['remaining_count'],1)
            self.assertTrue(s.start_next_round())

    def test_completed_active_point_is_not_selected_again(self):
        a,b = point('a'),point('b')
        s = scheduler([a,b])
        s.next_point()
        s.register_pick(a,'one','water')
        self.assertEqual(s.next_point(),b)

    def test_multiple_views_have_independent_counts(self):
        a,b = point('a',1,'table'),point('b',2,'table')
        s = scheduler([a,b])
        s.register_pick(a,'one','water')
        self.assertEqual(s.states['b']['remaining_count'],2)
        self.assertEqual(s.pending(),[b])

    def test_resume_preserves_count_and_deduplication(self):
        a,b = point('a',3),point('b')
        s = scheduler([a,b])
        s.next_point()
        s.register_pick(a,'one','water')
        restored = scheduler([a,b],s.snapshot())
        restored.register_pick(a,'one','water')
        self.assertEqual(restored.states['a']['remaining_count'],2)
        self.assertEqual(restored.next_point(),a)

    def test_legacy_snapshot_uses_confirmed_ids_only(self):
        a,b = point('a',2),point('b')
        old = {'scheduler_version':2,'states':{'a':{'visits':2,'disabled':True},
               'b':{'empty_streak':2}},'furniture':{'a':{'picked':{
               'confirmed':{'label':'water','point_id':'a'},
               'suspected':{'label':'unknown','point_id':'a'}},'delivered':[],'lost':[]}}}
        restored = scheduler([a,b],old,['confirmed'])
        self.assertEqual(restored.states['a']['remaining_count'],1)
        self.assertEqual(restored.states['b']['remaining_count'],1)
        self.assertEqual(restored.pending(),[a,b])

    def test_invalid_counter_is_rejected(self):
        for value in (True,-1,1.5,'2'):
            with self.subTest(value=value),self.assertRaises(ValueError):
                scheduler([point('a',value)])

    def test_workflow_records_confirmation_and_rebuild_keeps_count(self):
        s = SimSession()
        s.scheduler = scheduler([POINT])
        held = {'object_id':'one','attempt_id':'one','label':'water','grasp_confirmed':False}
        w = Workflow(s)
        w._register_grasp(POINT,held)
        self.assertEqual(s.scheduler.states[POINT['id']]['remaining_count'],1)
        held['grasp_confirmed'] = True
        w._register_grasp(POINT,held)
        w._register_grasp(POINT,held)
        self.assertEqual(s.scheduler.states[POINT['id']]['remaining_count'],0)
        def fail(): raise RuntimeError('模拟调度异常')
        s.scheduler.next_point = fail
        self.assertIsNone(w._schedule_call('next_point'))
        self.assertEqual(s.scheduler.states[POINT['id']]['remaining_count'],0)

    def test_zero_remaining_still_delivers_existing_load(self):
        s = SimSession()
        s.initial_nav_completed = s.entry_completed = True
        s.config.values['speech']['announce_stages'] = False
        held = {'object_id':'one','attempt_id':'one','label':'water','point_id':POINT['id'],
                'target':{'label':'water'},'grasp_confirmed':True}
        saved = scheduler([POINT])
        saved.register_pick(POINT,'one','water')
        s.restored = {'scheduler':saved.snapshot()}
        s.evidence.record['confirmed_object_ids'] = ['one']
        a = SimpleNamespace(load_state='holding',holding=held,require_travel_ready=lambda:None)
        def put(*args):
            a.holding = None
            a.load_state = 'empty'
            return {'object_id':'one','point_id':POINT['id'],'status':'delivered',
                    'delivery_confirmed':True}
        a.put = put
        s.actions = a
        w = Workflow(s)
        w._refresh_navigation = lambda reason:None
        w._initialize_route()
        self.assertEqual(s.evidence.record['delivery_actions_completed'],1)
        self.assertEqual(s.scheduler.pending(),[])
        self.assertEqual(s.evidence.record['place_interface_calls'],1)

    def test_failed_pick_is_one_attempt_per_visit(self):
        s = SimSession()
        s.speech_print = True
        a = GraspActions()
        calls = []
        a.grasp = lambda *args:(calls.append(args) or {'status':'no_target'})
        s.actions = a
        w = Workflow(s)
        w._head_scan = lambda p:{'candidates':[{'label':'cola'}]}
        self.assertEqual(w._inspect_point(POINT),('no_target',False))
        self.assertEqual(len(calls),1)

    def test_configuration_defaults_and_preserves_physical_values(self):
        path = ROOT/'config/navigation.local.json'
        raw = json.loads(path.read_text())
        parsed = load_configuration(path,ROOT)
        runtime = load_runtime_configuration(path,ROOT)
        for source,a,b in zip(raw['detection_points'],parsed.values['detection_points'],runtime.values['detection_points']):
            self.assertEqual(a['object_count'],1)
            self.assertEqual(b['object_count'],1)
            self.assertEqual(a['pose'],source['pose'])
        self.assertEqual(parsed.values['actions']['home_pose'],raw['actions']['home_pose'])
        self.assertNotIn('empty_confirmations',parsed.values['strategy'])

    def test_missing_count_defaults_to_one_and_invalid_count_fails_preflight(self):
        raw = json.loads((ROOT/'config/navigation.local.json').read_text())
        for item in raw['detection_points']: item.pop('object_count',None)
        with tempfile.NamedTemporaryFile(mode='w',suffix='.json',dir=ROOT/'config',delete=False) as stream:
            path = Path(stream.name)
        try:
            path.write_text(json.dumps(raw))
            parsed = load_configuration(path,ROOT)
            self.assertTrue(all(p['object_count']==1 for p in parsed.values['detection_points']))
            raw['detection_points'][0]['object_count'] = True
            path.write_text(json.dumps(raw))
            with self.assertRaises(ConfigurationRejected): load_configuration(path,ROOT)
        finally:
            path.unlink()

    def test_checkpoint_roundtrip_preserves_count_and_rejects_corruption(self):
        s = SimSession()
        p = point('a',2)
        schedule = scheduler([p])
        schedule.register_pick(p,'one','water')
        data = {'schema_version':1,'run_dir':str(ROOT),'entry_completed':True,'started':0,
                'record':s.evidence.record,'actions':None,'scheduler':schedule.snapshot()}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'checkpoint.json'
            path.write_text(json.dumps(data))
            restored = read_checkpoint(path,CFG,strict=False)
            self.assertEqual(restored['scheduler']['states']['a']['remaining_count'],1)
            data['scheduler']['states']['a']['remaining_count'] = False
            path.write_text(json.dumps(data))
            with self.assertRaises(ValueError): read_checkpoint(path,CFG,strict=False)


class PickerTests(unittest.TestCase):
    def test_wrong_kinect_label_does_not_filter_surface_targets(self):
        s = SimSession()
        controller = SimpleNamespace(session=s,pending_pick=None)
        targets = ['water_bottle','white_pump_bottle','blue_bowl']
        seen = []
        def pick(robot,target_items):
            seen.extend(target_items)
            robot.selected_target_name = 'white_pump_bottle'
            return True
        original = SimpleNamespace(TARGET_NAMES=targets,HOME_POSE=[0]*6,pick_once=pick)
        def invoke(c,target,p,callback,home_pose):
            c.pending_pick = {'target':dict(target),'label':target['label']}
            return callback(SimpleNamespace())
        with patch('arm.driver.load_original',return_value=original),patch('arm.pick_surface.call_original',side_effect=invoke):
            self.assertTrue(catch_table(controller,{'label':'cola'},POINT,'cola'))
        self.assertEqual(seen,targets)
        self.assertEqual(controller.pending_pick['target']['head_label_hint'],'cola')
        self.assertEqual(controller.pending_pick['label'],'white_pump_bottle')

    def test_no_kinect_detection_still_runs_surface_recognition(self):
        s = SimSession()
        c = SimpleNamespace(session=s,pending_pick={})
        original = SimpleNamespace(TARGET_NAMES=['water_bottle'],HOME_POSE=[0]*6)
        with patch('arm.driver.load_original',return_value=original),patch('arm.pick_surface.call_original',return_value={'status':'grasped'}) as invoke:
            self.assertEqual(catch_table(c,{},POINT,None),{'status':'grasped'})
        self.assertEqual(invoke.call_args.args[1]['label'],'unknown')

    def test_explicit_exclusions_still_apply(self):
        s = SimSession()
        s.config.values['excluded_labels'] = ['water']
        c = SimpleNamespace(session=s,pending_pick=None)
        seen = []
        def pick(robot,targets): seen.extend(targets); return False
        original = SimpleNamespace(TARGET_NAMES=['water_bottle','white_cup'],HOME_POSE=[0]*6,pick_once=pick)
        with patch('arm.driver.load_original',return_value=original),patch('arm.pick_surface.call_original',side_effect=lambda c,t,p,invoke,home_pose:invoke(SimpleNamespace())):
            catch_table(c,{'label':'water'},POINT,'water')
        self.assertEqual(seen,['white_cup'])

    def test_ground_uses_all_wrist_classes_without_head_filter(self):
        s = SimSession()
        c = SimpleNamespace(session=s,cfg=s.config.values['actions'],pending_pick=None,observation_reporter=None)
        detected = []
        class Detector:
            def __init__(self,**kw): pass
            def detect_targets(self,targets,**kw): detected.extend(targets); return None
        class Robot:
            def image_to_arm(self,*args): pass
            def catch_ground(self,weights_path):
                module.RealSenseYolo11Detector().detect_targets(['paper_ball'])
                return False
        module = SimpleNamespace(KinovaRobotGroud=Robot,RealSenseYolo11Detector=Detector)
        with patch('arm.driver.load_original',return_value=module),patch('arm.pick_ground.call_original',side_effect=lambda c,t,p,invoke:invoke(SimpleNamespace())):
            catch_ground(c,{'label':'water'},dict(POINT,grasp_profile='ground'),'water')
        expected = [name for name in c.cfg['profiles']['ground']['model']['labels'] if name not in s.config.values['excluded_labels']]
        self.assertEqual(detected,expected)
        self.assertIs(module.RealSenseYolo11Detector,Detector)


def load_detector(filename):
    created = []
    class Model:
        def __init__(self,path):
            created.append(path)
            self.names = {0:'water'}
            self.images = []
        def to(self,device): return self
        def __call__(self,image,**kw): self.images.append(image); return []
    class Pipeline:
        def __init__(self): self.started = self.stopped = 0
        def start(self,config): self.started += 1
        def stop(self): self.stopped += 1
        def get_active_profile(self):
            sensor = SimpleNamespace(set_option=lambda *args:None)
            device = SimpleNamespace(first_color_sensor=lambda:sensor)
            return SimpleNamespace(get_device=lambda:device)
        def wait_for_frames(self): return None
    np = ModuleType('numpy')
    np.ndarray = object
    np.random = SimpleNamespace(randint=lambda *args:0)
    torch = ModuleType('torch')
    torch.cuda = SimpleNamespace(is_available=lambda:False)
    rs = ModuleType('pyrealsense2')
    rs.pipeline = Pipeline
    rs.config = lambda:SimpleNamespace(enable_stream=lambda *args:None)
    rs.align = lambda *args:object()
    rs.stream = SimpleNamespace(color=1,depth=2)
    rs.format = SimpleNamespace(z16=1,bgr8=2)
    rs.option = SimpleNamespace(enable_auto_white_balance=1)
    ultra = ModuleType('ultralytics')
    ultra.YOLO = Model
    utils = ModuleType('ultralytics.utils')
    utils.ops = SimpleNamespace()
    modules = {'numpy':np,'cv2':ModuleType('cv2'),'torch':torch,'pyrealsense2':rs,
               'ultralytics':ultra,'ultralytics.utils':utils}
    spec = importlib.util.spec_from_file_location('cache_test_'+Path(filename).stem,ROOT/'catch_ground/src'/filename)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules,modules): spec.loader.exec_module(module)
    return module,created


class CacheTests(unittest.TestCase):
    def test_cache_is_keyed_by_weight_and_device(self):
        for filename in ('realsense_yolo11_ref.py','realsense_yolo11.py'):
            with self.subTest(filename=filename):
                module,created = load_detector(filename)
                one = module.get_cached_model('weights/a.pt','cpu')
                self.assertIs(module.get_cached_model('weights/../weights/a.pt','cpu'),one)
                self.assertIsNot(module.get_cached_model('weights/b.pt','cpu'),one)
                self.assertIsNot(module.get_cached_model('weights/a.pt','cuda'),one)
                self.assertEqual(len(created),3)

    def test_detector_reuses_model_but_not_camera_or_new_images(self):
        for filename in ('realsense_yolo11_ref.py','realsense_yolo11.py'):
            with self.subTest(filename=filename):
                module,created = load_detector(filename)
                one = module.RealSenseYolo11Detector(Path('weights/a.pt'))
                two = module.RealSenseYolo11Detector(Path('weights/a.pt'))
                self.assertIs(one.model,two.model)
                self.assertIsNot(one.pipeline,two.pipeline)
                class Image:
                    def copy(self): return self
                images = [Image(),Image()]
                for detector,image in zip((one,two),images):
                    detector._get_realsense_data = lambda image=image:(image,None,None)
                    if hasattr(detector,'_white_balance_warmup'):
                        detector._white_balance_warmup = lambda:None
                    with patch.object(module.time,'sleep',return_value=None):
                        self.assertIsNone(detector.detect_targets(['water'],max_retry=1,show_window=False))
                    self.assertEqual(detector.pipeline.started,1)
                    self.assertEqual(detector.pipeline.stopped,1)
                self.assertEqual(one.model.images,images)
                self.assertEqual(len(created),1)

    def test_prepare_preloads_enabled_models_without_starting_camera(self):
        s = SimSession()
        for p in s.config.values['detection_points']: p['enabled'] = True
        # 柜层和桌面同一权重只预加载一次；地面是另一模型。
        calls = []
        module = SimpleNamespace(get_cached_model=lambda weights:calls.append(weights))
        with patch('arm.driver.load_original',return_value=module):
            Actions(s)._prepare_vision_models()
        self.assertEqual(len(calls),2)

    def test_prepare_skips_models_in_exit_window(self):
        s = SimSession(570)
        with patch('arm.driver.load_original') as load:
            Actions(s)._prepare_vision_models()
        load.assert_not_called()


if __name__=='__main__':
    unittest.main(verbosity=2)
