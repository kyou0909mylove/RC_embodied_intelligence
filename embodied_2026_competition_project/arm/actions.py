# -*- coding: utf-8 -*-
"""机械臂资源与持物状态；家具选择、动作执行、放置分别在独立模块。"""
import copy
import uuid
from core.faults import StateMismatch
from .pick_surface import catch_table
from .pick_shelf import catch_shelf
from .pick_ground import catch_ground
from .put import put as put_in_zone, print_put


def grasp(target, detection_point):
    print('[抓取打印] 机械臂未执行动作',flush=True)
    return {'physical_action':False,'grasp_confirmed':False,'status':'print'}


def put(target, drop_pose):
    return print_put(target,drop_pose)


def place(target, drop_pose):
    return put(target,drop_pose)


class Actions:
    def __init__(self, session):
        self.session,self.cfg = session,session.config.values['actions']
        self.driver = self.feedback = self.wrist = None
        saved = (session.restored or {}).get('actions') or {}
        self.holding = copy.deepcopy(saved.get('holding'))
        self.pending_pick = copy.deepcopy(saved.get('pending_pick'))
        self._load_unknown = saved.get('load_state')=='unknown'
        self.storage_states = copy.deepcopy(saved.get('storage_states',{}))
        self.shelf_next_layers = copy.deepcopy(saved.get('shelf_next_layers') or {})
        self.travel_ready = False
        self.put_stage = saved.get('put_stage')
        self.observation_reporter = None
        self.wrist_restart_needed = False

    @property
    def load_state(self):
        return 'unknown' if self._load_unknown else ('holding' if self.holding else 'empty')

    def snapshot(self):
        return copy.deepcopy({'holding':self.holding,'pending_pick':self.pending_pick,
                              'load_state':self.load_state,'travel_ready':self.travel_ready,
                              'storage_states':self.storage_states,'put_stage':self.put_stage,
                              'shelf_next_layers':self.shelf_next_layers})

    def prepare(self):
        from .driver import KinovaDriver
        from .feedback import FingerFeedback
        from .startup import reset_for_new_match, restore_interrupted_match
        from camera.wrist import WristVision
        self.driver = KinovaDriver(self.cfg,self.session.rospy,self.session.gate)
        self.driver.connect()
        self.feedback = FingerFeedback(self.driver,self.cfg['gripper_feedback'],self.session.gate)
        if self.session.resume:
            # 恢复先判断实际负载。空爪可以自动复位；持物不执行张爪动作。
            restore_interrupted_match(self)
        else:
            reset_for_new_match(self)
        self.reconcile_storage_on_resume()
        self.wrist = WristVision(self.session.config,self.session.evidence)
        # 原抓取文件自行使用 RealSense，按需启动旧观测接口，避免占用同一相机。
        self.wrist_restart_needed = True
        self._prepare_vision_models()
        self.session.checkpoint()

    def _prepare_vision_models(self):
        """任务计时前只预加载已启用点位的腕部模型，不打开 RealSense。"""
        from .driver import load_original
        if self.session.clock()>=self.session.work_deadline:
            return  # 恢复已进入离场窗口时不再预加载抓取模型。
        values = self.session.config.values
        loaded = set()
        for point in values['detection_points']:
            if (not point.get('enabled',True) or point.get('object_count',1)==0
                    or point['difficulty'] not in values['strategy']['enabled_levels']):
                continue
            profile = point['grasp_profile']
            name = 'realsense_yolo11' if profile=='ground' else 'realsense_yolo11_ref'
            weights = self.cfg['profiles'][profile]['model']['weights']
            key = (name,weights)
            if key not in loaded:
                self.session.check()
                load_original(name).get_cached_model(weights)
                loaded.add(key)

    def require_travel_ready(self):
        if (not self.travel_ready or
                any(state.get('handle_engaged') for state in self.storage_states.values())):
            raise StateMismatch('尚未安全收臂或仍在拉家具把手；先复位再导航')

    def _home(self):
        self.travel_ready = False
        self.session.checkpoint()
        with self.session.operation(self.cfg['home_timeout_s']):
            self.driver.move(self.cfg['home_pose'],verify_pose=True,stage='收臂')
        self.travel_ready = True
        self.session.checkpoint()


    def _recovered_context(self):
        """没有完整抓取记录时只保留疑似负载，不伪造一次真实抓取。"""
        point = next(p for p in self.session.config.values['detection_points'] if p.get('enabled', True))
        oid = 'recovered_'+uuid.uuid4().hex
        return {'attempt_id':oid, 'object_id':oid, 'label':'unknown', 'target':{},
                'point_id':point['id'], 'furniture_id':point['furniture_id'],
                'grasp_confirmed':False, 'retreat_poses':[], 'retreat_index':0}

    def reset_stalled_attempt(self):
        """取消旧动作、沿已有回程退出、收臂，再判断中断抓取的负载。"""
        from .put import released_receipt
        print('[恢复] 取消旧动作并收臂；投放阶段={}'.format(self.put_stage or '无'), flush=True)
        self.session.stop_navigation()
        self.driver.cancel_motion()
        context = self.holding or self.pending_pick
        # PUT已张爪但回执未保存时，保留原来的真实释放确认。
        if self.put_stage == 'opening' and context:
            receipt = self.feedback.confirm('place')
            if receipt['released']:
                if getattr(self.session, 'drop_arrived', False):
                    self.session.last_put_receipt = released_receipt(self, context, receipt)
                self.holding = self.pending_pick = None
                self.put_stage = 'released'
                context = None
        if self.put_stage == 'released':
            self.holding = self.pending_pick = None
            context = None
        if context and context.get('grasp_phase') in ('approach','closing','retreat'):
            path = context.get('retreat_poses', [])
            for index in range(context.get('retreat_index', 0), len(path)):
                self.driver.move(path[index], verify_pose=False, stage='恢复抓取回程')
                context['retreat_index'] = index+1
                self.session.checkpoint()
            context['grasp_phase'] = 'home'
        self._home()
        if self.holding is None and context and self._load_unknown:
            receipt = self.feedback.confirm('grasp',context['attempt_id'],context['label'])
            if receipt['holding']:
                self.holding = copy.deepcopy(context)
                self.holding['grasp_confirmed'] = bool(receipt.get('holding_confirmed'))
        if self.holding is None:
            print('[恢复张爪] 已完成收臂，未记录持物，执行复位张爪', flush=True)
            self.driver.fingers(self.cfg['finger_open'])
        self._load_unknown = False
        self.pending_pick = None
        self.put_stage = None
        if self.wrist is not None and self.wrist.busy:
            self.wrist.close()
            self.wrist_restart_needed = True
        self.session.checkpoint()
        return {'holding':self.holding is not None, 'gripper_opened':self.holding is None,
                'home_completed':True}

    def fallback_reset(self):
        """恢复软件出错时使用已有回程及home；未知负载不伪造持物记录。"""
        self.session.stop_navigation()
        self.driver.cancel_motion()
        context = self.holding or self.pending_pick
        if context and context.get('grasp_phase') in ('approach','closing','retreat'):
            for index in range(context.get('retreat_index',0),len(context.get('retreat_poses',[]))):
                self.driver.move(context['retreat_poses'][index],verify_pose=False,stage='最小恢复回程')
                context['retreat_index'] = index+1
                self.session.checkpoint()
        if self.holding is None:
            print('[最小恢复张爪] 未记录持物，先张爪再收臂', flush=True)
            self.driver.fingers(self.cfg['finger_open'])
        self._home()
        self._load_unknown = False
        self.pending_pick = None
        if self.wrist is not None and self.wrist.busy:
            self.wrist.close()
            self.wrist_restart_needed = True
        self.session.checkpoint()
        return {'holding':bool(self.holding),'gripper_opened':not bool(self.holding),'home_completed':True}


    def grasp(self, target, point):
        # 几何配置在启动读取时已验证，运行中不重复按源示教角度门禁。
        if point['difficulty']==3:
            state = self.storage_states.get(point['furniture_id'],{})
            if state.get('status')!='open_motion_completed':
                return {'status':'storage_uncertain','grasp_confirmed':False,'physical_action':False}
        if self.load_state=='unknown':
            self.reset_stalled_attempt()
        if self.holding:
            return {'status':'holding','grasp_confirmed':False,'physical_action':False}
        self.require_travel_ready()
        label = self.cfg['profiles'][point['grasp_profile']]['label_map'].get(target.get('label'))
        if label is None:
            label = target.get('label')
        self.session.last_put_receipt = None
        self.travel_ready = False
        self.session.checkpoint()
        picker = {'surface':catch_table,'shelf':catch_shelf,'ground':catch_ground}[point['grasp_profile']]
        return picker(self,target,point,label)

    def put(self, target, drop_pose):
        return put_in_zone(self,target,drop_pose)

    def abandon(self, reason):
        from .release import abandon
        return abandon(self, reason)

    def place(self, target, drop_pose):
        return self.put(target,drop_pose)


    def open_storage(self, point):
        from .open_cabinet import open_cabinet
        from .open_drawer import open_drawer
        return {'cabinet':open_cabinet,'drawer':open_drawer}[point['storage']['kind']](self,point)


    def reconcile_storage_on_resume(self):
        """恢复时核对空爪/收臂和人工确认；中断的开门不会自动重放。"""
        confirmed = set(getattr(self.session,'opened_storage',[]))
        for furniture,state in self.storage_states.items():
            if state.get('status')=='opening' or state.get('handle_engaged'):
                state.update(status='uncertain',handle_engaged=False)
        if confirmed:
            if not self.session.resume or self.holding or self.pending_pick:
                raise RuntimeError('人工确认开门仅用于恢复且必须空爪')
            if not self.feedback.confirm('initial')['empty']:
                raise RuntimeError('人工确认开门还需要实际空爪')
            self.driver.tool_pose(expected=self.cfg['home_pose'])
            for furniture in confirmed:
                self.storage_states[furniture] = {'status':'open_motion_completed','handle_engaged':False,
                    'stage':'operator_confirmed','door_sensor_verified':False}
                self.session.evidence.event('storage_operator_confirmed',furniture_id=furniture)

    def close(self):
        errors = []
        for name,resource in [('腕部相机',self.wrist),('三指反馈',self.feedback),('机械臂',self.driver)]:
            if resource is not None:
                try: resource.close()
                except Exception as exc: errors.append(name+': '+str(exc))
        if errors:
            raise RuntimeError('; '.join(errors))
