# -*- coding: utf-8 -*-
"""机械臂资源与持物状态；家具选择、动作执行、放置分别在独立模块。"""
import copy
import uuid
from core.faults import ParameterFault, StateMismatch
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
        self.travel_ready = False
        self.put_stage = saved.get('put_stage')
        self.observation_reporter = None
        self._ground_fallback_noted = set()
        self.wrist_restart_needed = False

    @property
    def load_state(self):
        return 'unknown' if self._load_unknown else ('holding' if self.holding else 'empty')

    def snapshot(self):
        return copy.deepcopy({'holding':self.holding,'pending_pick':self.pending_pick,
                              'load_state':self.load_state,'travel_ready':self.travel_ready,
                              'storage_states':self.storage_states,'put_stage':self.put_stage})

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
        self.wrist.start(self.session.gate)
        self.session.checkpoint()

    def require_travel_ready(self):
        if (self.load_state=='unknown' or not self.travel_ready or
                any(state.get('handle_engaged') for state in self.storage_states.values())):
            raise StateMismatch('负载状态待恢复或尚未收臂；先复位，再发送底盘目标')

    def _home(self):
        self.travel_ready = False
        self.session.checkpoint()
        with self.session.operation(self.cfg['home_timeout_s']):
            self.driver.move(self.cfg['home_pose'],stage='收臂')
        self.travel_ready = True
        self.session.checkpoint()

    def _scan(self, point, label, layer=None):
        if self.wrist_restart_needed:
            with self.session.operation(self.session.config.values['timing']['prepare_timeout_s']):
                self.wrist.restart(self.session.gate)
            self.wrist_restart_needed = False
        with self.session.operation(self.cfg['wrist']['scan_timeout_s']):
            candidates = self.wrist.scan(point,label,self.session.gate,layer)
        # 有框但尚未稳定，不等于没有物品；供各家具的完整判空逻辑使用。
        self.scan_saw_objects = bool(candidates or getattr(self.wrist, 'last_scan', {}).get('raw_detection_count'))
        if point['grasp_profile'] == 'ground':
            fallback = self.cfg['profiles']['ground'].get('angle_fallback_deg', 0.0)
            for item in candidates:
                if item.get('camera_xyz_m') is not None and item.get('angle_deg') is None and fallback is not None:
                    # 与往年脚本的无轮廓返回0度一致；明确记录，不冒充测量角度。
                    item.update(angle_deg=fallback, angle_basis='legacy_fallback')
                    self.session.evidence.event('ground_angle_fallback', point_id=point['id'],
                                                label=item['label'], angle_deg=fallback)
                    if point['id'] not in self._ground_fallback_noted:
                        print('[地面角度] 未取得稳定轮廓角度，沿用旧代码默认{}度'.format(fallback), flush=True)
                        self._ground_fallback_noted.add(point['id'])
        if self.observation_reporter is not None:
            self.observation_reporter(point, candidates, layer)
        return candidates

    def _recovered_context(self):
        """没有完整抓取记录时只保留疑似负载，不伪造一次真实抓取。"""
        point = next(p for p in self.session.config.values['detection_points'] if p.get('enabled', True))
        oid = 'recovered_'+uuid.uuid4().hex
        return {'attempt_id':oid, 'object_id':oid, 'label':'unknown', 'target':{},
                'point_id':point['id'], 'furniture_id':point['furniture_id'],
                'grasp_confirmed':False, 'retreat_poses':[], 'retreat_index':0}

    def reset_stalled_attempt(self):
        """超时/软件错误恢复。先取消，保留可能持物，沿已记录回程退出并收臂。

        已经执行张爪的PUT如果只是后续收臂失败，不再次放置、也不丢失回执。
        普通检测失败确认空爪后张爪；中断的闭爪继续加紧，再按三指闭合程度判断。
        """
        from .put import released_receipt
        self.session.stop_navigation()
        self.driver.cancel_motion()
        context = self.holding or self.pending_pick
        state = self.feedback.confirm('initial')
        if self.put_stage == 'opening' and state['classification'] == 'open' and context:
            # 仅有真实导航到投放点、释放位动作完成并已尝试张爪，才能补投放回执。
            if getattr(self.session, 'drop_arrived', False):
                receipt = released_receipt(self, context, state)
                self.session.last_put_receipt = receipt
            self.holding = self.pending_pick = None
            self._load_unknown = False
            self.put_stage = 'released'
            context = None
            self.session.checkpoint()
        elif context and self.put_stage == 'released':
            # 从旧快照恢复时以已完成的释放回执为准。
            self.holding = self.pending_pick = None
            self._load_unknown = False
            context = None
        elif self._load_unknown or self.pending_pick:
            if state['classification'] == 'open':
                self.holding = None
            else:
                context = context or self._recovered_context()
                point = next((p for p in self.session.config.values['detection_points']
                              if p['id'] == context['point_id']), None)
                profile = self.cfg['profiles'][point['grasp_profile']] if point else {}
                self.driver.fingers(profile.get('finger_tight', self.cfg['finger_tight']))
                settled = self.feedback.confirm('grasp', context['attempt_id'], context['label'])
                if settled['holding']:
                    context['grasp_confirmed'] = bool(settled.get('holding_confirmed'))
                    self.holding = copy.deepcopy(context)
                else:
                    self.holding = None
            self._load_unknown = False
        # 在前伸途中失败，已经走过的步骤保存下来；只执行剩余的既有回程。
        path_context = self.holding or self.pending_pick or context
        if path_context and path_context.get('grasp_phase') in ('approach','closing','retreat'):
            if not self.holding:
                self.driver.fingers(self.cfg['finger_open'])
            path = path_context.get('retreat_poses', [])
            start = path_context.get('retreat_index', 0)
            for index in range(start, len(path)):
                self.driver.move(path[index], verify_pose=False, stage='恢复抓取回程 '+str(index+1))
                path_context['retreat_index'] = index+1
                self.session.checkpoint()
            path_context['grasp_phase'] = 'home'
        if self.holding is None:
            self.driver.fingers(self.cfg['finger_open'])
        self._home()
        if self.wrist is not None and self.wrist.busy:
            self.wrist.close()
            self.wrist_restart_needed = True
        self.pending_pick = None
        self.put_stage = None
        self.session.checkpoint()
        return {'holding':self.holding is not None, 'gripper_opened':self.holding is None,
                'home_completed':True}

    def fallback_reset(self):
        """恢复代码本身报错时使用最小路径；仍核对通信，保留所有疑似负载。"""
        self.session.stop_navigation()
        self.driver.cancel_motion()
        if self.holding is None and (self._load_unknown or self.pending_pick):
            self.holding = copy.deepcopy(self.pending_pick or self._recovered_context())
            self.holding['grasp_confirmed'] = False
        self._load_unknown = False
        context = self.holding or self.pending_pick
        if context and context.get('grasp_phase') in ('approach','closing','retreat'):
            for index in range(context.get('retreat_index',0),len(context.get('retreat_poses',[]))):
                self.driver.move(context['retreat_poses'][index],verify_pose=False,stage='最小恢复回程')
                context['retreat_index'] = index+1
                self.session.checkpoint()
        if self.holding is None:
            self.driver.fingers(self.cfg['finger_open'])
        self._home()
        self.pending_pick = None
        if self.wrist is not None and self.wrist.busy:
            self.wrist.close()
            self.wrist_restart_needed = True
        self.session.checkpoint()
        return {'holding':bool(self.holding),'gripper_opened':not bool(self.holding),'home_completed':True}

    def _check_grasp_geometry(self, point):
        """仅在要使用该家具时核对坐标公式所需几何条件，不属于启动预检。"""
        errors = []
        kind = point['grasp_profile']
        if kind == 'ground':
            from core.storage_configuration import validate_ground_point, validate_ground_profile
            profile = self.cfg['profiles']['ground']
            validate_ground_profile(profile, errors)
            validate_ground_point(point, profile, errors)
        elif kind == 'shelf':
            for layer in point['layers']:
                for pose in [layer['detect_pose']] + layer.get('alternate_detect_poses', []):
                    self.driver.check_pose(pose)
                    if any(abs(a-b) > .01 for a,b in zip(
                            pose[3:], self.cfg['profiles']['shelf']['detect_euler_deg'])):
                        errors.append('柜层朝向与现有坐标公式不符')
        if errors:
            raise ParameterFault('detection_points.'+point['id'], '抓取参数：'+'；'.join(errors))

    def grasp(self, target, point):
        self._check_grasp_geometry(point)
        if point['difficulty']==3:
            state = self.storage_states.get(point['furniture_id'],{})
            if state.get('status')!='open_motion_completed':
                return {'status':'storage_uncertain','grasp_confirmed':False,'physical_action':False}
        if self.load_state!='empty':
            raise StateMismatch('已有负载或状态待恢复，先投放/恢复再开始新抓取')
        self.require_travel_ready()
        label = self.cfg['profiles'][point['grasp_profile']]['label_map'].get(target.get('label'))
        if target.get('label') is not None and label is None:
            return {'status':'blocked','grasp_confirmed':False,'physical_action':False}
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

    def grasp_closed_storage(self, target, point):
        """兼容旧扩展入口；Workflow已先调用open_storage，不在这里盲目开门。"""
        if point.get('grasp_profile') not in ('shelf','ground'):
            return {'status':'unsupported','physical_action':False,'grasp_confirmed':False}
        return self.grasp(target,point)

    def open_storage(self, point):
        from .open_cabinet import open_cabinet
        from .open_drawer import open_drawer
        return {'cabinet':open_cabinet,'drawer':open_drawer}[point['storage']['kind']](self,point)

    def discover(self, point):
        self._check_grasp_geometry(point)
        from .observe import discover
        return discover(self,point)

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
