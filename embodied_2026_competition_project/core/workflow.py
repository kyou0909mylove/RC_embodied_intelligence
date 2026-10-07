# -*- coding: utf-8 -*-
"""按配置顺序逐轮搜索：有限恢复、一次一件、受阻先释放、时间不足持物离场。

策略在这里；设备连接在session，抓取路径在arm各模块，阈值在local.json。
普通点位/软件错误只影响本轮。设备持续不可达、错误参数及主动停止向上传递。
"""
import copy
import time
from arm import actions
from camera.vision import CameraFailed
from .diagnostics import software_error
from .faults import DeviceUnavailable, ParameterFault, StateMismatch
from .recovery import Recovery
from .safety import ExitRequested, OperationStopped, interruptible
from .strategy import PointScheduler
from .session import NavigationFailed


class Workflow:
    def __init__(self, session):
        self.session,self.cfg = session,session.config.values
        self.announced_counts = {}
        self.recovery = Recovery(session,self._stage)
        self.active_stage = 'START'
        self.camera_timeouts = {'Vision':0,'WristVision':0}

    def _stage(self, name, **details):
        self.active_stage = name
        self.session.evidence.event('stage',name=name,**details)
        print('[阶段] '+name,flush=True)

    def _may_retry(self):
        s = self.session
        return not s.gate.stopped and not s.rospy.is_shutdown() and s.clock()<s.match_deadline

    def _wait(self, seconds):
        self.recovery.wait(seconds)

    def _point_for(self, context):
        return next((p for p in self.cfg['detection_points']
                     if p['id']==context.get('point_id')),None)

    def _refresh_navigation(self, reason):
        """清旧代价后等激光重建；不能把仍在场的机器人从障碍地图永久删除。"""
        s = self.session
        self._stage('REFRESH_OBSTACLES',reason=reason)
        try:
            s.refresh_navigation(reason)
        except (ExitRequested,DeviceUnavailable,ParameterFault):
            raise
        except Exception as exc:
            s.check()
            s.evidence.event('costmap_refresh_failed',reason=reason,error=str(exc))
            print('[导航提醒] 清图未完成，继续使用当前代价地图：'+str(exc),flush=True)
        self._wait(self.cfg['recovery']['costmap_settle_s'])

    def _navigate(self, name, pose, retries=None):
        """只把move_base=3算到达；4等状态不是自动转圈命令。"""
        s = self.session
        retries = self.cfg['recovery']['nav_retries'] if retries is None else retries
        for attempt in range(retries+1):
            try:
                s.navigate(name,pose)
                return
            except (ExitRequested,DeviceUnavailable,ParameterFault):
                raise
            except Exception as exc:
                s.check()
                if isinstance(exc,StateMismatch) and s.actions:
                    self.recovery.arm('发送导航前恢复收臂/负载状态')
                elif (not isinstance(exc,NavigationFailed) or exc.state==9):
                    # 父阶段已经耗尽时先离开该阶段，不能在旧计时器里诊断出假通信超时。
                    if s.clock()<s.gate.deadline:
                        self.recovery.probe('导航')
                if attempt==retries:
                    raise
                s.evidence.event('navigation_retry',name=name,attempt=attempt+1,reason=str(exc))
                self._stage('WAIT_NAV_RETRY',seconds=self.cfg['recovery']['retry_delay_s'])
                self._wait(self.cfg['recovery']['retry_delay_s'])
                self._refresh_navigation(name+'重试')

    def _speak(self, text, kind, simulated_detection=None, blocking=True):
        s,policy = self.session,self.cfg['speech']
        timeout = min(policy['max_timeout_s'],max(policy['timeout_s'],5+len(text)*.3))
        for attempt in range(self.cfg['recovery']['speech_retries']+1):
            try:
                with s.operation(timeout+policy['completion_slack_s']),interruptible(s.gate):
                    s.evidence.event('speech_requested',text=text,kind=kind,
                        simulated_detection=simulated_detection,hardware_request=not s.speech_print)
                    if s.speech_print:
                        print('[语音打印] '+text,flush=True)
                        if blocking:
                            self._wait(policy['pause_s'])
                        return True
                    if s.speaker.tts_pub.get_num_connections()==0:
                        self.recovery.probe('语音')
                    s.evidence.record['speech_requests'] += 1
                    wait_done = blocking and policy['wait_done']
                    done = s.speaker.speak(text,wait_done=wait_done,timeout=timeout)
                    if not done:
                        if s.speaker.tts_pub.get_num_connections()==0:
                            self.recovery.probe('语音')
                        raise RuntimeError('播报请求/匹配的summer_tts_done未确认')
                    if wait_done:
                        s.evidence.record['speech_done_requests'] += 1
                    elif blocking:
                        self._wait(policy['pause_s'])
                    s.evidence.event('speech_completion',text=text,
                                     backend_done_confirmed=bool(wait_done and done),queued=not blocking)
                    return True
            except (ExitRequested,DeviceUnavailable,ParameterFault):
                raise
            except Exception as exc:
                s.check()
                s.evidence.event('speech_retry_or_skip',kind=kind,error=str(exc),attempt=attempt+1)
                if attempt<self.cfg['recovery']['speech_retries']:
                    self._wait(self.cfg['recovery']['retry_delay_s'])
        # 节点仍连接但done不匹配/播放软件异常：记录后继续，不反复扫描同一目标。
        return False

    def _head_scan(self, point):
        s = self.session
        if getattr(s,'head_restart_needed',False):
            self.recovery.camera(s.vision,'头部相机','上一扫描已过期')
            s.head_restart_needed = False
        limit = self.cfg['recovery']['camera_restarts']
        for attempt in range(limit+1):
            try:
                with s.operation(self.cfg['timing']['scan_timeout_s']),interruptible(s.gate):
                    return s.vision.scan(point,s.gate)
            except (ExitRequested,DeviceUnavailable,ParameterFault):
                raise
            except CameraFailed as exc:
                if not exc.device_related:
                    raise  # 推理/回包代码问题由本点恢复，不能被当成家具为空。
                if attempt==limit:
                    raise DeviceUnavailable('头部相机','有限重启后仍采图/通信失败：'+str(exc)) from exc
                self.recovery.camera(s.vision,'头部相机',exc)

    def _announce_all(self, point, candidates, source='head'):
        """报出全部稳定实例与同类数量；腕部非阻塞播报，保留测量的新鲜度。"""
        counts = {}
        for item in candidates:
            counts[item['label']] = counts.get(item['label'],0)+1
        names = []
        for label,count in counts.items():
            if count<=self.announced_counts.get(label,0):
                continue
            name = self.cfg['speech']['label_names'].get(label,label)
            names.append(name if count==1 else '{}件{}'.format(count,name))
            self.announced_counts[label] = count
        if not names:
            return False
        s = self.session
        s.evidence.event('objects_announced',point_id=point['id'],source=source,counts=counts,names=names)
        prefix = ('模拟检测，' if s.test_mode else '')+point['name']+'发现'
        for start in range(0,len(names),4):
            text = prefix+'、'.join(names[start:start+4])
            if not self._speak(text,'detection',simulated_detection=s.test_mode,blocking=source!='wrist'):
                print('[语音未确认完成] '+text+'；继续当前抓取',flush=True)
        return True

    def _report_wrist(self, point, candidates, layer=None):
        mapping = self.cfg['actions']['profiles'][point['grasp_profile']]['label_map']
        inverse = {wrist:head for head,wrist in mapping.items()}
        return self._announce_all(point,[dict(item,label=inverse[item['label']]) for item in candidates
                                        if item['label'] in inverse],source='wrist')

    def _register_grasp(self, point, held):
        """记录疑似负载也保留实例编号；只有稳定持物反馈才增加真实抓取数。"""
        s = self.session
        oid = held.get('object_id') or held.get('attempt_id')
        if not oid:
            oid = s.actions._recovered_context()['object_id']
            held.update(object_id=oid,attempt_id=oid,grasp_confirmed=False)
        if held.get('grasp_confirmed',True) and oid not in s.evidence.record['confirmed_object_ids']:
            s.evidence.record['confirmed_object_ids'].append(oid)
            s.evidence.record['physical_grasps'] += 1
        if s.scheduler and point and point['id'] in s.scheduler.states:
            self._schedule_call('register_pick',point,oid,held.get('label','unknown'))
        s.checkpoint()

    def _register_delivery(self, receipt):
        s = self.session
        point,oid = self._point_for(receipt),receipt.get('object_id')
        if receipt.get('delivery_confirmed') and oid:
            if oid in s.evidence.record['confirmed_object_ids']:
                if oid not in s.evidence.record['delivered_object_ids']:
                    s.evidence.record['delivered_object_ids'].append(oid)
                    s.evidence.record['physical_deliveries'] += 1
                    s.evidence.record['delivery_actions_completed'] += 1
                    if receipt.get('zone_sensor_verified'):
                        s.evidence.record['zone_sensor_verified_deliveries'] += 1
                if (s.scheduler and point and point['id'] in s.scheduler.states
                        and not s.gate.stopped and s.clock()<s.match_deadline):
                    self._schedule_call('register_delivery',point,oid)
            else:
                # 记录缺失影响计数，不阻止已经空爪的机器人继续比赛。
                receipt = dict(receipt,delivery_confirmed=False,record_warning='缺少确认抓取记录')
        elif receipt.get('status')=='lost_in_transit' and s.scheduler and point:
            self._schedule_call('register_delivery',point,oid,True)
        s.evidence.event('delivery_recorded',**receipt)
        s.last_put_receipt = None
        s.checkpoint()

    def _abandon_before_next_point(self, reason):
        """投放导航耗尽：在当前安全收臂处张爪并确认开爪，再去下一家具。"""
        s = self.session
        self._stage('ABANDON_LOAD',reason=reason)
        for attempt in range(self.cfg['recovery']['reset_retries']+1):
            try:
                with s.operation(self.cfg['actions']['abandon_timeout_s']),interruptible(s.gate):
                    s.actions.abandon(reason)
                return 'abandoned'
            except (ExitRequested,ParameterFault):
                raise
            except Exception as exc:
                s.check()
                software_error(s.evidence,'ABANDON_LOAD',exc)
                self.recovery.arm('放弃负载后的张爪/收臂恢复')
                if s.actions.load_state=='empty':
                    return 'abandoned'
                if attempt==self.cfg['recovery']['reset_retries']:
                    raise DeviceUnavailable('机械臂执行','张爪重试耗尽，仍未确认释放；禁止带物去家具') from exc
        return 'abandoned'

    def _deliver_pending(self):
        """最多3次导航、总窗口120秒；到点直接PUT，不预先读取持物反馈。"""
        s = self.session
        if s.actions.load_state=='unknown':
            self.recovery.arm('投放前恢复软件负载状态')
        held = s.actions.holding
        if not held:
            return 'load_missing'
        point = self._point_for(held)
        self._register_grasp(point,held)
        s.drop_arrived = False
        self._stage('DROP_NAV',point_id=held.get('point_id'))
        failure = None
        try:
            with s.operation(self.cfg['recovery']['drop_retry_timeout_s']):
                for attempt in range(self.cfg['recovery']['drop_max_attempts']):
                    try:
                        self._navigate('己方得分区停靠点',self.cfg['locations']['drop'],retries=0)
                        s.drop_arrived = True
                        break
                    except (ExitRequested,DeviceUnavailable,ParameterFault):
                        raise
                    except Exception as exc:
                        s.check()
                        failure = exc
                        s.evidence.event('drop_navigation_retry',attempt=attempt+1,error=str(exc),
                                         object_id=held['object_id'])
                        if attempt+1==self.cfg['recovery']['drop_max_attempts']:
                            break
                        self._stage('WAIT_DROP_RETRY',attempt=attempt+1,
                                    seconds=self.cfg['recovery']['drop_retry_wait_s'])
                        self._wait(self.cfg['recovery']['drop_retry_wait_s'])
                        self._refresh_navigation('持物投放重试')
        except (ExitRequested,DeviceUnavailable,ParameterFault):
            raise
        except Exception as exc:
            s.check()
            failure = exc
        if not s.drop_arrived:
            self.recovery.probe('导航')  # 退出旧窗口后诊断；通信仍在才允许换点。
            return self._abandon_before_next_point('投放点限次/限时仍不可达：'+str(failure))
        s.checkpoint()
        for attempt in range(self.cfg['recovery']['reset_retries']+1):
            self._stage('PUT',attempt=attempt+1,point_id=held.get('point_id'))
            try:
                with s.operation(self.cfg['actions']['place_timeout_s']),interruptible(s.gate):
                    result = s.actions.put(held.get('target',{}),self.cfg['locations']['drop'])
                s.evidence.record['place_interface_calls'] += 1
                self._register_delivery(result)
                return result['status']
            except (ExitRequested,ParameterFault):
                raise
            except Exception as exc:
                s.check()
                software_error(s.evidence,'PUT',exc)
                self.recovery.arm('PUT未完成，先取消/收臂再有限重试')
                if s.last_put_receipt:
                    result = s.last_put_receipt
                    s.evidence.record['place_interface_calls'] += 1
                    self._register_delivery(result)
                    return result['status']
                if s.actions.load_state=='empty':
                    return 'released_without_receipt'
                if attempt==self.cfg['recovery']['reset_retries']:
                    return self._abandon_before_next_point('PUT软件恢复后仍未完成')
        return 'put_failed'

    def _inspect_point(self, point):
        s = self.session
        if point['difficulty']==3 and not s.test_mode:
            self._stage('OPEN_STORAGE',point_id=point['id'])
            with s.operation(point['storage']['timeout_s']),interruptible(s.gate):
                opening = s.actions.open_storage(point)
            if opening['status']!='opened':
                return opening['status'],False
        self._stage('HEAD_DETECT',point_id=point['id'])
        if s.test_mode:
            target = {'label':point.get('test_label','cola'),'simulated':True}
            observation = {'point_id':point['id'],'target':target,'candidates':[target],'valid_frames':0}
        else:
            observation = self._head_scan(point)
        s.evidence.event('head_scan_completed',**observation)
        candidates = observation.get('candidates') or ([observation['target']] if observation.get('target') else [])
        self._announce_all(point,candidates)
        if s.test_mode:
            actions.grasp(candidates[0],point)
            self._navigate('己方得分区停靠点',self.cfg['locations']['drop'])
            actions.put(candidates[0],self.cfg['locations']['drop'])
            return 'print',False
        attempt = 0
        s.actions.observation_reporter = self._report_wrist
        try:
            while True:
                s.check()
                attempt += 1
                self._stage('WRIST_AND_GRASP',point_id=point['id'],attempt=attempt)
                with s.operation(point.get('grasp_timeout_s',self.cfg['actions']['grasp_timeout_s'])),interruptible(s.gate):
                    result = s.actions.grasp({},point)
                s.evidence.record['grasp_interface_calls'] += 1
                s.evidence.event('grasp_result',**dict(result,point_id=point['id']))
                if s.actions.load_state=='holding':
                    return 'grasped',bool(s.actions.holding.get('grasp_confirmed',True))
                if s.actions.load_state!='empty':
                    raise StateMismatch('抓取后负载待恢复')
                outcome = result['status']
                if outcome=='no_target':
                    if not candidates and not observation.get('detections'):
                        return 'no_target',False
                    outcome = 'blocked'  # 头部看到但腕部没定位，不能判空。
                if outcome in ('time_insufficient','unsupported','storage_uncertain'):
                    return outcome,False
                self._stage('RETRY_POINT',point_id=point['id'],reason=result.get('reason',outcome))
                self._wait(self.cfg['recovery']['retry_delay_s'])
        finally:
            s.actions.observation_reporter = None

    def _reset_point_attempt(self, point, reason):
        result = self.recovery.arm(reason)
        s = self.session
        if s.vision is not None and getattr(s.vision,'busy',False):
            s.vision.close()
            s.head_restart_needed = True
        s.evidence.event('point_attempt_reset',point_id=point['id'],reason=reason,**result)
        return result

    def _one_point(self, point):
        s = self.session
        self._stage('DETECT_NAV',point_id=point['id'],furniture_id=point['furniture_id'])
        try:
            self._navigate(point['name'],point['pose'])
        except (ExitRequested,DeviceUnavailable,ParameterFault):
            raise
        except Exception as exc:
            s.check()
            self.recovery.probe('导航')
            s.evidence.record['detection_points_skipped'].append(point['id'])
            s.evidence.event('point_temporarily_unreachable',point_id=point['id'],error=str(exc))
            print('[本轮跳过] '+point['name']+'：下轮重新导航',flush=True)
            return 'unreachable',False
        self.announced_counts = {}
        timeout = point.get('wait_timeout_s',max(self.cfg['recovery']['point_wait_timeout_s'],
                                                point.get('grasp_timeout_s',0)))
        point_end = s.clock()+timeout
        limit = self.cfg['recovery']['camera_restarts']
        outcome,confirmed = 'recoverable_error',False
        for attempt in range(limit+1):
            try:
                with s.operation(max(.001,point_end-s.clock())):
                    outcome,confirmed = self._inspect_point(point)
                self.camera_timeouts = {'Vision':0,'WristVision':0}
                break
            except (ExitRequested,ParameterFault):
                raise
            except DeviceUnavailable as exc:
                # 短暂反馈缺失先复核机械臂；持续不可达仍直接向上传递。
                if '机械臂' not in exc.module:
                    raise
                self.recovery.probe('机械臂')
                reset = self._reset_point_attempt(point,str(exc))
                outcome = 'grasped' if reset['holding'] else 'feedback_recovered'
                break
            except CameraFailed as exc:
                s.check()
                wrist_failed = self.active_stage=='WRIST_AND_GRASP' or getattr(s.actions.wrist,'busy',False)
                self._reset_point_attempt(point,str(exc))
                vision = s.actions.wrist if wrist_failed else s.vision
                # reset可能已关闭busy腕部；按最近出错阶段判相机，不用已清除的busy位。
                if exc.device_related:
                    self.recovery.camera(vision,'腕部相机' if vision is s.actions.wrist else '头部相机',exc)
                    s.actions.wrist_restart_needed = False if vision is s.actions.wrist else s.actions.wrist_restart_needed
                    if attempt==limit:
                        raise DeviceUnavailable('相机','重启后仍采图失败：'+str(exc)) from exc
                    if s.clock()<point_end and s.actions.load_state=='empty':
                        continue
                else:
                    software_error(s.evidence,'视觉处理',exc)
                    vision.close()
                    if vision is s.actions.wrist:
                        s.actions.wrist_restart_needed = True
                    else:
                        s.head_restart_needed = True
                outcome = 'camera_recovered_error'
                break
            except Exception as exc:
                s.check()
                software_error(s.evidence,self.active_stage,exc)
                reset = self._reset_point_attempt(point,str(exc))
                camera = getattr(exc,'camera_source',None)
                if isinstance(exc,OperationStopped) and getattr(exc,'camera_phase',None)=='capture' and camera:
                    self.camera_timeouts[camera] += 1
                    vision = s.actions.wrist if camera=='WristVision' else s.vision
                    self.recovery.camera(vision,'腕部相机' if camera=='WristVision' else '头部相机',exc)
                    if camera=='WristVision':
                        s.actions.wrist_restart_needed = False
                    else:
                        s.head_restart_needed = False
                    if self.camera_timeouts[camera]>self.cfg['recovery']['camera_restarts']:
                        raise DeviceUnavailable('相机采图','SDK采图重启后仍持续无响应：'+str(exc)) from exc
                outcome = 'grasped' if reset['holding'] else ('point_timeout' if isinstance(exc,OperationStopped) else 'software_error')
                break
        if s.actions and s.actions.holding:
            held = s.actions.holding
            confirmed = bool(held.get('grasp_confirmed',True))
            self._register_grasp(point,held)
            return self._deliver_pending(),confirmed
        if s.actions and not s.actions.travel_ready:
            self._reset_point_attempt(point,'本点结束前收臂')
            if s.actions.holding:
                confirmed = bool(s.actions.holding.get('grasp_confirmed',True))
                return self._deliver_pending(),confirmed
        if point['id'] not in s.evidence.record['detection_points_completed']:
            s.evidence.record['detection_points_completed'].append(point['id'])
        return outcome,confirmed

    def _initialize_route(self):
        s = self.session
        if not s.test_mode:
            try:
                with s.operation(self.cfg['timing']['state_timeout_s']),interruptible(s.gate):
                    s.scheduler = PointScheduler(self.cfg['detection_points'],self.cfg['strategy'],s.clock,
                        s.evidence.event,copy.deepcopy((s.restored or {}).get('scheduler')))
            except (ExitRequested,DeviceUnavailable,ParameterFault):
                raise
            except Exception as exc:
                s.check()
                software_error(s.evidence,'RESTORE_SCHEDULER',exc)
                # 物理抓放去重保存在record，不依赖可重建的访问顺序快照。
                s.scheduler = PointScheduler(self.cfg['detection_points'],self.cfg['strategy'],
                                             s.clock,s.evidence.event)
            confirmed = set(getattr(s,'opened_storage',[]))
            for p in s.scheduler.points:
                if p['difficulty']==3 and p['furniture_id'] in confirmed:
                    s.scheduler.states[p['id']].update(disabled=False,empty_streak=0,blocked_until=0)
        if s.last_put_receipt:
            self._register_delivery(s.last_put_receipt)
        if s.actions and (s.actions.holding or s.actions.load_state=='unknown'):
            self._stage('RESUME_DELIVERY')
            self._deliver_pending()
        if (not s.initial_nav_completed and not s.entry_completed
                and self.cfg['localization']['navigate_to_initial_pose']):
            self._stage('INITIAL_NAV')
            self._navigate('初始点',self.cfg['initial_pose'])
            s.initial_nav_completed = True
            if self.cfg['initial_pose']==self.cfg['locations']['entry']:
                s.entry_completed = True
            s.checkpoint()
        if not s.entry_completed:
            self._stage('ENTRY')
            self._navigate('入场点',self.cfg['locations']['entry'])
            s.entry_completed = True
            s.checkpoint()
        if self.cfg['speech']['announce_stages']:
            self._speak('已到达入场点，开始搜索比赛物品','stage')
        if s.scheduler:
            self._refresh_navigation('第{}轮开始'.format(s.scheduler.round_number))

    def _schedule_call(self, method, *args):
        s = self.session
        try:
            with s.operation(self.cfg['timing']['state_timeout_s']),interruptible(s.gate):
                return getattr(s.scheduler,method)(*args)
        except (ExitRequested,DeviceUnavailable,ParameterFault):
            raise
        except Exception as exc:
            s.check()
            software_error(s.evidence,'SCHEDULER.'+method,exc)
            # 真实抓放计数在Evidence按object_id去重，重建调度不会重复计数。
            s.scheduler = PointScheduler(self.cfg['detection_points'],self.cfg['strategy'],
                                         s.clock,s.evidence.event)
            s.evidence.event('scheduler_rebuilt',reason=str(exc))
            if method=='next_point':
                return s.scheduler.points[0] if s.scheduler.points else None
            if method in ('start_next_round','pending'):
                return bool(s.scheduler.points)
            return None

    def _search(self):
        s = self.session
        sequential = iter(p for p in self.cfg['detection_points'] if p.get('enabled',True))
        while True:
            s.check()
            if s.actions and s.actions.load_state!='empty':
                self._deliver_pending()
                continue
            point = self._schedule_call('next_point') if s.scheduler else next(sequential,None)
            if point is None:
                if not s.scheduler or not self._schedule_call('pending'):
                    return '所有待查点已完成配置次数的完整判空'
                self._stage('NEXT_ROUND',round_number=s.scheduler.round_number+1)
                self._wait(self.cfg['strategy']['round_wait_s'])
                pending = self._schedule_call('start_next_round')
                if not pending:
                    return '全部待查点已完成'
                self._refresh_navigation('第{}轮开始'.format(s.scheduler.round_number))
                s.checkpoint()
                continue
            s.checkpoint()
            try:
                outcome,confirmed = self._one_point(point)
            except (ExitRequested,DeviceUnavailable,ParameterFault):
                raise
            except Exception as exc:
                s.check()
                software_error(s.evidence,'POINT',exc)
                if s.actions:
                    self._reset_point_attempt(point,'点位遗漏异常恢复')
                    if s.actions.holding:
                        self._deliver_pending()
                outcome,confirmed = 'software_error',False
            if s.scheduler:
                self._schedule_call('observe',point,outcome,confirmed)
            s.checkpoint()

    def _exit(self, reason):
        """不受搜索软截止约束；允许持物。离场受阻重试直到600秒总截止。"""
        s = self.session
        with s.gate.suspend_exit():
            if s.actions and (not s.actions.travel_ready or s.actions.load_state=='unknown'):
                self.recovery.arm('离场前取消当前动作、完成既有回程并收臂；保留负载')
            # 释放已成功但收臂期间触发软截止：先补登记已保存回执，不能漏计或重放。
            if s.last_put_receipt:
                self._register_delivery(s.last_put_receipt)
            s.evidence.record['has_confirmed_delivery'] = bool(s.evidence.record['delivery_actions_completed']>=1)
            s.evidence.record['exit_motion_completed'] = False
            s.begin_exit(reason)
            attempt = 0
            while True:
                try:
                    s.check()
                    attempt += 1
                    self._stage('EXIT',attempt=attempt)
                    self._navigate('离场点',self.cfg['locations']['exit'],retries=0)
                    s.evidence.record['exit_motion_completed'] = True
                    s.status,s.exit_code = 'FLOW_COMPLETED',0
                    break
                except (DeviceUnavailable,ParameterFault):
                    raise
                except Exception as exc:
                    if s.gate.stopped or s.rospy.is_shutdown():
                        s.check()
                    if s.clock()>=s.match_deadline:
                        s.status,s.exit_code = 'TIME_UP',0
                        s.evidence.event('exit_not_completed',reason='比赛截止',last_error=str(exc))
                        break
                    s.evidence.event('exit_navigation_retry',attempt=attempt,error=str(exc))
                    self.recovery.probe('导航')
                    self._stage('WAIT_EXIT_RETRY',seconds=self.cfg['recovery']['exit_retry_wait_s'])
                    try:
                        self._wait(self.cfg['recovery']['exit_retry_wait_s'])
                        self._refresh_navigation('离场重试')
                    except OperationStopped:
                        if s.clock()>=s.match_deadline and not s.gate.stopped:
                            s.status,s.exit_code = 'TIME_UP',0
                            break
                        raise
            s.checkpoint()
            print('[结束] 状态={}；确认抓取={}，投放动作完成={}，离场到达={}'.format(
                s.status,s.evidence.record['physical_grasps'],s.evidence.record['delivery_actions_completed'],
                s.evidence.record['exit_motion_completed']),flush=True)

    def _time_up(self):
        """总截止后只登记已完成释放，不发新动作，也不重建/运行过期调度。"""
        s = self.session
        if s.last_put_receipt:
            try:
                self._register_delivery(s.last_put_receipt)
            except Exception as exc:
                software_error(s.evidence,'TIME_UP_RECEIPT',exc)
        s.status,s.exit_code = 'TIME_UP',0
        s.evidence.record['has_confirmed_delivery'] = bool(s.evidence.record['delivery_actions_completed']>=1)
        s.evidence.record['exit_motion_completed'] = False
        s.checkpoint()

    def run(self):
        s = self.session
        try:
            self._initialize_route()  # 初始/入场导航失败允许中止，搜索开始后改为本轮跳过。
            reason = self._search()
        except ExitRequested as exc:
            reason = str(exc)
            s.evidence.event('priority_exit_requested',reason=reason,stage=self.active_stage)
        except OperationStopped:
            if s.clock()>=s.match_deadline and not s.gate.stopped:
                self._time_up()
                return
            raise
        try:
            self._exit(reason)
        except OperationStopped:
            # 离场收臂、通信复核或等待中恰好到总截止，也按时间到结束，不要求resume。
            if s.clock()>=s.match_deadline and not s.gate.stopped:
                self._time_up()
                return
            raise
