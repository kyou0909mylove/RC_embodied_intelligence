# -*- coding: utf-8 -*-
"""复用原 KinovaRobot 动作函数；这里只对接任务资源和实际反馈。"""
import math
import time
import importlib
import sys
from pathlib import Path
from core.safety import GuardedClient, ActionFailed, OperationStopped, cancel_unfinished_goal
from core.faults import DeviceUnavailable, ParameterFault, PoseNotReached
from .coordinates import euler_xyz_to_quaternion


def load_original(name):
    """保留原抓取动作，通过真实模块别名满足往年脚本的局部导入。"""
    if not isinstance(name, str) or not name.isidentifier():
        raise ValueError('原功能模块名必须是 arm 内的 Python 文件名，不含扩展名')
    speaker = importlib.import_module('speech.summer_tts_speaker')
    sys.modules['summer_tts_speaker'] = speaker
    source_dir = Path(__file__).resolve().parents[1] / 'catch_ground' / 'src'
    source_package = importlib.import_module('catch_ground.src')
    if source_dir not in [Path(path).resolve() for path in source_package.__path__]:
        raise ImportError('往年功能依赖须来自本工程 catch_ground/src，不能使用外部同名包')
    originals = {}
    for dependency in ('realsense_yolo11', 'realsense_yolo11_desk', 'realsense_yolo11_ref', 'catch'):
        module = importlib.import_module('catch_ground.src.' + dependency)
        if Path(module.__file__).resolve() != source_dir / (dependency + '.py'):
            raise ImportError('原功能依赖路径不符：' + dependency)
        sys.modules[dependency] = module
        originals[dependency] = module
    if name in originals:
        return originals[name]
    module = importlib.import_module('.' + name, __package__)
    # 模型集中在 models；只在调用层指定路径，不改上传的功能文件。
    model_bindings = {
        'task2_pick_v4': ('MODEL_CANDIDATES', 'best(1).pt'),
        'xg_table_pick': ('DESK_MODEL_CANDIDATES', 'best(1).pt'),
    }
    if name in model_bindings:
        attribute, filename = model_bindings[name]
        model_dir = Path(__file__).resolve().parents[1] / 'models'
        setattr(module, attribute, [str(model_dir / filename)])
    return module


def _compatible_robot(original_class, driver):
    """在原类外限制初始化等待；动作和单位转换继续执行原类方法。"""
    class ConnectedKinovaRobot(original_class):
        def __init__(self, robot_type):
            self._initializing = True
            self._cartesian_sub = self._finger_sub = None
            try:
                super().__init__(robot_type)
            finally:
                self._initializing = False

        def _feedback_message(self, topic, message_type, callback, handle_name):
            remaining = driver.gate.check()
            if getattr(self, handle_name) is None:
                setattr(self, handle_name, driver.rospy.Subscriber(
                    topic, message_type, callback, queue_size=1))
            timeout = min(remaining, driver.cfg['initial_feedback_timeout_s'])
            try:
                message = driver.rospy.wait_for_message(topic, message_type, timeout=timeout)
            except driver.rospy.ROSException as exc:
                raise DeviceUnavailable('机械臂反馈', topic + '：' + str(exc)) from exc
            driver.gate.check()
            callback(message)

        def getcurrentCartesianCommand(self):
            import kinova_msgs.msg
            self._feedback_message('/' + self.prefix + 'driver/out/cartesian_command',
                kinova_msgs.msg.KinovaPose, self.setcurrentCartesianCommand, '_cartesian_sub')

        def getCurrentFingerPosition(self):
            import kinova_msgs.msg
            self._feedback_message('/' + self.prefix + 'driver/out/finger_position',
                kinova_msgs.msg.FingerPosition, self.setCurrentFingerPosition, '_finger_sub')

        def arm_run(self, unit='mdeg', pose_target=None, relative=False):
            if self._initializing:
                return None
            return super().arm_run(unit=unit, pose_target=pose_target, relative=relative)

        def finger_run(self, unit='percent', finger_target=None, relative=False):
            if self._initializing:
                return None
            return super().finger_run(unit=unit, finger_target=finger_target, relative=relative)

    return ConnectedKinovaRobot


class KinovaDriver:
    def __init__(self, config, rospy, gate):
        self.cfg, self.rospy, self.gate = config, rospy, gate
        self.robot = None
        self.arm_client = self.finger_client = self.pose_sub = self.finger_sub = None
        self.latest_pose = self.latest_fingers = None
        self.finger_sequence = 0
        self.status_subscribers = []
        self.latest_statuses = {}

    def connect(self):
        KinovaRobot = load_original('catch').KinovaRobot
        import kinova_msgs.msg
        import geometry_msgs.msg
        from actionlib_msgs.msg import GoalStatusArray
        # 原构造器的自动张爪/回位由 startup 统一执行，恢复持物时不执行。
        # 先保存实例，原初始化失败时 close 仍能释放已创建的反馈订阅。
        robot_class = _compatible_robot(KinovaRobot, self)
        self.robot = robot_class.__new__(robot_class)
        robot_class.__init__(self.robot, self.cfg['robot_type'])
        self.arm_client = getattr(self.robot, 'client_arm', None)
        self.finger_client = getattr(self.robot, 'client_finger', None)
        if (self.arm_client is None or self.finger_client is None or
                not hasattr(self.robot, 'goal_arm') or not hasattr(self.robot, 'goal_finger') or
                self.rospy.is_shutdown()):
            raise DeviceUnavailable('机械臂','原KinovaRobot未完成驱动连接')
        prefix = '/' + self.cfg['robot_type'] + '_driver/'
        # 沿用任务的停止/截止处理，包装原客户端；不创建第二套客户端或目标消息。
        self.arm_client = GuardedClient(self.arm_client, self.gate, self.rospy.Duration,
                                        name=prefix+'pose_action/tool_pose',
                                        result_timeout_s=self.cfg['arm_action_timeout_s'],
                                        log_commands=True, require_success=False)
        self.finger_client = GuardedClient(self.finger_client, self.gate, self.rospy.Duration,
                                           name=prefix+'fingers_action/finger_positions',
                                           result_timeout_s=self.cfg['finger_action_timeout_s'],
                                           log_commands=True, require_success=False)
        self.robot.client_arm = self.arm_client
        self.robot.client_finger = self.finger_client
        self.robot.goal_arm.pose.header.frame_id = self.cfg['tool_frame']
        self.pose_sub = self.rospy.Subscriber(prefix+'out/tool_pose',
            geometry_msgs.msg.PoseStamped, self._pose_received, queue_size=1)
        self.finger_sub = self.rospy.Subscriber(prefix+'out/finger_position',
            kinova_msgs.msg.FingerPosition, self._fingers_received, queue_size=1)
        # 新进程没有本客户端GoalHandle时，get_state()本身也返回LOST。
        # 恢复不能把这个默认值当旧目标仍在活动，须核对服务器的新状态数组。
        for kind,namespace in (('arm','pose_action/tool_pose'),('finger','fingers_action/finger_positions')):
            self.status_subscribers.append(self.rospy.Subscriber(prefix+namespace+'/status',
                GoalStatusArray,lambda message,kind=kind:self._status_received(kind,message),queue_size=1))

    def _status_received(self, kind, message):
        self.latest_statuses[kind] = (self.gate.clock(),tuple(status.status for status in message.status_list))

    def _pose_received(self, message):
        if str(getattr(message,'_connection_header',{}).get('latching','0')) != '1':
            self.latest_pose = (self.gate.clock(), message)

    def _fingers_received(self, message):
        if str(getattr(message,'_connection_header',{}).get('latching','0')) == '1':
            return
        values = tuple(float(getattr(message, 'finger'+str(i))) for i in (1,2,3))
        if not all(math.isfinite(v) and 0<=v<=7000 for v in values):
            return
        self.finger_sequence += 1
        self.latest_fingers = (self.finger_sequence, values, self.gate.clock())

    def finger_sample(self):
        """返回新鲜反馈快照；实际发布次数用于防止把同一条消息重复当成多个样本。"""
        self.gate.check()
        result = self.latest_fingers
        if result and self.gate.clock()-result[2] <= self.cfg['gripper_feedback']['max_age_s']:
            return result
        return None

    def tool_pose(self, expected=None):
        end = min(self.gate.deadline, self.gate.clock()+self.cfg['tool_pose_timeout_s'])
        latest_valid = None
        while self.gate.clock() < end:
            self.gate.check()
            snapshot = self.latest_pose
            if snapshot and self.gate.clock()-snapshot[0] <= self.cfg['tool_pose_max_age_s']:
                message = snapshot[1]
                if message.header.frame_id != self.cfg['tool_frame']:
                    raise ParameterFault('actions.tool_frame','与tool_pose的frame_id不符：'+message.header.frame_id)
                p,q = message.pose.position,message.pose.orientation
                xyz,quat = [p.x,p.y,p.z],[q.x,q.y,q.z,q.w]
                if (not all(math.isfinite(v) for v in xyz+quat) or
                        any(abs(v)>3 for v in xyz) or sum(v*v for v in quat)<1e-12):
                    raise DeviceUnavailable('机械臂位姿反馈','坐标或四元数无效')
                norm = math.sqrt(sum(v*v for v in quat))
                quat = [v/norm for v in quat]
                latest_valid = (xyz,quat)
                if expected is not None:
                    wanted = euler_xyz_to_quaternion([math.radians(v) for v in expected[3:]])
                    angle = 2*math.acos(min(1.0,abs(sum(a*b for a,b in zip(quat,wanted)))))
                    if math.dist(xyz,expected[:3])>self.cfg['pose_tolerance_m'] or angle>math.radians(self.cfg['orientation_tolerance_deg']):
                        time.sleep(.02)
                        continue
                return xyz,quat
            time.sleep(.02)
        self.gate.check()
        if latest_valid is not None and expected is not None:
            exc = PoseNotReached('有效反馈未满足软件到位容差')
            exc.actual = latest_valid
            raise exc
        raise DeviceUnavailable('机械臂位姿反馈','持续没有新鲜tool_pose')

    def check_pose(self, pose):
        # 用户取消自定义XYZ包围范围；保留数值/单位契约，不宣称做了碰撞规划。
        if not isinstance(pose,list) or len(pose)!=6 or not all(
                type(v) in (int,float) and math.isfinite(v) for v in pose):
            raise ValueError('机械臂姿态须为 [x,y,z,tx,ty,tz]，米/度')
        if any(abs(v)>3 for v in pose[:3]):
            raise ValueError('机械臂坐标明显离谱：检查米/毫米单位及视觉坐标')

    def _motion_details(self, pose, stage, started):
        """只读取已有反馈，不为诊断发送动作或延长等待。"""
        now = self.gate.clock()
        details = {'stage':stage,'target_pose_mdeg':list(pose),'tool_frame':self.cfg['tool_frame']}
        snapshot = self.latest_pose
        if snapshot is None:
            details['feedback_note'] = '没有工具位姿反馈'
            return details
        received, message = snapshot
        details.update(feedback_age_s=max(0.0,now-received),feedback_after_goal=received>=started)
        if now-received>self.cfg['tool_pose_max_age_s']:
            details['feedback_note'] = '工具位姿反馈已过期，不能用于判断本次到位'
            return details
        if message.header.frame_id!=self.cfg['tool_frame']:
            details['feedback_note'] = '工具位姿坐标系与配置不符'
            return details
        p,q = message.pose.position,message.pose.orientation
        xyz,quat = [p.x,p.y,p.z],[q.x,q.y,q.z,q.w]
        if not all(math.isfinite(v) for v in xyz+quat) or abs(sum(v*v for v in quat)-1)>.02:
            details['feedback_note'] = '工具位姿反馈数值无效'
            return details
        norm = math.sqrt(sum(v*v for v in quat))
        quat = [v/norm for v in quat]
        wanted = euler_xyz_to_quaternion([math.radians(v) for v in pose[3:]])
        angle = 2*math.acos(min(1.0,abs(sum(a*b for a,b in zip(quat,wanted)))))
        details.update(actual_xyz_m=xyz,actual_quaternion_xyzw=quat,
                       position_error_m=math.dist(xyz,pose[:3]),
                       orientation_error_deg=math.degrees(angle),
                       feedback_note=('本次动作期间收到的最新反馈' if received>=started
                                      else '本次动作前的反馈，不能证明失败时的位置'))
        return details

    def _check_running(self):
        self.gate.check()
        if self.rospy.is_shutdown():
            raise OperationStopped('ROS节点已关闭，停止机械臂动作')

    def move(self, pose, verify_pose=False, stage=None):
        self._check_running()
        self.check_pose(pose)
        is_home = (verify_pose or stage in ('收臂','原文件收臂') or
                   (stage is None and (list(pose)==self.cfg['home_pose'] or
                                       list(pose)==list(self.robot.homePositionMdeg))))
        stage = stage or ('收臂' if is_home else '末端移动')
        started = self.gate.clock()
        print('[机械臂] {}，目标（米/度）={}'.format(stage,list(pose)),flush=True)
        # 米/度转换、构造 goal、发送与等待全部执行上传原函数。
        previous_policy = self.arm_client.require_success
        self.arm_client.require_success = is_home
        try:
            # 客户端有限等待；普通动作返回后继续，收臂核对本次真实成功。
            previous_sequence = self.arm_client.goal_sequence
            self.robot.arm_run(unit='mdeg', pose_target=list(pose), relative=False)
            self._check_running()
            if is_home:
                self._require_success(self.arm_client, previous_sequence)
        except ActionFailed as exc:
            details = self._motion_details(pose,stage,started)
            exc.motion_details = details
            detail_text = '步骤={}，目标={}，实测XYZ={}，位置误差={}米，姿态误差={}度，{}'.format(
                stage,list(pose),details.get('actual_xyz_m'),details.get('position_error_m'),
                details.get('orientation_error_deg'),details['feedback_note'])
            exc.args = (str(exc)+'；'+detail_text,)
            print('[机械臂动作失败] '+detail_text,flush=True)
            raise
        finally:
            self.arm_client.require_success = previous_policy
        result = self.arm_client.last_goal_result or {}
        succeeded = (self.arm_client.goal_sequence>previous_sequence and
                     result.get('sequence')==self.arm_client.goal_sequence and
                     result.get('status')=='succeeded')
        if succeeded:
            print('[机械臂] {}动作成功，继续下一步'.format(stage),flush=True)
        else:
            print('[机械臂] {}调用已返回，状态={}，{}；按往年顺序继续'.format(
                stage,result.get('state'),result.get('reason','未确认执行成功')),flush=True)
        if not verify_pose:
            return succeeded
        try:
            return self.tool_pose(expected=pose)
        except PoseNotReached as exc:
            # SUCCEEDED与新鲜反馈都存在：额外毫米/角度门槛只记提醒。
            print('[到位提醒] {}：{}；驱动动作成功，继续流程'.format(stage,exc),flush=True)
            return exc.actual

    def fingers(self, values, verify_result=False):
        self._check_running()
        if len(values)!=3 or any(not math.isfinite(v) or not 0<=v<=100 for v in values):
            raise ValueError('三指目标须为0–100百分比')
        print('[夹爪] 开始发送目标（百分比）={}'.format(list(values)),flush=True)
        previous_sequence = self.finger_client.goal_sequence
        previous_policy = self.finger_client.require_success
        self.finger_client.require_success = bool(verify_result)
        try:
            self.robot.finger_run(unit='percent', finger_target=list(values), relative=False)
            self._check_running()
            if verify_result:
                self._require_success(self.finger_client, previous_sequence)
        finally:
            self.finger_client.require_success = previous_policy
        result = self.finger_client.last_goal_result or {}
        succeeded = (self.finger_client.goal_sequence>previous_sequence and
                     result.get('sequence')==self.finger_client.goal_sequence and
                     result.get('status')=='succeeded')
        if succeeded:
            print('[夹爪] 动作成功（百分比）={}'.format(list(values)),flush=True)
        else:
            print('[夹爪] 调用已返回，状态={}，{}；按往年顺序继续'.format(
                result.get('state'),result.get('reason','未确认执行成功')),flush=True)
        return succeeded

    def arm_run(self,unit='mdeg',pose_target=None,relative=False,stage=None):
        """原抓取脚本的调用形式；动作发送/换算仍执行catch.KinovaRobot。"""
        if unit!='mdeg' or relative:
            raise ValueError('主流程抓取接口使用绝对米/度姿态')
        pose = []
        for value in pose_target:
            if isinstance(value, bool) or getattr(value, 'dtype', None) == bool:
                raise ValueError('机械臂位姿不能使用布尔值')
            # 原地面坐标转换产生单元素numpy数组；只提取标量，不改变坐标。
            scalar = value.item() if hasattr(value, 'item') else value
            pose.append(float(scalar))
        return self.move(pose,verify_pose=False,stage=stage)

    def finger_run(self,unit='percent',finger_target=None,relative=False):
        """原抓取脚本的百分比夹爪接口。"""
        if unit!='percent' or relative:
            raise ValueError('主流程抓取接口使用绝对夹爪百分比')
        return self.fingers(list(finger_target))

    def _require_success(self, client, previous_sequence):
        # 原方法无返回值，而且可能吞ROSInterruptException；只接受本次新Goal
        # 已由GuardedClient读取的成功结果，不重复读状态，也不沿用上一Goal。
        result = client.last_goal_result or {}
        if client.goal_sequence <= previous_sequence:
            reason = '原动作函数已返回，但本次没有成功下发新Goal（前#{}，后#{}）'.format(
                previous_sequence, client.goal_sequence)
            if result.get('reason'):
                reason += '：'+result['reason']
            raise ActionFailed(client.name, None, reason)
        if (client.successful_goal_sequence != client.goal_sequence or
                result.get('sequence') != client.goal_sequence or
                result.get('status') != 'succeeded' or result.get('state') != 3):
            reason = result.get('reason') or '原动作函数已返回，但本次新Goal没有确认SUCCEEDED'
            raise ActionFailed(client.name, result.get('state'), reason)

    def cancel_motion(self):
        """复位前仅取消本客户端的目标，并等待其结束，避免旧目标覆盖复位动作。"""
        for kind,client in (('arm',self.arm_client),('finger',self.finger_client)):
            if client is None:
                continue
            if hasattr(client,'gh') and client.gh is None:
                self._confirm_server_idle(kind)
                continue
            cancel_unfinished_goal(client)
            end = min(self.gate.deadline, self.gate.clock() + self.cfg['cancel_timeout_s'])
            while client.get_state() in (0, 1, 6, 7) and self.gate.clock() < end:
                self.gate.check()
                time.sleep(.05)
            if client.get_state() not in (2, 3, 4, 5, 8):
                raise DeviceUnavailable('机械臂取消接口','旧动作未确认结束，不发送叠加目标')

    def _confirm_server_idle(self, kind):
        """没有当前客户端目标可取消时，等待服务器确认无活动目标。

        不用cancel_all_goals取消其他控制程序的目标。旧进程通常已在退出时
        取消自身目标；如果仍有活动目标，等待有限时间仍不结束就暂停排障。
        """
        end = min(self.gate.deadline,self.gate.clock()+self.cfg['cancel_timeout_s'])
        while self.gate.clock()<end:
            self.gate.check()
            sample = self.latest_statuses.get(kind)
            if (sample and self.gate.clock()-sample[0]<=self.cfg['tool_pose_max_age_s']
                    and all(state in (2,3,4,5,8) for state in sample[1])):
                return
            time.sleep(.05)
        self.gate.check()
        raise DeviceUnavailable('机械臂取消接口',
            kind+'新客户端无旧GoalID，服务器未确认空闲；检查旧/其他控制程序或status反馈')

    def check_connection(self, timeout):
        end = min(self.gate.deadline,self.gate.clock()+timeout)
        for client in (self.arm_client,self.finger_client):
            left = end-self.gate.clock()
            self.gate.check()
            if left <= 0 or not client.wait_for_server(self.rospy.Duration(left)):
                raise DeviceUnavailable('机械臂','动作服务器无响应')
        while self.gate.clock() < end:
            self.gate.check()
            if self.finger_sample() and self.latest_pose and (
                    self.gate.clock()-self.latest_pose[0] <= self.cfg['tool_pose_max_age_s']):
                return True
            time.sleep(.05)
        raise DeviceUnavailable('机械臂','服务器在线，但三指/tool_pose持续没有新鲜数据')

    def close(self):
        errors = []
        for client in (self.arm_client,self.finger_client):
            if client is not None:
                try: cancel_unfinished_goal(client)
                except Exception as exc: errors.append(str(exc))
        subscribers = [self.pose_sub, self.finger_sub]+self.status_subscribers
        if self.robot is not None:
            subscribers += [getattr(self.robot, '_cartesian_sub', None),
                            getattr(self.robot, '_finger_sub', None)]
            # kinova_wn 原构造器自建语音实例，其ROS资源也随机械臂关闭。
            speaker = getattr(self.robot, 'speak', None)
            if speaker is not None:
                subscribers += [getattr(speaker, 'tts_done_sub', None),
                                getattr(speaker, 'tts_pub', None)]
        for sub in subscribers:
            if sub is not None:
                try: sub.unregister()
                except Exception as exc: errors.append(str(exc))
        if errors:
            raise RuntimeError('; '.join(errors))
