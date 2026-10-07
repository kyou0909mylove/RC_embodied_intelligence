# -*- coding: utf-8 -*-
"""复用原 KinovaRobot 动作函数；这里只对接任务资源和实际反馈。"""
import math
import time
from core.safety import GuardedClient, ActionFailed, cancel_unfinished_goal
from core.faults import DeviceUnavailable, ParameterFault, PoseNotReached
from .coordinates import euler_xyz_to_quaternion


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
        from .original_kinova import KinovaRobot
        import kinova_msgs.msg
        import geometry_msgs.msg
        from actionlib_msgs.msg import GoalStatusArray
        # 原构造器的自动张爪/回位由 startup 统一执行，恢复持物时不执行。
        self.robot = KinovaRobot(self.cfg['robot_type'], initialize_motion=False,
                                feedback_timeout_s=self.cfg['initial_feedback_timeout_s'],
                                server_timeout_s=self.cfg['server_timeout_s'],
                                arm_timeout_s=self.cfg['arm_action_timeout_s'],
                                finger_timeout_s=self.cfg['finger_action_timeout_s'])
        self.arm_client = getattr(self.robot, 'client_arm', None)
        self.finger_client = getattr(self.robot, 'client_finger', None)
        if (self.arm_client is None or self.finger_client is None or
                not hasattr(self.robot, 'goal_arm') or not hasattr(self.robot, 'goal_finger') or
                self.rospy.is_shutdown()):
            raise DeviceUnavailable('机械臂','原KinovaRobot未完成驱动连接')
        prefix = '/' + self.cfg['robot_type'] + '_driver/'
        # 沿用任务的停止/截止处理，包装原客户端；不创建第二套客户端或目标消息。
        self.arm_client = GuardedClient(self.arm_client, self.gate, self.rospy.Duration,
                                        name=prefix+'pose_action/tool_pose')
        self.finger_client = GuardedClient(self.finger_client, self.gate, self.rospy.Duration,
                                           name=prefix+'fingers_action/finger_positions')
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
                if not all(math.isfinite(v) for v in xyz+quat) or abs(sum(v*v for v in quat)-1)>.02:
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

    def move(self, pose, verify_pose=True, stage=None):
        self.gate.check()
        self.check_pose(pose)
        stage = stage or ('收臂' if list(pose)==self.cfg['home_pose'] else '末端移动')
        started = self.gate.clock()
        print('[机械臂] {}，目标（米/度）={}'.format(stage,list(pose)),flush=True)
        # 米/度转换、构造 goal、发送与等待全部执行上传原函数。
        try:
            self.robot.arm_run(unit='mdeg', pose_target=list(pose), relative=False)
            self._require_success(self.arm_client)
        except ActionFailed as exc:
            details = self._motion_details(pose,stage,started)
            exc.motion_details = details
            detail_text = '步骤={}，目标={}，实测XYZ={}，位置误差={}米，姿态误差={}度，{}'.format(
                stage,list(pose),details.get('actual_xyz_m'),details.get('position_error_m'),
                details.get('orientation_error_deg'),details['feedback_note'])
            exc.args = (str(exc)+'；'+detail_text,)
            print('[机械臂动作失败] '+detail_text,flush=True)
            raise
        if not verify_pose:
            return True
        try:
            return self.tool_pose(expected=pose)
        except PoseNotReached as exc:
            # SUCCEEDED与新鲜反馈都存在：额外毫米/角度门槛只记提醒。
            print('[到位提醒] {}：{}；驱动动作成功，继续流程'.format(stage,exc),flush=True)
            return exc.actual

    def fingers(self, values, verify_result=False):
        self.gate.check()
        if len(values)!=3 or any(not math.isfinite(v) or not 0<=v<=100 for v in values):
            raise ValueError('三指目标须为0–100百分比')
        self.robot.finger_run(unit='percent', finger_target=list(values), relative=False)
        self._require_success(self.finger_client)
        if verify_result:
            return True

    def _require_success(self, client):
        # 原 arm_run/finger_run 没有返回值，不能把 None 判成失败或成功。
        # 任务继续执行的依据仍是驱动状态，实际持物另由三指反馈判断。
        self.gate.check()
        state = client.get_state()
        if state != 3:
            raise ActionFailed(client.name, state, client._failure_reason(state))

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
        for sub in subscribers:
            if sub is not None:
                try: sub.unregister()
                except Exception as exc: errors.append(str(exc))
        if errors:
            raise RuntimeError('; '.join(errors))
