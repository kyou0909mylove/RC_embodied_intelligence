# -*- coding: utf-8 -*-
"""调用原功能文件并登记结果；不规划、复制或改写原抓取动作。"""
import uuid
from core.faults import PoseNotReached
from core.safety import interruptible


class OriginalRobotInterface:
    """把原脚本动作交给已连接的机械臂；仅保留基本数值和真实执行保护。"""
    def __init__(self, controller, home_pose=None):
        self.controller = controller
        self.home_pose = list(home_pose if home_pose is not None
                              else controller.driver.robot.homePositionMdeg)
        self.home_succeeded = False
        self.last_pose = None
        self.closed = False
        self.failure = None
        self.action_name = '腕部识别/位姿计算'

    def __getattr__(self, name):
        return getattr(self.controller.driver.robot, name)

    def arm_run(self, unit='mdeg', pose_target=None, relative=False):
        self.action_name = '原文件机械臂移动'
        try:
            pose = [float(value.item() if hasattr(value, 'item') else value)
                    for value in pose_target]
            is_home = pose == self.home_pose
            if is_home:
                self.action_name = '原文件收臂'
            self.home_succeeded = False
            result = self.controller.driver.arm_run(
                unit, pose_target, relative, stage='原文件收臂' if is_home else None)
        except Exception as exc:
            self.failure = exc
            raise
        self.last_pose = pose
        # 原收臂仍由driver确认本次真实执行结果，普通前伸不按成功状态门禁。
        self.home_succeeded = is_home
        return result

    def finger_run(self, unit='percent', finger_target=None, relative=False):
        self.action_name = '原文件夹爪动作'
        try:
            result = self.controller.driver.finger_run(unit, finger_target, relative)
        except Exception as exc:
            self.failure = exc
            raise
        self.closed = any(v > 50 for v in finger_target)
        if self.controller.pending_pick is not None:
            self.controller.pending_pick['grasp_phase'] = 'closing' if self.closed else 'approach'
        return result


def call_original(controller, target, point, invoke, home_pose=None):
    """动作按原文件执行；完成收臂之后才判断是否抓到。"""
    if controller.wrist is not None:
        controller.wrist.close()
        controller.wrist_restart_needed = True
    if home_pose is None:
        home_pose = controller.driver.robot.homePositionMdeg
    robot = OriginalRobotInterface(controller, home_pose)
    oid = uuid.uuid4().hex
    target = dict(target or {})
    target.setdefault('label', 'unknown')
    controller.pending_pick = {'attempt_id':oid, 'object_id':oid, 'label':target['label'],
        'target':target, 'point_id':point['id'], 'furniture_id':point['furniture_id'],
        'retreat_poses':[], 'retreat_index':0, 'grasp_phase':'approach',
        'grasp_confirmed':False}
    controller._load_unknown = True
    controller.session.checkpoint()
    print('[原抓取] 点位={}，方式={}，头部提示={}'.format(
        point['id'], point['grasp_profile'], target.get('head_label_hint', target['label'])), flush=True)
    # 仅用整场硬截止/急停；不添加会截断前伸与闭爪的阶段预算。
    try:
        with interruptible(controller.session.gate):
            completed = bool(invoke(robot))
    except Exception as exc:
        print('[原抓取中断] 最近调用={}，{}: {}'.format(
            robot.action_name, type(exc).__name__, exc), flush=True)
        raise
    controller.session.check()
    if robot.failure is not None:
        # 部分往年方法捕获了执行异常；由外层恢复，不能误认为已收臂。
        print('[原抓取中断] 原文件捕获了{}的失败：{}'.format(
            robot.action_name, robot.failure), flush=True)
        raise robot.failure
    print('[原抓取返回] 返回值={}，已调用合爪动作={}'.format(completed, robot.closed), flush=True)
    if robot.last_pose != list(home_pose) or not robot.home_succeeded:
        controller._home()
    else:
        # 原函数返回前已执行其自身 home，不追加另一套抓取/收臂动作。
        try:
            controller.driver.tool_pose(expected=robot.last_pose)
        except PoseNotReached as exc:
            print('[收臂提醒] '+str(exc)+'；原 home 动作已成功', flush=True)
        controller.travel_ready = True
    context = controller.pending_pick
    target = context['target']
    if completed and robot.closed:
        receipt = controller.feedback.confirm('grasp', oid, target['label'])
    else:
        receipt = {'holding':False, 'holding_confirmed':False,
                   'classification':'open' if not robot.closed else 'unknown'}
    if receipt['holding']:
        context['grasp_phase'] = 'home'
        context['grasp_confirmed'] = bool(receipt.get('holding_confirmed'))
        controller.holding = dict(context)
    elif robot.closed:
        print('[抓取结束张爪] 收臂后的三指结果={}，未记录持物，执行复位张爪'.format(
            receipt.get('classification', 'unknown')), flush=True)
        controller.driver.fingers(controller.cfg['finger_open'])
    controller._load_unknown = False
    controller.pending_pick = None
    controller.session.checkpoint()
    result = {'status':'grasped' if receipt['holding'] else ('empty' if completed else 'no_target'),
        'physical_action':True, 'grasp_confirmed':bool(receipt.get('holding_confirmed')),
        'target':target, 'attempt_id':oid, 'object_id':oid, 'feedback':receipt}
    controller.session.evidence.event('original_pick_completed', point_id=point['id'], **result)
    return result

