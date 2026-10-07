# -*- coding: utf-8 -*-
"""到观察位的有限重试，以及头部看不到地面/柜内物品时的腕部发现。"""
from core.safety import ActionFailed


class ObservationUnavailable(RuntimeError):
    """未找到物品且有观察位未能到达；失败后的收臂已经确认。"""


def move_to_view(controller, point, pose, view_id):
    """观察位失败先收臂，重试同一位一次，再交给调用方尝试备用位/其他层。

    只处理闭爪前的末端ABORTED/REJECTED；不能据此判断家具在地图上不可达。
    收臂失败、超时、停止、夹爪动作失败或负载未知仍向上抛出，禁止继续移动。
    """
    for attempt in range(controller.cfg['observe_retries']+1):
        controller.session.check()
        # 即使上一失败后已收臂，本次再次伸手也必须撤销底盘可行驶标志。
        controller.travel_ready = False
        controller.session.checkpoint()
        try:
            # 使用驱动成功结果到位；坐标计算读取实际工具位姿，取消额外毫米/角度门槛。
            controller.driver.move(pose,verify_pose=False,stage='观察位 '+view_id)
            return True
        except ActionFailed as exc:
            if (controller.load_state!='empty' or exc.state not in (4,5)
                    or not exc.name.endswith('/pose_action/tool_pose')
                    or not controller.session.config.values['flow']['skip_unreachable_detection']):
                raise
            controller.session.check()
            controller.session.evidence.event('arm_observation_failed',point_id=point['id'],
                view_id=view_id,attempt=attempt+1,reason=str(exc),
                motion=getattr(exc,'motion_details',None))
            try:
                controller._home()
            except Exception as home_error:
                # 外层不能再把收臂失败当普通观察位失败而尝试其他姿态。
                home_error.safe_retreat_failed = True
                raise
            controller.last_view_failure = exc
            print('[观察重试] {}：{}失败，已确认收臂；{}'.format(
                point['name'],view_id,'有限重试本姿态' if attempt<controller.cfg['observe_retries'] else '尝试后续已配置姿态'),flush=True)
    return False


def discover(controller, point):
    if controller.load_state!='empty':
        raise RuntimeError('腕部发现类别前必须空爪')
    controller.require_travel_ready()
    controller.travel_ready = False
    controller.session.checkpoint()
    if point['grasp_profile']=='shelf':
        from .pick_shelf import shelf_search_order
        views = [(pose,layer) for layer in shelf_search_order(point['layers'],point['layer_hint'])
                 for pose in [layer['detect_pose']]+layer.get('alternate_detect_poses',[])]
    else:
        views = [(pose,None) for pose in [point['detect_pose']]+point.get('alternate_detect_poses',[])]
    candidates = []
    reached_view = False
    failed_view = False
    for pose,layer in views:
        view_id = point['id']+('/'+layer['id'] if layer else '')
        if not move_to_view(controller,point,pose,view_id):
            failed_view = True
            continue
        reached_view = True
        profile = controller.cfg['profiles'][point['grasp_profile']]
        if point['grasp_profile']=='ground':
            from .pick_ground import wait_for_observation
            wait_for_observation(controller)
        controller.driver.fingers(profile.get('finger_open',controller.cfg['finger_open']))
        candidates = controller._scan(point,None,layer)
        if candidates:
            break
    if not controller.travel_ready:
        controller._home()
    if not candidates and (failed_view or not reached_view):
        raise ObservationUnavailable('本次观察未完成：存在未到达的观察姿态，不能判空；'+str(controller.last_view_failure))
    return candidates
