# -*- coding: utf-8 -*-
"""共有抓取：新鲜腕部测量 -> 前伸 -> 闭爪 -> 抬起/退出 -> 收臂 -> 判断持物。"""
import uuid
from .coordinates import grasp_pose


def execute_grasp(controller, target, point, candidate, detect_pose, special=None, planner=None,
                  align_only=False):
    if candidate is None:
        if not controller.travel_ready:
            controller._home()
        status = special or 'no_target'
        reasons = {'no_target':'所有配置观察姿态均完成，未检测到物品',
                   'reposition_required':'看到物品，但缺少稳定深度/角度；不是空家具',
                   'arm_failed':'至少一个观察位未到达，不能判定家具为空'}
        return {'status':status,'grasp_confirmed':False,'physical_action':True,
                'reason':reasons.get(status,status)}
    profile = controller.cfg['profiles'][point['grasp_profile']]
    label = profile['label_map'][target['label']]
    layer = next((d for d in point.get('layers',[]) if d['id']==candidate.get('layer_id')),None)
    retries = controller.cfg['wrist']['max_fresh_retries']
    # 不缓存跨姿态/跨底盘点的XYZ。CPU推理或到位核对耗时较长时主动重拍。
    for attempt in range(retries+1):
        # 采用新鲜实际工具位姿计算坐标，不额外要求与示教姿态相差不足15毫米。
        xyz,quat = controller.driver.tool_pose()
        if controller.session.clock()-candidate['capture_monotonic'] <= controller.cfg['wrist']['max_age_s']:
            break
        if attempt==retries:
            controller._home()
            return {'status':'blocked','grasp_confirmed':False,
                    'reason':'重拍后测量仍超过{}秒；检查腕部推理耗时'.format(controller.cfg['wrist']['max_age_s'])}
        fresh = controller._scan(point,label,layer)
        if point['grasp_profile']=='shelf':
            from .pick_shelf import shelf_choice
            state,candidate = shelf_choice(fresh)
            if state!='candidate':
                controller._home()
                return {'status':'reposition_required','grasp_confirmed':False,
                        'reason':'播报后在原观察位重拍，仍未得到可用深度'}
            if layer:
                candidate['layer_id'] = layer['id']
        else:
            valid = [d for d in fresh if d.get('camera_xyz_m') is not None
                     and (planner is None or d.get('angle_deg') is not None)]
            if not valid:
                controller._home()
                return {'status':'blocked','grasp_confirmed':False,
                        'reason':'播报后重拍未取得有效深度/地面角度'}
            candidate = valid[0]
    if planner is None:
        pose = grasp_pose(profile,candidate['camera_xyz_m'],xyz+list(detect_pose[3:]),quat)
        lift = list(pose)
        lift[2] += point['lift_dz_m']
        approach,retreat = [pose],[lift]
    else:
        approach,retreat = planner(profile,point,candidate,xyz,detect_pose,controller.cfg['home_pose'])
        pose = approach[-1]
        # catch_ty 的二级地面闭爪后直接 home，没有另外的抬起/反向退出点。
        lift = retreat[0] if retreat else None
    for waypoint in approach+retreat+[controller.cfg['home_pose']]:
        controller.driver.check_pose(waypoint)
    # 只要求本次抓取及安全回程能完成；时间不足时允许持物离场。
    # 这些为保守估计，单动作硬上限仍由driver和外层阶段共同限制。
    reach_budget = len(approach)*timing_estimate(controller, 'reach_estimate_s', 5)
    close_finish = ((len(retreat)+1)*timing_estimate(controller, 'return_estimate_s', 5)
                    + timing_estimate(controller, 'close_estimate_s', 5)
                    + controller.cfg['gripper_feedback']['timeout_s'])
    if (controller.session.match_deadline-controller.session.clock()<close_finish+reach_budget or
            controller.session.gate.check()<close_finish+reach_budget):
        controller._home()
        return {'status':'time_insufficient','grasp_confirmed':False,
                'reason':'本阶段剩余时间不足以完成抓取及安全回程',
                'required_stage_s':close_finish+reach_budget}
    attempt_id = uuid.uuid4().hex
    controller.pending_pick = {'attempt_id':attempt_id,'object_id':attempt_id,'label':target['label'],
                               'target':target,'point_id':point['id'],'furniture_id':point['furniture_id'],
                               'retreat_poses':[list(v) for v in retreat], 'retreat_index':0,
                               'grasp_phase':'approach','grasp_confirmed':False}
    controller.session.evidence.event('grasp_planned',attempt_id=attempt_id,point_id=point['id'],
        wrist_target=candidate,grasp_pose=pose,lift_pose=lift,approach=approach,retreat=retreat,
        return_pose=controller.cfg['home_pose'])
    controller.session.checkpoint()
    # 规划前已在同一观察位核对/重拍。底盘与机械臂在保存记录期间保持静止，
    # 不再因为写恢复文件的耗时而撤销整套动作、重新播报。
    if align_only:
        # 现场调中间点时只移动一次并暂停；不能把源脚本的 True 当成抓取成功。
        from core.safety import OperationStopped
        controller.driver.move(approach[0],stage='柜层调试对准')
        controller.pending_pick = None
        controller.session.checkpoint()
        stopped = OperationStopped('柜层 align_only 已到中间点；确认路径后改回 full 并恢复流程')
        stopped.require_operator = True
        raise stopped
    # 地面分段横移/前伸/下降；柜层中间点/抓取点。两者均先核对新鲜测量。
    for index,waypoint in enumerate(approach):
        controller.driver.move(waypoint,verify_pose=False,stage='抓取前伸 '+str(index+1))
    # 在第一次闭爪前保存未知状态；进程此刻中断不能被恢复逻辑当成空手。
    controller._load_unknown = True
    controller.pending_pick['grasp_phase'] = 'closing'
    controller.session.checkpoint()
    controller.driver.fingers(profile.get('finger_close',controller.cfg['finger_close']))
    controller.driver.fingers(profile.get('finger_tight',controller.cfg['finger_tight']))
    # 闭爪后完整执行抬起/退出/收臂，最后一次反馈再判断成功，避免中间误判抢断动作。
    controller.pending_pick['grasp_phase'] = 'retreat'
    controller.session.checkpoint()
    for index,waypoint in enumerate(retreat):
        controller.driver.move(waypoint,verify_pose=False,stage='抓取回程 '+str(index+1))
        controller.pending_pick['retreat_index'] = index+1
        controller.session.checkpoint()
    controller.pending_pick['grasp_phase'] = 'home'
    controller._home()
    receipt = controller.feedback.confirm('grasp',attempt_id,target['label'])
    controller._load_unknown = False
    if receipt['holding']:
        controller.pending_pick['grasp_confirmed'] = bool(receipt.get('holding_confirmed'))
        controller.holding = dict(controller.pending_pick)
    controller.pending_pick = None
    controller.session.checkpoint()
    return {'status':'grasped' if receipt['holding'] else 'empty',
            'grasp_confirmed':bool(receipt.get('holding_confirmed')),'physical_action':True,'target':target,
            'attempt_id':attempt_id,'object_id':attempt_id,'feedback':receipt}


def timing_estimate(controller, key, fallback):
    """动作预算的估计秒数；实际超时另由arm_action_timeout_s等硬上限控制。"""
    return controller.cfg.get(key, fallback)
