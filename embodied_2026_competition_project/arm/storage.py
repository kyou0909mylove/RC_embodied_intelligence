# -*- coding: utf-8 -*-
"""开门公共动作与中断记录；门把手不属于比赛物品，不增加抓取/交付数量。

动作到位和松开把手只说明路径执行完成；没有门状态传感器，不能证明门已完全打开。
真实力矩/碰撞检测仍由原驱动及现场人员负责，配置只接受已实测的非零路径。
"""


def execute_opening(controller, point, kind):
    session = controller.session
    furniture = point['furniture_id']
    existing = controller.storage_states.get(furniture,{})
    if existing.get('status')=='open_motion_completed':
        return {'status':'opened','physical_action':False,'door_sensor_verified':False}
    if existing.get('status') in ('opening','uncertain'):
        return {'status':'storage_uncertain','physical_action':False,
                'reason':'开门曾中断，需要现场确认开门、空爪和收臂后恢复'}
    # 启动不扫描所有家具；到实际开门时才拒绝未标定的全零路径。
    from core.storage_configuration import validate_storage
    errors = []
    validate_storage(point, errors)
    if errors:
        raise ValueError('开门参数：' + '；'.join(errors))
    controller.require_travel_ready()
    if controller.load_state!='empty':
        raise RuntimeError('开门前必须空爪')
    config = point['storage']
    if config['kind']!=kind:
        raise ValueError('开门模块与storage.kind不符')
    # 每个move最久10秒+工具到位确认；三次夹爪动作及反馈另留预算。
    duration = ((len(config['open_path'])+4)*(10+controller.cfg['tool_pose_timeout_s'])
                +15+(len(config['open_path'])+2)*controller.cfg['gripper_feedback']['timeout_s'])
    timing = session.config.values['timing']
    remaining = (timing['scan_estimate_s']+timing['grasp_estimate_s']
                 +timing['nav_timeout_s']+controller.cfg['place_timeout_s']+timing['exit_reserve_s'])
    if (session.match_deadline-session.clock()<duration+remaining or
            session.gate.deadline-session.clock()<duration):
        return {'status':'time_insufficient','physical_action':False}
    state = {'status':'opening','kind':kind,'handle_engaged':False,'stage':'pregrasp'}
    controller.storage_states[furniture] = state
    controller.travel_ready = False
    session.checkpoint()
    controller.driver.fingers(config['finger_open'])
    controller.driver.move(config['pregrasp_pose'])
    controller.driver.move(config['handle_pose'])
    # 闭爪前标记把手负载未知。任何异常都保留记录，不自动收臂或再次拉门。
    state.update(handle_engaged=True,stage='grip_handle')
    controller._load_unknown = True
    session.checkpoint()
    controller.driver.fingers(config['finger_grasp'])
    receipt = controller.feedback.confirm('grasp','handle-'+furniture)
    if not receipt['holding']:
        controller.driver.fingers(config['finger_open'])
        if not controller.feedback.confirm('place','handle-'+furniture)['released']:
            raise RuntimeError('把手未确认松开，禁止自动收臂')
        state.update(status='unopened',handle_engaged=False,stage='handle_not_gripped')
        controller._load_unknown = False
        controller._home()
        return {'status':'blocked','physical_action':True,'reason':'把手闭合反馈未显示受阻'}
    for index,pose in enumerate(config['open_path']):
        state['stage'] = 'open_path_'+str(index)
        session.checkpoint()
        controller.driver.move(pose)
        if not controller.feedback.confirm('grasp','handle-'+furniture)['holding']:
            raise RuntimeError('开门过程中把手疑似脱落；现场检查后恢复，禁止自动重拉')
    state['stage'] = 'release_handle'
    session.checkpoint()
    controller.driver.fingers(config['finger_open'])
    if not controller.feedback.confirm('place','handle-'+furniture)['released']:
        raise RuntimeError('把手未确认松开，禁止自动收臂')
    state.update(handle_engaged=False,stage='clear_handle')
    controller._load_unknown = False
    session.checkpoint()
    controller.driver.move(config['clear_pose'])
    controller._home()
    state.update(status='open_motion_completed',stage='completed')
    session.checkpoint()
    session.evidence.event('storage_opening_completed',point_id=point['id'],furniture_id=furniture,
                           kind=kind,door_sensor_verified=False)
    return {'status':'opened','physical_action':True,'door_sensor_verified':False}
