# -*- coding: utf-8 -*-
"""实际导航到得分区后直接put；不做投放前持物复核，保留释放后的张爪确认。"""


def print_put(target, drop_pose):
    print('[放置打印] 未执行放置，不增加交付动作数',flush=True)
    return {'physical_action':False,'delivery_confirmed':False,'status':'print'}


def released_receipt(controller, held, receipt):
    known = held.get('grasp_confirmed',True)
    return {'status':'delivered' if known else 'released_unconfirmed_pick',
            'physical_action':True,'delivery_confirmed':known,
            'delivery_basis':'recorded_pick_and_drop_navigation_and_finger_open',
            'zone_sensor_verified':False,'released_confirmed':True,
            'object_id':held['object_id'],'point_id':held['point_id'],'feedback':receipt}


def put(controller, target, drop_pose):
    from .feedback import FeedbackUnknown
    from core.faults import StateMismatch
    session = controller.session
    if not getattr(session,'drop_arrived',False):
        raise StateMismatch('put缺少实际到达结果，应重新执行DROP_NAV')
    held = controller.holding or controller.pending_pick
    if not held:
        if session.last_put_receipt:
            controller._home()
            return session.last_put_receipt
        raise StateMismatch('put缺少抓取上下文；不能补造成功交付')
    controller.travel_ready = False
    controller.put_stage = 'moving'
    session.checkpoint()
    print('[投放] 已到投放点，开始释放位动作', flush=True)
    try:
        controller.driver.move(controller.cfg['place_pose'],stage='释放位')
    except Exception as exc:
        print('[投放中断] 释放位动作调用中断，本次尚未发送投放张爪；{}: {}'.format(
            type(exc).__name__, exc), flush=True)
        raise
    controller.put_stage = 'opening'
    controller._load_unknown = True
    print('[投放] 释放位动作调用已返回，按往年顺序准备张爪', flush=True)
    session.checkpoint()
    # 已实际到达得分区，直接释放。没有confirm('holding')。
    print('[投放张爪] 发送张爪动作', flush=True)
    try:
        controller.driver.fingers(controller.cfg['finger_open'])
    except Exception as exc:
        print('[投放中断] 张爪动作调用中断；{}: {}'.format(
            type(exc).__name__, exc), flush=True)
        raise
    receipt = controller.feedback.confirm('place',held['attempt_id'],held['label'],held['object_id'])
    if not receipt['released']:
        print('[投放提醒] 已调用张爪，实际三指释放反馈未确认，进入有限恢复', flush=True)
        raise FeedbackUnknown('张爪尚未确认，进入PUT有限恢复')
    result = released_receipt(controller,held,receipt)
    controller.holding = controller.pending_pick = None
    controller._load_unknown = False
    controller.put_stage = 'released'
    session.last_put_receipt = result
    session.checkpoint()
    controller._home()
    controller.put_stage = None
    return result
