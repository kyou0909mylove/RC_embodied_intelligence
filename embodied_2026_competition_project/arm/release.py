# -*- coding: utf-8 -*-
"""投放路线受阻后主动放弃持物：底盘停止、张爪、确认、收臂。

独立于得分区put，不伪造到达投放点，也不登记成功交付。
"""


def abandon(controller, reason):
    from .feedback import FeedbackUnknown
    session = controller.session
    session.stop_navigation()
    held = controller.holding or controller.pending_pick
    controller.driver.fingers(controller.cfg['finger_open'])
    receipt = controller.feedback.confirm('place')
    if not receipt['released']:
        raise FeedbackUnknown('主动放弃持物后，张爪尚未确认')
    # 张开后立即保存弃物回执；之后收臂失败也不能把它重新当成待投放物品。
    controller.holding = controller.pending_pick = None
    controller._load_unknown = False
    session.drop_arrived = False
    session.evidence.record.setdefault('abandoned_objects', []).append({
        'object':held, 'reason':reason, 'feedback':receipt})
    session.checkpoint()
    controller._home()
    session.evidence.event('load_abandoned', reason=reason, object=held,
                           delivery_confirmed=False, opened_before_next_navigation=True)
    return {'status':'abandoned', 'delivery_confirmed':False}
