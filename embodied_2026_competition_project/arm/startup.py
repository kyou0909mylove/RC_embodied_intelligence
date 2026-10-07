# -*- coding: utf-8 -*-
"""沿用 jujia26 的新任务顺序：先张爪，再回配置的收臂位。

只等待驱动动作结果，不要求人员提前把机械臂摆到收臂位。
恢复流程先读实际夹爪反馈，再按空爪/持物/把手中断分别处理。
"""


def reset_for_new_match(actions):
    """初始化动作不登记抓取/交付；任务中的到位和夹持检查仍由动作模块执行。"""
    if actions.session.resume:
        raise RuntimeError('--resume 恢复持物任务不能自动张爪')
    actions.travel_ready = False
    actions._load_unknown = True
    actions.session.checkpoint()
    actions.session.evidence.event('arm_startup_reset_requested',
        home_pose=actions.cfg['home_pose'], finger_open=actions.cfg['finger_open'],
        physical_action=False)
    print('[初始化] 夹爪：执行张爪动作', flush=True)
    with actions.session.operation(actions.cfg['startup_open_timeout_s']):
        actions.driver.fingers(actions.cfg['finger_open'], verify_result=True)
    print('[初始化] 机械臂：返回配置的收臂位', flush=True)
    with actions.session.operation(actions.cfg['startup_home_timeout_s']):
        actions.driver.move(actions.cfg['home_pose'], verify_pose=False)
    actions.travel_ready = True
    actions.holding = None
    actions.pending_pick = None
    actions._load_unknown = False
    actions.session.checkpoint()
    actions.session.evidence.event('arm_startup_reset_completed', physical_action=True,
        verification_basis='action_result', home_pose_confirmed=False,
        fingers_open_confirmed=False)
    print('[初始化] 收臂和张爪动作完成', flush=True)


def restore_interrupted_match(actions):
    """同场恢复不重置比赛时间；复用保存的回程，不先把疑似持物张开。"""
    from .feedback import FeedbackUnknown
    interrupted = any(s.get('handle_engaged') or s.get('status') in ('opening','uncertain')
                      for s in actions.storage_states.values())
    if interrupted:
        # 三级为保留接口。把手还在爪上时，不自动拉门或驾驶底盘。
        actions.driver.tool_pose(expected=actions.cfg['home_pose'])
        if not actions.feedback.confirm('initial')['empty']:
            raise FeedbackUnknown('三级开门中断；确认把手松开，再以--opened-storage恢复')
        actions.holding = actions.pending_pick = None
        actions._load_unknown = False
        actions.travel_ready = True
        return
    actions.reset_stalled_attempt()
    actions.session.evidence.event('arm_resume_reset_completed', physical_action=True,
                                   holding=bool(actions.holding), return_path_reused=True)
    print('[恢复] 已恢复负载及收臂；持物继续投放，空爪继续搜索',flush=True)
