# -*- coding: utf-8 -*-
"""接入 jujia26 已跑通的 catch_ty 地面动作，使用本项目已有腕部相机。

不创建第二个 KinovaRobot 或 RealSense 管线。目标类别来自当前赛事模型；
几何由 ground_coordinates 保留源轴交换、补偿和角度，闭爪确认由共享模块负责。
"""
import time
from .grasp import execute_grasp
from .ground_coordinates import ground_path
from .observe import move_to_view


def wait_for_observation(controller):
    """源 catch_ty 到检测位后等3秒；等待期间仍响应停止及任务截止。"""
    seconds = controller.cfg['profiles']['ground'].get('observe_settle_s', 3.0)
    end = controller.session.clock() + seconds
    # 外层抓取已经有中断定时器，这里只分段等候，避免再嵌套一层定时器。
    while controller.session.clock() < end:
        controller.session.gate.check()
        time.sleep(min(0.05, max(0.0, end-controller.session.clock())))
    controller.session.gate.check()


def find_candidate(controller, point, label):
    poses = [point['detect_pose']]+point.get('alternate_detect_poses',[])
    saw_object = False
    reached_view = False
    failed_view = False
    for index,pose in enumerate(poses):
        if not move_to_view(controller,point,pose,point['id']+'/view'+str(index+1)):
            failed_view = True
            continue
        reached_view = True
        wait_for_observation(controller)
        controller.driver.fingers(controller.cfg['profiles']['ground']['finger_open'])
        candidates = controller._scan(point,label)
        saw_object = saw_object or bool(candidates) or getattr(controller,'scan_saw_objects',False)
        valid = [item for item in candidates if item.get('camera_xyz_m') is not None
                 and item.get('angle_deg') is not None]
        if valid:
            return valid[0],pose,None
    if saw_object:
        special = 'reposition_required'
    elif failed_view or not reached_view:
        special = 'arm_failed'
    else:
        special = 'no_target'
    return None,poses[-1],special


def catch_ground(controller, target, point, label):
    candidate,pose,special = find_candidate(controller,point,label)
    if candidate is not None:
        inverse = {wrist:head for head,wrist in controller.cfg['profiles']['ground']['label_map'].items()}
        target = dict(target,label=inverse[candidate['label']])
    return execute_grasp(controller,target,point,candidate,pose,special,planner=ground_path)
