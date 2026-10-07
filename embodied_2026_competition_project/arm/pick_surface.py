# -*- coding: utf-8 -*-
"""一级家具表面：主观察位、备用观察位轮换；不负责底盘导航或得分区。"""
from .grasp import execute_grasp
from .observe import move_to_view


def find_candidate(controller, point, label):
    poses = [point['detect_pose']] + point.get('alternate_detect_poses',[])
    saw_object = False
    reached_view = False
    failed_view = False
    for index,pose in enumerate(poses):
        if not move_to_view(controller,point,pose,point['id']+'/view'+str(index+1)):
            failed_view = True
            continue
        reached_view = True
        controller.driver.fingers(controller.cfg['finger_open'])
        candidates = controller._scan(point,label)
        saw_object = saw_object or bool(candidates) or getattr(controller,'scan_saw_objects',False)
        valid = [d for d in candidates if d.get('camera_xyz_m') is not None]
        if valid:
            return valid[0],pose,None
    if saw_object:
        special = 'reposition_required'
    elif failed_view or not reached_view:
        special = 'arm_failed'  # 未完成所有配置观察位，不能记为空家具。
    else:
        special = 'no_target'
    return None,poses[-1],special


def catch_table(controller, target, point, label):
    candidate,pose,special = find_candidate(controller,point,label)
    if candidate is not None:
        inverse = {wrist:head for head,wrist in controller.cfg['profiles']['surface']['label_map'].items()}
        target = dict(target,label=inverse[candidate['label']])
    return execute_grasp(controller,target,point,candidate,pose,special)
