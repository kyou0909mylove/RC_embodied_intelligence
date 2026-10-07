# -*- coding: utf-8 -*-
"""三层柜：按最新脚本选择画面中心的有效物品，逐层重拍后完整抓取。"""
from .grasp import execute_grasp
from .shelf_coordinates import shelf_path
from .observe import move_to_view


def shelf_search_order(layers, hint):
    i = [layer['id'] for layer in layers].index(hint)
    return layers[i:] + list(reversed(layers[:i]))


def shelf_choice(candidates):
    """对应 task2_pick_v2._view_decision；不再用旧版左边缘/上下分区。"""
    valid = [item for item in candidates if item.get('camera_xyz_m') is not None]
    if not valid:
        return ('reposition_required' if candidates else 'no_target'), None
    def distance(item):
        x1,y1,x2,y2 = item['box_xyxy']
        w,h = item['image_wh']
        return ((x1+x2)/2-w/2)**2 + ((y1+y2)/2-h/2)**2
    return 'candidate', min(valid,key=distance)


def find_candidate(controller, point, label):
    profile = controller.cfg['profiles']['shelf']
    layers = shelf_search_order(point['layers'],point['layer_hint'])
    selected_pose = None
    needs_reposition = False
    reached_view = False
    failed_view = False
    controller.driver.fingers(profile.get('finger_open',controller.cfg['finger_open']))
    for layer in layers:
        poses = [layer['detect_pose']] + layer.get('alternate_detect_poses',[])
        for index,pose in enumerate(poses):
            selected_pose = pose
            if not move_to_view(controller,point,pose,point['id']+'/'+layer['id']+'/view'+str(index+1)):
                failed_view = True
                continue
            reached_view = True
            # 最新脚本允许抓取另一类别的中心物品，不能只检测头部指定类别。
            for _ in range(profile['detect_tries']):
                candidates = controller._scan(point,None,layer)
                state,target = shelf_choice(candidates)
                if state=='candidate':
                    target = dict(target,layer_id=layer['id'])
                    return target,pose,None
                needs_reposition = (needs_reposition or state=='reposition_required'
                                    or getattr(controller,'scan_saw_objects',False))
        # 主观察位两次无结果，再看本层已配置备用位；随后按提示层向上、再向下。
    if needs_reposition:
        special = 'reposition_required'
    elif failed_view or not reached_view:
        special = 'arm_failed'  # 有一层未观察到，不能靠其他空层判整个柜子为空。
    else:
        special = 'no_target'
    return None,selected_pose,special


def catch_shelf(controller, target, point, label):
    candidate,pose,special = find_candidate(controller,point,label)
    if candidate is not None:
        inverse = {wrist:head for head,wrist in controller.cfg['profiles']['shelf']['label_map'].items()}
        actual_label = inverse[candidate['label']]
        if actual_label != target.get('label'):
            controller.session.evidence.event('wrist_target_changed',point_id=point['id'],
                head_label=target.get('label'),actual_label=actual_label)
        target = dict(target,label=actual_label)
    return execute_grasp(controller,target,point,candidate,pose,special,planner=shelf_path,
        align_only=controller.cfg['profiles']['shelf']['execution_mode']=='align_only')
