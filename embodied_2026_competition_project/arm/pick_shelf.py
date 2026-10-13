# -*- coding: utf-8 -*-
"""柜层调用接口；main 直接调用上传的 task2_pick_v4.pick_on_shelf。"""
from .grasp import call_original


def catch_shelf(controller, target, point, label):
    from .driver import load_original
    original = load_original('task2_pick_v4')
    # 头部类别只作提示，未匹配时按 v4 自身规则选取腕部目标。
    target_name = label if label in original.SHELF_TARGETS else None
    next_layers = getattr(controller, 'shelf_next_layers', None)
    if next_layers is None:
        next_layers = controller.shelf_next_layers = {}
    furniture_id = point.get('furniture_id', point['id'])
    hint_layer = next_layers.get(furniture_id) or point.get('layer_hint') or original.DEFAULT_LAYER
    order = original.LAYER_ORDER
    next_layer = order[(order.index(hint_layer)+1) % len(order)]
    reported = dict(target or {})
    reported['head_label_hint'] = reported.get('label')
    # v4 可能抓取更靠近中心的另一件；其布尔返回值不提供实际类别。
    reported['label'] = 'unknown'

    def invoke(kinova):
        # 本次调用每层最多识别5次；不改上传文件和原动作/选物逻辑。
        previous_tries = original.DETECT_TRIES
        try:
            original.DETECT_TRIES = 5
            print('[柜层识别] 每层最多尝试{}次，识别成功立即继续抓取'.format(
                original.DETECT_TRIES), flush=True)
            print('[柜层轮换] 本次优先{}，下一次优先{}'.format(
                original.SHELF_LAYERS[hint_layer]['name'],
                original.SHELF_LAYERS[next_layer]['name']), flush=True)
            return controller.session.pick_shelf_original(kinova, target_name, hint_layer)
        finally:
            original.DETECT_TRIES = previous_tries

    # 原函数遇物即返回；下次换起始层，避免每次都被中层截住。
    # call_original 的原有检查点会一并保存下一层，不增加动作间等待。
    next_layers[furniture_id] = next_layer
    return call_original(controller, reported, point,
        invoke, home_pose=original.HOME_POSE)


