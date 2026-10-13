# -*- coding: utf-8 -*-
"""Kinect用于播报，桌面由腕部18类模型自行选物；原抓取动作保持独立。"""
from .grasp import call_original

HEAD_TO_DESK = {
    "biscuit": "oreo_box",
    "sprite": "sprite_bottle",
    "shampoo": "green_shampoo_bottle",
    "water": "water_bottle",
    "bowl": "blue_bowl",
    "cup": "white_cup",
}


def catch_table(controller, target, point, label):
    from .driver import load_original

    original = load_original("xg_table_pick")
    inverse = {desk:head for head,desk in HEAD_TO_DESK.items()}
    excluded = set(controller.session.config.values.get('excluded_labels',[]))
    target_items = [name for name in original.TARGET_NAMES if inverse.get(name,name) not in excluded]
    if not target_items:
        return {'status':'no_target','physical_action':False,'grasp_confirmed':False}
    reported = dict(target or {})
    reported['head_label_hint'] = reported.get('label')
    reported['label'] = 'unknown'

    def invoke(robot):
        completed = original.pick_once(robot, target_items)
        selected = getattr(robot, "selected_target_name", None)
        if selected is not None:
            context = controller.pending_pick
            actual = inverse.get(selected,selected)
            context["label"] = actual
            context["target"] = dict(
                context["target"],
                label=actual,
                wrist_label=selected,
            )
        return completed

    return call_original(
        controller, reported, point, invoke,
        home_pose=original.HOME_POSE,
    )
