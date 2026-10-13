# -*- coding: utf-8 -*-
"""桌面使用新18类模型，头部保留21类。"""
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

    head_label = (target or {}).get("label")
    desk_label = HEAD_TO_DESK.get(head_label)
    if head_label is not None and desk_label is None:
        return {
            "status": "blocked",
            "physical_action": False,
            "grasp_confirmed": False,
            "reason": "缺少明确的新桌面类别对应：" + str(head_label),
        }

    original = load_original("xg_table_pick")
    target_items = [desk_label] if desk_label else list(HEAD_TO_DESK.values())

    def invoke(robot):
        completed = original.pick_once(robot, target_items)
        selected = getattr(robot, "selected_target_name", None)
        if selected is not None:
            inverse = {desk: head for head, desk in HEAD_TO_DESK.items()}
            context = controller.pending_pick
            context["label"] = inverse[selected]
            context["target"] = dict(
                context["target"],
                label=inverse[selected],
                wrist_label=selected,
            )
        return completed

    return call_original(
        controller, target, point, invoke,
        home_pose=original.HOME_POSE,
    )
