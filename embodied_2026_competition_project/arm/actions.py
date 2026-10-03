# -*- coding: utf-8 -*-
"""只保留待完善的抓取/投放接口。这里不导入或控制机械臂、夹爪。"""


def grasp(target, detection_point):
    """后续在此接入自己的抓取函数；target=None 表示没有检测到物品。"""
    label = target["label"] if target else "未检测到物品，仅演练接口"
    print("[抓取占位] 已完成抓取接口调用：{}，检测点={}；机械臂未执行动作".format(
        label, detection_point["id"]), flush=True)
    return {"interface_completed": True, "physical_action": False, "target": target}


def place(target, drop_pose):
    """到达己方投放点后调用；当前只打印，未来必须验证物品落在1m×1m得分区内。"""
    label = target["label"] if target else "空载接口演练"
    print("[投放占位] 已完成投放接口调用：{}；机械臂未执行动作".format(label), flush=True)
    return {"interface_completed": True, "physical_action": False, "target": target}
