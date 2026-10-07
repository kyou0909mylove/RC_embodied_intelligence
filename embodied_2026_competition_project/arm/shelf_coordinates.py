# -*- coding: utf-8 -*-
"""柜层路径：沿用 task2_pick_v2 的实际公式，连接完整闭爪和回程。"""
from .coordinates import grasp_pose


def shelf_path(profile, point, candidate, actual_xyz, detect_pose, home_pose):
    # 位置用反馈实测值，朝向沿用当前层配置，避免把命令到位误差加进目标位置。
    grasp = grasp_pose(profile, candidate['camera_xyz_m'],
                       list(actual_xyz) + list(detect_pose[3:]))
    align = list(grasp)
    # 源码实际改的是 Y：(观察位Y + 抓取位Y)*2/3，X/Z保持抓取值。
    # 源注释写成 X 中点，与执行语句不同；这里明确沿用执行语句。
    align[1] = (detect_pose[1] + grasp[1]) * profile['align_y_factor']
    lift = list(grasp)
    lift[2] += point['lift_dz_m']
    retreat = list(lift)
    # 抓取后先抬起，再按源 return_home 的 XY 中点过渡到配置收臂位。
    retreat[0] = (lift[0] + home_pose[0]) / 2
    retreat[1] = (lift[1] + home_pose[1]) / 2
    return [align, grasp], [lift, retreat]
