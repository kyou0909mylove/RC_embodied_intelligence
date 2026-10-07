# -*- coding: utf-8 -*-
"""地面路径来自 hangzhou2026/tongyong_25/catch_ty.py 的 catch_ground。

这是固定观察朝向下的经验算法，不是任意视角的通用手眼变换。
源动作是横移、前伸、下降、两次闭爪，然后直接回收臂位。
抽屉仍单独设置内部几何，并保留抬起和分段退出，不照搬地面回程。
"""
import math


def ground_path(profile, point, candidate, actual_xyz, detect_pose=None, home_pose=None):
    camera = candidate['camera_xyz_m']
    angle = candidate.get('angle_deg')
    if (camera is None or len(camera)!=3 or not all(math.isfinite(v) for v in camera)
            or angle is None or not math.isfinite(angle)):
        raise ValueError('地面抓取需要有效的三维坐标和物品角度')
    # 原调用 image_to_arm(result.x, result.z, result.y)，不能改成常规XYZ顺序。
    swapped = [camera[0],camera[2],camera[1]]
    measured = [sum(row[i]*swapped[i] for i in range(3))+offset
                for row,offset in zip(profile['hand_eye_R'],profile['hand_eye_T_m'])]
    geometry = dict(profile,**point.get('ground',{}))
    reference = profile['reference_detect_pose']
    # 只允许与源观察位同朝向；平移改变由当前工具位置的差值补入锚点。
    # 自定义锚点以该点主观察位为基准；源默认锚点以源观察位为基准。
    if 'anchor_m' in point.get('ground',{}):
        reference = point['detect_pose']
    anchor = [v+actual_xyz[i]-reference[i] for i,v in enumerate(geometry['anchor_m'])]
    orientation = list(profile['grasp_euler_xy_deg'])+[angle-profile['yaw_offset_deg']]
    lateral = [anchor[0],anchor[1]-measured[2]-profile['side_clearance_m'],anchor[2]]+orientation
    forward = [anchor[0]-measured[1]+profile['forward_offset_m'],anchor[1]-measured[2],anchor[2]]+orientation
    descend = list(forward)
    descend[2] = geometry['grasp_z_m']  # 机械臂基座坐标Z，不是地图坐标或地面海拔。
    approach = [lateral,forward,descend]
    if point.get('difficulty', 2) == 3:
        # 抽屉口有边沿，延续已有分段退出；三级参数不因地面更新而改变。
        lift = list(descend)
        lift[2] += point['lift_dz_m']
        return approach, [lift,forward,lateral]
    # 二级地面照源动作闭爪后直接 home；共享执行器负责配置收臂位及持物反馈。
    return approach, []
