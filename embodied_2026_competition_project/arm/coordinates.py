# -*- coding: utf-8 -*-
"""纯坐标/姿态运算。一级与二级保留各自来源公式，不连接 ROS 或运动硬件。"""
import math


def euler_xyz_to_quaternion(angles_rad):
    """原 KinovaRobot.EulerXYZ2Quaternion 公式；输入弧度，输出 x/y/z/w。"""
    tx, ty, tz = angles_rad
    sx, cx = math.sin(tx/2), math.cos(tx/2)
    sy, cy = math.sin(ty/2), math.cos(ty/2)
    sz, cz = math.sin(tz/2), math.cos(tz/2)
    return [sx*cy*cz+cx*sy*sz, -sx*cy*sz+cx*sy*cz,
            sx*sy*cz+cx*cy*sz, -sx*sy*sz+cx*cy*cz]

def _matvec(matrix, vector):
    return [sum(a*b for a,b in zip(row, vector)) for row in matrix]

def _tool_rotation(quaternion):
    """沿用原 KinovaRobot 四元数 w/x/y/z 公式，输入显式为 x/y/z/w。"""
    x, y, z, w = quaternion
    return [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]

def grasp_pose(profile, xyz, detect_pose, quaternion=None):
    """保留两套来源公式，禁止共用/自动推断相机标定。

    surface：R_tool * R_hand_eye * (p_cam - TCP_GRIP_CAM)，加实际检测位。
    shelf：task2_pick_v2 交换 y/z -> R*p+T -> [-p_y,p_x,p_z]+经验补偿。
    shelf 公式是历史经验标定约定，不能解释成通用 SE(3) 标定后随意修改。
    """
    if profile['transform'] == 'surface':
        delta = _matvec(profile['hand_eye_R'], [a-b for a,b in zip(xyz, profile['tcp_grip_cam_m'])])
        delta = _matvec(_tool_rotation(quaternion), delta)
    elif profile['transform'] == 'shelf':
        raw = _matvec(profile['hand_eye_R'], [xyz[0], xyz[2], xyz[1]])
        p = [a+b for a,b in zip(raw, profile['hand_eye_T_m'])]
        delta = [a+b for a,b in zip([-p[1], p[0], p[2]], profile['compensation_m'])]
    else:
        raise ValueError('未实现的抓取坐标变换')
    return [a+b for a,b in zip(detect_pose[:3], delta)] + list(detect_pose[3:])
