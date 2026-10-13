# -*- coding: utf-8 -*-
"""驱动反馈比较使用的欧拉角转四元数；抓取坐标计算由原功能文件负责。"""
import math


def euler_xyz_to_quaternion(angles_rad):
    """原 KinovaRobot.EulerXYZ2Quaternion 公式；输入弧度，输出 x/y/z/w。"""
    tx, ty, tz = angles_rad
    sx, cx = math.sin(tx/2), math.cos(tx/2)
    sy, cy = math.sin(ty/2), math.cos(ty/2)
    sz, cz = math.sin(tz/2), math.cos(tz/2)
    return [sx*cy*cz+cx*sy*sz, -sx*cy*sz+cx*sy*cz,
            sx*sy*cz+cx*cy*sz, -sx*sy*sz+cx*cy*cz]


