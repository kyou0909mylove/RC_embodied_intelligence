# -*- coding: utf-8 -*-
"""柜门专用入口，对应 catch_3.py 的 open_door_g：把手沿铰链圆弧运动。"""
from .storage import execute_opening


def open_cabinet(controller, point):
    return execute_opening(controller,point,'cabinet')
