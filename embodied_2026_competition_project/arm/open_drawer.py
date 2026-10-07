# -*- coding: utf-8 -*-
"""抽屉专用入口，对应 catch_3.py 的 open_door_c：把手沿滑轨分段拉出。"""
from .storage import execute_opening


def open_drawer(controller, point):
    return execute_opening(controller,point,'drawer')
