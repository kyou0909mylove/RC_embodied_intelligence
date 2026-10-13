# -*- coding: utf-8 -*-
"""区分设备不可用与软件恢复；异常名字本身不能用来判定硬件故障。"""


class DeviceUnavailable(RuntimeError):
    """连续复核后设备/通信仍不可用，允许停止本场并保留恢复记录。"""
    def __init__(self, module, detail):
        self.module = module
        super().__init__('设备不可用【{}】：{}'.format(module, detail))


class StateMismatch(RuntimeError):
    """软件状态与动作/反馈不一致；由阶段恢复处理。"""


class PoseNotReached(RuntimeError):
    """有效反馈存在，但未满足软件到位容差，不等同于通信断开。"""


class ParameterFault(ValueError):
    def __init__(self, parameter, detail):
        self.parameter = parameter
        super().__init__('参数 {}：{}'.format(parameter, detail))
