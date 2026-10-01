# -*- coding: utf-8 -*-
"""运行进度与状态名。只保存数据，导入本文件不会初始化硬件。"""
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Optional

MATCH_SECONDS = 600.0


class TextState(str, Enum):
    """与旧字符串、JSON 和日志兼容，同时给状态名提供统一定义。"""

    def __str__(self):
        """日志显示 SELECT 等原状态值。"""
        return self.value


class State(TextState):
    """流程步骤，与是否持物分开保存。"""
    PREPARE = "PREPARE"
    WAIT_START = "WAIT_START"
    SELECT = "SELECT"
    NAVIGATE = "NAVIGATE"
    SCAN = "SCAN"
    APPROACH = "APPROACH"
    GRASP = "GRASP"
    GRIP = "GRIP"
    STOW = "STOW"
    PLACE = "PLACE"
    STOP = "STOP"


class HoldingState(TextState):
    """夹爪判断；位置反馈只提供持物辅助证据。"""
    EMPTY = "EMPTY"
    UNKNOWN = "UNKNOWN"
    HOLD_INDICATED = "HOLD_INDICATED"
    RELEASE_UNVERIFIED = "RELEASE_UNVERIFIED"


class ScanMode(TextState):
    """同一观察管线的四种检查目的。"""
    SEARCH = "SEARCH"
    RECHECK = "RECHECK"
    SLOT_BEFORE = "SLOT_BEFORE"
    SLOT_AFTER = "SLOT_AFTER"


@dataclass
class RunContext:
    """单场共享进度；硬件对象由 RobotSession 持有。"""
    state: Any = State.PREPARE
    holding: Any = HoldingState.EMPTY
    started: Optional[float] = None
    deadline: Optional[float] = None
    operation_deadline: Optional[float] = None
    stage: Any = None
    now: Optional[float] = None
    remaining: Optional[float] = None
    task: Any = None
    area: Any = None
    slot: Any = None
    target_map: Any = None
    destination: Any = None
    nav_kind: Any = None
    nav_after: Any = None
    scan_mode: Any = None
    grip_after: Any = None
    attempt: Any = None
    before_zone: Any = None
    cycle_started: Optional[float] = None
    return_cost: float = 0.0
    exit_cost: float = 0.0
    limits: dict = field(default_factory=dict)
    result_code: int = 0
    deliveries: list = field(default_factory=list)
    inventory: dict = field(default_factory=dict)
    record: dict = field(default_factory=dict)
    journal_cursor: int = 0

    def normalize_states(self):
        """阶段结束时核对状态名；保持旧字符串赋值位置与动作顺序。"""
        self.state = State(self.state)
        self.holding = HoldingState(self.holding)
        if self.scan_mode is not None:
            self.scan_mode = ScanMode(self.scan_mode)
        if self.nav_after is not None:
            self.nav_after = State(self.nav_after)
        if self.grip_after is not None and self.grip_after != "SCORE_VIEW":
            self.grip_after = State(self.grip_after)
