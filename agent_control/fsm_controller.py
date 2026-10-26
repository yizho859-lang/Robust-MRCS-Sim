"""三态有限状态机。

NORMAL：按全局轨迹行走。
DEGRADED：超时后只靠本地传感器避障。
RESYNC：上报差分状态并等待修复轨迹。
转移条件在阶段三实现，首周智能体只开环跟踪路径。
"""

from __future__ import annotations

from enum import Enum


class FsmState(Enum):
    NORMAL = "NORMAL"
    DEGRADED = "DEGRADED"
    RESYNC = "RESYNC"


class FsmController:
    def __init__(self) -> None:
        self.state = FsmState.NORMAL

    def step(self, *_args, **_kwargs) -> FsmState:
        raise NotImplementedError("阶段三实现：NORMAL / DEGRADED / RESYNC 转移")
