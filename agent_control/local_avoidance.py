"""失联态本地安全避障。

断网后冻结全局路径，只读取有限半径内的占用并等待或单步礼让。
首周不接入主循环，丢包时智能体保持原地。
"""

from __future__ import annotations


def avoid(*_args, **_kwargs):
    raise NotImplementedError("阶段三实现：失联态本地避障")
