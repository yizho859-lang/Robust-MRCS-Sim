"""增量式局部窗口路径修复。

网络恢复后，协调器根据 DIFF_SYNC 里的实际位姿和时间偏移，
在有限窗口内重做 CBS。首周不接入主循环。
"""

from __future__ import annotations


def repair_window(*_args, **_kwargs):
    raise NotImplementedError("阶段三实现：Windowed-CBS 局部修复")
