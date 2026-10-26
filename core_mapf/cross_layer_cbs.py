"""跨层网络感知型 CBS。

阶段三再把心跳统计得到的延迟和丢包率写入底层代价，并在冲突树里
按弱网占用时间膨胀时空约束。首周不接入主循环。
"""

from __future__ import annotations

import math


def cross_layer_step_cost(
    g_u: float,
    tau_hat: float,
    p_hat: float,
    omega1: float,
    omega2: float,
    tau_th: float,
    eps: float = 1e-6,
) -> float:
    """方案中的底层代价：g(u) + 1 + ω1·τ̂/τth + ω2·p̂/(1-p̂+ε)。"""
    if tau_th <= 0:
        raise ValueError("延迟阈值必须为正")
    return g_u + 1.0 + omega1 * (tau_hat / tau_th) + omega2 * (p_hat / (1.0 - p_hat + eps))


def occupancy_extra_steps(tau_hat: float, dt_step: float) -> int:
    """高层占用膨胀步数 ⌈τ̂ / Δt⌉。"""
    if dt_step <= 0:
        raise ValueError("时间步长必须为正")
    return math.ceil(tau_hat / dt_step)


def plan(*_args, **_kwargs):
    raise NotImplementedError("阶段三实现：把跨层代价和占用膨胀接入 CBS")
