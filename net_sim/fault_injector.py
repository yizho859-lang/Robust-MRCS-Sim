"""故障注入。

首周的伯努利丢包写在 main_sim.py 主循环里，判据是
``rng.random() < loss_rate``。本文件留下阶段二要接上的空间盲区和
Gilbert-Elliott 突发丢包接口。
"""

from __future__ import annotations


def spatial_loss_probability(x: int, y: int, center: tuple[int, int], p_max: float, sigma: float) -> float:
    """阶段二：P_loss(x,y) = Pmax · exp(-||p-c||^2 / 2σ^2)。"""
    raise NotImplementedError("阶段二实现：高斯衰减盲区")


def gilbert_elliott_step(state: str, p_gb: float, p_bg: float, rng) -> str:
    """阶段二：两态马尔可夫链，返回 GOOD 或 BAD。"""
    raise NotImplementedError("阶段二实现：脉冲丢包")


def sample_jitter(tau_base: float, rng) -> float:
    """阶段二：Δτ = τ_base + Jitter()。"""
    raise NotImplementedError("阶段二实现：长尾延迟")
