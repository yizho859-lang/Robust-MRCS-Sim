"""PATH_CMD / HEARTBEAT / DIFF_SYNC 的 JSON 载荷。

字段与方案中的 WebSocket / MQTT 数据契约一致。首周只下发 PATH_CMD。
"""

from __future__ import annotations


def build_path_cmd(
    seq_id: int,
    timestamp: float,
    agent_id: int,
    window_path: list[list[float]],
    net_risk_level: float = 0.0,
) -> dict:
    return {
        "msg_type": "PATH_CMD",
        "seq_id": seq_id,
        "timestamp": timestamp,
        "agent_id": agent_id,
        "window_path": window_path,
        "net_risk_level": net_risk_level,
    }


def build_heartbeat(
    ack_seq: int,
    agent_id: int,
    curr_pos: list[float],
    velocity: float,
    fsm_state: str,
    local_obs: list,
) -> dict:
    return {
        "msg_type": "HEARTBEAT",
        "ack_seq": ack_seq,
        "agent_id": agent_id,
        "curr_pos": curr_pos,
        "velocity": velocity,
        "fsm_state": fsm_state,
        "local_obs": local_obs,
    }


def build_diff_sync(
    agent_id: int,
    actual_pos: list[float],
    planned_pos: list[float],
    time_drift_dt: int,
    blocked_cells: list[list[int]],
) -> dict:
    return {
        "msg_type": "DIFF_SYNC",
        "agent_id": agent_id,
        "actual_pos": actual_pos,
        "planned_pos": planned_pos,
        "time_drift_dt": time_drift_dt,
        "blocked_cells": blocked_cells,
    }
