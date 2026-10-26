"""传统理想通信 CBS。

假设每条指令都能按时送达。低层用带时空约束的 A*，高层用冲突树
对顶点冲突和交换冲突做二分。通信丢包不在本模块处理。
"""

from __future__ import annotations

import heapq
from dataclasses import dataclass
from typing import Iterable

Pos = tuple[int, int]
_MOVES = ((0, 0), (1, 0), (-1, 0), (0, 1), (0, -1))


class Grid:
    """四连通栅格。blocked 中的格子是货架或墙。"""

    def __init__(self, width: int, height: int, blocked: Iterable[Pos]) -> None:
        self.width = width
        self.height = height
        self.blocked = set(blocked)

    def passable(self, x: int, y: int) -> bool:
        return 0 <= x < self.width and 0 <= y < self.height and (x, y) not in self.blocked


@dataclass
class CbsSolution:
    paths: list[list[Pos]]
    cost: int
    expanded: int


def first_conflict(paths: list[list[Pos]]) -> dict | None:
    """返回时间最早的一类冲突；路径在结束后视为停在终点。"""

    def at(agent: int, t: int) -> Pos:
        path = paths[agent]
        if t < len(path):
            return path[t]
        return path[-1]

    agent_count = len(paths)
    max_t = max(len(path) for path in paths) - 1
    for t in range(max_t + 1):
        occupied: dict[Pos, int] = {}
        for agent in range(agent_count):
            pos = at(agent, t)
            other = occupied.get(pos)
            if other is not None:
                return {
                    "type": "vertex",
                    "time": t,
                    "pos": pos,
                    "agents": (other, agent),
                }
            occupied[pos] = agent
        if t == max_t:
            continue
        for left in range(agent_count):
            for right in range(left + 1, agent_count):
                left_now, left_next = at(left, t), at(left, t + 1)
                right_now, right_next = at(right, t), at(right, t + 1)
                crossed = left_now == right_next and right_now == left_next and left_now != left_next
                if crossed:
                    return {
                        "type": "edge",
                        "time": t + 1,
                        "agents": (left, right),
                        "move_a": (left_now, left_next),
                        "move_b": (right_now, right_next),
                    }
    return None


def _low_level(
    grid: Grid,
    start: Pos,
    goal: Pos,
    vertex: set[tuple[int, int, int]],
    edge: set[tuple[int, int, int, int, int]],
    max_time: int,
) -> list[Pos] | None:
    if not grid.passable(*start) or not grid.passable(*goal):
        return None
    if (0, start[0], start[1]) in vertex:
        return None

    goal_blocks = {t for (t, x, y) in vertex if (x, y) == goal}

    def can_finish(t: int, x: int, y: int) -> bool:
        if (x, y) != goal:
            return False
        # 到达后会一直占用终点，因此不能在任一更晚的约束时刻停在终点。
        return all(blocked_t < t for blocked_t in goal_blocks)

    start_state = (start[0], start[1], 0)
    parent: dict[tuple[int, int, int], tuple[int, int, int] | None] = {start_state: None}
    seq = 0
    heuristic = abs(start[0] - goal[0]) + abs(start[1] - goal[1])
    heap: list[tuple[int, int, int, tuple[int, int, int]]] = [(heuristic, 0, seq, start_state)]
    closed: set[tuple[int, int, int]] = set()

    while heap:
        _, _, _, state = heapq.heappop(heap)
        if state in closed:
            continue
        closed.add(state)
        x, y, t = state
        if can_finish(t, x, y):
            path: list[Pos] = []
            cursor: tuple[int, int, int] | None = state
            while cursor is not None:
                path.append((cursor[0], cursor[1]))
                cursor = parent[cursor]
            path.reverse()
            return path
        if t >= max_time:
            continue
        for dx, dy in _MOVES:
            nx, ny = x + dx, y + dy
            nt = t + 1
            if not grid.passable(nx, ny):
                continue
            if (nt, nx, ny) in vertex:
                continue
            if (nt, x, y, nx, ny) in edge:
                continue
            nxt = (nx, ny, nt)
            if nxt in closed or nxt in parent:
                continue
            parent[nxt] = state
            seq += 1
            step_h = abs(nx - goal[0]) + abs(ny - goal[1])
            heapq.heappush(heap, (nt + step_h, nt, seq, nxt))
    return None


def _split(conflict: dict) -> list[tuple[int, str, tuple]]:
    if conflict["type"] == "vertex":
        x, y = conflict["pos"]
        payload = (conflict["time"], x, y)
        return [(agent, "vertex", payload) for agent in conflict["agents"]]

    arrival = conflict["time"]
    splits = []
    for agent, move in zip(conflict["agents"], (conflict["move_a"], conflict["move_b"])):
        src, dst = move
        splits.append((agent, "edge", (arrival, src[0], src[1], dst[0], dst[1])))
    return splits


def baseline_cbs(
    grid: Grid,
    starts: list[Pos],
    goals: list[Pos],
    *,
    max_nodes: int = 10000,
    max_time: int = 96,
) -> CbsSolution:
    """求理想通信下的无碰撞路径。代价是各车路径时间之和。"""
    if len(starts) != len(goals):
        raise ValueError("起点与终点数量不一致")
    if len(starts) == 0:
        raise ValueError("至少需要一个智能体")

    constraints = [(set(), set()) for _ in starts]
    paths: list[list[Pos]] = []
    for index, (start, goal) in enumerate(zip(starts, goals)):
        path = _low_level(grid, start, goal, constraints[index][0], constraints[index][1], max_time)
        if path is None:
            raise RuntimeError(f"智能体 {index} 在无约束时无法到达终点 {goal}")
        paths.append(path)

    counter = 0
    root_cost = sum(len(path) - 1 for path in paths)
    heap: list[tuple] = [(root_cost, counter, constraints, paths)]
    expanded = 0

    while heap:
        cost, _, node_constraints, node_paths = heapq.heappop(heap)
        expanded += 1
        if expanded > max_nodes:
            raise RuntimeError("CBS 展开节点超过上限")
        conflict = first_conflict(node_paths)
        if conflict is None:
            return CbsSolution(paths=node_paths, cost=cost, expanded=expanded)

        for agent, kind, payload in _split(conflict):
            child_constraints = [(set(vertex), set(edge)) for vertex, edge in node_constraints]
            vertex_set, edge_set = child_constraints[agent]
            if kind == "vertex":
                vertex_set.add(payload)
            else:
                edge_set.add(payload)
            repaired = _low_level(
                grid,
                starts[agent],
                goals[agent],
                vertex_set,
                edge_set,
                max_time,
            )
            if repaired is None:
                continue
            child_paths = list(node_paths)
            child_paths[agent] = repaired
            child_cost = sum(len(path) - 1 for path in child_paths)
            counter += 1
            heapq.heappush(heap, (child_cost, counter, child_constraints, child_paths))

    raise RuntimeError("CBS 未找到无碰撞路径")
