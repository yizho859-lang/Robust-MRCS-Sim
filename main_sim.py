"""联合仿真主入口，对应首周无界面最小闭环。

1. 用传统 CBS 为 10 个智能体规划互不碰撞的路径。
2. 主循环按时间步下发下一跳；理想通信时导出 replay.json。
3. 下发前按丢包率丢弃指令。丢包的车停在原地，其余车继续走，
   从而在交叉口复现碰撞或死锁。
"""

from __future__ import annotations

import argparse
import json
import random
import sys
from pathlib import Path

import yaml

from core_mapf.baseline_cbs import Grid, baseline_cbs, first_conflict
from net_sim.protocol_handler import build_path_cmd

ROOT = Path(__file__).resolve().parent
PATH_CMD_KEYS = {"msg_type", "seq_id", "timestamp", "agent_id", "window_path", "net_risk_level"}


def load_config(path: Path) -> dict:
    with path.open(encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"配置文件格式不正确: {path}")
    return data


def build_grid(cfg: dict) -> Grid:
    spec = cfg["map"]
    width, height = int(spec["width"]), int(spec["height"])
    aisles: set[tuple[int, int]] = set()
    for row in spec["horizontal_aisles"]:
        y = int(row["y"])
        for x in range(int(row["x0"]), int(row["x1"]) + 1):
            aisles.add((x, y))
    for col in spec["vertical_aisles"]:
        x = int(col["x"])
        for y in range(int(col["y0"]), int(col["y1"]) + 1):
            aisles.add((x, y))
    blocked = {(x, y) for y in range(height) for x in range(width) if (x, y) not in aisles}
    return Grid(width, height, blocked)


def load_agents(cfg: dict, grid: Grid) -> list[dict]:
    agents = []
    seen_starts: set[tuple[int, int]] = set()
    seen_goals: set[tuple[int, int]] = set()
    for index, item in enumerate(cfg["agents"]):
        if int(item["id"]) != index:
            raise ValueError(f"智能体 id 必须从 0 连续编号，第 {index} 项是 {item['id']}")
        start = (int(item["start"][0]), int(item["start"][1]))
        goal = (int(item["goal"][0]), int(item["goal"][1]))
        if not grid.passable(*start):
            raise ValueError(f"智能体 {index} 的起点 {start} 不在巷道上")
        if not grid.passable(*goal):
            raise ValueError(f"智能体 {index} 的终点 {goal} 不在巷道上")
        if start in seen_starts or goal in seen_goals:
            raise ValueError(f"智能体 {index} 的起点或终点与其他车重叠")
        seen_starts.add(start)
        seen_goals.add(goal)
        agents.append({"id": index, "start": start, "goal": goal})
    if len(agents) != 10:
        raise ValueError(f"首周清单要求 10 个智能体，当前是 {len(agents)} 个")
    return agents


def next_hop(path: list[tuple[int, int]], index: int) -> tuple[tuple[int, int], int]:
    if index >= len(path) - 1:
        return path[-1], index
    return path[index + 1], index + 1


def simulate(
    paths: list[list[tuple[int, int]]],
    goals: list[tuple[int, int]],
    *,
    loss_rate: float,
    seed: int,
    max_steps: int,
    stall_limit: int,
) -> dict:
    """按时间步下发下一跳。丢包时该车本步不移动。"""
    agent_count = len(paths)
    current = [paths[i][0] for i in range(agent_count)]
    progress = [0 for _ in range(agent_count)]
    rng = random.Random(seed)
    seq_id = 1
    last_progress = 0
    steps = [
        {
            "t": 0,
            "agents": [
                {
                    "id": i,
                    "pos": [current[i][0], current[i][1]],
                    "cmd_lost": False,
                    "seq_id": None,
                }
                for i in range(agent_count)
            ],
        }
    ]
    result: dict = {"status": "incomplete", "step": None, "event": None}

    for t in range(max_steps):
        clock = t + 1
        intended = list(current)
        new_progress = list(progress)
        lost = [False for _ in range(agent_count)]
        seqs: list[int | None] = [None for _ in range(agent_count)]

        for i in range(agent_count):
            hop, hopped = next_hop(paths[i], progress[i])
            cmd = build_path_cmd(
                seq_id=seq_id,
                timestamp=float(clock),
                agent_id=i,
                window_path=[[hop[0], hop[1], clock]],
                net_risk_level=0.0,
            )
            seqs[i] = seq_id
            seq_id += 1
            if not PATH_CMD_KEYS <= cmd.keys():
                raise RuntimeError("PATH_CMD 缺少数据契约字段")
            # 首周故障拦截：下发位置指令前按丢包率丢弃，丢失则本步不移动。
            if rng.random() < loss_rate:
                lost[i] = True
                continue
            intended[i] = hop
            new_progress[i] = hopped

        event = detect_collision(current, intended)
        steps.append(
            {
                "t": clock,
                "agents": [
                    {
                        "id": i,
                        "pos": [intended[i][0], intended[i][1]],
                        "cmd_lost": lost[i],
                        "seq_id": seqs[i],
                    }
                    for i in range(agent_count)
                ],
            }
        )
        if event is not None:
            event["step"] = clock
            event["cmd_lost"] = {str(i): lost[i] for i in event["agents"]}
            result = {"status": event["kind"], "step": clock, "event": event}
            break

        moved = any(intended[i] != current[i] for i in range(agent_count))
        current = intended
        progress = new_progress
        if all(current[i] == goals[i] for i in range(agent_count)):
            result = {"status": "success", "step": clock, "event": None}
            break
        if moved:
            last_progress = clock
        elif clock - last_progress >= stall_limit:
            result = {
                "status": "deadlock",
                "step": clock,
                "event": {
                    "kind": "deadlock",
                    "step": clock,
                    "stall_limit": stall_limit,
                    "positions": [[pos[0], pos[1]] for pos in current],
                },
            }
            break

    result["steps"] = steps
    return result


def detect_collision(
    previous: list[tuple[int, int]],
    intended: list[tuple[int, int]],
) -> dict | None:
    occupied: dict[tuple[int, int], int] = {}
    for index, pos in enumerate(intended):
        other = occupied.get(pos)
        if other is not None:
            return {
                "kind": "collision",
                "collision_type": "vertex",
                "step": None,
                "pos": [pos[0], pos[1]],
                "agents": [other, index],
            }
        occupied[pos] = index
    for left in range(len(intended)):
        for right in range(left + 1, len(intended)):
            swapped = (
                previous[left] == intended[right]
                and previous[right] == intended[left]
                and previous[left] != intended[left]
            )
            if swapped:
                return {
                    "kind": "collision",
                    "collision_type": "edge",
                    "step": None,
                    "pos": [intended[left][0], intended[left][1]],
                    "agents": [left, right],
                }
    return None


def dump_replay(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def replay_payload(cfg: dict, grid: Grid, agents: list[dict], paths, scenario: str, seed: int | None, loss_rate: float, run: dict) -> dict:
    obstacles = sorted(grid.blocked)
    return {
        "project": "面向弱通信与高延迟环境的多智能体高鲁棒性协同控制系统研究",
        "scenario": scenario,
        "loss_rate": loss_rate,
        "seed": seed,
        "map": {
            "width": grid.width,
            "height": grid.height,
            "obstacles": [[x, y] for x, y in obstacles],
        },
        "agents": [
            {
                "id": agent["id"],
                "start": [agent["start"][0], agent["start"][1]],
                "goal": [agent["goal"][0], agent["goal"][1]],
                "path": [[x, y] for x, y in paths[agent["id"]]],
            }
            for agent in agents
        ],
        "result": {"status": run["status"], "step": run["step"], "event": run["event"]},
        "steps": run["steps"],
    }


def describe_event(run: dict) -> str:
    event = run["event"]
    if run["status"] == "success":
        return f"10 个智能体在第 {run['step']} 步全部到达终点，无碰撞。"
    if run["status"] == "deadlock":
        return f"第 {event['step']} 步发生死锁：连续 {event['stall_limit']} 步没有任何车移动，且仍有车未到终点。"
    agents = event["agents"]
    pos = event["pos"]
    lost = event.get("cmd_lost", {})
    stalled = [agent for agent in agents if lost.get(str(agent))]
    if event["collision_type"] == "vertex" and stalled:
        others = [agent for agent in agents if agent not in stalled]
        return (
            f"第 {event['step']} 步发生顶点碰撞，位置 ({pos[0]}, {pos[1]})，"
            f"智能体 {stalled} 本步丢包后滞留，智能体 {others} 按下一跳进入同一栅格。"
        )
    kind = "顶点碰撞" if event["collision_type"] == "vertex" else "对向交换碰撞"
    extra = ""
    if event["collision_type"] == "vertex" and not stalled:
        extra = "本步指令都已送达，此前丢包打乱了 CBS 的让行时序，两车同时进入该栅格。"
    return f"第 {event['step']} 步发生{kind}，智能体 {agents}，位置 ({pos[0]}, {pos[1]})。{extra}"


def schedule_note(paths: list[list[tuple[int, int]]], agent_ids: list[int], pos: list[int]) -> str:
    cell = (pos[0], pos[1])
    parts = []
    for agent_id in agent_ids:
        times = [t for t, point in enumerate(paths[agent_id]) if point == cell]
        if not times:
            continue
        shown = "、".join(f"t={t}" for t in times)
        parts.append(f"车 {agent_id} 在 {shown}")
    if len(parts) < 2:
        return ""
    return "理想通信下两车错开经过该格：" + "，".join(parts) + "。"


def pair_trace(run: dict) -> list[str]:
    event = run.get("event") or {}
    agent_ids = event.get("agents") or []
    if not agent_ids:
        return []
    lines = ["   相关车辆逐步坐标："]
    previous: dict[int, list[int]] = {}
    for step in run["steps"]:
        by_id = {item["id"]: item for item in step["agents"]}
        parts = []
        for agent_id in agent_ids:
            item = by_id[agent_id]
            pos = item["pos"]
            if step["t"] == 0:
                flag = "初始"
            elif item["cmd_lost"]:
                flag = "丢包停住"
            elif previous.get(agent_id) == pos:
                flag = "收到等待"
            else:
                flag = "收到指令"
            parts.append(f"车{agent_id} ({pos[0]},{pos[1]}) {flag}")
            previous[agent_id] = pos
        lines.append(f"   t={step['t']}: " + "；".join(parts))
    return lines


def write_log(
    path: Path,
    ideal_text: str,
    fault_text: str,
    replay_name: str,
    fault_name: str,
    reproduce: str,
    extra_lines: list[str],
) -> None:
    lines = [
        "首周开工检查",
        f"1. 理想通信：通过。{ideal_text}轨迹：{replay_name}",
        f"2. 故障注入：通过。{fault_text}",
        *extra_lines,
        f"   轨迹：{fault_name}",
        f"   复现命令：{reproduce}",
        "",
    ]
    path.write_text("\n".join(lines), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="首周最小闭环：理想通信走完，再注入丢包并记录第一次碰撞或死锁。")
    parser.add_argument("--config", default=str(ROOT / "config" / "week1.yaml"))
    parser.add_argument("--seed", type=int, default=None, help="指定故障实验的随机种子；默认从配置种子开始向后搜索")
    parser.add_argument("--loss-rate", type=float, default=None)
    return parser.parse_args()


def main() -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")

    args = parse_args()
    cfg = load_config(Path(args.config))
    grid = build_grid(cfg)
    agents = load_agents(cfg, grid)
    starts = [agent["start"] for agent in agents]
    goals = [agent["goal"] for agent in agents]
    solution = baseline_cbs(grid, starts, goals)
    if first_conflict(solution.paths) is not None:
        raise RuntimeError("CBS 返回的路径仍有冲突")

    loss_rate = float(cfg["loss_rate"] if args.loss_rate is None else args.loss_rate)
    max_steps = int(cfg["max_steps"])
    stall_limit = int(cfg["stall_limit"])

    ideal = simulate(
        solution.paths,
        goals,
        loss_rate=0.0,
        seed=0,
        max_steps=max_steps,
        stall_limit=stall_limit,
    )
    if ideal["status"] != "success":
        raise RuntimeError(f"理想通信未能走完：{ideal['status']} @ {ideal['step']}")
    for step in ideal["steps"]:
        for agent in step["agents"]:
            index = agent["id"]
            planned = solution.paths[index][min(step["t"], len(solution.paths[index]) - 1)]
            if tuple(agent["pos"]) != planned or agent["cmd_lost"]:
                raise RuntimeError(f"理想通信轨迹偏离 CBS 路径：智能体 {index}，第 {step['t']} 步")

    ideal_payload = replay_payload(cfg, grid, agents, solution.paths, "ideal", None, 0.0, ideal)
    dump_replay(ROOT / "replay.json", ideal_payload)
    ideal_text = describe_event(ideal)

    if args.seed is not None:
        seeds = [args.seed]
    else:
        origin = int(cfg["seed"])
        seeds = list(range(origin, origin + int(cfg["seed_search"])))

    fault_run = None
    fault_seed = None
    fallback = None
    for seed in seeds:
        run = simulate(
            solution.paths,
            goals,
            loss_rate=loss_rate,
            seed=seed,
            max_steps=max_steps,
            stall_limit=stall_limit,
        )
        if run["status"] == "collision":
            fault_run = run
            fault_seed = seed
            break
        if run["status"] == "deadlock" and fallback is None:
            fallback = (seed, run)

    if fault_run is None and fallback is not None:
        fault_seed, fault_run = fallback
    if fault_run is None or fault_seed is None:
        print("故障注入未捕获碰撞或死锁。可提高 loss_rate，或增大 seed_search 后重试。")
        return 1

    fault_payload = replay_payload(cfg, grid, agents, solution.paths, "fault", fault_seed, loss_rate, fault_run)
    dump_replay(ROOT / "replay_fault.json", fault_payload)
    fault_text = f"种子 {fault_seed}，丢包率 {loss_rate}，{describe_event(fault_run)}"
    reproduce = f"python main_sim.py --seed {fault_seed} --loss-rate {loss_rate}"
    extra_lines = []
    event = fault_run.get("event") or {}
    if event.get("pos") and event.get("agents"):
        note = schedule_note(solution.paths, event["agents"], event["pos"])
        if note:
            extra_lines.append(f"   {note}")
    extra_lines.extend(pair_trace(fault_run))
    write_log(
        ROOT / "collision_log.txt",
        ideal_text,
        fault_text,
        "replay.json",
        "replay_fault.json",
        reproduce,
        extra_lines,
    )

    print(f"CBS 规划完成：总代价 {solution.cost}，高层节点 {solution.expanded}。")
    print(f"[理想通信] {ideal_text}轨迹已写入 replay.json")
    print(f"[故障注入] {fault_text}")
    print("日志已写入 collision_log.txt，故障轨迹已写入 replay_fault.json")
    print(f"复现：{reproduce}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
