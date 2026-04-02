"""Run benchmark on all predefined scenarios and export metrics + figures."""

from __future__ import annotations

import argparse
import csv
import json
import time
from collections import deque
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Set, Tuple

import matplotlib.pyplot as plt
import numpy as np

from eval.metrics import evaluate_metrics

Point = Tuple[int, int]


def neighbors(cell: Point, width: int, height: int) -> Iterable[Point]:
    x, y = cell
    for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
        if 0 <= nx < width and 0 <= ny < height:
            yield (nx, ny)


def bfs_path(start: Point, goal: Point, blocked: Set[Point], width: int, height: int) -> List[Point]:
    if start == goal:
        return [start]
    q = deque([start])
    parent: Dict[Point, Point | None] = {start: None}
    while q:
        node = q.popleft()
        for nxt in neighbors(node, width, height):
            if nxt in blocked or nxt in parent:
                continue
            parent[nxt] = node
            if nxt == goal:
                path = [goal]
                while path[-1] != start:
                    path.append(parent[path[-1]])
                path.reverse()
                return path
            q.append(nxt)
    return []


def build_scan_order(width: int, height: int, free_cells: Set[Point]) -> List[Point]:
    order: List[Point] = []
    for y in range(height):
        xs = range(width) if y % 2 == 0 else range(width - 1, -1, -1)
        for x in xs:
            cell = (x, y)
            if cell in free_cells:
                order.append(cell)
    return order


def active_dynamic_cells(dynamic_events: Sequence[Dict], frame_id: int) -> Set[Point]:
    active: Set[Point] = set()
    for event in dynamic_events:
        if event["start_frame"] <= frame_id <= event["end_frame"]:
            active.update(tuple(item) for item in event.get("cells", []))
    return active


def nearest_reachable_target(
    current: Point,
    targets: Set[Point],
    blocked: Set[Point],
    width: int,
    height: int,
) -> Point | None:
    q = deque([current])
    seen = {current}
    while q:
        node = q.popleft()
        if node in targets:
            return node
        for nxt in neighbors(node, width, height):
            if nxt in seen or nxt in blocked:
                continue
            seen.add(nxt)
            q.append(nxt)
    return None


def run_scenario(scenario_path: Path, trace_dir: Path, figure_dir: Path) -> Dict[str, float | str]:
    with open(scenario_path, "r", encoding="utf-8") as f:
        scenario = json.load(f)

    width = int(scenario["width"])
    height = int(scenario["height"])
    static_obstacles = {tuple(item) for item in scenario.get("obstacles", [])}
    dynamic_events = scenario.get("dynamic_obstacles", [])

    all_cells = {(x, y) for x in range(width) for y in range(height)}
    free_cells = all_cells - static_obstacles
    free_count = len(free_cells)

    order = build_scan_order(width, height, free_cells)
    if not order:
        raise ValueError(f"No free cells in scenario {scenario_path}")

    current = order[0]
    path: List[Point] = [current]
    visited_count: Dict[Point, int] = {current: 1}
    uncovered = set(free_cells)
    uncovered.discard(current)

    replan_segments: List[Tuple[int, int]] = []
    replan_durations: List[float] = []

    idx = 1
    while uncovered:
        frame_id = len(path) - 1
        dynamic_obstacles = active_dynamic_cells(dynamic_events, frame_id)
        blocked = static_obstacles | dynamic_obstacles
        blocked.discard(current)

        preferred = order[idx] if idx < len(order) else None
        if preferred is not None and preferred in uncovered:
            target = preferred
            idx += 1
        else:
            target = None

        if target is None or target in blocked:
            target = nearest_reachable_target(current, uncovered, blocked, width, height)
            if target is None:
                break

        t0 = time.perf_counter()
        segment = bfs_path(current, target, blocked, width, height)
        dt = time.perf_counter() - t0
        if not segment:
            uncovered.discard(target)
            continue

        is_replan = len(segment) > 2
        if is_replan:
            start_idx = len(path) - 1
            replan_durations.append(dt)

        for node in segment[1:]:
            path.append(node)
            visited_count[node] = visited_count.get(node, 0) + 1
            uncovered.discard(node)

        if is_replan:
            replan_segments.append((start_idx, len(path) - 1))

        current = path[-1]

    covered_count = len(visited_count)
    revisited_count = sum(1 for cnt in visited_count.values() if cnt > 1)

    metrics = evaluate_metrics(
        path=path,
        free_count=free_count,
        covered_count=covered_count,
        revisited_count=revisited_count,
        replan_durations=replan_durations,
    )

    trace = {
        "scenario": scenario.get("name", scenario_path.stem),
        "path": path,
        "replan_segments": replan_segments,
    }
    trace_path = trace_dir / f"{scenario_path.stem}_trace.json"
    with open(trace_path, "w", encoding="utf-8") as f:
        json.dump(trace, f)

    heat = np.zeros((height, width), dtype=float)
    for (x, y), cnt in visited_count.items():
        heat[y, x] = cnt
    for x, y in static_obstacles:
        heat[y, x] = np.nan

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    axes[0].set_title("Coverage heatmap")
    cmap = plt.cm.get_cmap("YlOrRd").copy()
    cmap.set_bad(color="black")
    axes[0].imshow(heat, cmap=cmap, origin="upper")

    axes[1].set_title("Path")
    axes[1].imshow(np.zeros((height, width)), cmap="Greys", alpha=0.15, origin="upper")
    if static_obstacles:
        obs = np.array(list(static_obstacles))
        axes[1].scatter(obs[:, 0], obs[:, 1], c="black", s=12)
    xs = [p[0] for p in path]
    ys = [p[1] for p in path]
    axes[1].plot(xs, ys, color="royalblue", linewidth=1)
    axes[1].scatter([xs[0]], [ys[0]], c="green", s=20)
    axes[1].scatter([xs[-1]], [ys[-1]], c="red", s=20)

    for ax in axes:
        ax.set_xlim(-0.5, width - 0.5)
        ax.set_ylim(height - 0.5, -0.5)
        ax.set_aspect("equal")

    fig.tight_layout()
    fig_path = figure_dir / f"{scenario_path.stem}.png"
    fig.savefig(fig_path, dpi=160)
    plt.close(fig)

    return {
        "scenario": scenario.get("name", scenario_path.stem),
        **metrics,
        "trace_file": str(trace_path),
        "figure_file": str(fig_path),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Run benchmark on all scenarios")
    parser.add_argument("--scenario-dir", default="sim/scenarios", help="Scenario folder")
    parser.add_argument("--output-dir", default="eval/artifacts", help="Result output folder")
    args = parser.parse_args()

    scenario_dir = Path(args.scenario_dir)
    output_dir = Path(args.output_dir)
    trace_dir = output_dir / "traces"
    figure_dir = output_dir / "figures"
    output_dir.mkdir(parents=True, exist_ok=True)
    trace_dir.mkdir(parents=True, exist_ok=True)
    figure_dir.mkdir(parents=True, exist_ok=True)

    scenario_files = sorted(scenario_dir.glob("*.json"))
    rows = [run_scenario(path, trace_dir, figure_dir) for path in scenario_files]

    csv_path = output_dir / "results.csv"
    with open(csv_path, "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    print(f"[benchmark] scenarios={len(rows)}")
    print(f"[benchmark] csv={csv_path}")


if __name__ == "__main__":
    main()
