"""Visual playback environment for map and trajectory traces."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np

Point = Tuple[int, int]


class VisualEnv:
    def __init__(self, scenario: Dict, trace: Dict):
        self.scenario = scenario
        self.trace = trace
        self.width = int(scenario["width"])
        self.height = int(scenario["height"])
        self.obstacles = {tuple(cell) for cell in scenario.get("obstacles", [])}
        self.dynamic_obstacles = scenario.get("dynamic_obstacles", [])

        self.path: List[Point] = [tuple(point) for point in trace.get("path", [])]
        self.replan_segments = [tuple(seg) for seg in trace.get("replan_segments", [])]

        self.coverage_grid = np.zeros((self.height, self.width), dtype=float)
        self.free_mask = np.ones((self.height, self.width), dtype=bool)
        for ox, oy in self.obstacles:
            if 0 <= ox < self.width and 0 <= oy < self.height:
                self.free_mask[oy, ox] = False

    @staticmethod
    def load_json(path: str | Path) -> Dict:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)

    def _dynamic_cells(self, frame_id: int) -> set[Point]:
        cells: set[Point] = set()
        for event in self.dynamic_obstacles:
            if event["start_frame"] <= frame_id <= event["end_frame"]:
                cells.update(tuple(item) for item in event.get("cells", []))
        return cells

    def play(self, speed: float = 1.0, step_mode: bool = False) -> None:
        if not self.path:
            raise ValueError("trace.path is empty")

        fig, ax = plt.subplots(figsize=(8, 6))
        ax.set_title(f"Scenario: {self.scenario.get('name', 'unknown')}")
        ax.set_xlim(-0.5, self.width - 0.5)
        ax.set_ylim(-0.5, self.height - 0.5)
        ax.set_aspect("equal")
        ax.invert_yaxis()

        obstacle_layer = np.zeros((self.height, self.width))
        for ox, oy in self.obstacles:
            if 0 <= ox < self.width and 0 <= oy < self.height:
                obstacle_layer[oy, ox] = 1

        covered_im = ax.imshow(self.coverage_grid, cmap="Blues", vmin=0, vmax=4, alpha=0.75)
        ax.imshow(obstacle_layer, cmap="Greys", alpha=0.9)

        path_line, = ax.plot([], [], color="gold", linewidth=2, label="current path")
        robot_dot, = ax.plot([], [], "ro", markersize=4, label="robot")
        replan_lines = []
        for _ in self.replan_segments:
            line, = ax.plot([], [], color="crimson", linewidth=2.5, alpha=0.9)
            replan_lines.append(line)

        dyn_scatter = ax.scatter([], [], s=40, c="black", marker="s", label="dynamic obstacles")
        ax.legend(loc="upper right")

        interval = max(10, int(120 / max(speed, 0.1)))

        def update(frame_id: int):
            x, y = self.path[frame_id]
            if 0 <= x < self.width and 0 <= y < self.height and self.free_mask[y, x]:
                self.coverage_grid[y, x] += 1

            sub_path = self.path[: frame_id + 1]
            path_line.set_data([item[0] for item in sub_path], [item[1] for item in sub_path])
            robot_dot.set_data([x], [y])
            covered_im.set_data(self.coverage_grid)

            active_dynamic = self._dynamic_cells(frame_id)
            if active_dynamic:
                dyn_xy = np.array(list(active_dynamic))
                dyn_scatter.set_offsets(dyn_xy)
            else:
                dyn_scatter.set_offsets(np.empty((0, 2)))

            for idx, (start, end) in enumerate(self.replan_segments):
                if frame_id < start:
                    replan_lines[idx].set_data([], [])
                    continue
                capped_end = min(frame_id, end)
                segment = self.path[start : capped_end + 1]
                replan_lines[idx].set_data([item[0] for item in segment], [item[1] for item in segment])

            ax.set_xlabel(f"frame={frame_id + 1}/{len(self.path)}")
            return [covered_im, path_line, robot_dot, dyn_scatter, *replan_lines]

        if step_mode:
            for frame_id in range(len(self.path)):
                update(frame_id)
                plt.pause(interval / 1000.0)
        else:
            animation.FuncAnimation(
                fig,
                update,
                frames=len(self.path),
                interval=interval,
                repeat=False,
                blit=False,
            )
        plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(description="Play trajectory trace on scenario map")
    parser.add_argument("--scenario", required=True, help="Path to scenario json")
    parser.add_argument("--trace", required=True, help="Path to trace json")
    parser.add_argument("--speed", type=float, default=1.0, help="Playback speed multiplier")
    parser.add_argument("--step", action="store_true", help="Frame-by-frame playback")
    args = parser.parse_args()

    scenario = VisualEnv.load_json(args.scenario)
    trace = VisualEnv.load_json(args.trace)
    env = VisualEnv(scenario, trace)
    env.play(speed=args.speed, step_mode=args.step)


if __name__ == "__main__":
    main()
