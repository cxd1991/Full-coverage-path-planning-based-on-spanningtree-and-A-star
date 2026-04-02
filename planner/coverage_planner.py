import heapq
import math
from collections import deque

import numpy as np

from PathPlanner import PathPlanner


class CoveragePlanner:
    def __init__(self, grid_map):
        self.map = grid_map
        self.visited_mask = np.zeros_like(self.map.free_mask, dtype=bool)
        self.coverage_ratio = 0.0
        self.redundant_ratio = 0.0
        self.uncovered_components = []
        self._visit_count = np.zeros_like(self.map.free_mask, dtype=np.int32)

    def plan(self, start_world=(20, 10), dynamic_obstacles=None):
        path = []

        # 1) 全局覆盖层（STC）
        path.extend(self.global_coverage(start_world))

        # 2) 补漏层（对未覆盖连通域按面积降序处理）
        path.extend(self.hole_fill(path))

        # 3) 执行层（动态障碍局部重规划）
        path = self.runtime_avoid(path, dynamic_obstacles or [])

        self._compute_metrics(path)
        return path

    def global_coverage(self, start_world):
        planner = PathPlanner(self.map, start_world[0], start_world[1])
        planner.spanning_tree()
        planner.draw_path()
        raw_path = planner.get_path()

        out = []
        for i, (x, y) in enumerate(raw_path):
            yaw = self._infer_yaw(raw_path, i)
            out.append((x, y, yaw, 'cover'))
            self._mark_visited(x, y)
        return out

    def hole_fill(self, existing_path):
        self.uncovered_components = self._extract_uncovered_components()
        self.uncovered_components.sort(key=len, reverse=True)

        if not existing_path:
            return []

        result = []
        current = (existing_path[-1][0], existing_path[-1][1])

        for comp in self.uncovered_components:
            target = min(comp, key=lambda p: abs(p[0] - current[0]) + abs(p[1] - current[1]))
            bridge = self._astar(current, target)
            result.extend(self._cells_to_path(bridge, 'replan'))

            sweep = self._component_sweep(comp, target)
            result.extend(self._cells_to_path(sweep, 'cover'))

            if sweep:
                current = sweep[-1]
            elif bridge:
                current = bridge[-1]

        # 补漏后再更新一次未覆盖分量
        self.uncovered_components = self._extract_uncovered_components()
        return result

    def runtime_avoid(self, planned_path, dynamic_obstacles):
        if not dynamic_obstacles:
            return planned_path

        occ = self.map.occ_mask.copy()
        for (x, y) in dynamic_obstacles:
            if 0 <= x <= self.map.width and 0 <= y <= self.map.height:
                occ[x, y] = True

        cells = [(p[0], p[1]) for p in planned_path]
        rebuilt = []
        avoid_indices = set()
        i = 0
        while i < len(cells):
            cx, cy = cells[i]
            if occ[cx, cy]:
                prev = rebuilt[-1] if rebuilt else cells[max(i - 1, 0)]
                j = i + 1
                while j < len(cells) and occ[cells[j][0], cells[j][1]]:
                    j += 1
                if j >= len(cells):
                    break
                bridge = self._astar(prev, cells[j], occ_mask=occ)
                base = len(rebuilt)
                rebuilt.extend(bridge[1:])
                for k in range(base, len(rebuilt)):
                    avoid_indices.add(k)
                i = j
            else:
                rebuilt.append((cx, cy))
                i += 1

        if not rebuilt:
            return planned_path

        out = []
        for idx, cell in enumerate(rebuilt):
            yaw = self._infer_yaw(rebuilt, idx)
            mode = 'avoid' if idx in avoid_indices else 'cover'
            out.append((cell[0], cell[1], yaw, mode))
        return out

    def _extract_uncovered_components(self):
        comps = []
        seen = np.zeros_like(self.map.free_mask, dtype=bool)

        for x in range(1, self.map.width):
            for y in range(1, self.map.height):
                if seen[x, y] or (not self.map.free_mask[x, y]) or self.visited_mask[x, y]:
                    continue
                q = deque([(x, y)])
                seen[x, y] = True
                comp = []
                while q:
                    cx, cy = q.popleft()
                    comp.append((cx, cy))
                    for nx, ny in self._neighbors4((cx, cy)):
                        if seen[nx, ny]:
                            continue
                        if self.map.free_mask[nx, ny] and (not self.visited_mask[nx, ny]):
                            seen[nx, ny] = True
                            q.append((nx, ny))
                if comp:
                    comps.append(comp)
        return comps

    def _component_sweep(self, comp, start):
        comp_set = set(comp)
        q = deque([start])
        seen = {start}
        order = [start]
        while q:
            c = q.popleft()
            for nxt in self._neighbors4(c):
                if nxt in comp_set and nxt not in seen:
                    seen.add(nxt)
                    q.append(nxt)
                    order.append(nxt)
        for c in comp:
            if c not in seen:
                order.append(c)
        return order

    def _astar(self, start, goal, occ_mask=None):
        occ = occ_mask if occ_mask is not None else self.map.occ_mask
        open_heap = []
        heapq.heappush(open_heap, (0, start))
        came = {start: None}
        g_cost = {start: 0}

        while open_heap:
            _, current = heapq.heappop(open_heap)
            if current == goal:
                break
            for nxt in self._neighbors4(current):
                if occ[nxt[0], nxt[1]]:
                    continue
                new_cost = g_cost[current] + 1
                if nxt not in g_cost or new_cost < g_cost[nxt]:
                    g_cost[nxt] = new_cost
                    f = new_cost + abs(nxt[0] - goal[0]) + abs(nxt[1] - goal[1])
                    heapq.heappush(open_heap, (f, nxt))
                    came[nxt] = current

        if goal not in came:
            return [start]

        rev = [goal]
        c = goal
        while came[c] is not None:
            c = came[c]
            rev.append(c)
        rev.reverse()
        return rev

    def _neighbors4(self, cell):
        x, y = cell
        nbs = []
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = x + dx, y + dy
            if 0 <= nx <= self.map.width and 0 <= ny <= self.map.height:
                nbs.append((nx, ny))
        return nbs

    def _mark_visited(self, x, y):
        if 0 <= x <= self.map.width and 0 <= y <= self.map.height and self.map.free_mask[x, y]:
            self.visited_mask[x, y] = True
            self._visit_count[x, y] += 1

    def _cells_to_path(self, cells, mode):
        path = []
        for i, (x, y) in enumerate(cells):
            yaw = self._infer_yaw(cells, i)
            path.append((x, y, yaw, mode))
            self._mark_visited(x, y)
        return path

    def _infer_yaw(self, seq, i):
        if len(seq) <= 1:
            return 0.0
        if i < len(seq) - 1:
            nx, ny = seq[i + 1][0], seq[i + 1][1]
            x, y = seq[i][0], seq[i][1]
        else:
            nx, ny = seq[i][0], seq[i][1]
            x, y = seq[i - 1][0], seq[i - 1][1]
        return math.atan2(ny - y, nx - x)

    def _compute_metrics(self, final_path):
        free_total = int(np.count_nonzero(self.map.free_mask))
        visited_total = int(np.count_nonzero(self.visited_mask))
        self.coverage_ratio = (visited_total / free_total) if free_total else 0.0

        for x, y, _, _ in final_path:
            if 0 <= x <= self.map.width and 0 <= y <= self.map.height and self.map.free_mask[x, y]:
                self._visit_count[x, y] += 1

        repeated = int(np.count_nonzero(self._visit_count > 1))
        self.redundant_ratio = (repeated / visited_total) if visited_total else 0.0
