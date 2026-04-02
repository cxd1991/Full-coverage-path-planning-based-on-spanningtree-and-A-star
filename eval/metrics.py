"""Evaluation metrics for coverage path planning experiments."""

from __future__ import annotations

from typing import Dict, Iterable, List, Sequence, Tuple

Point = Tuple[int, int]


def _path_steps(path: Sequence[Point]) -> Iterable[Tuple[Point, Point]]:
    for idx in range(1, len(path)):
        yield path[idx - 1], path[idx]


def coverage_rate(covered_count: int, free_count: int) -> float:
    if free_count <= 0:
        return 0.0
    return covered_count / free_count


def revisited_rate(revisited_count: int, covered_count: int) -> float:
    if covered_count <= 0:
        return 0.0
    return revisited_count / covered_count


def path_length_per_area(path: Sequence[Point], free_count: int) -> float:
    if len(path) < 2 or free_count <= 0:
        return 0.0
    total_length = 0.0
    for src, dst in _path_steps(path):
        total_length += abs(dst[0] - src[0]) + abs(dst[1] - src[1])
    return total_length / free_count


def turn_metrics(path: Sequence[Point], sharp_turn_threshold_deg: float = 90.0) -> Dict[str, float]:
    if len(path) < 3:
        return {"turn_count": 0.0, "sharp_turn_ratio": 0.0}

    turn_count = 0
    sharp_turn_count = 0

    for idx in range(2, len(path)):
        p0 = path[idx - 2]
        p1 = path[idx - 1]
        p2 = path[idx]

        v1 = (p1[0] - p0[0], p1[1] - p0[1])
        v2 = (p2[0] - p1[0], p2[1] - p1[1])

        if v1 == v2:
            continue

        turn_count += 1

        dot = v1[0] * v2[0] + v1[1] * v2[1]
        if dot <= 0 and sharp_turn_threshold_deg <= 90.0:
            sharp_turn_count += 1

    sharp_turn_ratio = sharp_turn_count / turn_count if turn_count else 0.0
    return {"turn_count": float(turn_count), "sharp_turn_ratio": sharp_turn_ratio}


def replan_metrics(replan_durations: Sequence[float]) -> Dict[str, float]:
    replan_count = len(replan_durations)
    avg_time = sum(replan_durations) / replan_count if replan_count else 0.0
    return {
        "replan_count": float(replan_count),
        "avg_replan_time_s": avg_time,
    }


def evaluate_metrics(
    path: Sequence[Point],
    free_count: int,
    covered_count: int,
    revisited_count: int,
    replan_durations: Sequence[float],
) -> Dict[str, float]:
    result = {
        "coverage_rate": coverage_rate(covered_count, free_count),
        "revisited_rate": revisited_rate(revisited_count, covered_count),
        "path_length_per_area": path_length_per_area(path, free_count),
    }
    result.update(turn_metrics(path))
    result.update(replan_metrics(replan_durations))
    return result
