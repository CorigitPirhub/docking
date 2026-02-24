import math
from typing import Iterable, List, Sequence, Tuple

import numpy as np


Vec2 = Tuple[float, float]


def wrap_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def clip(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def rotation(theta: float) -> np.ndarray:
    c = math.cos(theta)
    s = math.sin(theta)
    return np.array([[c, -s], [s, c]], dtype=float)


def heading(theta: float) -> np.ndarray:
    return np.array([math.cos(theta), math.sin(theta)], dtype=float)


def perp(vec: np.ndarray) -> np.ndarray:
    return np.array([-vec[1], vec[0]], dtype=float)


def cross2d(a: np.ndarray, b: np.ndarray) -> float:
    return float(a[0] * b[1] - a[1] * b[0])


def polyline_length(points: Sequence[Vec2]) -> float:
    if len(points) < 2:
        return 0.0
    acc = 0.0
    for i in range(1, len(points)):
        p0 = np.array(points[i - 1], dtype=float)
        p1 = np.array(points[i], dtype=float)
        acc += float(np.linalg.norm(p1 - p0))
    return acc


def nearest_point_index(points: Sequence[Vec2], query: Vec2) -> int:
    if not points:
        return 0
    q = np.array(query, dtype=float)
    d = [float(np.linalg.norm(np.array(p, dtype=float) - q)) for p in points]
    return int(np.argmin(d))


def cumulative_lengths(points: Sequence[Vec2]) -> List[float]:
    if not points:
        return []
    s = [0.0]
    for i in range(1, len(points)):
        p0 = np.array(points[i - 1], dtype=float)
        p1 = np.array(points[i], dtype=float)
        s.append(s[-1] + float(np.linalg.norm(p1 - p0)))
    return s


def interpolate_along_polyline(points: Sequence[Vec2], dist: float) -> Vec2:
    if not points:
        return (0.0, 0.0)
    if len(points) == 1 or dist <= 0.0:
        return points[0]

    s = cumulative_lengths(points)
    if dist >= s[-1]:
        return points[-1]

    for i in range(1, len(points)):
        if s[i] >= dist:
            seg_len = max(s[i] - s[i - 1], 1e-9)
            t = (dist - s[i - 1]) / seg_len
            p0 = np.array(points[i - 1], dtype=float)
            p1 = np.array(points[i], dtype=float)
            p = (1.0 - t) * p0 + t * p1
            return (float(p[0]), float(p[1]))
    return points[-1]


def softmax(scores: Iterable[float], temperature: float = 1.0) -> List[float]:
    vals = np.array(list(scores), dtype=float)
    if vals.size == 0:
        return []
    temperature = max(temperature, 1e-6)
    vals = vals / temperature
    vals -= np.max(vals)
    expv = np.exp(vals)
    z = float(np.sum(expv))
    if z <= 1e-12:
        return [1.0 / len(expv)] * len(expv)
    return [float(v / z) for v in expv]
