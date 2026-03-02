from __future__ import annotations

import math
from typing import Iterable

import numpy as np


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def angle_diff(target: float, source: float) -> float:
    return wrap_angle(target - source)


def rot2d(theta: float) -> np.ndarray:
    c = math.cos(theta)
    s = math.sin(theta)
    return np.array([[c, -s], [s, c]], dtype=float)


def body_to_world(local_xy: np.ndarray, pose_xy: np.ndarray, yaw: float) -> np.ndarray:
    return (rot2d(yaw) @ local_xy.T).T + pose_xy


def world_to_body(world_xy: np.ndarray, pose_xy: np.ndarray, yaw: float) -> np.ndarray:
    return (rot2d(-yaw) @ (world_xy - pose_xy).T).T


def poly_edges(poly: np.ndarray) -> Iterable[np.ndarray]:
    for i in range(len(poly)):
        p0 = poly[i]
        p1 = poly[(i + 1) % len(poly)]
        yield p1 - p0


def unit(v: np.ndarray, eps: float = 1e-9) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < eps:
        return np.zeros_like(v)
    return v / n
