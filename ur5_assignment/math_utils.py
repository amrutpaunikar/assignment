import math

import numpy as np


def normalize_quat(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    return q / n


def quat_distance(q1: np.ndarray, q2: np.ndarray) -> float:
    q1 = normalize_quat(q1)
    q2 = normalize_quat(q2)
    dot = np.clip(abs(np.dot(q1, q2)), 0.0, 1.0)
    return 2.0 * math.acos(dot)


def path_length(points: np.ndarray) -> np.ndarray:
    d = np.linalg.norm(np.diff(points, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(d)])
