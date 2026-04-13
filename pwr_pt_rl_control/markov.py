from __future__ import annotations

import numpy as np
from scipy.linalg import expm


def discrete_transition_matrix(generator: np.ndarray, dt: float) -> np.ndarray:
    matrix = expm(generator * dt)
    matrix = np.clip(matrix, 0.0, 1.0)
    return matrix / matrix.sum(axis=1, keepdims=True)


def sample_mode_path(
    generator: np.ndarray,
    dt: float,
    horizon: float,
    seed: int,
    initial_mode: int = 0,
) -> tuple[np.ndarray, np.ndarray]:
    rng = np.random.default_rng(seed)
    times = np.arange(0.0, horizon + dt, dt)
    transitions = discrete_transition_matrix(generator, dt)
    modes = np.zeros(times.shape[0], dtype=int)
    modes[0] = initial_mode
    for idx in range(1, times.shape[0]):
        modes[idx] = rng.choice(transitions.shape[0], p=transitions[modes[idx - 1]])
    return times, modes
