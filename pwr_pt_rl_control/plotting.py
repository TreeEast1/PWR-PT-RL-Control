from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from .config import CaseDefinition
from .simulation import SimulationResult


def create_plots(
    result: SimulationResult,
    case: CaseDefinition,
    output_dir: Path,
    show: bool = True,
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)

    time = result.time
    nr = result.plant_state[:, 0]
    tf = result.plant_state[:, 2]
    tl = result.plant_state[:, 3]
    c_hat = result.observer_state[:, 0]
    tf_hat = result.observer_state[:, 1]
    tl_hat = result.differentiator_state[:, 0]

    fig, axes = plt.subplots(3, 1, figsize=(11, 12), sharex=True)

    axes[0].plot(time, result.reference, label="Reference power", linewidth=2.0)
    axes[0].plot(time, nr, label="Actual power", linewidth=1.6)
    axes[0].set_ylabel("Relative power (FP)")
    axes[0].set_title("PWR load-following under PC-1 Markov jumps")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()

    axes[1].plot(time, result.reference - nr, label="Tracking error", color="tab:red")
    axes[1].plot(time, result.control * 1e5, label="Control reactivity x1e5 pcm", color="tab:green")
    axes[1].set_ylabel("Error / scaled control")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()

    axes[2].step(time, result.modes, where="post", linewidth=1.8, color="tab:purple")
    axes[2].set_yticks([0, 1, 2], [mode.name for mode in case.simulation.modes])
    axes[2].set_ylabel("PC-1 mode")
    axes[2].set_xlabel("Time (s)")
    axes[2].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(output_dir / "load_following_summary.png", dpi=220)

    fig2, axes2 = plt.subplots(2, 1, figsize=(11, 8), sharex=True)
    axes2[0].plot(time, tl, label="True outlet coolant temperature", linewidth=1.8)
    axes2[0].plot(time, result.noisy_tl, label="Noisy measurement", alpha=0.55)
    axes2[0].plot(time, tl_hat, label="FSD estimate", linewidth=1.8)
    axes2[0].set_ylabel("T_l (deg C)")
    axes2[0].grid(True, alpha=0.3)
    axes2[0].legend()

    axes2[1].plot(time, tf, label="True fuel temperature", linewidth=1.8)
    axes2[1].plot(time, tf_hat, label="Observer estimate", linewidth=1.6)
    axes2[1].plot(time, result.plant_state[:, 1], label="True precursor density", linewidth=1.4)
    axes2[1].plot(time, c_hat, label="Observer precursor estimate", linewidth=1.2)
    axes2[1].set_ylabel("Estimated states")
    axes2[1].set_xlabel("Time (s)")
    axes2[1].grid(True, alpha=0.3)
    axes2[1].legend()

    fig2.tight_layout()
    fig2.savefig(output_dir / "observer_differentiator_summary.png", dpi=220)

    if show:
        plt.show()
    plt.close("all")
