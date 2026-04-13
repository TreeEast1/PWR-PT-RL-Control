from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.integrate import solve_ivp

from .config import CaseDefinition
from .controller import PreassignedTimeController
from .markov import sample_mode_path
from .model import PWRCoreModel, reference_power
from .observer import FiniteTimeStateObserver, FixedTimeStableDifferentiator


@dataclass
class SimulationResult:
    time: np.ndarray
    plant_state: np.ndarray
    observer_state: np.ndarray
    differentiator_state: np.ndarray
    noisy_tl: np.ndarray
    reference: np.ndarray
    control: np.ndarray
    modes: np.ndarray


def run_simulation(case: CaseDefinition) -> SimulationResult:
    core_model = PWRCoreModel(case.core)
    differentiator = FixedTimeStableDifferentiator(case.observer)
    observer = FiniteTimeStateObserver(case.core, case.observer)
    controller = PreassignedTimeController(case.core, case.controller)

    times, modes = sample_mode_path(
        case.simulation.transition_generator,
        case.simulation.dt,
        case.simulation.t_final,
        case.simulation.random_seed,
    )
    rng = np.random.default_rng(case.simulation.random_seed)
    noise = rng.normal(0.0, case.simulation.noise_std, size=times.shape[0])

    state = np.concatenate(
        [
            case.simulation.initial_state,
            case.simulation.initial_observer_state[:2],
            case.simulation.initial_observer_state[2:],
        ]
    )
    history = np.zeros((times.shape[0], state.shape[0]), dtype=float)
    control_history = np.zeros(times.shape[0], dtype=float)
    reference = np.array([reference_power(t) for t in times], dtype=float)
    noisy_tl = np.zeros(times.shape[0], dtype=float)

    history[0] = state
    noisy_tl[0] = state[3] + noise[0]
    for idx in range(times.shape[0] - 1):
        t0 = times[idx]
        t1 = times[idx + 1]
        mode = case.simulation.modes[modes[idx]]

        plant_state = state[:4]
        observer_state = state[4:6]
        diff_state = state[6:]
        measured_tl = plant_state[3] + noise[idx]
        noisy_tl[idx] = measured_tl
        filtered_tl, tl_dot_hat = diff_state
        control = controller.control(
            t0,
            plant_state[0],
            reference[idx],
            observer_state[1],
            filtered_tl,
            mode.reactivity_bias,
        )
        control_history[idx] = control

        def closed_loop_rhs(t: float, y: np.ndarray) -> np.ndarray:
            plant = y[:4]
            observer_y = y[4:6]
            diff_y = y[6:]
            plant_rhs = core_model.rhs(t, plant, control, mode)
            measured_tl_local = plant[3] + np.interp(t, [t0, t1], [noise[idx], noise[idx + 1]])
            diff_rhs = differentiator.rhs(measured_tl_local, diff_y)
            obs_rhs = observer.rhs(
                measured_nr=plant[0],
                filtered_tl=diff_y[0],
                estimated_tl_dot=diff_y[1],
                observer_state=observer_y,
                control_reactivity=control,
            )
            return np.concatenate([plant_rhs, obs_rhs, diff_rhs])

        sol = solve_ivp(
            closed_loop_rhs,
            (t0, t1),
            state,
            method="RK45",
            max_step=case.simulation.dt / 5.0,
            rtol=1e-6,
            atol=1e-8,
        )
        state = sol.y[:, -1]
        history[idx + 1] = state

    noisy_tl[-1] = history[-1, 3] + noise[-1]
    control_history[-1] = control_history[-2]
    return SimulationResult(
        time=times,
        plant_state=history[:, :4],
        observer_state=history[:, 4:6],
        differentiator_state=history[:, 6:],
        noisy_tl=noisy_tl,
        reference=reference,
        control=control_history,
        modes=modes,
    )


def ensure_output_dir(root: Path) -> Path:
    output = root / "outputs"
    output.mkdir(parents=True, exist_ok=True)
    return output
