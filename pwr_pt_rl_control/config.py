from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class CoreMode:
    name: str
    reactivity_bias: float
    heat_transfer_scale: float
    coolant_loss_scale: float


@dataclass(frozen=True)
class CoreParameters:
    beta: float
    neutron_lifetime: float
    precursor_decay: float
    alpha_fuel: float
    alpha_coolant: float
    nominal_power_mw: float
    inlet_temperature: float
    fuel_time_constant: float
    coolant_time_constant: float
    fuel_power_gain: float
    coolant_coupling_gain: float
    controller_limit: tuple[float, float]


@dataclass(frozen=True)
class ObserverParameters:
    fsd_k1: float
    fsd_k2: float
    fsd_k3: float
    fsd_k4: float
    fsd_a1: float
    fsd_a2: float
    fsd_a3: float
    fsd_a4: float
    c_gain_1: float
    c_gain_2: float
    tf_gain_1: float
    tf_gain_2: float


@dataclass(frozen=True)
class ControllerParameters:
    tp: float
    k_fast: float
    k_linear: float
    k_temp: float
    post_gain: float


@dataclass(frozen=True)
class SimulationParameters:
    t_final: float
    dt: float
    noise_std: float
    random_seed: int
    initial_state: np.ndarray
    initial_observer_state: np.ndarray
    transition_generator: np.ndarray
    modes: tuple[CoreMode, ...]


@dataclass(frozen=True)
class CaseDefinition:
    core: CoreParameters
    observer: ObserverParameters
    controller: ControllerParameters
    simulation: SimulationParameters


def build_default_case() -> CaseDefinition:
    """Return a numerically well-scaled PC-1 case for the project demo."""
    core = CoreParameters(
        beta=6.5e-3,
        neutron_lifetime=2.0e-4,
        precursor_decay=0.08,
        alpha_fuel=-2.4e-5,
        alpha_coolant=-1.8e-5,
        nominal_power_mw=900.0,
        inlet_temperature=290.0,
        fuel_time_constant=6.0,
        coolant_time_constant=9.5,
        fuel_power_gain=36.0,
        coolant_coupling_gain=0.72,
        controller_limit=(-3.5e-3, 3.5e-3),
    )
    observer = ObserverParameters(
        fsd_k1=6.0,
        fsd_k2=2.0,
        fsd_k3=12.0,
        fsd_k4=4.0,
        fsd_a1=0.5,
        fsd_a2=1.5,
        fsd_a3=0.25,
        fsd_a4=1.25,
        c_gain_1=3.0,
        c_gain_2=1.2,
        tf_gain_1=5.0,
        tf_gain_2=2.0,
    )
    controller = ControllerParameters(
        tp=35.0,
        k_fast=1.8e-3,
        k_linear=1.2e-3,
        k_temp=3.0e-5,
        post_gain=1.1e-3,
    )
    modes = (
        CoreMode("Normal operation", 0.0, 1.00, 1.00),
        CoreMode("Minor fuel damage", -1.5e-4, 0.93, 1.05),
        CoreMode("SG leakage", -2.2e-4, 0.88, 1.18),
    )
    # Continuous-time generator for the three PC-1 modes. The discrete transition
    # matrix used during simulation is expm(Q * dt).
    transition_generator = np.array(
        [
            [-0.018, 0.012, 0.006],
            [0.036, -0.062, 0.026],
            [0.042, 0.018, -0.060],
        ],
        dtype=float,
    )
    simulation = SimulationParameters(
        t_final=420.0,
        dt=0.1,
        noise_std=0.25,
        random_seed=7,
        initial_state=np.array([1.0, 0.08125, 304.0, 296.0], dtype=float),
        initial_observer_state=np.array([0.08, 304.0, 296.0, 0.0], dtype=float),
        transition_generator=transition_generator,
        modes=modes,
    )
    return CaseDefinition(
        core=core,
        observer=observer,
        controller=controller,
        simulation=simulation,
    )
