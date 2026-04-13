from __future__ import annotations

import numpy as np

from .config import CoreParameters, ObserverParameters


def sig(value: float, power: float) -> float:
    return np.sign(value) * (abs(value) ** power)


class FixedTimeStableDifferentiator:
    def __init__(self, params: ObserverParameters):
        self.params = params

    def rhs(self, measured_tl: float, state: np.ndarray) -> np.ndarray:
        z1, z2 = state
        error = z1 - measured_tl
        dz1 = (
            z2
            - self.params.fsd_k1 * sig(error, self.params.fsd_a1)
            - self.params.fsd_k2 * sig(error, self.params.fsd_a2)
        )
        dz2 = (
            -self.params.fsd_k3 * sig(error, self.params.fsd_a3)
            - self.params.fsd_k4 * sig(error, self.params.fsd_a4)
        )
        return np.array([dz1, dz2], dtype=float)


class FiniteTimeStateObserver:
    def __init__(self, core: CoreParameters, params: ObserverParameters):
        self.core = core
        self.params = params

    def rhs(
        self,
        measured_nr: float,
        filtered_tl: float,
        estimated_tl_dot: float,
        observer_state: np.ndarray,
        control_reactivity: float,
    ) -> np.ndarray:
        c_hat, tf_hat = observer_state
        nr_error = c_hat - (self.core.beta / self.core.precursor_decay / self.core.neutron_lifetime) * measured_nr
        tl_error = tf_hat - filtered_tl
        dc_hat = (
            (self.core.beta / self.core.neutron_lifetime) * measured_nr
            - self.core.precursor_decay * c_hat
            - self.params.c_gain_1 * sig(nr_error, 0.5)
            - self.params.c_gain_2 * nr_error
        )
        inferred_heat_balance = estimated_tl_dot + (filtered_tl - self.core.inlet_temperature) / self.core.coolant_time_constant
        d_tf_hat = (
            filtered_tl
            + inferred_heat_balance / max(self.core.coolant_coupling_gain, 1e-6)
            - tf_hat
            - self.params.tf_gain_1 * sig(tl_error, 0.5)
            - self.params.tf_gain_2 * tl_error
            + 0.05 * control_reactivity
        )
        return np.array([dc_hat, d_tf_hat], dtype=float)
