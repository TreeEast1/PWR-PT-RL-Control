from __future__ import annotations

import numpy as np

from .config import CoreMode, CoreParameters


def reference_power(t: float) -> float:
    ramp_rate = 0.0025
    if t <= 200.0:
        return 1.0 - ramp_rate * t
    return 0.5


class PWRCoreModel:
    def __init__(self, params: CoreParameters):
        self.params = params

    def thermal_reactivity(self, tf: float, tl: float) -> float:
        return (
            self.params.alpha_fuel * (tf - 304.0)
            + self.params.alpha_coolant * (tl - 296.0)
        )

    def rhs(
        self,
        t: float,
        state: np.ndarray,
        control_reactivity: float,
        mode: CoreMode,
    ) -> np.ndarray:
        nr, cr, tf, tl = state
        total_rho = (
            control_reactivity
            + self.thermal_reactivity(tf, tl)
            + mode.reactivity_bias
        )
        dn = (
            ((total_rho - self.params.beta) / self.params.neutron_lifetime) * nr
            + self.params.precursor_decay * cr
        )
        dc = (
            (self.params.beta / self.params.neutron_lifetime) * nr
            - self.params.precursor_decay * cr
        )
        fuel_source = self.params.fuel_power_gain * mode.heat_transfer_scale * (nr - 0.5)
        d_tf = (
            fuel_source - (tf - tl) / self.params.fuel_time_constant
        )
        d_tl = (
            self.params.coolant_coupling_gain * (tf - tl)
            - mode.coolant_loss_scale * (tl - self.params.inlet_temperature) / self.params.coolant_time_constant
        )
        return np.array([dn, dc, d_tf, d_tl], dtype=float)
