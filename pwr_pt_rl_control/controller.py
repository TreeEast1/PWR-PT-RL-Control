from __future__ import annotations

import numpy as np

from .config import ControllerParameters, CoreParameters


def sign_power(value: float, power: float) -> float:
    return np.sign(value) * (abs(value) ** power)


class PreassignedTimeController:
    def __init__(self, core: CoreParameters, params: ControllerParameters):
        self.core = core
        self.params = params

    def control(
        self,
        t: float,
        nr: float,
        nr_ref: float,
        tf_hat: float,
        tl_hat: float,
        mode_bias: float,
    ) -> float:
        error = nr - nr_ref
        remaining = max(self.params.tp - min(t, self.params.tp - 1e-3), 1e-3)
        scheduled_gain = (self.params.tp / remaining) ** 2
        fast_term = -self.params.k_fast * scheduled_gain * sign_power(error, 0.5)
        linear_term = -self.params.k_linear * scheduled_gain * error
        if t >= self.params.tp:
            fast_term = -self.params.post_gain * sign_power(error, 0.5)
            linear_term = -self.params.post_gain * error
        thermal_comp = -self.params.k_temp * ((tf_hat - 304.0) + 0.6 * (tl_hat - 296.0))
        raw_control = fast_term + linear_term + thermal_comp - mode_bias
        return float(np.clip(raw_control, *self.core.controller_limit))
