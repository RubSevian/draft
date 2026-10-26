"""Two-node winding/housing thermal network, optional and explicitly calibrated.

The motor profile's Kt is output torque / RMS phase current. Copper losses are
1.5 * line-to-line resistance * I_rms² for a balanced three-phase motor.
Unmodelled iron, gearbox and inverter losses limit the prediction's accuracy.
"""
from __future__ import annotations

import math

import numpy as np


class ThermalModel:
    def __init__(self, motor: dict, count: int, initial_c: float, ambient_c: float):
        cfg = motor.get("thermal")
        if not isinstance(cfg, dict):
            raise ValueError("Temperature prediction needs thermal coefficients in the motor profile; "
                             "leave thermal_enabled=false until they are available.")
        self.cfg = cfg
        for key in ("winding_heat_capacity_j_k", "housing_heat_capacity_j_k",
                    "winding_to_housing_k_w", "housing_to_air_k_w"):
            v = cfg.get(key)
            if isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v) or v <= 0:
                raise ValueError(f"Thermal {key} must be finite and positive")
        for key in ("derate_start_c", "shutdown_c"):
            v = cfg.get(key)
            if isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v):
                raise ValueError(f"Thermal {key} is required and must be finite")
        if cfg["shutdown_c"] <= cfg["derate_start_c"]:
            raise ValueError("shutdown_c must exceed derate_start_c")
        if not all(math.isfinite(v) and v > -273.15 for v in (initial_c, ambient_c)):
            raise ValueError("Temperatures must be finite and above absolute zero")
        self.motor = motor
        self.winding = np.full(count, initial_c, dtype=float)
        self.housing = np.full(count, initial_c, dtype=float)
        self.ambient = ambient_c
        self.initial = initial_c

    def reset(self) -> None:
        self.winding[:] = self.initial
        self.housing[:] = self.initial

    def torque_scale(self) -> np.ndarray:
        lo, hi = self.cfg["derate_start_c"], self.cfg["shutdown_c"]
        return np.clip((hi - self.winding) / (hi - lo), 0.0, 1.0)

    def step(self, torque: np.ndarray, dt: float) -> None:
        if not math.isfinite(dt) or dt <= 0 or not np.isfinite(torque).all():
            raise ValueError("Thermal integration requires a positive finite dt and finite torque")
        c = self.cfg
        cw, ch = c["winding_heat_capacity_j_k"], c["housing_heat_capacity_j_k"]
        rw, ra = c["winding_to_housing_k_w"], c["housing_to_air_k_w"]
        # Bound the explicit integration step for even very small heat capacities.
        safe_dt = 0.1 * min(cw * rw, ch / (1 / rw + 1 / ra))
        steps = max(1, math.ceil(dt / safe_dt))
        if steps > 10000:
            raise ValueError("Thermal time constants are too small for this simulation timestep")
        current = np.asarray(torque) / self.motor["output_torque_constant_nm_per_a_rms"]
        for _ in range(steps):
            resistance = self.motor["line_resistance_ohm"] * (
                1 + self.motor.get("copper_temperature_coefficient", 0.0039)
                * (self.winding - self.motor.get("resistance_reference_c", 25.0)))
            loss = 1.5 * np.maximum(resistance, 0) * current**2
            transfer = (self.winding - self.housing) / rw
            cooling = (self.housing - self.ambient) / ra
            self.winding += (dt / steps) * (loss - transfer) / cw
            self.housing += (dt / steps) * (transfer - cooling) / ch
        if not np.isfinite(self.winding).all() or not np.isfinite(self.housing).all():
            raise FloatingPointError("Motor temperature integration became non-finite")
