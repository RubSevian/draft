"""Uncalibrated lumped motor temperature from measured rollout torque.

One thermal node represents an effective module temperature, not the winding
hotspot or the built-in sensor. RC values must be supplied explicitly.
"""
import numpy as np


def estimate_temperature(time_s, torque, peak_torque, no_load_speed, *,
                         ambient_C, initial_C, resistance_K_W, capacity_J_K):
    """Exact RC update for piecewise-constant losses over each sample interval.

    Uses the paper's theoretical copper-loss proxy tau² * omega_NL / tau_peak.
    No electrical/controller/gearbox losses or temperature feedback are included.
    """
    t = np.asarray(time_s, dtype=float)
    tau = np.asarray(torque, dtype=float)
    peak, speed = np.asarray(peak_torque, dtype=float), np.asarray(no_load_speed, dtype=float)
    if (t.ndim != 1 or len(t) < 2 or tau.ndim != 2 or tau.shape[0] != len(t)
            or peak.shape != tau.shape[1:] or speed.shape != peak.shape):
        raise ValueError('Expected time [samples], torque [samples,joints], limits [joints].')
    if not all(np.all(np.isfinite(x)) for x in (t, tau, peak, speed,
            ambient_C, initial_C, resistance_K_W, capacity_J_K)):
        raise ValueError('All inputs must be finite.')
    if (np.any(np.diff(t) <= 0) or np.any(peak <= 0) or np.any(speed <= 0)
            or resistance_K_W <= 0 or capacity_J_K <= 0):
        raise ValueError('Time must increase; limits and thermal coefficients must be positive.')
    loss = tau**2 * speed / peak
    temperature = np.full(tau.shape, float(initial_C))
    for i, dt in enumerate(np.diff(t), 1):
        decay = np.exp(-dt / (resistance_K_W * capacity_J_K))
        equilibrium = ambient_C + resistance_K_W * loss[i - 1]
        temperature[i] = equilibrium + (temperature[i - 1] - equilibrium) * decay
    return temperature, loss
