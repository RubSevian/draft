"""Configuration and validation shared by the editor and headless importer."""
from __future__ import annotations

import math
import re
from pathlib import Path

import yaml

from draft.paths import imported_configs_dir, repo_root

PARAMETER_LABELS = {
    "kp": "Position gain Kp (N·m/rad)", "kd": "Velocity gain Kd (N·m·s/rad)",
    "effort_limit_nm": "Peak torque limit (N·m)",
    "joint_speed_limit_rad_s": "Joint speed limit (rad/s)",
    "armature_kgm2": "Reflected rotor inertia (kg·m²)",
    "joint_damping": "Joint damping (N·m·s/rad)",
    "joint_frictionloss": "Joint dry friction (N·m)",
    "stance_hip_roll": "Hip roll in standing pose (rad)",
    "stance_hip_pitch": "Hip pitch in standing pose (rad)",
    "stance_knee": "Knee in standing pose (rad)",
    "payload_mass_kg": "Payload mass (kg)",
    "payload_x": "Payload centre X (m)", "payload_y": "Payload centre Y (m)",
    "payload_z": "Payload centre Z (m)",
    "payload_size_x": "Payload size X (m)", "payload_size_y": "Payload size Y (m)",
    "payload_size_z": "Payload size Z (m)",
    "ground_friction": "Sliding friction coefficient",
    "full_body_collisions": "Enable body and leg contacts",
    "timestep": "Physics timestep (s)",
    "thermal_enabled": "Enable temperature prediction (requires thermal data)",
    "initial_temperature_c": "Initial motor temperature (°C)",
    "ambient_temperature_c": "Ambient temperature (°C)",
}


def parameter_group(key: str) -> str:
    if key.startswith("payload_"):
        return "Payload"
    if key.startswith("stance_"):
        return "Standing pose"
    if "temperature" in key or key == "thermal_enabled":
        return "Temperature"
    if key in ("ground_friction", "full_body_collisions", "timestep"):
        return "Environment"
    return "Drive settings"


def read_yaml(path: Path) -> dict:
    value = yaml.safe_load(path.read_text())
    if not isinstance(value, dict):
        raise ValueError(f"Expected a mapping in {path}")
    return value


def profiles() -> dict[str, Path]:
    return {p.stem: p for p in sorted(imported_configs_dir().glob("*.yaml"))}


def load_profile(path: Path, overrides: dict | None = None) -> tuple[dict, dict, dict]:
    profile = read_yaml(Path(path))
    if not re.fullmatch(r"[a-z][a-z0-9_]*", str(profile.get("name", ""))):
        raise ValueError("Robot name must be a lowercase identifier")
    if overrides is not None and not isinstance(overrides, dict):
        raise ValueError("Parameter overrides must be a YAML mapping")
    motor = read_yaml(repo_root() / profile["motor"])
    params = dict(profile["parameters"])
    unknown = set(overrides or {}) - params.keys()
    if unknown:
        raise ValueError(f"Unknown imported robot parameters: {sorted(unknown)}")
    params.update(overrides or {})
    for key, value in params.items():
        if key in ("thermal_enabled", "full_body_collisions"):
            if not isinstance(value, bool):
                raise ValueError(f"{key} must be a boolean")
        elif isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"{key} must be a finite number")
    positive = ("kp", "effort_limit_nm", "joint_speed_limit_rad_s", "timestep",
                "payload_size_x", "payload_size_y", "payload_size_z")
    for key in positive:
        if params[key] <= 0:
            raise ValueError(f"{key} must be positive")
    for key in ("kd", "payload_mass_kg", "ground_friction", "armature_kgm2",
                "joint_damping", "joint_frictionloss"):
        if params[key] < 0:
            raise ValueError(f"{key} cannot be negative")
    if params["timestep"] > 0.01:
        raise ValueError("timestep must be <= 0.01 s")
    for key in ("peak_torque_nm", "no_load_speed_rad_s", "output_torque_constant_nm_per_a_rms",
                "line_resistance_ohm"):
        value = motor.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"Motor {key} must be finite and positive")
    if params["effort_limit_nm"] > motor["peak_torque_nm"]:
        raise ValueError("effort_limit_nm exceeds the motor profile's peak torque")
    if params["thermal_enabled"]:
        from draft.imported.thermal import ThermalModel
        ThermalModel(motor, 1, params["initial_temperature_c"], params["ambient_temperature_c"])
    return profile, params, motor
