"""CPU PD simulation with torque-speed limits and optional thermal feedback."""
from __future__ import annotations

import csv
from pathlib import Path

import mujoco
import numpy as np

from draft.imported.profile import read_yaml
from draft.imported.thermal import ThermalModel


def torque_bounds(velocity, effort, no_load_speed, speed_limit):
    """Four-quadrant linear envelope; overspeed inhibits driving, not braking."""
    lo = np.maximum(-effort, effort * (-1 - velocity / no_load_speed))
    hi = np.minimum(effort, effort * (1 - velocity / no_load_speed))
    # Above no-load speed there is no accelerating torque; avoid inverted bounds.
    hi = np.maximum(hi, -effort)
    lo = np.minimum(lo, effort)
    hi = np.where(velocity >= speed_limit, np.minimum(hi, 0), hi)
    lo = np.where(velocity <= -speed_limit, np.maximum(lo, 0), lo)
    return lo, hi


class MotorController:
    def __init__(self, model: mujoco.MjModel, config: dict):
        self.model, self.config = model, config
        self.params = config["params"]
        self.motor = config["motor"]
        self.names = list(config["joints"])
        self.jids = np.array([model.joint(n).id for n in self.names])
        self.qadr = model.jnt_qposadr[self.jids]
        self.dadr = model.jnt_dofadr[self.jids]
        self.aids = np.array([model.actuator(n).id for n in self.names])
        self.effort = np.array([config["joints"][n]["effort"] for n in self.names])
        self.speed = np.array([config["joints"][n]["speed"] for n in self.names])
        self.target = np.zeros(len(self.names))
        self.applied = np.zeros(len(self.names))
        self.thermal = (ThermalModel(self.motor, len(self.names),
                                    self.params["initial_temperature_c"],
                                    self.params["ambient_temperature_c"])
                        if self.params["thermal_enabled"] else None)

    @classmethod
    def from_folder(cls, model: mujoco.MjModel, folder: Path):
        path = Path(folder) / "simulation.yaml"
        return cls(model, read_yaml(path)) if path.is_file() else None

    def reset(self, data: mujoco.MjData) -> None:
        key = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, "home")
        if key < 0:
            raise ValueError("Imported model is missing its home pose")
        mujoco.mj_resetDataKeyframe(self.model, data, key)
        self.target = data.qpos[self.qadr].copy()
        self.applied[:] = 0
        if self.thermal:
            self.thermal.reset()
        mujoco.mj_forward(self.model, data)

    def capture_pose(self, data: mujoco.MjData) -> None:
        self.target = data.qpos[self.qadr].copy()

    def step(self, data: mujoco.MjData) -> None:
        q, velocity = data.qpos[self.qadr], data.qvel[self.dadr]
        request = self.params["kp"] * (self.target - q) - self.params["kd"] * velocity
        lo, hi = torque_bounds(velocity, self.effort, self.motor["no_load_speed_rad_s"], self.speed)
        if self.thermal:
            scale = self.thermal.torque_scale()
            # Intersect thermal current limits with the speed envelope.
            lo, hi = np.maximum(lo, -self.effort * scale), np.minimum(hi, self.effort * scale)
            # A disabled driver applies zero torque, even when back-EMF would brake.
            lo, hi = np.minimum(lo, 0), np.maximum(hi, 0)
        data.ctrl[self.aids] = np.clip(request, lo, hi)
        before_time = float(data.time)
        mujoco.mj_step(self.model, data)
        if data.time <= before_time or not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
            raise RuntimeError("MuJoCo became unstable; reduce timestep/gains or check the configuration")
        self.applied = data.qfrc_actuator[self.dadr].copy()
        if self.thermal:
            self.thermal.step(self.applied, self.model.opt.timestep)

    def telemetry(self, data: mujoco.MjData) -> dict:
        rows = []
        for i, name in enumerate(self.names):
            rows.append({"joint": name, "torque_nm": float(self.applied[i]),
                         "velocity_rad_s": float(data.qvel[self.dadr[i]]),
                         "winding_c": float(self.thermal.winding[i]) if self.thermal else None,
                         "housing_c": float(self.thermal.housing[i]) if self.thermal else None})
        return {"time_s": float(data.time), "thermal_enabled": self.thermal is not None,
                "thermal_calibrated": bool(self.motor.get("thermal", {}).get("calibrated", False))
                    if self.thermal else False,
                "motors": rows}


def run_headless(scene: Path, seconds: float, csv_path: Path) -> dict:
    """Run exactly simulated seconds, independent of rendering or wall time."""
    if not np.isfinite(seconds) or seconds <= 0:
        raise ValueError("seconds must be finite and positive")
    model = mujoco.MjModel.from_xml_path(str(Path(scene).resolve()))
    data = mujoco.MjData(model)
    controller = MotorController.from_folder(model, Path(scene).parent)
    if controller is None:
        raise ValueError("No simulation.yaml beside the scene; build an imported profile first")
    controller.reset(data)
    csv_path = Path(csv_path)
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["time_s", "joint", "torque_nm", "velocity_rad_s", "winding_c", "housing_c"])
        writer.writeheader()
        sample_every = max(1, round(0.02 / model.opt.timestep))
        steps = int(np.ceil(seconds / model.opt.timestep))
        for i in range(steps):
            controller.step(data)
            if i % sample_every == 0 or i == steps - 1:
                state = controller.telemetry(data)
                for row in state["motors"]:
                    writer.writerow({"time_s": state["time_s"], **row})
    result = controller.telemetry(data)
    result["base_height_m"] = float(data.qpos[2])
    result["csv"] = str(csv_path)
    return result
