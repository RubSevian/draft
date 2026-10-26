"""The 3D half of the editor: a viser scene showing one generated model.

Owns the scene and physics loop and exposes the joints as data; the joint
sliders and simulate toggle live on the editor page (`editor/web/`), which runs
this class in the same process.
"""

from __future__ import annotations

import threading
import math
import time
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import viser
from mjviser.scene import ViserMujocoScene

from draft.generation import mjcf_assets
from draft.imported.runtime import MotorController


class Viewer3D:
    """A viser server showing one MuJoCo model. The editor page drives it."""

    def __init__(self, port: int = 8080, verbose: bool = True) -> None:
        self.server = viser.ViserServer(host="127.0.0.1", port=port)
        self.server.gui.configure_theme(control_layout="collapsible",
                                        show_logo=False, show_share_button=False)
        # The bound port: viser moves to the next free one if `port` is taken.
        self.port = self.server.get_port()
        if verbose:
            print(f"[viewer] 3D scene on http://localhost:{self.port}", flush=True)

        self._mj_model: mujoco.MjModel | None = None
        self._mj_data: mujoco.MjData | None = None
        self._scene: ViserMujocoScene | None = None
        self._scene_lock = threading.Lock()
        self._simulate = False
        self._controller: MotorController | None = None
        self.error: str | None = None
        self._stop = threading.Event()
        self._camera_frame = None
        self._camera_model = None
        self.server.on_client_connect(self._frame_client)
        #: One entry per movable joint: name, limits, and its qpos address.
        #: Handed to the page, which draws the sliders.
        self._joints: list[dict[str, Any]] = []
        self._status = "Ready."

    # ── what the page drives ─────────────────────────────────────────────────

    def set_status(self, msg: str) -> None:
        self._status = msg

    @property
    def simulate(self) -> bool:
        return self._simulate

    def set_simulate(self, on: bool) -> None:
        """Run physics, or hold the slider pose (resetting on the way out)."""
        with self._scene_lock:
            if on and self._mj_model is None:
                raise ValueError("Build and load a robot before starting physics")
            self._simulate = bool(on)
            if on and self._controller:
                self._controller.capture_pose(self._mj_data)
        if not self._simulate:
            self.reset_pose()

    def telemetry(self) -> dict | None:
        with self._scene_lock:
            return self._controller.telemetry(self._mj_data) if self._controller else None

    def clear(self) -> None:
        with self._scene_lock:
            self._simulate = False
            self._controller = None
            self._mj_model = self._mj_data = self._scene = None
            self._joints = []
            self.error = None
            self._camera_frame = None
            self._camera_model = None
            self.server.scene.reset()

    def _frame_client(self, client) -> None:
        frame = self._camera_frame
        if frame is not None:
            center, position = frame
            client.camera.up_direction = (0, 0, 1)
            client.camera.position = position
            client.camera.look_at = center

    def joints(self) -> list[dict]:
        """The movable joints of the loaded model, with their current angles."""
        out = []
        with self._scene_lock:
            for j in self._joints:
                q = (float(self._mj_data.qpos[j["qadr"]])
                     if self._mj_data is not None else 0.0)
                out.append({**j, "value": q})
        return out

    def set_joint(self, qadr: int, value: float) -> None:
        """Pose one joint. Ignored while physics is running, which owns qpos."""
        if self._simulate:
            return
        with self._scene_lock:
            if self._mj_data is None or self._mj_model is None:
                return
            joint = next((j for j in self._joints if j["qadr"] == qadr), None)
            if joint is None or not math.isfinite(value):
                raise ValueError("Invalid joint or angle")
            value = min(max(value, joint["lo"]), joint["hi"])
            self._mj_data.qpos[qadr] = float(value)
            mujoco.mj_forward(self._mj_model, self._mj_data)
            if self._scene:
                self._scene.update_from_mjdata(self._mj_data)

    # ── model ─────────────────────────────────────────────────────────────────

    def load_model(self, mjcf_path: Path) -> bool:
        """Replace the 3D scene with a freshly-generated MJCF, in place."""
        mjcf_path = Path(mjcf_path)
        scene_xml = Path(mjcf_path).parent / "scene.xml"
        load_path = scene_xml if scene_xml.is_file() else Path(mjcf_path)
        try:
            # Not `from_xml_path`: MuJoCo caches meshes by path, and every build
            # rewrites the same folder.
            model = mjcf_assets.load_model(load_path)
            data = mujoco.MjData(model)
            controller = MotorController.from_folder(model, load_path.parent)
            if controller:
                controller.reset(data)
            else:
                mujoco.mj_resetData(model, data)
            mujoco.mj_forward(model, data)
            with self._scene_lock:
                self.server.scene.reset()
                self._scene = ViserMujocoScene(self.server, model, num_envs=1)
                self._mj_model = model
                self._mj_data = data
                self._controller = controller
                self._simulate = False
                self.error = None
                self._scene.update_from_mjdata(data)
                self._scan_joints()
                # Exclude the floor and frame the actual robot for new clients.
                # Rebuilding parameters preserves an existing client's camera.
                points = data.xpos[1:]
                if len(points):
                    low, high = points.min(axis=0), points.max(axis=0)
                    center = (low + high) / 2
                    distance = max(1.2, 2.2 * np.linalg.norm(high - low))
                    direction = np.array([1.0, -1.0, 0.7])
                    position = center + distance * direction / np.linalg.norm(direction)
                    self._camera_frame = (tuple(center), tuple(position))
                    if self._camera_model != mjcf_path.name:
                        for client in self.server.get_clients().values():
                            self._frame_client(client)
                    self._camera_model = mjcf_path.name
        except Exception as exc:
            self.error = str(exc)
            self.set_status(f"**Error loading model:** {exc}")
            return False
        return True

    def reset_pose(self) -> None:
        with self._scene_lock:
            if self._mj_model and self._mj_data and self._scene:
                if self._controller:
                    self._controller.reset(self._mj_data)
                else:
                    mujoco.mj_resetData(self._mj_model, self._mj_data)
                self.error = None
                mujoco.mj_forward(self._mj_model, self._mj_data)
                self._scene.update_from_mjdata(self._mj_data)

    # ── joints ────────────────────────────────────────────────────────────────

    def _scan_joints(self) -> None:
        """Record the movable joints of the freshly-loaded model."""
        self._joints = []
        model = self._mj_model
        if model is None:
            return
        HINGE = int(mujoco.mjtJoint.mjJNT_HINGE)
        SLIDE = int(mujoco.mjtJoint.mjJNT_SLIDE)
        for jid in range(model.njnt):
            jtype = int(model.jnt_type[jid])
            if jtype not in (HINGE, SLIDE):
                continue
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid) or f"joint_{jid}"
            if model.jnt_limited[jid]:
                lo, hi = float(model.jnt_range[jid, 0]), float(model.jnt_range[jid, 1])
            else:
                # Unlimited: +-pi for a hinge, +-1 m for a slide.
                lo, hi = (-3.14159, 3.14159) if jtype == HINGE else (-1.0, 1.0)
            self._joints.append({
                "name": name, "lo": lo, "hi": hi,
                "qadr": int(model.jnt_qposadr[jid]),
                "unit": "rad" if jtype == HINGE else "m",
            })

    # ── loop ──────────────────────────────────────────────────────────────────

    def start(self) -> None:
        """Run the render/physics loop on a daemon thread."""
        threading.Thread(target=self._loop, daemon=True).start()

    def stop(self) -> None:
        self._stop.set()
        self.server.stop()

    def _loop(self) -> None:
        last = time.monotonic()
        accumulator = 0.0
        while not self._stop.is_set():
            started = time.monotonic()
            elapsed, last = min(started - last, 0.1), started
            with self._scene_lock:
                if self._scene and self._mj_data and self._mj_model:
                    if self._simulate:
                        accumulator += elapsed
                        dt = self._mj_model.opt.timestep
                        try:
                            while accumulator >= dt:
                                if self._controller:
                                    self._controller.step(self._mj_data)
                                else:
                                    mujoco.mj_step(self._mj_model, self._mj_data)
                                accumulator -= dt
                        except Exception as exc:
                            self._simulate = False
                            self.error = str(exc)
                            accumulator = 0.0
                    else:
                        accumulator = 0.0
                    self._scene.update_from_mjdata(self._mj_data)
            time.sleep(max(0.001, 1 / 60 - (time.monotonic() - started)))
