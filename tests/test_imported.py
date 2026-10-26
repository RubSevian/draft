"""Preservation of hardware data, portable builds and usable controlled dynamics."""
import hashlib
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import pytest
import yaml

from draft.editor.core.session import EditorSession, is_parameter_yaml
from draft.editor.shared import discover_robot_types
from draft.imported.builder import build_imported, LEGS
from draft.imported.runtime import MotorController, run_headless, torque_bounds
from draft.imported.thermal import ThermalModel
from draft.paths import repo_root

ROOT = repo_root()
PROFILE = ROOT / "configs/robots/mevius2.yaml"


@pytest.fixture(scope="module")
def built(tmp_path_factory):
    out = tmp_path_factory.mktemp("mevius2")
    build_imported(PROFILE, out)
    return out


def test_import_preserves_vendor_mass_inertias_axes_and_limits(built):
    source = ROOT / "urdf/mevius2/mevius2.urdf"
    root = ET.parse(source).getroot()
    for mesh in root.findall(".//mesh"):
        mesh.set("filename", str(source.parent / mesh.get("filename")))
    compiler = ET.SubElement(ET.SubElement(root, "mujoco"), "compiler")
    compiler.attrib.update(discardvisual="false", fusestatic="false", strippath="false")
    original_spec = mujoco.MjSpec.from_string(ET.tostring(root, encoding="unicode"))
    original_spec.body("base_link").add_freejoint()
    original = original_spec.compile()
    imported = mujoco.MjModel.from_xml_path(str(built / "mevius2.xml"))
    cfg = yaml.safe_load((built / "simulation.yaml").read_text())
    assert (imported.nq, imported.nv, imported.nu) == (19, 18, 12)
    assert imported.body_mass.sum() == pytest.approx(original.body_mass.sum(), abs=5e-5)
    for old, new in cfg["body_map"].items():
        for field in ("body_mass", "body_inertia", "body_ipos", "body_iquat", "body_pos", "body_quat"):
            a = getattr(original, field)[original.body(old).id]
            b = getattr(imported, field)[imported.body(new).id]
            np.testing.assert_allclose(a, b, rtol=2e-5, atol=2e-6)
    for old, new in cfg["joint_map"].items():
        for field in ("jnt_axis", "jnt_range", "jnt_pos"):
            np.testing.assert_allclose(getattr(original, field)[original.joint(old).id],
                                       getattr(imported, field)[imported.joint(new).id], atol=2e-6)
    for leg in LEGS:
        assert imported.site(f"{leg}_foot").id >= 0
        assert imported.geom(f"{leg}_lower_leg_link_sphere").size[0] == pytest.approx(0.03)


def test_build_is_portable_and_does_not_reference_the_original_checkout(built, tmp_path):
    dest = tmp_path / "moved"
    shutil.copytree(built, dest)
    for path in dest.glob("*.xml"):
        for mesh in ET.parse(path).getroot().findall(".//mesh"):
            assert not Path(mesh.get("file", "")).is_absolute()
        assert str(ROOT) not in path.read_text()
    model = mujoco.MjModel.from_xml_path(str(dest / "scene.xml"))
    assert model.nu == 12


def test_mapping_cannot_attach_a_foot_to_another_leg(tmp_path):
    profile = yaml.safe_load(PROFILE.read_text())
    profile["legs"]["fl"]["foot"] = profile["legs"]["rr"]["foot"]
    path = tmp_path / "bad-profile.yaml"
    path.write_text(yaml.safe_dump(profile))
    with pytest.raises(ValueError, match="foot must be fixed"):
        build_imported(path, tmp_path / "output")


def test_default_stance_remains_upright_and_respects_torque_limits(built):
    model = mujoco.MjModel.from_xml_path(str(built / "scene.xml"))
    data = mujoco.MjData(model)
    controller = MotorController.from_folder(model, built)
    controller.reset(data)
    for _ in range(round(10 / model.opt.timestep)):
        controller.step(data)
        assert np.max(np.abs(controller.applied) - controller.effort) < 1e-6
    assert data.time == pytest.approx(10)
    assert data.qpos[2] > 0.4
    assert data.xmat[model.body("torso_link").id].reshape(3, 3)[2, 2] > np.cos(np.deg2rad(5))
    assert controller.telemetry(data)["motors"][0]["winding_c"] is None
    controller.reset(data)
    assert data.time == 0
    assert np.all(data.ctrl == 0)


def test_payload_updates_mass_and_inertia_without_replacing_robot(built, tmp_path):
    build_imported(PROFILE, tmp_path, {"payload_mass_kg": 5.0, "payload_x": 0.05})
    base = mujoco.MjModel.from_xml_path(str(built / "scene.xml"))
    loaded = mujoco.MjModel.from_xml_path(str(tmp_path / "scene.xml"))
    assert loaded.body_mass.sum() - base.body_mass.sum() == pytest.approx(5.0)
    assert loaded.body("payload").ipos[0] == 0
    np.testing.assert_allclose(loaded.body("payload").pos, [0.05, 0, 0.23])
    for name in ("torso_link", "fl_lower_leg_link"):
        np.testing.assert_allclose(loaded.body(name).inertia, base.body(name).inertia)


@pytest.mark.parametrize("override", [
    {"kp": float("nan")}, {"payload_mass_kg": -1}, {"effort_limit_nm": 61},
    {"stance_knee": 0}, {"thermal_enabled": True}, {"timestep": 0.1},
    {"unknown": 1}, {"full_body_collisions": "false"},
])
def test_invalid_configuration_preserves_last_good_build(built, tmp_path, override):
    out = tmp_path / "model"
    shutil.copytree(built, out)
    before = {p.name: hashlib.sha256(p.read_bytes()).digest() for p in out.glob("*") if p.is_file()}
    with pytest.raises((ValueError, TypeError)):
        build_imported(PROFILE, out, override)
    assert before == {p.name: hashlib.sha256(p.read_bytes()).digest() for p in out.glob("*") if p.is_file()}


def test_full_contacts_and_foot_only_are_explicit(built, tmp_path):
    build_imported(PROFILE, tmp_path, {"full_body_collisions": False})
    full = mujoco.MjModel.from_xml_path(str(built / "mevius2.xml"))
    feet = mujoco.MjModel.from_xml_path(str(tmp_path / "mevius2.xml"))
    assert np.count_nonzero(full.geom_contype) > 4
    assert np.count_nonzero(feet.geom_contype) == 4


def test_editor_roundtrip_keeps_import_settings_and_excludes_reports(tmp_path):
    configs = discover_robot_types(ROOT)
    assert {"mevius2", "humanoid", "quadruped"} <= configs.keys()
    session = EditorSession(configs["mevius2"], tmp_path)
    session.set("kp", 120)
    session.set("payload_mass_kg", 2)
    saved = session.save_as("loaded-dog")
    other = EditorSession(configs["mevius2"], tmp_path)
    other.folders.select(saved.name)
    other.load_selected_folder()
    assert other.collect()["kp"] == 120
    assert other.collect()["payload_mass_kg"] == 2
    assert not is_parameter_yaml(saved / "simulation.yaml")
    assert not is_parameter_yaml(saved / "import_report.yaml")
    other.reset_to_defaults()
    assert other.collect()["payload_mass_kg"] == 0


def test_headless_csv_contains_real_torque_and_no_fabricated_temperature(built, tmp_path):
    import csv
    path = tmp_path / "trace.csv"
    result = run_headless(built / "scene.xml", 0.2, path)
    rows = list(csv.DictReader(path.open()))
    assert len({row["joint"] for row in rows}) == 12
    assert any(abs(float(row["torque_nm"])) > 0.1 for row in rows)
    assert all(row["winding_c"] == row["housing_c"] == "" for row in rows)
    assert result["thermal_enabled"] is False


def test_speed_limit_blocks_accelerating_torque_but_allows_braking():
    lo, hi = torque_bounds(np.array([-25, -10, 0, 10, 25]), np.full(5, 60), 30, 20)
    assert lo[0] == 0 and hi[-1] == 0
    assert hi[0] > 0 and lo[-1] < 0
    assert hi[2] == 60 and lo[2] == -60
    assert hi[3] < hi[2] and np.all(lo <= hi)


def synthetic_motor():
    # Synthetic test fixture, not asserted RobStride thermal data.
    return {"output_torque_constant_nm_per_a_rms": 2.0,
            "line_resistance_ohm": 0.4, "copper_temperature_coefficient": 0.0,
            "thermal": {"winding_heat_capacity_j_k": 10., "housing_heat_capacity_j_k": 20.,
                        "winding_to_housing_k_w": 1., "housing_to_air_k_w": 2.,
                        "derate_start_c": 60., "shutdown_c": 80.}}


def test_thermal_energy_balance_and_steady_state():
    motor = synthetic_motor()
    thermal = ThermalModel(motor, 1, 25., 25.)
    thermal.step(np.array([4.]), 0.001)
    # I=2 Arms, heat=1.5*0.4*4=2.4 W, initially no heat flow to ambient.
    energy = 10 * (thermal.winding[0] - 25) + 20 * (thermal.housing[0] - 25)
    assert energy == pytest.approx(2.4 * 0.001)
    for _ in range(20000):
        thermal.step(np.array([4.]), 0.05)
    assert thermal.housing[0] == pytest.approx(25 + 2.4 * 2, abs=0.01)
    assert thermal.winding[0] == pytest.approx(25 + 2.4 * 3, abs=0.01)
    hot = thermal.winding[0]
    for _ in range(2000):
        thermal.step(np.array([0.]), 0.05)
    assert 25 <= thermal.winding[0] < hot
    thermal.winding[:] = 70
    np.testing.assert_allclose(thermal.torque_scale(), 0.5)
    thermal.winding[:] = 85
    assert thermal.torque_scale()[0] == 0


def test_controller_shutdown_uses_applied_torque_for_heat(built):
    model = mujoco.MjModel.from_xml_path(str(built / "scene.xml"))
    config = yaml.safe_load((built / "simulation.yaml").read_text())
    config["motor"]["thermal"] = synthetic_motor()["thermal"]
    config["params"]["thermal_enabled"] = True
    controller = MotorController(model, config)
    data = mujoco.MjData(model)
    controller.reset(data)
    controller.thermal.winding[:] = 90
    controller.target += 0.1
    controller.step(data)
    np.testing.assert_allclose(controller.applied, 0)
    assert np.all(controller.thermal.winding < 90)
