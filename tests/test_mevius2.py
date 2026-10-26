"""MEVIUS2 uses the native generator/editor with the source kinematic chain."""
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest
import yaml

from draft.editor.core.session import EditorSession
from draft.editor.shared import discover_robot_types
from draft.generation.generator import RobotGenerator
from draft.paths import repo_root
from draft.robots import robot_dir


@pytest.fixture(scope="module")
def baseline(tmp_path_factory):
    out = tmp_path_factory.mktemp("mevius2-native")
    RobotGenerator(robot_dir("mevius2")).generate(out)
    return out


def model(out):
    return mujoco.MjModel.from_xml_path(str(out / "mevius2.xml"))


@pytest.fixture(scope="module")
def source():
    path = repo_root() / "urdf/mevius2/mevius2.urdf"
    tree = ET.parse(path).getroot()
    for mesh in tree.findall(".//mesh"):
        mesh.set("filename", str(path.parent / mesh.get("filename")))
    ET.SubElement(ET.SubElement(tree, "mujoco"), "compiler", {"fusestatic": "false"})
    return mujoco.MjModel.from_xml_string(ET.tostring(tree, encoding="unicode"))


@pytest.mark.parametrize("angles", [(0, 0, 0), (0.1, 0.7, -1.2), (-0.2, -0.3, -2)])
def test_joint_axes_limits_and_foot_kinematics_match_urdf(baseline, source, angles):
    native = model(baseline)
    a, b = mujoco.MjData(source), mujoco.MjData(native)
    # Compare in the same base frame, independent of the editor's starting height.
    b.qpos[:3] = 0
    for old, leg in zip(("FL", "FR", "BL", "BR"), ("fl", "fr", "rl", "rr")):
        for raw, role, angle in zip(("collar", "hip", "knee"), ("hip_roll", "hip_pitch", "knee"), angles):
            src, dst = source.joint(f"{old}_{raw}_joint"), native.joint(f"{leg}_{role}")
            a.qpos[src.qposadr[0]] = angle
            b.qpos[dst.qposadr[0]] = angle
            np.testing.assert_allclose(src.range, dst.range, atol=1e-6)
    mujoco.mj_forward(source, a)
    mujoco.mj_forward(native, b)
    for old, leg in zip(("FL", "FR", "BL", "BR"), ("fl", "fr", "rl", "rr")):
        for raw, role in zip(("collar", "hip", "knee"), ("hip_roll", "hip_pitch", "knee")):
            sid, nid = source.joint(f"{old}_{raw}_joint").id, native.joint(f"{leg}_{role}").id
            np.testing.assert_allclose(a.xanchor[sid], b.xanchor[nid], atol=2e-6)
            np.testing.assert_allclose(a.xaxis[sid], b.xaxis[nid], atol=2e-6)
        np.testing.assert_allclose(a.body(f"{old}_foot_link").xpos, b.site(f"{leg}_foot").xpos, atol=2e-6)
    assert (native.nq, native.nv, native.nu) == (19, 18, 0)  # native Draft torque-joint contract
    for leg in ("fl", "fr", "rl", "rr"):
        assert native.geom(f"{leg}_lower_leg_link_sphere").size[0] == pytest.approx(0.03)


def test_lengths_and_trunk_dimensions_regenerate_geometry_and_mass(baseline, tmp_path):
    RobotGenerator(robot_dir("mevius2")).generate(tmp_path, params_override={
        "upper_leg_link_length": 0.36, "lower_leg_link_length": 0.32,
        "torso_lx": 0.48, "torso_ly": 0.24, "torso_lz": 0.21,
        "shoulder_width": 0.19})
    old, new = model(baseline), model(tmp_path)
    for leg, front, side in (("fl", 1, 1), ("fr", 1, -1), ("rl", -1, 1), ("rr", -1, -1)):
        np.testing.assert_allclose(new.body(f"{leg}_roll_motor_link").pos,
                                   [front * 0.243, side * 0.095, 0.062])
        assert new.body(f"{leg}_lower_leg_link").pos[2] == pytest.approx(-0.36)
        assert new.site(f"{leg}_foot").pos[2] == pytest.approx(-0.32)
    assert new.body_mass.sum() > old.body_mass.sum()
    assert new.body("torso_link").mass > old.body("torso_link").mass
    assert not np.allclose(new.body("fl_upper_leg_link").inertia,
                           old.body("fl_upper_leg_link").inertia)


def test_motor_class_selection_updates_joint_limits_and_charges_each_motor_once(tmp_path):
    RobotGenerator(robot_dir("mevius2")).generate(tmp_path, params_override={
        "hip_pitch_mot": "M", "knee_mot": "S"})
    m = model(tmp_path)
    for leg in ("fl", "fr", "rl", "rr"):
        for role, effort in (("hip_roll", 60), ("hip_pitch", 17), ("knee", 48)):
            np.testing.assert_allclose(m.jnt_actfrcrange[m.joint(f"{leg}_{role}").id], [-effort, effort])
    report = yaml.safe_load((tmp_path / "feasibility_report.yaml").read_text())
    assert report["actuator_packing"]["n_actuator_geoms"] == 12
    assert report["actuator_packing"]["n_overlapping_pairs"] == 0
    assert report["actuator_mass_audit"]["emitted_over_charged"] == pytest.approx(1.0, abs=1e-3)


def test_native_editor_discovers_changes_and_saves_mevius2(tmp_path):
    cfg = discover_robot_types(repo_root())["mevius2"]
    assert cfg.yaml_path == robot_dir("mevius2") / "parameters.yaml"
    session = EditorSession(cfg, tmp_path)
    assert session.schema.motor_classes == ("L", "M", "S")
    session.set("upper_leg_link_length", 0.34)
    session.set("hip_pitch_mot", "M")
    session.motors.set({"L_motor_effort": 55})
    saved = session.save_as("longer-legs")
    assert (saved / "feasibility_report.yaml").is_file()
    loaded = EditorSession(cfg, tmp_path)
    loaded.folders.select(saved.name)
    loaded.load_selected_folder()
    assert loaded.collect()["upper_leg_link_length"] == 0.34
    assert loaded.collect()["hip_pitch_mot"] == "M"
    assert loaded.collect()["L_motor_effort"] == 55
    assert not (saved / "simulation.yaml").exists()
    assert not (saved / "import_report.yaml").exists()
