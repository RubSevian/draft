"""Compile a configured URDF without replacing vendor inertias with trends."""
from __future__ import annotations

import hashlib
import json
import shutil
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import yaml

from draft.imported.profile import load_profile
from draft.paths import repo_root

ROLES = {"hip_roll": "roll_link", "hip_pitch": "upper_leg_link", "knee": "lower_leg_link"}
LEGS = ("fl", "fr", "rl", "rr")


def _write_yaml(path: Path, value: dict) -> None:
    path.write_text(yaml.safe_dump(value, sort_keys=False))


def build_imported(profile_path: Path, output_dir: Path, overrides: dict | None = None) -> Path:
    """Validate in a staging directory before replacing any last good build."""
    profile, params, motor = load_profile(profile_path, overrides)
    output_dir = Path(output_dir).resolve()
    output_dir.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".import-", dir=output_dir.parent) as tmp:
        stage = Path(tmp)
        _compile(profile, params, motor, stage)
        output_dir.mkdir(parents=True, exist_ok=True)
        for path in stage.iterdir():
            if path.is_dir():
                shutil.copytree(path, output_dir / path.name, dirs_exist_ok=True)
            else:
                shutil.copy2(path, output_dir / path.name)
    return output_dir / f"{profile['name']}.xml"


def _compile(profile: dict, params: dict, motor: dict, out: Path) -> None:
    source = repo_root() / profile["urdf"]
    root = ET.parse(source).getroot()
    if root.tag != "robot":
        raise ValueError("Expected a URDF <robot> root")
    links = {link.get("name"): link for link in root.findall("link")}
    joints = {j.get("name"): j for j in root.findall("joint")}
    base = profile["base_link"]
    if base not in links:
        raise ValueError(f"Missing base link {base!r}")
    if any(j.get("type") == "floating" for j in joints.values()):
        raise ValueError("Import a base-rooted URDF; the importer adds its floating base")
    parents = {j.find("child").get("link"): j.find("parent").get("link") for j in joints.values()}
    parent_joints = {j.find("child").get("link"): j for j in joints.values()}
    def fixed_descendant(link, ancestor):
        visited = set()
        while link != ancestor:
            if link in visited or link not in parent_joints:
                return False
            visited.add(link)
            joint = parent_joints[link]
            if joint.get("type") != "fixed":
                return False
            link = joint.find("parent").get("link")
        return True
    if set(links) - set(parents) != {base}:
        raise ValueError("base_link must be the single root of the URDF")
    mass_before = sum(float(m.get("value")) for m in root.findall("link/inertial/mass"))
    renames = {base: "torso_link"}
    joint_map, limits = {}, {}
    if set(profile["legs"]) != set(LEGS):
        raise ValueError("legs must explicitly map fl, fr, rl and rr")
    for leg in LEGS:
        mapping = profile["legs"][leg]
        if set(mapping["joints"]) != set(ROLES):
            raise ValueError(f"{leg}: expected hip_roll, hip_pitch and knee")
        ancestor = base
        for role in ROLES:
            old = mapping["joints"][role]
            if old in joint_map or old not in joints:
                raise ValueError(f"Duplicate or missing joint {old!r}")
            j = joints[old]
            if not fixed_descendant(j.find("parent").get("link"), ancestor):
                raise ValueError(f"{leg}: {old} does not follow the mapped leg chain")
            ancestor = j.find("child").get("link")
            if j.get("type") != "revolute":
                raise ValueError(f"{old}: only limited revolute leg joints are supported")
            lim = j.find("limit")
            if lim is None:
                raise ValueError(f"{old}: joint limits are missing")
            lo, hi, effort, speed = (float(lim.get(k, 'nan')) for k in ("lower", "upper", "effort", "velocity"))
            if not np.isfinite([lo, hi, effort, speed]).all() or lo >= hi or min(effort, speed) <= 0:
                raise ValueError(f"{old}: invalid joint limits")
            if not lo <= params[f"stance_{role}"] <= hi:
                raise ValueError(f"stance_{role} is outside {old}'s range [{lo}, {hi}]")
            new = f"{leg}_{role}"
            joint_map[old] = new
            limits[new] = {"effort": min(effort, params["effort_limit_nm"]),
                           "speed": min(speed, params["joint_speed_limit_rad_s"])}
            renames[j.find("child").get("link")] = f"{leg}_{ROLES[role]}"
        foot = mapping["foot"]
        if foot not in links:
            raise ValueError(f"Missing foot {foot!r}")
        if not fixed_descendant(foot, ancestor):
            raise ValueError(f"{leg}: foot must be fixed to its mapped lower leg")
        renames[foot] = f"{leg}_foot_link"
    driven = {j.get("name") for j in joints.values() if j.get("type") != "fixed"}
    if set(joint_map) != driven:
        raise ValueError("The profile must map every movable joint exactly once")
    if len(set(renames.values())) != len(renames):
        raise ValueError("Profile maps multiple links to the same name")

    # The only changes to the source tree are paths and explicit name mappings.
    assets = out / "meshes"
    assets.mkdir()
    copied = {}
    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename", "")
        if "://" in filename:
            raise ValueError(f"Mesh URI {filename!r} needs a relative path in the bundled URDF")
        asset = (source.parent / filename).resolve()
        if not asset.is_file():
            raise FileNotFoundError(f"Missing mesh: {asset}")
        if asset.name in copied and copied[asset.name] != asset:
            raise ValueError(f"Ambiguous mesh basename: {asset.name}")
        copied[asset.name] = asset
        shutil.copy2(asset, assets / asset.name)
        mesh.set("filename", str(assets / asset.name))
    for link in links.values():
        link.set("name", renames.get(link.get("name"), link.get("name")))
    for j in joints.values():
        j.set("name", joint_map.get(j.get("name"), j.get("name")))
        for tag in ("parent", "child"):
            ref = j.find(tag)
            ref.set("link", renames.get(ref.get("link"), ref.get("link")))
    for existing in root.findall("mujoco"):
        root.remove(existing)
    mj = ET.SubElement(root, "mujoco")
    ET.SubElement(mj, "compiler", {"discardvisual": "false", "fusestatic": "false",
                                   "strippath": "false", "balanceinertia": "false"})
    prepared = out / "_prepared.urdf"
    ET.ElementTree(root).write(prepared, encoding="utf-8", xml_declaration=True)
    spec = mujoco.MjSpec.from_file(str(prepared))
    torso = spec.body("torso_link")
    torso.add_freejoint(name="root")
    spec.option.timestep = params["timestep"]
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    if params["payload_mass_kg"] > 0:
        payload = torso.add_body(name="payload", pos=[params[f"payload_{a}"] for a in "xyz"])
        payload.add_geom(name="payload", type=mujoco.mjtGeom.mjGEOM_BOX,
                         size=[params[f"payload_size_{a}"] / 2 for a in "xyz"],
                         mass=params["payload_mass_kg"], rgba=[0.95, 0.5, 0.1, 1])
    for geom in spec.geoms:
        if geom.contype or geom.conaffinity:
            geom.friction = [params["ground_friction"], 0.005, 0.0001]

    # Locate feet through the profile rather than selecting arbitrary small spheres.
    probe = spec.compile()
    data = mujoco.MjData(probe)
    mujoco.mj_forward(probe, data)
    for leg in LEGS:
        foot = spec.body(f"{leg}_foot_link")
        spheres = [g for g in foot.geoms if g.type == mujoco.mjtGeom.mjGEOM_SPHERE
                   and (g.contype or g.conaffinity)]
        if len(spheres) != 1:
            raise ValueError(f"{leg}: expected exactly one collision sphere on the configured foot")
        geom = spheres[0]
        # The foot may have a fixed offset/rotation relative to the shank.
        fid = mujoco.mj_name2id(probe, mujoco.mjtObj.mjOBJ_BODY, foot.name)
        shank = spec.body(f"{leg}_lower_leg_link")
        sid = mujoco.mj_name2id(probe, mujoco.mjtObj.mjOBJ_BODY, shank.name)
        world = data.xpos[fid] + data.xmat[fid].reshape(3, 3) @ geom.pos
        local = data.xmat[sid].reshape(3, 3).T @ (world - data.xpos[sid])
        geom.name = f"{leg}_lower_leg_link_sphere"
        shank.add_site(name=f"{leg}_foot", pos=local, size=[0.005]*3)
    if not params["full_body_collisions"]:
        feet = {f"{leg}_lower_leg_link_sphere" for leg in LEGS}
        for geom in spec.geoms:
            if geom.name not in feet:
                geom.contype = geom.conaffinity = 0
    torso.add_site(name="imu", size=[0.005]*3)
    for name, kind in (("imu_ang_vel", mujoco.mjtSensor.mjSENS_GYRO),
                       ("imu_lin_vel", mujoco.mjtSensor.mjSENS_VELOCIMETER),
                       ("imu_lin_acc", mujoco.mjtSensor.mjSENS_ACCELEROMETER)):
        spec.add_sensor(name=name, type=kind, objtype=mujoco.mjtObj.mjOBJ_SITE, objname="imu")
    spec.add_sensor(name="root_angmom", type=mujoco.mjtSensor.mjSENS_SUBTREEANGMOM,
                    objtype=mujoco.mjtObj.mjOBJ_BODY, objname="torso_link")
    for name, limit in limits.items():
        joint = spec.joint(name)
        joint.armature = params["armature_kgm2"]
        joint.damping = [params["joint_damping"], 0.0, 0.0]
        joint.frictionloss = params["joint_frictionloss"]
        joint.actfrcrange = [-limit["effort"], limit["effort"]]
        joint.actfrclimited = mujoco.mjtLimited.mjLIMITED_TRUE
        spec.add_actuator(name=name, target=name, trntype=mujoco.mjtTrn.mjTRN_JOINT,
                          ctrllimited=mujoco.mjtLimited.mjLIMITED_TRUE,
                          forcelimited=mujoco.mjtLimited.mjLIMITED_TRUE,
                          ctrlrange=[-limit["effort"], limit["effort"]],
                          forcerange=[-limit["effort"], limit["effort"]])
        spec.add_numeric(name=f"{name}_velocity_limit", data=[motor["no_load_speed_rad_s"]])

    model = spec.compile()
    data = mujoco.MjData(model)
    for leg in LEGS:
        for role in ROLES:
            j = model.joint(f"{leg}_{role}")
            data.qpos[j.qposadr[0]] = params[f"stance_{role}"]
    mujoco.mj_forward(model, data)
    bottom = min(float(data.geom_xpos[model.geom(f"{leg}_lower_leg_link_sphere").id, 2]
                       - model.geom(f"{leg}_lower_leg_link_sphere").size[0]) for leg in LEGS)
    data.qpos[2] += 0.002 - bottom
    spec.add_key(name="home", qpos=data.qpos.tolist(), ctrl=[0.0]*model.nu)
    model = spec.compile()
    mass = float(model.body_mass.sum())
    if abs(mass - mass_before - params["payload_mass_kg"]) > 1e-7:
        raise ValueError("Import changed the robot's total mass")
    # XML assets must be relocatable, including when generated/ is copied elsewhere.
    spec.meshdir = "meshes"
    for mesh in spec.meshes:
        mesh.file = Path(mesh.file).name
    robot = out / f"{profile['name']}.xml"
    robot.write_text(spec.to_xml())
    scene = ET.Element("mujoco", {"model": profile["name"] + "_scene"})
    ET.SubElement(scene, "include", {"file": robot.name})
    world = ET.SubElement(scene, "worldbody")
    ET.SubElement(world, "light", {"pos": "0 -2 3", "dir": "0 1 -2", "directional": "true"})
    ET.SubElement(world, "geom", {"name": "floor", "type": "plane", "size": "5 5 0.1",
                                  "friction": f"{params['ground_friction']} 0.005 0.0001",
                                  "rgba": "0.25 0.28 0.32 1"})
    ET.ElementTree(scene).write(out / "scene.xml", encoding="utf-8", xml_declaration=True)
    check = mujoco.MjModel.from_xml_path(str(out / "scene.xml"))
    if (check.nq, check.nv, check.nu) != (19, 18, 12):
        raise ValueError("Imported quadruped must have a floating base and 12 actuators")
    # MuJoCo serializes floating point fields to finite precision.
    serialized_mass = float(check.body_mass.sum())
    if abs(serialized_mass - mass) > max(1e-4, mass * 2e-5):
        raise ValueError("Serialized model mass differs from the compiled URDF")
    prepared.unlink()
    runtime = {"version": 1, "robot": profile["name"], "params": params, "motor": motor,
               "joints": limits, "joint_map": joint_map, "body_map": renames}
    _write_yaml(out / "simulation.yaml", runtime)
    _write_yaml(out / "parameters_from_gui.yaml", params)
    _write_yaml(out / "parameters_resolved.yaml", params)
    warnings = ["Motor torque-speed envelope is a linear approximation; no continuous-duty limit without thermal data.",
                "Reflected rotor inertia is an explicit simulation assumption, not a motor measurement."]
    if not params["thermal_enabled"]:
        warnings.append("Temperature prediction is disabled: thermal coefficients have not been supplied.")
    if not params["full_body_collisions"]:
        warnings.append("Only foot contacts are enabled; body and leg collisions are disabled.")
    report = {"source": profile["urdf"], "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
              "total_mass_kg": serialized_mass, "source_mass_kg": mass_before,
              "serialization_mass_error_kg": serialized_mass - mass,
              "mass_kg": {"robot": mass_before, "payload": params["payload_mass_kg"]},
              "joint_map": joint_map, "warnings": warnings}
    _write_yaml(out / "import_report.yaml", report)
    (out / "source_profile.json").write_text(json.dumps(profile, indent=2) + "\n")
