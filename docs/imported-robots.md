# Imported robots and MEVIUS2

The fork adds a second build path alongside Draft's parametric generator.
Imported models preserve their URDF masses, inertias, geometry and limits;
fitted actuator trends do not replace these properties. The original generator
and paper baselines remain separate.

## Files and workflow

- `urdf/mevius2/`: original STL URDF, 13 meshes, upstream license and checksums.
- `configs/robots/mevius2.yaml`: explicit link/joint mapping and editable settings.
- `configs/motors/robstride03.yaml`: electrical/mechanical motor data and sources.
- `generated/<name>/`: portable MJCF, floor scene, mesh copies, resolved settings,
  simulation configuration and import report. This output is ignored by Git.

Install the package as documented in the root README (MuJoCo 3.8 or newer).
Run `python scripts/robot_tuner.py --robot mevius2`; the first build is automatic.
The robot selector also retains Draft's original robots. Edits rebuild after a
short pause; **Save as** creates an independent named variant. Selecting a saved
folder restores its parameters. Invalid builds retain the last successful model.

While paused, joint sliders set a pose. **Simulate with motors** holds that pose
using PD control. **Reset pose** and stopping reset the initial pose, time and
thermal state. The table shows applied joint torque, speed and, when enabled,
winding/housing temperatures. It refreshes twice per second.

For batch runs:

```shell
python scripts/generate_robot.py --robot mevius2 --output-dir generated/mevius2 --fixed-output-dir
python scripts/simulate_robot.py generated/mevius2/scene.xml --seconds 60 --csv logs/mevius2.csv
```

An override YAML containing a subset of the flat `parameters` keys can be passed
to generation with `--overrides path/to/settings.yaml`. A run exports one row
per joint approximately every 20 ms of **simulation time**, independent of wall
time. Disabled temperatures are empty CSV cells, not a constant room temperature.

## What is preserved and what is configured

The source robot mass is **22.906507 kg**, excluding optional payload. The model
has 12 revolute joints and four foot collision spheres with radius 0.03 m.
Rear legs `BL/BR` map explicitly to `rl/rr`; collar/hip/knee joints map to
`hip_roll/hip_pitch/knee`. The importer adds a free root (19 position coordinates,
18 velocity coordinates) and verifies the mass before and after XML serialization.
The report records the small rounding error in MuJoCo's serialized XML.

The default pose is hip roll 0, hip pitch 0.7, knee −1.2 radians, from the source
controller. The CPU standing preset uses **Kp=100, Kd=5**; these are simulation
tuning values, distinct from the source controller's 50/2. A ten-second test
checks upright posture and torque limits. This is pose control, not active
balance, walking, or a validated controller for physical hardware.

Drive, payload, pose and contact settings can be changed in the editor. Geometry
and link inertias remain fixed: changing a leg length requires a revised URDF
with corresponding inertial data, or a separate Draft parametric model.
Payload is a box with configured mass, dimensions and centre relative to the
torso; its inertia is derived from those dimensions.

Full body contacts are on by default, using the URDF's collision approximations.
Turning them off leaves foot contacts only. The friction value applies to the
floor and collidable robot geometry. Reflected rotor inertia defaults to
0.02 kg·m² from the vendor simulation; this is not a measured motor constant.

## Motor model

The controller computes `Kp*(target-position) - Kd*velocity`, then applies a
linear torque–speed envelope and the source URDF's effort/speed limits. At the
speed limit, accelerating torque is blocked while braking remains possible.
The URDF caps are 60 N·m and 18.85 rad/s. The RobStride03 profile's no-load speed
is 195 rpm (20.42035 rad/s). The profile also records the 20 N·m rated torque.

The 60 N·m value is a **peak** limit. Without calibrated thermal data the model
does not enforce a continuous-duty duration. The envelope approximates motor
behaviour; it does not simulate inverter switching, current-loop dynamics,
battery voltage sag, gearbox elasticity or communication delay. Loading the
generated XML alone supplies bounded torque actuators; the editor and
`simulate_robot.py` additionally run the PD, speed and thermal logic.

## Temperature over time

`thermal: null` in the shipped motor profile is intentional. Set it to a mapping
with all the following values from measurements or reliable motor data:

| Key | Meaning and unit |
| --- | --- |
| `winding_heat_capacity_j_k` | Winding heat capacity, J/K |
| `housing_heat_capacity_j_k` | Housing heat capacity, J/K |
| `winding_to_housing_k_w` | Winding-to-housing thermal resistance, K/W |
| `housing_to_air_k_w` | Housing-to-environment thermal resistance for the actual mount, K/W |
| `derate_start_c` | Winding temperature at which torque reduction starts, °C |
| `shutdown_c` | Winding temperature at which drive torque reaches zero, °C |
| `calibrated` | Optional boolean; true only after validating against measurements |

Then enable `thermal_enabled`, specify initial and ambient temperature, and
rebuild. Missing/invalid coefficients reject the build with an explanatory error.
Changing the motor profile requires rebuilding; saved output retains a snapshot
of the motor data used for that build.

The two-node model integrates:

```
I_phase_rms = applied_joint_torque / Kt_output
P_copper = 1.5 * R_line_to_line(T_winding) * I_phase_rms²
C_w * dT_w/dt = P_copper - (T_w - T_h)/R_wh
C_h * dT_h/dt = (T_w - T_h)/R_wh - (T_h - T_ambient)/R_ha
```

The motor profile uses output-side `Kt=2.36 N·m/Arms` and line-to-line resistance
0.39 Ω at the configured reference temperature. Gear ratio must not be applied
again to that torque constant. Copper resistance rises with temperature.
Torque derating acts before the physics step; heating uses the applied actuator
torque. The numerical model is tested with synthetic thermal parameters, not
claimed RobStride measurements. Iron, gearbox and inverter losses are omitted;
accurate predictions require calibration for the actual mounting and airflow.

## Extending the importer

The current backend supports a base-rooted quadruped with exactly four mapped
legs, three limited revolute joints per leg and one collision sphere per foot.
It is not a universal URDF importer. Another quadruped can supply its own profile
without robot-name branches in Python; Hero2/humanoid support needs a separate
topology adapter. New bundled assets must have a compatible license, pinned
provenance, checksums and an explicit allowance in the structure test.
