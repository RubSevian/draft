<p align="center">
  <img src="assets/draft-splay-bold-light.svg" alt="Draft" width="480">
</p>

<p align="center"><em>A parametric tool for robot design exploration</em></p>

<p align="center">
  <a href="https://arxiv.org/pdf/2609.38405">Paper (arXiv)</a> ·
  <a href="https://youtu.be/egzlEqfwwLU">Video</a>
</p>

[![Draft video](https://img.youtube.com/vi/egzlEqfwwLU/maxresdefault.jpg)](https://youtu.be/egzlEqfwwLU)

Draft compiles a short declarative robot description into a simulation-ready
MuJoCo model, without CAD. You state link lengths and the torque and speed each
joint needs. Trends fitted to 114 catalogued actuators and 49 published robot
descriptions fill in the masses, motor sizes and reflected inertias, then check
whether the assembled machine could be built. Design variants take seconds to
generate, and every one can be trained with the included quadruped RL tasks.

## Quick start

```shell
conda env create -f environment.yml && conda activate draft
export PYTHONNOUSERSITE=1          # keep ~/.local packages out of the env
pip install -e ".[editor,twins,figures,dev]" -c constraints.txt
python scripts/robot_tuner.py                       # browser editor
python scripts/generate_robot.py --robot quadruped  # headless, writes generated/
```

Training needs a CUDA GPU: add the `rl` extra (`pip install -e ".[rl]" -c
constraints.txt`) and see [`docs/tasks.md`](docs/tasks.md).
Everything else runs on a laptop CPU.

## MEVIUS2 simulation in this fork

The bundled MEVIUS2 imports the original link geometry, masses, inertias and
joint limits. The editor can change drive settings, pose, payload and contacts,
and run a CPU simulation with 12 PD-controlled motors.

```shell
python -m venv .venv
source .venv/bin/activate
pip install -e ".[editor,dev]" -c constraints.txt
python scripts/robot_tuner.py --robot mevius2
```

Open the editor URL printed in the terminal, then click **Simulate with motors**.
For repeatable runs without a browser:

```shell
python scripts/generate_robot.py --robot mevius2 --output-dir generated/mevius2 --fixed-output-dir
python scripts/simulate_robot.py generated/mevius2/scene.xml --seconds 60 --csv logs/mevius2.csv
```

Temperature prediction is implemented but **disabled by default**: the motor's
thermal coefficients and mounting conditions are not known. See
[`docs/imported-robots.md`](docs/imported-robots.md) for configuration, assumptions
and the required thermal data. Pose control is included; a MEVIUS2 walking
policy is not included.

## Layout

```
src/draft/
  generation/   the compiler: parameter table + kinematic tree -> MJCF
  trends/       fitted trends, the feasibility checks, and their coefficients
  robots/       the shipped specifications (humanoid, quadruped)
  twins/        four Unitree robots rebuilt from specs and scored
  tasks/        quadruped locomotion environments and curricula
  lineup/       the paper's training and evaluation sweep
  editor/       the browser editor
  imported/     URDF import, controlled CPU dynamics and optional thermal model
urdf/           attributed source URDFs and mesh assets (MEVIUS2)
configs/robots/  imported robot mappings and simulation presets
configs/motors/  actuator specifications and optional thermal coefficients
datasets/       actuator catalogue and robot-description survey, with fitting scripts
experiments/    design variants, sweep spec, frozen results
scripts/        command-line entry points and figure scripts
tests/          pytest; --runslow adds the twin rebuilds
```

MEVIUS2 is bundled with its upstream license and source manifest. The paper's
other vendor models are fetched by `python scripts/setup_data.py`.
The full documentation is in [`docs/`](docs/README.md).

## Citing

If you use Draft or its datasets, please cite the paper:

```bibtex
@article{nguyen2026draft,
  title         = {Draft: A Parametric Tool for Robot Design Exploration},
  author        = {Nguyen, David and Coelho, Marcelo and Kim, Sangbae},
  journal       = {arXiv preprint arXiv:2609.38405},
  year          = {2026},
  eprint        = {2609.38405},
  archivePrefix = {arXiv},
  primaryClass  = {cs.RO},
  url           = {https://arxiv.org/abs/2609.38405}
}
```

## License

MIT, see [LICENSE](LICENSE). Fetched vendor descriptions keep their own
licences.
