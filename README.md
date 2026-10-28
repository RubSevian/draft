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

## Локальный запуск через uv (MEVIUS2)

**[Пошаговая инструкция: модель → обучение → испытания → температура → сравнение](docs/mevius2-workflow.md).**

Рабочая среда — `.venv`, Python 3.11. В этой копии она уже создана.
Для новой копии сначала выполните `uv venv --python 3.11 .venv`.
Все команды ниже выполняются из корня `draft`; активация окружения не нужна.

```shell
cd "/home/ruben/Desktop/RAS/Robot dog/draft"
# Генератор, редактор и графики температуры (без CUDA):
uv pip install --python .venv/bin/python -e ".[editor,dev]" mjviser matplotlib -c constraints.txt
.venv/bin/python scripts/generate_robot.py --robot mevius2 --output-dir generated/mevius2 --fixed-output-dir
.venv/bin/python scripts/robot_tuner.py --port 8081 --viser-port 8080
```

Открыть **http://localhost:8081**, выбрать **mevius2**. Терминал оставить
открытым; остановка — `Ctrl+C`. Если редактор уже запущен службой, перед
ручным запуском остановите её: `systemctl --user stop draft-editor`.
Генерация по команде выше перезаписывает `generated/mevius2`; сохраните
изменённый вариант через редактор перед повторной генерацией.

Для управления моторами и обучения нужен Linux с NVIDIA/CUDA
(`nvidia-smi` должен видеть GPU). Дополнительные пакеты из `[rl]`:
PyTorch, mjlab, MuJoCo Warp, Warp, rsl-rl и W&B; версии заданы в
`pyproject.toml`. Загрузка занимает несколько ГБ. Наш конфиг пишет логи
локально в TensorBoard и не требует входа в W&B.

```shell
uv pip install --python .venv/bin/python -e ".[rl]" -c constraints.txt
# Проверка CUDA после установки:
.venv/bin/python -c "import torch; print(torch.__version__, torch.cuda.is_available())"
# Короткая проверка среды с нулевыми действиями — контроллер ходьбы ещё не обучен:
.venv/bin/python scripts/check_robot_rollout.py --config experiments/mevius2.yaml
# Обучение собственного контроллера MEVIUS2 (256 экземпляров, 1000 итераций):
.venv/bin/python scripts/train.py --config experiments/mevius2.yaml
# После обучения — просмотр последнего checkpoint:
.venv/bin/python scripts/play.py --config experiments/mevius2.yaml --viewer viser
```

Обучение сохраняется в `logs/mevius2_flat/`. Адрес просмотра печатается
в терминале. При завершении просмотра сохраняется `motor_trace.npz` рядом
с checkpoint. В этой рабочей копии уже завершён первый запуск из 1000 итераций;
путь к весам указан в пошаговой инструкции.

Предварительный нагрев для **48 В, воздуха 25 °C, крепления к дюрали**:

```shell
.venv/bin/python scripts/estimate_motor_temperature.py --trace logs/mevius2_check/motor_trace.npz
```

Результаты: `logs/mevius2_thermal/temperature.png` и CSV по суставам.
Параметры: `experiments/mevius2_thermal.yaml`. Теплоёмкость и теплоотвод
пока заданы предположениями: это сценарии эффективной температуры модуля,
а не калиброванный прогноз датчика или обмотки. Проверочная запись относится
к нулевым действиям; для анализа ходьбы укажите запись из обученного контроллера.
Подробнее: [MEVIUS2](docs/mevius2.md).

## MEVIUS2 in this fork

MEVIUS2 is a native Draft specification under `src/draft/robots/mevius2/`.
It uses the same generator, editor, motor classes and feasibility checks as the
other robots. No separate import mode or motor-profile directory is needed.

```shell
python scripts/generate_robot.py --robot mevius2 --output-dir generated/mevius2 --fixed-output-dir
python scripts/robot_tuner.py
```

Select **mevius2** in the top row. Change **Link Lengths**, **Geometry**,
**Motor Classes** and **Actuator design**, then save variants as usual.
Joint locations/axes/ranges come from the original URDF; geometry uses Draft's
primitives and masses/inertias are recalculated by its shared rules.
The original URDF/STLs remain under `urdf/mevius2/` as a licensed reference.
See [`docs/mevius2.md`](docs/mevius2.md) for dimensions and motor configuration.

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
datasets/       actuator catalogue and robot-description survey, with fitting scripts
experiments/    design variants, sweep spec, frozen results
scripts/        command-line entry points and figure scripts
tests/          pytest; --runslow adds the twin rebuilds
```

The fork bundles the licensed MEVIUS2 reference URDF and meshes. The paper's
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
