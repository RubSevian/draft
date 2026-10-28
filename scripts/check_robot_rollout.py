"""Short GPU integration check with zero policy actions (not learned walking)."""
import argparse
import os
os.environ.setdefault('MUJOCO_GL', 'egl')
import draft
import torch
import numpy as np
from draft.tasks import get_task
from draft.tasks.config_loader import load_run_config
from draft.tasks.motor_logger import MotorLogger


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', default='experiments/mevius2.yaml')
    p.add_argument('--steps', type=int, default=500)
    p.add_argument('--output', default='logs/mevius2_check/motor_trace.npz')
    a = p.parse_args()
    run = load_run_config(a.config)
    task = get_task(run['task'])
    cfg = task.make_env_cfg(n_envs=1, play=True, **run['env_kwargs'])
    env = task.env_class()(cfg, device='cuda:0')
    try:
        env.reset()
        logger = MotorLogger(env)
        actions = torch.zeros((1, env.action_manager.total_action_dim), device='cuda:0')
        resets = 0
        for _ in range(a.steps):
            env.step(actions)
            logger.record()
            resets += int(env.reset_buf[0].item())
        if not np.all(np.isfinite(logger._tau)) or not np.all(np.isfinite(logger._vel)):
            raise RuntimeError("Non-finite motor state during rollout")
        logger.save_trace(a.output)
        print(f'PASS: {a.steps} physics/control steps; {len(logger.joint_names)} joints; {resets} episode resets. Zero actions, NOT trained gait.')
    finally:
        env.close()


if __name__ == '__main__':
    main()
