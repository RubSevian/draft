"""Estimate effective module temperatures from a MotorLogger NPZ trace."""
import argparse
from pathlib import Path
import numpy as np
import yaml
from draft.tasks.thermal import estimate_temperature


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--trace', required=True)
    p.add_argument('--config', default='experiments/mevius2_thermal.yaml')
    p.add_argument('--output-dir', default='logs/mevius2_thermal')
    a = p.parse_args()
    cfg = yaml.safe_load(Path(a.config).read_text())
    if cfg['loss_model'] != 'paper_copper_proxy':
        raise ValueError('Unsupported loss model')
    out = Path(a.output_dir); out.mkdir(parents=True, exist_ok=True)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    with np.load(a.trace, allow_pickle=False) as trace:
        t, tau = trace['time_s'], trace['torque_Nm']
        if 'episode_reset' in trace:
            print(f"Recorded episode resets: {int(trace['episode_reset'].sum())}; inspect gait stability.")
        fig, ax = plt.subplots()
        for name, rc in cfg['scenarios'].items():
            temp, loss = estimate_temperature(t, tau, trace['peak_torque_Nm'],
                trace['no_load_speed_rad_s'], ambient_C=cfg['ambient_C'],
                initial_C=cfg['initial_C'], **rc)
            np.savetxt(out / f'{name}.csv', np.column_stack((t, temp)), delimiter=',',
                header=','.join(['time_s', *trace['joint_names'].tolist()]), comments='')
            ax.plot(t, temp.max(axis=1), label=name)
            print(f'{name}: highest effective temperature {temp.max():.2f} C')
        ax.set(xlabel='Simulation time (s)', ylabel='Effective module temperature (°C)',
               title='Uncalibrated RC scenarios — hottest joint at each instant')
        ax.legend(); fig.tight_layout(); fig.savefig(out / 'temperature.png'); plt.close(fig)
    (out / 'assumptions.yaml').write_text(yaml.safe_dump(cfg))
    print('Not winding/sensor temperature; not a validated operating limit. No trace repetition.')


if __name__ == '__main__':
    main()
