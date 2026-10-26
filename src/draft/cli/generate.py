#!/usr/bin/env python3
"""``scripts/generate_robot.py`` — compile one robot specification into an MJCF.

Resolves the robot's ``parameters.yaml`` + ``tree.yaml``, fills unstated values
from the fitted trends, runs the feasibility gate, and writes the model.

    python scripts/generate_robot.py --robot quadruped
    python scripts/generate_robot.py --robot humanoid --output-dir generated/tall
    python scripts/generate_robot.py --robot quadruped --overrides experiments/quadruped_variants/bear.yaml

The output directory also gets ``feasibility_report.yaml`` (derivations and
warnings) and ``parameters_resolved.yaml`` (the complete design as generated).
"""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

import yaml

from draft.generation.generator import RobotGenerator
from draft.paths import generated_dir
from draft.robots import available, robot_dir
from draft.imported.profile import profiles


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot", required=True,
                    help=f"which specification to compile ({', '.join(available() + list(profiles()))})")
    ap.add_argument("--output-dir", "--output_dir", dest="output_dir", default=None,
                    help="where to write (default: generated/<robot>_<timestamp>)")
    ap.add_argument("--fixed-output-dir", action="store_true",
                    help="write straight into --output-dir instead of a timestamped subfolder")
    ap.add_argument("--overrides", "--params-override", dest="overrides",
                    default=None, metavar="YAML",
                    help="a YAML of parameter overrides merged over the robot's own table, "
                         "which is what a named design variant is")
    args = ap.parse_args()

    imported = profiles().get(args.robot)
    try:
        if imported:
            from draft.imported.builder import build_imported
            base = Path(args.output_dir).expanduser().resolve() if args.output_dir else generated_dir()
            out = base if args.output_dir and args.fixed_output_dir else RobotGenerator.make_output_dir(base, args.robot)
            overrides = yaml.safe_load(Path(args.overrides).read_text()) if args.overrides else None
            model = build_imported(imported, out, overrides)
            print(f"Built {model}\nScene: {out / 'scene.xml'}")
            return 0
        src = robot_dir(args.robot)
    except (FileNotFoundError, ValueError, TypeError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    if not ((src / "tree.yaml").exists() or (src / "tree.py").exists()):
        print(f"Error: {src} has a parameter table but no tree.", file=sys.stderr)
        return 1

    if args.output_dir:
        base = Path(args.output_dir).expanduser().resolve()
        out = base if args.fixed_output_dir else RobotGenerator.make_output_dir(base, args.robot)
    else:
        out = RobotGenerator.make_output_dir(generated_dir(), args.robot)

    overrides, override_src = None, None
    if args.overrides:
        override_src = Path(args.overrides).expanduser().resolve()
        overrides = yaml.safe_load(override_src.read_text()) or {}

    RobotGenerator(src).generate(out, params_override=overrides or None)
    # Keep the override file with the output for provenance.
    if override_src and override_src.parent.resolve() != out.resolve():
        shutil.copy2(override_src, out / override_src.name)
    return 0

