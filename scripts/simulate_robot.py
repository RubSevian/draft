#!/usr/bin/env python3
"""CPU simulation of a built imported robot; writes per-motor CSV traces."""
import argparse
import json
from pathlib import Path

from draft.imported.runtime import run_headless


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scene", type=Path)
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--csv", type=Path, default=Path("logs/imported/motors.csv"))
    args = parser.parse_args()
    print(json.dumps(run_headless(args.scene, args.seconds, args.csv), indent=2))


if __name__ == "__main__":
    main()
