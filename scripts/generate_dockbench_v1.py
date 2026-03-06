#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from docking.config import load_config
from docking.dockbench import dataset_root
from docking.dockbench_generator import build_dockbench_v1


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Generate DockBench-v1 under data/.")
    p.add_argument("--out-dir", type=str, default=str(dataset_root()), help="Dataset root directory.")
    p.add_argument("--base-seed", type=int, default=20260306, help="Master seed for deterministic dataset generation.")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    cfg = load_config()
    summary = build_dockbench_v1(cfg, root=args.out_dir, base_seed=int(args.base_seed))
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
