#!/usr/bin/env python3
"""Report per-axis position/velocity scales from held-out baseline rollouts."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def rmse(chunks: list[np.ndarray]) -> float:
    values = np.concatenate([np.ravel(chunk) for chunk in chunks])
    return float(np.sqrt(np.mean(values**2)))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("result", type=Path)
    parser.add_argument("rollouts", type=Path)
    args = parser.parse_args()
    result = json.loads(args.result.read_text(encoding="utf-8"))
    data = np.load(args.rollouts)
    report: dict[str, object] = {}
    for joint in range(1, 8):
        rows = [
            row
            for row in result["rollout_index"]
            if row["role"] == "validation" and row["excitation"] == f"J{joint}_step"
        ]
        if not rows:
            continue
        q_errors = [
            data[f'{row["prefix"]}_simulated_q_rad'][joint - 1]
            - data[f'{row["prefix"]}_measured_q_rad'][joint - 1]
            for row in rows
        ]
        dq_errors = [
            data[f'{row["prefix"]}_simulated_dq_rad_s'][joint - 1]
            - data[f'{row["prefix"]}_measured_dq_rad_s'][joint - 1]
            for row in rows
        ]
        q_scale = rmse(q_errors)
        dq_scale = rmse(dq_errors)
        report[f"J{joint}"] = {
            "position_rmse": q_scale,
            "velocity_rmse": dq_scale,
            "position_weight": 1.0 / q_scale if q_scale > 0 else 1.0,
            "velocity_weight": 1.0 / dq_scale if dq_scale > 0 else 1.0,
            "validation_chunks": [row["name"] for row in rows],
        }
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
