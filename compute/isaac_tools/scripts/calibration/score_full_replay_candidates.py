#!/usr/bin/env python3
"""Score fixed-vector candidates against every held-out active-axis replay."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def rmse(value: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.ravel(value) ** 2)))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("base", type=Path)
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument("--glob", required=True)
    args = parser.parse_args()
    baseline_result = json.loads((args.base / f"{args.side}_baseline_full_result.json").read_text())
    baseline_data = np.load(args.base / f"{args.side}_baseline_full_rollouts.npz")
    baseline_index = {
        row["name"]: row for row in baseline_result["rollout_index"] if row.get("role") == "validation"
    }
    reports = []
    for result_path in sorted(args.base.glob(args.glob)):
        result = json.loads(result_path.read_text())
        data = np.load(args.base / Path(result["validation_rollout_npz"]).name)
        per_axis = []
        ratios = []
        for joint in range(1, 8):
            rows = [row for row in result["validation_rollout_index"] if row["excitation"] == f"J{joint}_step"]
            if not rows:
                continue
            bq, bv, fq, fv = [], [], [], []
            for row in rows:
                bprefix = baseline_index[row["name"]]["prefix"]
                fprefix = row["prefix"]
                bq.append(baseline_data[f"{bprefix}_simulated_q_rad"][joint - 1] - baseline_data[f"{bprefix}_measured_q_rad"][joint - 1])
                bv.append(baseline_data[f"{bprefix}_simulated_dq_rad_s"][joint - 1] - baseline_data[f"{bprefix}_measured_dq_rad_s"][joint - 1])
                fq.append(data[f"{fprefix}_simulated_q_rad"][joint - 1] - data[f"{fprefix}_measured_q_rad"][joint - 1])
                fv.append(data[f"{fprefix}_simulated_dq_rad_s"][joint - 1] - data[f"{fprefix}_measured_dq_rad_s"][joint - 1])
            bqr, bvr = rmse(np.concatenate(bq)), rmse(np.concatenate(bv))
            fqr, fvr = rmse(np.concatenate(fq)), rmse(np.concatenate(fv))
            qratio, vratio = fqr / bqr, fvr / bvr
            ratios.extend((qratio, vratio))
            per_axis.append(
                {
                    "joint": joint,
                    "position_improvement_percent": (1 - qratio) * 100,
                    "velocity_improvement_percent": (1 - vratio) * 100,
                }
            )
        reports.append(
            {
                "tag": result_path.name.removesuffix("_fixed_validation_result.json"),
                "theta_scales": result["theta_scales"],
                "normalized_score": float(np.sqrt(np.mean(np.square(ratios)))),
                "mean_improvement_percent": float((1 - np.mean(ratios)) * 100),
                "per_axis": per_axis,
            }
        )
    print(json.dumps(reports, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
