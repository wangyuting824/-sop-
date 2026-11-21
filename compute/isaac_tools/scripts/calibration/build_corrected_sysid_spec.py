#!/usr/bin/env python3
"""Build a corrected active-axis NVIDIA SysID test spec without editing source artifacts."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument("--joint", type=int, choices=range(1, 8), required=True)
    parser.add_argument("--max-iterations", type=int, default=6)
    parser.add_argument("--max-train-chunks", type=int, default=2)
    parser.add_argument("--position-weight", type=float, default=1.0)
    parser.add_argument("--velocity-weight", type=float, default=0.5)
    args = parser.parse_args()

    if args.destination.exists():
        raise FileExistsError(f"Refusing to overwrite {args.destination}")
    spec = json.loads(args.source.read_text(encoding="utf-8"))
    spec["solver"].update(
        {
            "optimizer": "levenberg_marquardt",
            "epsilon": 0.05,
            "damping_initial": 0.01,
            "max_iterations": args.max_iterations,
            "use_analytical_jacobian": False,
            "use_analytical_presolve": False,
        }
    )
    # NVIDIA's drive-calibration preset uses lower velocity weighting than the
    # original ad-hoc spec.  This still retains the measured dynamic response.
    spec["residuals"]["position_weight"] = args.position_weight
    spec["residuals"]["velocity_weight"] = args.velocity_weight
    spec["parameters"]["space"]["include_command_delay"] = False
    selected = [
        row
        for row in spec["parameters"]["selected"]
        if int(row.get("dof_index", -1)) == args.joint - 1
    ]
    spec["parameters"]["selected"] = selected
    excitation = f"J{args.joint}_step"
    matching_chunks = [
        row for row in spec["telemetry"]["chunks"] if row.get("excitation") == excitation
    ]
    train_chunks = [row for row in matching_chunks if row.get("role") == "train"]
    validation_chunks = [row for row in matching_chunks if row.get("role") == "validation"]
    if not selected:
        raise ValueError(f"No parameters found for {args.side} J{args.joint}")
    if len(train_chunks) < 2 or not validation_chunks:
        raise ValueError(
            f"Need at least two train chunks and one validation chunk for {args.side} J{args.joint}; "
            f"found {len(train_chunks)} train and {len(validation_chunks)} validation"
        )
    if len(train_chunks) > args.max_train_chunks:
        train_chunks = [train_chunks[0], train_chunks[-1]]
    spec["telemetry"]["chunks"] = train_chunks + validation_chunks
    spec["telemetry"]["command_alignment_seconds"] = 0.0
    tag = f"{args.side}_J{args.joint}_active_iter{args.max_iterations}"
    base = args.destination.parent
    spec["outputs"].update(
        {
            "apply_parameters_to_usd": False,
            "write_usd_provenance": False,
            "export_provenance_sidecar": True,
            "provenance_path": str(base / f"{tag}_official_provenance.json"),
            "compute_parameter_confidence": True,
            "parameter_confidence_rollout_budget": 500000,
        }
    )
    spec["zhifeng_corrected_objective"] = {
        "side": args.side,
        "joint": args.joint,
        "residual_scope": "active joint only",
        "finite_difference_epsilon": 0.05,
        "position_weight": args.position_weight,
        "velocity_weight": args.velocity_weight,
        "command_alignment_seconds": 0.0,
        "train_chunks": [row["name"] for row in train_chunks],
        "validation_chunks": [row["name"] for row in validation_chunks],
    }
    args.destination.write_text(
        json.dumps(spec, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(args.destination)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
