#!/usr/bin/env python3
"""Expand a side RunSpec to all seven drive-gain pairs without changing telemetry."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("reference", type=Path)
    parser.add_argument("destination", type=Path)
    args = parser.parse_args()
    if args.destination.exists():
        raise FileExistsError(args.destination)
    spec = json.loads(args.source.read_text(encoding="utf-8"))
    reference = json.loads(args.reference.read_text(encoding="utf-8"))
    selected = list(spec["parameters"]["selected"])
    existing = {(int(row["dof_index"]), row["param_type"]) for row in selected}
    for row in reference["parameters"]["selected"]:
        key = (int(row["dof_index"]), row["param_type"])
        if key not in existing:
            selected.append(dict(row))
            existing.add(key)
    selected.sort(key=lambda row: (int(row["dof_index"]), row["param_type"] != "joint_stiffness"))
    if [(int(row["dof_index"]), row["param_type"]) for row in selected] != [
        (joint, kind) for joint in range(7) for kind in ("joint_stiffness", "joint_damping")
    ]:
        raise ValueError("Expected seven stiffness/damping parameter pairs")
    spec["parameters"]["selected"] = selected
    spec["outputs"]["apply_parameters_to_usd"] = False
    spec["outputs"]["write_usd_provenance"] = False
    spec["outputs"]["compute_parameter_confidence"] = False
    spec["zhifeng_final_full_vector_replay"] = True
    args.destination.write_text(json.dumps(spec, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(args.destination)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
