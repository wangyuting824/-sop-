#!/usr/bin/env python3
"""Prepare real-URDF FK, then use the existing offline hand-eye solver."""
from __future__ import annotations
import argparse
import json
import math
from datetime import datetime, timezone
from pathlib import Path
import cv2
import numpy as np
from camera_common import UrdfKinematics, sha256, write_json_atomic
from solve_extrinsic import matrix, inv, selected, solve_eye_in_hand, solve_eye_to_hand, quaternion_from_rotation

ROOT = Path(__file__).resolve().parents[1]


def excitation(transforms):
    rotations = np.asarray([cv2.Rodrigues(transforms[0][:3, :3].T @ t[:3, :3])[0].reshape(3) for t in transforms[1:]])
    singular = np.linalg.svd(rotations, compute_uv=False)
    if len(singular) < 2 or singular[1] < .01 or singular[1] / max(singular[0], 1e-12) < .05:
        raise ValueError("Robot poses have insufficient independent rotation axes; repeated poses/single-axis sweeps cannot solve hand-eye calibration")
    return {"rotation_vector_singular_values": singular.tolist(), "second_first_ratio": float(singular[1]/singular[0]),
            "note": "Basic degeneracy check only; not a calibration accuracy guarantee"}


def prepare(document, urdf, chain_config):
    base, tool = chain_config.get("reference_frame"), chain_config.get("tool_frame")
    if not base or not tool or base == tool:
        raise ValueError("Fill the real reference_frame and tool_frame in config/robot_frames.json before solving")
    mapping = chain_config.get("joint_map", {})
    good = [s for s in document["samples"] if s.get("use_for_calibration")]
    frames = {s["image"]["header"]["frame_id"] for s in good}
    if len(frames) != 1:
        raise ValueError("Expected exactly one camera optical frame in accepted samples")
    for sample in good:
        source = sample["robot_feedback"]["q_rad"]
        values = {mapping.get(name, name): float(value) for name, value in source.items()}
        if len(values) != len(source):
            raise ValueError("Duplicate destinations in joint_map")
        fk = UrdfKinematics(urdf, list(values), base, tool)
        # This site input contains revolute arm feedback in degrees, converted to radians.
        # Do not apply that conversion to a missing waist/prismatic chain.
        if any(j.attrib.get("type") == "prismatic" for j in fk.chain):
            raise ValueError("Arm-only capture cannot supply a prismatic/waist joint in the requested FK chain")
        sample["T_base_tool"] = {"from_frame": tool, "to_frame": base,
                                 "matrix": fk.fk(np.asarray(list(values.values()))).tolist()}
    fit, hold = selected(good, "fit"), selected(good, "holdout")
    if len(fit) < 10 or len(hold) < 5:
        raise ValueError(f"Need at least 10 fit and 5 independent holdout poses; got {len(fit)} + {len(hold)}")
    quality = excitation([matrix(s["T_base_tool"]["matrix"], "robot pose") for s in fit])
    return fit, hold, frames.pop(), quality


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path, help="Capture directory or samples.json")
    parser.add_argument("--config", type=Path, default=ROOT / "config/robot_frames.json")
    parser.add_argument("--method", choices=("park", "tsai", "horaud", "andreff", "daniilidis"), default="park")
    args = parser.parse_args()
    samples_path = args.session / "samples.json" if args.session.is_dir() else args.session
    document = json.loads(samples_path.read_text(encoding="utf-8"))
    config = json.loads(args.config.read_text(encoding="utf-8"))
    if not config.get("urdf_path"):
        raise ValueError("Raw capture is saved. To solve, fill urdf_path and real frames in config/robot_frames.json; joint angles alone cannot define the robot geometry")
    urdf = Path(config["urdf_path"]).expanduser()
    if not urdf.is_absolute():
        urdf = args.config.resolve().parent / urdf
    if not urdf.is_file():
        raise ValueError(f"URDF not found: {urdf}")
    chain = config["arms"][document["arm"]]
    fit, hold, optical, quality = prepare(document, urdf, chain)
    if document["mode"] == "eye_in_hand":
        result = solve_eye_in_hand(fit, hold, args.method)
        transform_key, parent = "T_tool_camera", chain["tool_frame"]
    elif document["mode"] == "eye_to_hand":
        result = solve_eye_to_hand(fit, hold, args.method)
        transform_key, parent = "T_base_camera", chain["reference_frame"]
    else:
        raise ValueError("Unsupported capture mode")
    T = matrix(result[transform_key], transform_key)
    result.update({"status": "candidate_holdout_scored_not_released", "camera_slot": document["camera_slot"],
                   "from_frame": optical, "to_frame": parent, "transform_convention": "T_A_B maps coordinates from B into A; translation in metres",
                   "translation_m": T[:3, 3].tolist(), "quaternion_xyzw": quaternion_from_rotation(T[:3, :3]).tolist(),
                   "robot_reference_frame": chain["reference_frame"], "robot_tool_frame": chain["tool_frame"],
                   "robot_urdf_sha256": sha256(urdf), "input_sha256": sha256(samples_path),
                   "fit_indices": [s["index"] for s in fit], "holdout_indices": [s["index"] for s in hold],
                   "excitation": quality, "commands_sent": False, "writes_tf": False,
                   "created_utc": datetime.now(timezone.utc).isoformat(),
                   "limitations": "Conditional on the supplied URDF, joint mapping, board dimensions and stationary mounts. Holdout consistency is not absolute accuracy. Optical frame is not necessarily camera_link."})
    output = samples_path.parent / "analysis" / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    output.mkdir(parents=True, exist_ok=False)
    write_json_atomic(output / "candidate.json", result)
    write_json_atomic(output / "samples_with_fk.json", document)
    write_json_atomic(output / "robot_frames_used.json", config)
    (output / "robot_used.urdf").write_bytes(urdf.read_bytes())
    report = (f"Camera: {document['camera_slot']}\nCandidate transform: {optical} -> {parent}\n"
              f"Fit poses: {len(fit)}; holdout poses: {len(hold)}\nTranslation metres: {result['translation_m']}\n"
              f"Quaternion xyzw: {result['quaternion_xyzw']}\n4x4 matrix:\n{np.array2string(T, precision=9)}\n"
              f"Holdout consistency:\n{json.dumps(result['holdout_consistency'], indent=2)}\n"
              "这是依赖所填机器人模型的候选外参；留出集一致性不是绝对精度。没有自动写入 TF/URDF/USD。\n")
    (output / "report.txt").write_text(report, encoding="utf-8")
    print(report)
    print("Result directory:", output)


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        raise SystemExit(f"无法求解：{exc}")
