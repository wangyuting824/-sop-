#!/usr/bin/env python3
"""Offline OpenCV solve and holdout scoring for this package's capture files.

This script never reads ROS, starts a camera, writes TF, or controls a robot.
It produces a *candidate*, not a released transform.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np


METHODS = {name: getattr(cv2, f"CALIB_HAND_EYE_{name.upper()}") for name in ("tsai", "park", "horaud", "andreff", "daniilidis")}


def matrix(value, label: str) -> np.ndarray:
    T = np.asarray(value, dtype=float)
    if T.shape != (4, 4) or not np.isfinite(T).all() or not np.allclose(T[3], [0, 0, 0, 1], atol=1e-9):
        raise ValueError(f"{label} is not a finite 4x4 transform")
    if not np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-5) or not np.isclose(np.linalg.det(T[:3, :3]), 1.0, atol=1e-5):
        raise ValueError(f"{label} rotation is invalid")
    return T


def inv(T: np.ndarray) -> np.ndarray:
    result = np.eye(4); result[:3, :3] = T[:3, :3].T; result[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return result


def rotation_angle_deg(A: np.ndarray, B: np.ndarray) -> float:
    cosine = np.clip((np.trace(A[:3, :3].T @ B[:3, :3]) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(cosine)))


def quaternion_from_rotation(R: np.ndarray) -> np.ndarray:
    # xyzw, deterministic sign; suitable for a rotation mean.
    trace = np.trace(R)
    if trace > 0:
        s = 2.0 * np.sqrt(trace + 1.0); q = np.asarray([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    else:
        index = int(np.argmax(np.diag(R)))
        if index == 0:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]); q = np.asarray([0.25*s, (R[0, 1]+R[1, 0])/s, (R[0, 2]+R[2, 0])/s, (R[2, 1]-R[1, 2])/s])
        elif index == 1:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]); q = np.asarray([(R[0, 1]+R[1, 0])/s, 0.25*s, (R[1, 2]+R[2, 1])/s, (R[0, 2]-R[2, 0])/s])
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]); q = np.asarray([(R[0, 2]+R[2, 0])/s, (R[1, 2]+R[2, 1])/s, 0.25*s, (R[1, 0]-R[0, 1])/s])
    return q / np.linalg.norm(q)


def rotation_from_quaternion(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q / np.linalg.norm(q)
    return np.asarray([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def mean_transform(values: list[np.ndarray]) -> np.ndarray:
    if not values:
        raise ValueError("cannot average empty transform list")
    reference = quaternion_from_rotation(values[0][:3, :3])
    quaternions = []
    for item in values:
        q = quaternion_from_rotation(item[:3, :3])
        quaternions.append(q if np.dot(q, reference) >= 0 else -q)
    _, vectors = np.linalg.eigh(sum(np.outer(q, q) for q in quaternions))
    qmean = vectors[:, -1]
    if np.dot(qmean, reference) < 0:
        qmean = -qmean
    answer = np.eye(4)
    answer[:3, :3] = rotation_from_quaternion(qmean)
    answer[:3, 3] = np.mean([item[:3, 3] for item in values], axis=0)
    return answer


def metrics(values: list[np.ndarray], reference: np.ndarray) -> dict:
    if not values:
        return {"count": 0}
    translations = np.asarray([np.linalg.norm(item[:3, 3] - reference[:3, 3]) * 1000.0 for item in values])
    rotations = np.asarray([rotation_angle_deg(item, reference) for item in values])
    summary = lambda array: {"mean": float(np.mean(array)), "rmse": float(np.sqrt(np.mean(array * array))), "p95": float(np.percentile(array, 95)), "max": float(np.max(array))}
    return {"count": len(values), "translation_error_mm": summary(translations), "rotation_error_deg": summary(rotations)}


def selected(samples: list[dict], split: str) -> list[dict]:
    return [sample for sample in samples if sample.get("split") == split and sample.get("use_for_calibration", True)]


def solve_eye_in_hand(fit: list[dict], holdout: list[dict], method: str) -> dict:
    if len(fit) < 6:
        raise ValueError("eye-in-hand needs at least six accepted fit samples")
    G = [matrix(item["T_base_tool"]["matrix"], f"fit {item['index']} T_base_tool") for item in fit]
    C = [matrix(item["T_camera_board"]["matrix"], f"fit {item['index']} T_camera_board") for item in fit]
    R, t = cv2.calibrateHandEye([value[:3, :3] for value in G], [value[:3, 3] for value in G],
                                [value[:3, :3] for value in C], [value[:3, 3] for value in C], method=METHODS[method])
    X = np.eye(4); X[:3, :3], X[:3, 3] = R, np.asarray(t).reshape(3)  # T_tool_camera
    fixed_board = [g @ X @ c for g, c in zip(G, C)]
    board_mean = mean_transform(fixed_board)
    hold = [matrix(item["T_base_tool"]["matrix"], "holdout T_base_tool") @ X @ matrix(item["T_camera_board"]["matrix"], "holdout T_camera_board") for item in holdout]
    return {
        "kind": "eye_in_hand_candidate", "selected_method": method,
        "equation": "T_base_tool @ T_tool_camera @ T_camera_board = T_base_board",
        "T_tool_camera": X.tolist(), "T_base_board_reference": board_mean.tolist(),
        "fit_consistency": metrics(fixed_board, board_mean), "holdout_consistency": metrics(hold, board_mean),
    }


def solve_eye_to_hand(fit: list[dict], holdout: list[dict], method: str) -> dict:
    if len(fit) < 6:
        raise ValueError("eye-to-hand needs at least six accepted fit samples")
    G = [matrix(item["T_base_tool"]["matrix"], f"fit {item['index']} T_base_tool") for item in fit]
    C = [matrix(item["T_camera_tag"]["matrix"], f"fit {item['index']} T_camera_tag") for item in fit]
    A = [inv(value) for value in G]
    R, t = cv2.calibrateHandEye([value[:3, :3] for value in A], [value[:3, 3] for value in A],
                                [value[:3, :3] for value in C], [value[:3, 3] for value in C], method=METHODS[method])
    X = np.eye(4); X[:3, :3], X[:3, 3] = R, np.asarray(t).reshape(3)  # T_base_camera
    tool_tag = [a @ X @ c for a, c in zip(A, C)]
    tag_mean = mean_transform(tool_tag)
    hold = [inv(matrix(item["T_base_tool"]["matrix"], "holdout T_base_tool")) @ X @ matrix(item["T_camera_tag"]["matrix"], "holdout T_camera_tag") for item in holdout]
    return {
        "kind": "eye_to_hand_candidate", "selected_method": method,
        "equation": "T_base_tool @ T_tool_tag = T_base_camera @ T_camera_tag",
        "T_base_camera": X.tolist(), "T_tool_tag_reference": tag_mean.tolist(),
        "fit_consistency": metrics(tool_tag, tag_mean), "holdout_consistency": metrics(hold, tag_mean),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True, help="samples.json from this package")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--method", choices=tuple(METHODS), default="park", help="Preselect before reviewing results; do not choose after fitting.")
    args = parser.parse_args()
    document = json.loads(args.input.read_text(encoding="utf-8"))
    samples = document.get("samples", [])
    fit, holdout = selected(samples, "fit"), selected(samples, "holdout")
    if document.get("mode") == "eye_in_hand":
        result = solve_eye_in_hand(fit, holdout, args.method)
    elif document.get("mode") == "eye_to_hand":
        result = solve_eye_to_hand(fit, holdout, args.method)
    else:
        raise ValueError("input mode must be eye_in_hand or eye_to_hand")
    result.update({
        "schema_version": 1, "status": "candidate_holdout_scored_not_released",
        "input": str(args.input.resolve()),
        "input_sha256": hashlib.sha256(args.input.read_bytes()).hexdigest(),
        "fit_sample_indices": [sample["index"] for sample in fit], "holdout_sample_indices": [sample["index"] for sample in holdout],
        "transform_convention": "T_A_B maps coordinates from B into A",
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "hardware_commands_sent": False, "writes_tf": False,
        "limitations": "Fit/holdout consistency is not absolute pose accuracy. Verify frame direction, tag/board dimensions, installation, and task performance before release.",
    })
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({"status": result["status"], "output": str(args.output), "fit": len(fit), "holdout": len(holdout), "commands_sent": False}, ensure_ascii=False))


if __name__ == "__main__":
    main()
