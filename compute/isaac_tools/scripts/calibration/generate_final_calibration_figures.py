#!/usr/bin/env python3
"""Create clean final plots and parameter tables from the fixed-vector Isaac Sim replays."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


BASELINE_KP = 625.0
BASELINE_KD = 0.2
FINAL_SCALES = {
    "right": [
        (1.0, 1.0),
        (2.0, 20.0),
        (2.0, 20.0),
        (1.0, 1.0),
        (1.0, 1.0),
        (1.0, 1.0),
        (1.0, 0.5),
    ],
    "left": [
        (1.0, 1.0),
        (2.0, 20.0),
        (2.0, 20.0),
        (1.0, 1.0),
        (1.0, 1.0),
        (1.0, 1.0),
        (1.0, 1.0),
    ],
}


def rmse(values: list[np.ndarray]) -> float:
    flat = np.concatenate([np.ravel(value) for value in values])
    return float(np.sqrt(np.mean(flat**2)))


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def index_by_name(result: dict, validation_only: bool = False) -> dict[str, dict]:
    rows = result.get("validation_rollout_index") or result.get("rollout_index") or []
    if validation_only:
        rows = [row for row in rows if row.get("role") == "validation"]
    return {row["name"]: row for row in rows}


def arrays(data: np.lib.npyio.NpzFile, prefix: str, joint: int) -> dict[str, np.ndarray]:
    return {
        "time": np.asarray(data[f"{prefix}_times_s"], dtype=float),
        "measured_q": np.asarray(data[f"{prefix}_measured_q_rad"], dtype=float)[joint],
        "simulated_q": np.asarray(data[f"{prefix}_simulated_q_rad"], dtype=float)[joint],
        "measured_dq": np.asarray(data[f"{prefix}_measured_dq_rad_s"], dtype=float)[joint],
        "simulated_dq": np.asarray(data[f"{prefix}_simulated_dq_rad_s"], dtype=float)[joint],
    }


def style_axis(ax: plt.Axes) -> None:
    ax.grid(True, alpha=0.22, linewidth=0.8)
    ax.spines[["top", "right"]].set_visible(False)
    ax.tick_params(labelsize=11)


def plot_axis(
    out: Path,
    side: str,
    joint: int,
    measured: np.ndarray,
    baseline: np.ndarray,
    calibrated: np.ndarray,
    time: np.ndarray,
    measured_dq: np.ndarray,
    baseline_dq: np.ndarray,
    calibrated_dq: np.ndarray,
) -> None:
    t = time - time[0]
    fig, axes = plt.subplots(2, 1, figsize=(11.5, 7.2), sharex=True)
    labels = ((measured, baseline, calibrated), (measured_dq, baseline_dq, calibrated_dq))
    for ax, curves in zip(axes, labels):
        ax.plot(t, curves[0], color="#111827", linewidth=2.2, label="Robot")
        ax.plot(t, curves[1], color="#9CA3AF", linewidth=2.0, linestyle="--", label="Baseline")
        ax.plot(t, curves[2], color="#2563EB", linewidth=2.2, label="Calibrated")
        style_axis(ax)
    axes[0].set_ylabel("Position (rad)", fontsize=12)
    axes[1].set_ylabel("Velocity (rad/s)", fontsize=12)
    axes[1].set_xlabel("Time (s)", fontsize=12)
    axes[0].legend(frameon=False, ncol=3, fontsize=11, loc="best")
    fig.suptitle(f"{side.title()} Arm J{joint}", fontsize=18, fontweight="bold")
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(out / f"{side}_J{joint}_response.png", dpi=180, bbox_inches="tight")
    plt.close(fig)


def plot_contact_sheet(out: Path, side: str, traces: dict[int, dict[str, np.ndarray]]) -> None:
    fig, axes = plt.subplots(4, 2, figsize=(14, 14))
    for joint, ax in enumerate(axes.flat, 1):
        if joint > 7:
            ax.axis("off")
            continue
        row = traces[joint]
        t = row["time"] - row["time"][0]
        ax.plot(t, row["measured_q"], color="#111827", linewidth=1.7, label="Robot")
        ax.plot(t, row["baseline_q"], color="#9CA3AF", linewidth=1.5, linestyle="--", label="Baseline")
        ax.plot(t, row["calibrated_q"], color="#2563EB", linewidth=1.7, label="Calibrated")
        ax.set_title(f"J{joint}", fontsize=14, fontweight="bold")
        ax.set_xlabel("Time (s)", fontsize=10)
        ax.set_ylabel("Position (rad)", fontsize=10)
        style_axis(ax)
    axes[0, 0].legend(frameon=False, ncol=3, fontsize=10)
    fig.suptitle(f"{side.title()} Arm Response", fontsize=22, fontweight="bold")
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(out / f"{side}_arm_response.png", dpi=180, bbox_inches="tight")
    plt.close(fig)


def plot_pd(out: Path) -> None:
    labels = [f"L{joint}" for joint in range(1, 8)] + [f"R{joint}" for joint in range(1, 8)]
    kp = [BASELINE_KP * value[0] for value in FINAL_SCALES["left"] + FINAL_SCALES["right"]]
    kd = [BASELINE_KD * value[1] for value in FINAL_SCALES["left"] + FINAL_SCALES["right"]]
    x = np.arange(len(labels))
    fig, axes = plt.subplots(2, 1, figsize=(14, 8.2), sharex=True)
    axes[0].bar(x, kp, color="#2563EB")
    axes[1].bar(x, kd, color="#0F766E")
    axes[0].set_ylabel("Kp", fontsize=13)
    axes[1].set_ylabel("Kd", fontsize=13)
    axes[1].set_xticks(x, labels, fontsize=11)
    for ax in axes:
        style_axis(ax)
    fig.suptitle("Isaac Sim Joint Drive Gains", fontsize=20, fontweight="bold")
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(out / "pd_gains.png", dpi=180, bbox_inches="tight")
    plt.close(fig)


def plot_improvements(out: Path, metrics: list[dict]) -> None:
    rows = [row for row in metrics if row["position_improvement_percent"] is not None]
    labels = [row["axis"] for row in rows]
    q = [row["position_improvement_percent"] for row in rows]
    dq = [row["velocity_improvement_percent"] for row in rows]
    x = np.arange(len(labels))
    width = 0.37
    fig, ax = plt.subplots(figsize=(13, 5.8))
    ax.bar(x - width / 2, q, width, color="#2563EB", label="Position")
    ax.bar(x + width / 2, dq, width, color="#0F766E", label="Velocity")
    ax.axhline(0, color="#111827", linewidth=1)
    ax.set_xticks(x, labels, fontsize=11)
    ax.set_ylabel("RMSE improvement (%)", fontsize=13)
    ax.legend(frameon=False, ncol=2, fontsize=11)
    style_axis(ax)
    fig.suptitle("Held-out Replay Improvement", fontsize=19, fontweight="bold")
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(out / "heldout_improvement.png", dpi=180, bbox_inches="tight")
    plt.close(fig)


def plot_gripper(out: Path) -> None:
    left_x = np.array([40.0, 60.0, 80.0])
    left_y = np.array([37.75, 58.56, 80.27])
    right_x = np.array([20.0, 40.0, 60.0, 80.0])
    right_y = np.array([17.72, 37.48, 57.57, 78.01])
    fig, ax = plt.subplots(figsize=(10, 6.4))
    ax.plot([15, 82], [15, 82], color="#9CA3AF", linestyle="--", linewidth=1.8, label="Ideal")
    for x, y, color, label in (
        (left_x, left_y, "#2563EB", "Left"),
        (right_x, right_y, "#0F766E", "Right"),
    ):
        slope, intercept = np.polyfit(x, y, 1)
        span = np.linspace(x.min(), x.max(), 100)
        ax.scatter(x, y, s=70, color=color, zorder=3)
        ax.plot(span, slope * span + intercept, color=color, linewidth=2.4, label=label)
    ax.set_xlabel("Commanded opening (mm)", fontsize=13)
    ax.set_ylabel("Measured opening (mm)", fontsize=13)
    ax.legend(frameon=False, ncol=3, fontsize=12)
    style_axis(ax)
    fig.suptitle("Gripper Opening Calibration", fontsize=20, fontweight="bold")
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(out / "gripper_opening.png", dpi=180, bbox_inches="tight")
    plt.close(fig)


def plot_camera(out: Path) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(13, 5.6))
    axes[0].bar(["Before", "Calibrated", "Cross-validation"], [37.8021, 3.1372, 4.4573], color=["#9CA3AF", "#2563EB", "#0F766E"])
    axes[0].set_ylabel("Reprojection RMSE (px)", fontsize=12)
    axes[0].set_title("Head Camera", fontsize=16, fontweight="bold")
    wrist_names = ["Left wrist", "Right wrist"]
    axes[1].bar(np.arange(2) - 0.18, [4.3786, 4.3964], 0.36, color="#2563EB", label="Translation (mm)")
    axes[1].bar(np.arange(2) + 0.18, [0.5188, 0.3933], 0.36, color="#0F766E", label="Rotation (deg)")
    axes[1].set_xticks(np.arange(2), wrist_names)
    axes[1].set_title("Wrist Camera Consistency", fontsize=16, fontweight="bold")
    axes[1].legend(frameon=False, fontsize=11)
    for ax in axes:
        style_axis(ax)
    fig.suptitle("Camera Calibration Check", fontsize=20, fontweight="bold")
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(out / "camera_calibration.png", dpi=180, bbox_inches="tight")
    plt.close(fig)


def plot_isolated_j1(base: Path, out: Path) -> None:
    baseline_result = load_json(base / "right_baseline_full_result.json")
    baseline_data = np.load(base / "right_baseline_full_rollouts.npz")
    tuned_result = load_json(base / "right_J1_active_iter8_tune_result.json")
    tuned_data = np.load(base / "right_J1_active_iter8_tuned_validation_rollouts.npz")
    baseline_index = index_by_name(baseline_result, validation_only=True)
    row = tuned_result["validation_rollout_index"][0]
    baseline = arrays(baseline_data, baseline_index[row["name"]]["prefix"], 0)
    tuned = arrays(tuned_data, row["prefix"], 0)
    t = tuned["time"] - tuned["time"][0]
    fig, axes = plt.subplots(2, 1, figsize=(11.5, 7.2), sharex=True)
    for ax, measured, base_curve, tuned_curve in (
        (axes[0], tuned["measured_q"], baseline["simulated_q"], tuned["simulated_q"]),
        (axes[1], tuned["measured_dq"], baseline["simulated_dq"], tuned["simulated_dq"]),
    ):
        ax.plot(t, measured, color="#111827", linewidth=2.2, label="Robot")
        ax.plot(t, base_curve, color="#9CA3AF", linewidth=2.0, linestyle="--", label="Baseline")
        ax.plot(t, tuned_curve, color="#2563EB", linewidth=2.2, label="Calibrated")
        style_axis(ax)
    axes[0].set_ylabel("Position (rad)", fontsize=12)
    axes[1].set_ylabel("Velocity (rad/s)", fontsize=12)
    axes[1].set_xlabel("Time (s)", fontsize=12)
    axes[0].legend(frameon=False, ncol=3, fontsize=11)
    fig.suptitle("Right Arm J1 - Isolated Fit", fontsize=18, fontweight="bold")
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(out / "right_J1_isolated_validation.png", dpi=180, bbox_inches="tight")
    plt.close(fig)


def process_side(base: Path, out: Path, side: str) -> tuple[list[dict], dict[int, dict[str, np.ndarray]]]:
    baseline_result = load_json(base / f"{side}_baseline_full_result.json")
    baseline_data = np.load(base / f"{side}_baseline_full_rollouts.npz")
    final_tag = "right_global_v4" if side == "right" else "left_global_v3"
    final_result = load_json(base / f"{final_tag}_fixed_validation_result.json")
    final_data = np.load(base / f"{final_tag}_fixed_validation_rollouts.npz")
    baseline_index = index_by_name(baseline_result, validation_only=True)
    final_rows = final_result["validation_rollout_index"]
    final_index = {row["name"]: row for row in final_rows}
    metrics: list[dict] = []
    traces: dict[int, dict[str, np.ndarray]] = {}
    passive_row = next((row for row in reversed(final_rows) if row["excitation"] == "J3_step"), final_rows[-1])
    for joint in range(1, 8):
        rows = [row for row in final_rows if row["excitation"] == f"J{joint}_step"]
        scoring_rows = [row for row in rows if row["name"] in baseline_index]
        if scoring_rows:
            bq: list[np.ndarray] = []
            bdq: list[np.ndarray] = []
            fq: list[np.ndarray] = []
            fdq: list[np.ndarray] = []
            for row in scoring_rows:
                b = arrays(baseline_data, baseline_index[row["name"]]["prefix"], joint - 1)
                f = arrays(final_data, row["prefix"], joint - 1)
                bq.append(b["simulated_q"] - b["measured_q"])
                bdq.append(b["simulated_dq"] - b["measured_dq"])
                fq.append(f["simulated_q"] - f["measured_q"])
                fdq.append(f["simulated_dq"] - f["measured_dq"])
            bq_rmse, bdq_rmse = rmse(bq), rmse(bdq)
            fq_rmse, fdq_rmse = rmse(fq), rmse(fdq)
            q_improve = (1.0 - fq_rmse / bq_rmse) * 100.0
            dq_improve = (1.0 - fdq_rmse / bdq_rmse) * 100.0
            plot_row = scoring_rows[0]
        else:
            bq_rmse = bdq_rmse = fq_rmse = fdq_rmse = None
            q_improve = dq_improve = None
            plot_row = passive_row
        b = arrays(baseline_data, baseline_index[plot_row["name"]]["prefix"], joint - 1)
        f = arrays(final_data, final_index[plot_row["name"]]["prefix"], joint - 1)
        traces[joint] = {
            "time": f["time"],
            "measured_q": f["measured_q"],
            "baseline_q": b["simulated_q"],
            "calibrated_q": f["simulated_q"],
        }
        plot_axis(
            out,
            side,
            joint,
            f["measured_q"],
            b["simulated_q"],
            f["simulated_q"],
            f["time"],
            f["measured_dq"],
            b["simulated_dq"],
            f["simulated_dq"],
        )
        kp_scale, kd_scale = FINAL_SCALES[side][joint - 1]
        metrics.append(
            {
                "axis": f"{side[0].upper()}J{joint}",
                "side": side,
                "joint": joint,
                "kp_scale": kp_scale,
                "kd_scale": kd_scale,
                "kp": BASELINE_KP * kp_scale,
                "kd": BASELINE_KD * kd_scale,
                "baseline_position_rmse": bq_rmse,
                "final_position_rmse": fq_rmse,
                "position_improvement_percent": q_improve,
                "baseline_velocity_rmse": bdq_rmse,
                "final_velocity_rmse": fdq_rmse,
                "velocity_improvement_percent": dq_improve,
                "validation": "held_out_active_axis" if scoring_rows else "passive_replay_only",
            }
        )
    plot_contact_sheet(out, side, traces)
    return metrics, traces


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("base", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    metrics: list[dict] = []
    for side in ("left", "right"):
        rows, _ = process_side(args.base, args.output, side)
        metrics.extend(rows)
    plot_pd(args.output)
    plot_improvements(args.output, metrics)
    plot_gripper(args.output)
    plot_camera(args.output)
    plot_isolated_j1(args.base, args.output)
    (args.output / "final_parameters.json").write_text(
        json.dumps({"baseline": {"kp": BASELINE_KP, "kd": BASELINE_KD}, "joints": metrics}, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    with (args.output / "final_parameters.csv").open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(metrics[0]))
        writer.writeheader()
        writer.writerows(metrics)
    print(json.dumps({"output": str(args.output), "figures": len(list(args.output.glob("*.png"))), "rows": len(metrics)}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
