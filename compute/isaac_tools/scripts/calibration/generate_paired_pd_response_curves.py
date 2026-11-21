#!/usr/bin/env python3
"""Plot paired robot, blank-PD, and adjusted-PD Isaac Sim validation replays."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

BASE_KP = 625.0
BASE_KD = 0.2
COLORS = {"robot": "#111827", "blank": "#9CA3AF", "adjusted": "#2563EB"}


def rmse(x: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(x)))) if x.size else float("nan")


def load_pair(root: Path, side: str):
    if side == "left":
        base_tag, tune_tag = "left_blank_allchunks_20261114", "left_adjusted_allchunks_20261114"
    else:
        base_tag, tune_tag = "right_baseline_final_20261114", "right_tuned_final_20261114"
    br = json.loads((root / f"{base_tag}_fixed_validation_result.json").read_text(encoding="utf-8"))
    tr = json.loads((root / f"{tune_tag}_fixed_validation_result.json").read_text(encoding="utf-8"))
    bd = np.load(root / f"{base_tag}_fixed_validation_rollouts.npz")
    td = np.load(root / f"{tune_tag}_fixed_validation_rollouts.npz")
    bi = {r["name"]: r for r in br["validation_rollout_index"]}
    ti = {r["name"]: r for r in tr["validation_rollout_index"]}
    if list(bi) != list(ti):
        raise ValueError(f"{side}: baseline/tuned validation chunks differ")
    chunks = []
    for name in bi:
        b, t = bi[name], ti[name]
        if b.get("excitation") != t.get("excitation"):
            raise ValueError(f"{side}: excitation mismatch in {name}")
        bp, tp = b["prefix"], t["prefix"]
        for suffix in ("times_s", "measured_q_rad", "measured_dq_rad_s", "command_q_rad"):
            if not np.allclose(bd[f"{bp}_{suffix}"], td[f"{tp}_{suffix}"], rtol=0, atol=1e-10):
                raise ValueError(f"{side}: input data mismatch in {name} ({suffix})")
        chunks.append((name, b.get("excitation", ""), bd, bp, td, tp))
    scales = np.asarray(tr["theta_scales"], dtype=float).reshape(7, 2)
    return chunks, scales


def collect(chunks, joint: int):
    out = {k: [] for k in ("t", "robot_q", "blank_q", "adjusted_q", "robot_dq", "blank_dq", "adjusted_dq")}
    offset = 0.0
    for _, _, bd, bp, td, tp in chunks:
        t = np.asarray(bd[f"{bp}_times_s"], float)
        t = t - t[0] + offset
        bq = np.asarray(bd[f"{bp}_simulated_q_rad"], float)[joint]
        tq = np.asarray(td[f"{tp}_simulated_q_rad"], float)[joint]
        rq = np.asarray(bd[f"{bp}_measured_q_rad"], float)[joint]
        bdq = np.asarray(bd[f"{bp}_simulated_dq_rad_s"], float)[joint]
        tdq = np.asarray(td[f"{tp}_simulated_dq_rad_s"], float)[joint]
        rdq = np.asarray(bd[f"{bp}_measured_dq_rad_s"], float)[joint]
        for key, vals in (("t", t), ("robot_q", rq), ("blank_q", bq), ("adjusted_q", tq),
                          ("robot_dq", rdq), ("blank_dq", bdq), ("adjusted_dq", tdq)):
            out[key].extend(vals.tolist() + [np.nan])
        offset = float(t[-1]) + 0.6
    return {k: np.asarray(v) for k, v in out.items()}


def style(ax):
    ax.grid(True, alpha=0.22)
    ax.spines[["top", "right"]].set_visible(False)
    ax.tick_params(labelsize=9)


def plot_arm(out: Path, side: str, chunks):
    metrics = []
    scope_title = f"{len(chunks)} recorded segments" if side == "left" else f"{len(chunks)} held-out segments"
    for mode in ("position", "velocity", "error"):
        fig, axs = plt.subplots(4, 2, figsize=(14, 13), sharex=True)
        for j, ax in enumerate(axs.flat[:7]):
            s = collect(chunks, j)
            if mode == "position":
                curves = ((s["robot_q"], "Robot", COLORS["robot"], "-"),
                          (s["blank_q"], "Blank PD", COLORS["blank"], "--"),
                          (s["adjusted_q"], "Adjusted PD", COLORS["adjusted"], "-"))
                ylabel = "Position (rad)"
            elif mode == "velocity":
                curves = ((s["robot_dq"], "Robot", COLORS["robot"], "-"),
                          (s["blank_dq"], "Blank PD", COLORS["blank"], "--"),
                          (s["adjusted_dq"], "Adjusted PD", COLORS["adjusted"], "-"))
                ylabel = "Velocity (rad/s)"
            else:
                curves = (((s["blank_q"] - s["robot_q"]) * 180 / np.pi, "Blank PD − robot", COLORS["blank"], "--"),
                          ((s["adjusted_q"] - s["robot_q"]) * 180 / np.pi, "Adjusted PD − robot", COLORS["adjusted"], "-"))
                ylabel = "Position error (deg)"
                ax.axhline(0, color="#374151", linewidth=0.8)
            for values, label, color, ls in curves:
                ax.plot(s["t"], values, color=color, linestyle=ls, linewidth=1.15, label=label)
            ax.set_title(f"J{j+1}", fontweight="bold")
            ax.set_ylabel(ylabel)
            style(ax)
            if j == 0:
                ax.legend(frameon=False, fontsize=8, ncol=1)
            if mode == "error":
                parts_bq, parts_tq = [], []
                for _, _, bd, bp, td, tp in chunks:
                    parts_bq.append(np.asarray(bd[f"{bp}_simulated_q_rad"], float)[j] - np.asarray(bd[f"{bp}_measured_q_rad"], float)[j])
                    parts_tq.append(np.asarray(td[f"{tp}_simulated_q_rad"], float)[j] - np.asarray(td[f"{tp}_measured_q_rad"], float)[j])
                bq, tq = np.concatenate(parts_bq), np.concatenate(parts_tq)
                ax.text(0.99, 0.04, f"RMSE blank {rmse(bq)*180/np.pi:.3f}° | adjusted {rmse(tq)*180/np.pi:.3f}°",
                        transform=ax.transAxes, ha="right", va="bottom", fontsize=7, color="#374151")
        axs.flat[7].axis("off")
        axs[-1, 0].set_xlabel("Concatenated validation time (s)")
        fig.suptitle(f"{side.title()} arm — {mode.title()} ({scope_title})", fontsize=18, fontweight="bold")
        fig.tight_layout(rect=(0, 0, 1, 0.97))
        fig.savefig(out / f"{side}_{mode}_all_joints.png", dpi=180, bbox_inches="tight")
        plt.close(fig)
    for j in range(7):
        bq, tq, bdq, tdq = [], [], [], []
        for _, _, bd, bp, td, tp in chunks:
            bq.append(np.asarray(bd[f"{bp}_simulated_q_rad"], float)[j] - np.asarray(bd[f"{bp}_measured_q_rad"], float)[j])
            tq.append(np.asarray(td[f"{tp}_simulated_q_rad"], float)[j] - np.asarray(td[f"{tp}_measured_q_rad"], float)[j])
            bdq.append(np.asarray(bd[f"{bp}_simulated_dq_rad_s"], float)[j] - np.asarray(bd[f"{bp}_measured_dq_rad_s"], float)[j])
            tdq.append(np.asarray(td[f"{tp}_simulated_dq_rad_s"], float)[j] - np.asarray(td[f"{tp}_measured_dq_rad_s"], float)[j])
        bq, tq, bdq, tdq = map(np.concatenate, (bq, tq, bdq, tdq))
        metrics.append({"axis": f"{side[0].upper()}J{j+1}", "side": side, "joint": j+1,
            "blank_position_rmse_rad": rmse(bq), "adjusted_position_rmse_rad": rmse(tq),
            "position_improvement_percent": (1-rmse(tq)/rmse(bq))*100 if rmse(bq)>0 else None,
            "blank_velocity_rmse_rad_s": rmse(bdq), "adjusted_velocity_rmse_rad_s": rmse(tdq),
            "velocity_improvement_percent": (1-rmse(tdq)/rmse(bdq))*100 if rmse(bdq)>0 else None})
    # One zoomable four-panel file per joint, preserving all validation trials.
    for j in range(7):
        s = collect(chunks, j)
        fig, axs = plt.subplots(4, 1, figsize=(12, 9), sharex=True)
        for ax, values, unit, err in (
            (axs[0], (s["robot_q"], s["blank_q"], s["adjusted_q"]), "Position (rad)", False),
            (axs[1], ((s["blank_q"]-s["robot_q"])*180/np.pi, (s["adjusted_q"]-s["robot_q"])*180/np.pi), "Position error (deg)", True),
            (axs[2], (s["robot_dq"], s["blank_dq"], s["adjusted_dq"]), "Velocity (rad/s)", False),
            (axs[3], (s["blank_dq"]-s["robot_dq"], s["adjusted_dq"]-s["robot_dq"]), "Velocity error (rad/s)", True),
        ):
            if err:
                ax.axhline(0, color="#374151", linewidth=0.8)
                labs = ("Blank PD − robot", "Adjusted PD − robot")
                cols = (COLORS["blank"], COLORS["adjusted"])
                lss = ("--", "-")
            else:
                labs = ("Robot", "Blank PD", "Adjusted PD")
                cols = (COLORS["robot"], COLORS["blank"], COLORS["adjusted"])
                lss = ("-", "--", "-")
            for val, lab, col, ls in zip(values, labs, cols, lss):
                ax.plot(s["t"], val, color=col, linestyle=ls, linewidth=1.2, label=lab)
            ax.set_ylabel(unit)
            style(ax)
            if ax is axs[0]: ax.legend(frameon=False, ncol=3, fontsize=9)
        axs[-1].set_xlabel("Concatenated validation time (s)")
        fig.suptitle(f"{side.title()} arm J{j+1} — Robot vs blank PD vs adjusted PD", fontsize=16, fontweight="bold")
        fig.tight_layout(rect=(0, 0, 1, 0.96))
        fig.savefig(out / f"{side}_J{j+1}_three_way.png", dpi=180, bbox_inches="tight")
        plt.close(fig)
    return metrics


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("data_dir", type=Path)
    ap.add_argument("output_dir", type=Path)
    a = ap.parse_args()
    a.output_dir.mkdir(parents=True, exist_ok=True)
    all_metrics, gain_rows = [], []
    manifest = {"comparison": "real robot vs blank PD vs adjusted PD",
                "simulator": "Isaac Sim 6.1.0 / NVIDIA SysID fixed-parameter validation replay",
                "input_pairing_verified": True,
                "scope_note": "Left plots use 43 of 44 source chunks; the first chunk was retained for SysID preparation. The left recordings actively excite J1-J3 only; J4-J7 are shown as measured passive channels, not active left-axis tests. Right plots use paired held-out chunks with active trials for J1-J7.",
                "sides": {}}
    for side in ("left", "right"):
        chunks, scales = load_pair(a.data_dir, side)
        all_metrics.extend(plot_arm(a.output_dir, side, chunks))
        manifest["sides"][side] = {"validation_chunks": [n for n, *_ in chunks], "scale_pairs_kp_kd": scales.tolist()}
        for j, (ks, ds) in enumerate(scales, 1):
            gain_rows.append({"side": side, "joint": j, "blank_kp": BASE_KP, "adjusted_kp": BASE_KP*ks,
                              "kp_scale": ks, "blank_kd": BASE_KD, "adjusted_kd": BASE_KD*ds, "kd_scale": ds})
    with (a.output_dir / "joint_response_metrics.csv").open("w", newline="", encoding="utf-8-sig") as f:
        w=csv.DictWriter(f, fieldnames=list(all_metrics[0])); w.writeheader(); w.writerows(all_metrics)
    with (a.output_dir / "pd_parameters_comparison.csv").open("w", newline="", encoding="utf-8-sig") as f:
        w=csv.DictWriter(f, fieldnames=list(gain_rows[0])); w.writeheader(); w.writerows(gain_rows)
    x=np.arange(14); labels=[f"L{j}" for j in range(1,8)]+[f"R{j}" for j in range(1,8)]
    fig,axs=plt.subplots(2,1,figsize=(14,8),sharex=True)
    for ax, key, ylabel in ((axs[0],"kp","Kp"),(axs[1],"kd","Kd")):
        blank=[r[f"blank_{key}"] for r in gain_rows]; tuned=[r[f"adjusted_{key}"] for r in gain_rows]
        ax.bar(x-0.19,blank,0.38,color=COLORS["blank"],label="Blank PD")
        ax.bar(x+0.19,tuned,0.38,color=COLORS["adjusted"],label="Adjusted PD")
        ax.set_ylabel(ylabel); style(ax); ax.legend(frameon=False)
    axs[1].set_xticks(x,labels)
    fig.suptitle("PD parameter comparison used in the paired replay",fontsize=17,fontweight="bold")
    fig.tight_layout(rect=(0,0,1,0.95)); fig.savefig(a.output_dir/"pd_parameters_blank_vs_adjusted.png",dpi=180,bbox_inches="tight"); plt.close(fig)
    (a.output_dir/"run_manifest.json").write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    print(json.dumps({"output":str(a.output_dir),"figures":len(list(a.output_dir.glob("*.png"))),"metrics":len(all_metrics)},ensure_ascii=False))


if __name__ == "__main__":
    main()
