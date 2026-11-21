#!/usr/bin/env python3
"""Run a bounded, offline NVIDIA Isaac Sim SysID gain solve with verified DOF mapping.

The installed NVIDIA SysID bridge defaults to raw DOF ``range(N)`` when no
indices are supplied.  This robot's live PhysX articulation interleaves the
arms.  The adapter below supplies the probe-verified raw indices only; all
replay, finite-difference Jacobians, optimization, validation and metrics are
provided by the installed NVIDIA 6.1 SysID extension.

The run spec must have ``apply_parameters_to_usd=false``.  Thus this launcher
does not write either source USD or the candidate stage; it emits an auditable
gain-scale result and the post-solve holdout rollouts for later review.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import os
from pathlib import Path
import sys
import time
import traceback

import numpy as np


ROOT = Path(os.environ["XUCHUANG_WORKSPACE"])
RUNTIME = Path(os.environ["XUCHUANG_RUNTIME"])
SYSID_EXTENSION = Path(
    os.environ.get(
        "ISAACSIM61_SYSID_EXTENSION",
        os.path.join(os.getenv("CALIBRATION_SIM_HOME", os.path.expanduser("~")), "isaacsim-6.1.0/exts/isaacsim.robot_setup.sysid"),
    )
)
OUT = ROOT / "xuchuang_response_work/sysid_20261113"
(RUNTIME / "kit").mkdir(parents=True, exist_ok=True)
(RUNTIME / "logs").mkdir(parents=True, exist_ok=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument("--run-spec", type=Path, required=True)
    parser.add_argument("--tag", required=True, help="Unique output suffix, for example left_J1_iter3.")
    parser.add_argument(
        "--active-joint",
        type=int,
        choices=range(1, 8),
        required=True,
        help="One-based joint whose position/velocity residuals define the solve objective.",
    )
    return parser.parse_args()


ARGS = parse_args()
RAW_DOF_INDICES = {"left": [0, 2, 4, 6, 8, 10, 12], "right": [1, 3, 5, 7, 9, 11, 13]}[ARGS.side]
app = None


def write_json(path: Path, payload: object) -> None:
    if path.exists():
        raise FileExistsError(f"Refusing to overwrite result artifact: {path}")
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


async def enable_sysid_extension() -> None:
    import isaacsim
    import omni.kit.app

    manager = omni.kit.app.get_app().get_extension_manager()
    manager.add_path(str(SYSID_EXTENSION))
    manager.set_extension_enabled_immediate("isaacsim.robot_setup.sysid", True)
    package_root = str(SYSID_EXTENSION / "isaacsim")
    if package_root not in isaacsim.__path__:
        isaacsim.__path__.append(package_root)
    if str(SYSID_EXTENSION) not in sys.path:
        sys.path.insert(0, str(SYSID_EXTENSION))
    for _ in range(4):
        await omni.kit.app.get_app().next_update_async()


def rollout_npz_payload(prefix: str, chunk, rollout) -> dict[str, np.ndarray]:  # noqa: ANN001
    return {
        prefix + "_times_s": np.asarray(rollout.times, dtype=np.float64),
        prefix + "_measured_q_rad": np.asarray(rollout.measured_positions, dtype=np.float64),
        prefix + "_simulated_q_rad": np.asarray(rollout.simulated_positions, dtype=np.float64),
        prefix + "_measured_dq_rad_s": np.asarray(rollout.measured_velocities, dtype=np.float64),
        prefix + "_simulated_dq_rad_s": np.asarray(rollout.simulated_velocities, dtype=np.float64),
        prefix + "_command_q_rad": np.asarray(chunk.trajectory.commands, dtype=np.float64),
    }


async def run() -> dict[str, object]:
    import omni.kit.app
    import omni.timeline
    import omni.usd

    await enable_sysid_extension()
    from isaacsim.robot_setup.sysid.execution import run_sysid
    from isaacsim.robot_setup.sysid.isaac_sim_sysid_bridge import IsaacSimSysIdBridge
    from isaacsim.robot_setup.sysid.run_controller import SysIdRunController
    from isaacsim.robot_setup.sysid.run_spec import SysIdRunSpec

    raw_spec = json.loads(ARGS.run_spec.read_text(encoding="utf-8"))
    reported = raw_spec.get("xuchuang_verified_physx_dof_indices")
    if reported != RAW_DOF_INDICES:
        raise ValueError(f"RunSpec raw DOF index guard failed: {reported} != {RAW_DOF_INDICES}")
    if bool(raw_spec.get("outputs", {}).get("apply_parameters_to_usd", False)):
        raise ValueError("This offline solve adapter requires apply_parameters_to_usd=false")
    spec = SysIdRunSpec.from_dict(raw_spec)
    context = omni.usd.get_context()
    ok, error = await context.open_stage_async(spec.stage.input_path)
    if not ok:
        raise RuntimeError(error)
    for _ in range(12):
        await omni.kit.app.get_app().next_update_async()
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(16):
        await omni.kit.app.get_app().next_update_async()
    stage = context.get_stage()
    if stage is None:
        raise RuntimeError("No replay stage after open")

    def bridge_factory(**kwargs):
        return IsaacSimSysIdBridge(
            robot_prim_path=kwargs["robot_prim_path"],
            source_env_path=kwargs["source_env_path"],
            env_paths_root=kwargs["env_paths_root"],
            use_parallel_clones=kwargs["use_parallel_clones"],
            co_locate_clones=kwargs["co_locate_clones"],
            fabric_clones=kwargs["fabric_clones"],
            offline_stepping=kwargs["offline_stepping"],
            end_effector_link_index=kwargs["end_effector_link_index"],
            joint_dof_indices=list(RAW_DOF_INDICES),
            selected_dof_paths=list(kwargs["selected_dof_paths"])[:7],
            physics_backend=kwargs["physics_backend"],
            actuator_runtime=kwargs["actuator_runtime"],
            explicit_actuator_policy=kwargs["explicit_actuator_policy"],
        )

    controller = SysIdRunController(spec, bridge_factory=bridge_factory)
    trajectory = controller.load_trajectory()
    prepared = controller.prepare(stage, trajectory=trajectory, timeline_playing=True)
    if not prepared.train_chunks or not prepared.validation_chunks:
        raise ValueError("Tuning spec requires at least one train and validation chunk")
    expected = [f"{ARGS.side}_arm_joint_{number}" for number in range(1, 8)]
    # The controller keeps unmatched joints at the tail.  Its documented
    # telemetry-order contract is that the first N paths correspond to the N
    # CSV columns, so validate exactly that prefix rather than incorrectly
    # requiring a seven-joint USD stage.
    selected_paths = [path.rsplit("/", 1)[-1] for path in prepared.joint_paths[:7]]
    if selected_paths != expected:
        raise RuntimeError(f"Prepared joint-path order mismatch: {selected_paths} != {expected}")

    # A single-axis experiment must be scored on that axis.  The stock residual
    # engine flattens every telemetry joint; with seven-column telemetry, noise
    # from the six stationary joints otherwise dominates the objective while the
    # optimizer is allowed to change only two gains.  Keep NVIDIA's bridge,
    # finite-difference LM solver, weighting and validation, but select the
    # physically excited residual column before the standard engine evaluates it.
    from isaacsim.robot_setup.sysid.residual_engine import RolloutResidualInputs, WeightedResidualEngine

    class ActiveJointResidualEngine(WeightedResidualEngine):
        def __init__(self, specs, active_index: int):  # noqa: ANN001
            super().__init__(specs)
            self._active_index = int(active_index)

        def _joint_column(self, value):  # noqa: ANN001
            if value is None:
                return None
            return value[..., self._active_index : self._active_index + 1]

        def compute(self, inputs):  # noqa: ANN001
            selected = RolloutResidualInputs(
                meas_pos=self._joint_column(inputs.meas_pos),
                meas_vel=self._joint_column(inputs.meas_vel),
                sim_pos=self._joint_column(inputs.sim_pos),
                sim_vel=self._joint_column(inputs.sim_vel),
                meas_torque=self._joint_column(inputs.meas_torque),
                sim_torque=self._joint_column(inputs.sim_torque),
                meas_ee_pose=inputs.meas_ee_pose,
                sim_ee_pose=inputs.sim_ee_pose,
                meas_contact=inputs.meas_contact,
                sim_contact=inputs.sim_contact,
                sample_weights=inputs.sample_weights,
            )
            return super().compute(selected)

    active_index = ARGS.active_joint - 1
    parameter_dofs = {
        int(entry.dof_index)
        for entry in prepared.config.param_entries
        if int(entry.dof_index) >= 0
    }
    if parameter_dofs != {active_index}:
        raise RuntimeError(
            f"Active-axis guard failed: objective J{ARGS.active_joint}, parameter DOFs {sorted(parameter_dofs)}"
        )
    original_engine = prepared.optimizer._residual_engine
    prepared.optimizer._residual_engine = ActiveJointResidualEngine(original_engine.specs, active_index)

    solve_started = time.monotonic()
    progress_stamp = [0.0]
    iteration_rows: list[dict[str, object]] = []

    def progress(message: str, fraction: float) -> None:
        now = time.monotonic()
        if now - progress_stamp[0] >= 5.0 or fraction >= 1.0:
            print(f"[tune {ARGS.side} {ARGS.tag}] {fraction:.1%} {message}", flush=True)
            progress_stamp[0] = now

    def iteration(status) -> None:  # noqa: ANN001
        row = {
            "iteration": int(status.iteration),
            "cost": float(status.cost),
            "damping": float(status.damping),
            "accepted": bool(status.accepted),
            "theta_scales": [float(value) for value in status.theta],
        }
        iteration_rows.append(row)
        print("[tune iteration] " + json.dumps(row, ensure_ascii=False), flush=True)

    try:
        result = await run_sysid(spec, prepared_run=prepared, on_iteration=iteration, on_progress=progress)
        validation_chunks = list(prepared.validation_chunks)
        if len(validation_chunks) != len(result.validation_rollouts):
            raise RuntimeError(
                "Expected a stored validation rollout for every validation chunk; "
                f"got {len(result.validation_rollouts)} for {len(validation_chunks)} chunks"
            )
        data: dict[str, np.ndarray] = {}
        rollout_index: list[dict[str, object]] = []
        for number, (chunk, rollout) in enumerate(zip(validation_chunks, result.validation_rollouts), 1):
            prefix = f"validation_{number:03d}"
            data.update(rollout_npz_payload(prefix, chunk, rollout))
            rollout_index.append(
                {
                    "prefix": prefix,
                    "role": "validation",
                    "name": chunk.spec.name,
                    "excitation": chunk.spec.excitation,
                    "source_start_s": chunk.spec.start,
                    "source_end_s": chunk.spec.end,
                    "rollout_samples": len(rollout.times),
                }
            )
        npz_path = OUT / f"{ARGS.tag}_tuned_validation_rollouts.npz"
        if npz_path.exists():
            raise FileExistsError(f"Refusing to overwrite result artifact: {npz_path}")
        np.savez_compressed(npz_path, **data)
        selected = result.selected_status
        result_path = OUT / f"{ARGS.tag}_tune_result.json"
        payload = {
            "kind": "official_nvidia_sysid_gain_scale_solve",
            "side": ARGS.side,
            "run_spec": str(ARGS.run_spec.resolve()),
            "stage": spec.stage.input_path,
            "physics_backend": prepared.physics_backend,
            "backend": prepared.backend_label,
            "raw_physx_dof_indices": RAW_DOF_INDICES,
            "joint_paths": prepared.joint_paths,
            "objective_joint": ARGS.active_joint,
            "objective_scope": "active_joint_position_and_velocity_only",
            "parameter_entries": [
                {
                    "param_type": entry.param_type.value,
                    "dof_index": int(entry.dof_index),
                    "link_index": int(entry.link_index),
                    "component_index": int(entry.component_index),
                    "category": entry.resolved_category().value,
                }
                for entry in prepared.config.param_entries
            ],
            "theta_initial_scales": [float(value) for value in prepared.config.theta_initial.detach().cpu().tolist()],
            "selected_status": {
                "iteration": int(selected.iteration),
                "cost": float(selected.cost),
                "damping": float(selected.damping),
                "accepted": bool(selected.accepted),
                "theta_scales": [float(value) for value in selected.theta],
                "source": result.selected_status_source,
            },
            "final_status": {
                "iteration": int(result.final_status.iteration),
                "cost": float(result.final_status.cost),
                "damping": float(result.final_status.damping),
                "accepted": bool(result.final_status.accepted),
                "theta_scales": [float(value) for value in result.final_status.theta],
            },
            "iterations": iteration_rows,
            "validation_metrics": [metric.to_dict() for metric in result.validation_metrics],
            "parameter_confidence": result.parameter_confidence,
            "validation_rollout_index": rollout_index,
            "validation_rollout_npz": str(npz_path),
            "parameters_written": bool(result.parameters_written),
            "source_usd_modified": False,
            "elapsed_seconds": time.monotonic() - solve_started,
            "scope_note": (
                "The reported values are multipliers of the baseline authored drive gains, not manufacturer gains. "
                "No USD writeback was performed by this solve adapter."
            ),
        }
        write_json(result_path, payload)
        return {"result": str(result_path), "rollouts": str(npz_path), "selected_theta": payload["selected_status"]["theta_scales"]}
    finally:
        timeline.stop()
        for _ in range(4):
            await omni.kit.app.get_app().next_update_async()


def main() -> int:
    global app
    from isaacsim import SimulationApp

    app = SimulationApp(
        {
            "headless": True,
            "hide_ui": True,
            "multi_gpu": False,
            "renderer": "RayTracedLighting",
            "limit_cpu_threads": 6,
            "create_new_stage": False,
            "enable_crashreporter": False,
            "extra_args": [
                "--portable",
                "--portable-root",
                str(RUNTIME / "kit"),
                "--/app/userConfigPath=" + str(RUNTIME / "config/isaac61_sysid_tune.json"),
                "--/log/file=" + str(RUNTIME / "logs/isaac61_sysid_tune.log"),
                "--/app/settings/persistent=false",
                "--/app/asyncRendering=false",
            ],
        }
    )
    task = asyncio.ensure_future(run())
    try:
        while not task.done():
            app.update()
        print(json.dumps(task.result(), ensure_ascii=False, indent=2))
    except BaseException:
        traceback.print_exc()
        raise
    finally:
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
