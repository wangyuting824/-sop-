#!/usr/bin/env python3
"""Replay fixed per-axis gain scales on NVIDIA SysID holdout chunks."""

from __future__ import annotations

import argparse
import asyncio
import json
import os
from pathlib import Path
import sys
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument("--run-spec", type=Path, required=True)
    parser.add_argument("--tag", required=True)
    parser.add_argument("--active-joint", type=int, choices=range(1, 8))
    parser.add_argument(
        "--theta",
        type=float,
        nargs="+",
        required=True,
        help="Gain-scale vector in the run spec parameter order. Use two values for an active-axis spec.",
    )
    return parser.parse_args()


ARGS = parse_args()
RAW_DOF_INDICES = {"left": [0, 2, 4, 6, 8, 10, 12], "right": [1, 3, 5, 7, 9, 11, 13]}[ARGS.side]
app = None


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


def rollout_payload(prefix: str, chunk, rollout) -> dict[str, np.ndarray]:  # noqa: ANN001
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
    from isaacsim.robot_setup.sysid.isaac_sim_sysid_bridge import IsaacSimSysIdBridge
    from isaacsim.robot_setup.sysid.run_controller import SysIdRunController
    from isaacsim.robot_setup.sysid.run_spec import SysIdRunSpec

    raw_spec = json.loads(ARGS.run_spec.read_text(encoding="utf-8"))
    if raw_spec.get("xuchuang_verified_physx_dof_indices") != RAW_DOF_INDICES:
        raise ValueError("RunSpec raw DOF index guard failed")
    spec = SysIdRunSpec.from_dict(raw_spec)
    context = omni.usd.get_context()
    ok, error = await context.open_stage_async(spec.stage.input_path)
    if not ok:
        raise RuntimeError(error)
    for _ in range(12):
        await omni.kit.app.get_app().next_update_async()
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()

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
    prepared = controller.prepare(context.get_stage(), trajectory=trajectory, timeline_playing=True)
    parameter_dofs = {int(entry.dof_index) for entry in prepared.config.param_entries if int(entry.dof_index) >= 0}
    if ARGS.active_joint is not None:
        active_index = ARGS.active_joint - 1
        if parameter_dofs != {active_index}:
            raise RuntimeError(f"Parameter DOF guard failed: {parameter_dofs}")
    elif len(ARGS.theta) != len(prepared.config.param_entries):
        raise RuntimeError(
            f"Full-vector replay needs {len(prepared.config.param_entries)} values; got {len(ARGS.theta)}"
        )
    theta = prepared.config.theta_initial.new_tensor(ARGS.theta)
    replay_fn = getattr(prepared.optimizer, "evaluate_validation_chunks_with_rollouts")
    replays = await replay_fn(prepared.config, list(prepared.validation_chunks), theta)
    payload: dict[str, np.ndarray] = {}
    index: list[dict[str, object]] = []
    for number, (chunk, replay) in enumerate(zip(prepared.validation_chunks, replays), 1):
        prefix = f"validation_{number:03d}"
        payload.update(rollout_payload(prefix, chunk, replay.rollout))
        index.append({"prefix": prefix, "name": chunk.spec.name, "excitation": chunk.spec.excitation})
    npz_path = OUT / f"{ARGS.tag}_fixed_validation_rollouts.npz"
    json_path = OUT / f"{ARGS.tag}_fixed_validation_result.json"
    if npz_path.exists() or json_path.exists():
        raise FileExistsError(f"Refusing to overwrite {ARGS.tag}")
    np.savez_compressed(npz_path, **payload)
    json_path.write_text(
        json.dumps(
            {
                "kind": "official_nvidia_sysid_fixed_theta_validation",
                "side": ARGS.side,
                "active_joint": ARGS.active_joint,
                "theta_scales": ARGS.theta,
                "validation_metrics": [replay.metric.to_dict() for replay in replays],
                "validation_rollout_index": index,
                "validation_rollout_npz": str(npz_path),
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    timeline.stop()
    cleanup = getattr(prepared.optimizer, "cleanup_bridge_async", None)
    if callable(cleanup):
        await cleanup()
    return {"result": str(json_path), "rollouts": str(npz_path)}


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
                "--/app/userConfigPath=" + str(RUNTIME / "config/isaac61_sysid_validate.json"),
                "--/log/file=" + str(RUNTIME / "logs/isaac61_sysid_validate.log"),
                "--/app/settings/persistent=false",
                "--/app/asyncRendering=false",
            ],
        }
    )
    task = asyncio.ensure_future(run())
    try:
        while not task.done():
            app.update()
        print(json.dumps(task.result(), ensure_ascii=False, indent=2), flush=True)
        return 0
    except BaseException:
        traceback.print_exc()
        raise
    finally:
        app.close()


if __name__ == "__main__":
    raise SystemExit(main())
