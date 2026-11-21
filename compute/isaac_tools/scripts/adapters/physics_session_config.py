"""Shared, session-only PhysX settings for paired tuning and validation."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path


PHYSICS_DT_S = 1.0 / 60.0
CONFIGURATION_ID = "tgs_external_forces_every_iteration_60hz_v1"
PHYSX_REFERENCE = (
    "https://nvidia-omniverse.github.io/PhysX/physx/5.8.0/docs/Simulation.html"
    "#tgs-steady-state-velocity-and-position-discrepancy"
)


def apply_physics_session_config(stage, *, stage_path: str, output: Path, tag: str) -> dict:
    """Configure the current stage before timeline play; do not save model files."""
    import omni.timeline
    from isaacsim.core.simulation_manager import SimulationManager
    from pxr import PhysxSchema, Usd, UsdPhysics

    if stage is None:
        raise RuntimeError("No stage for session-only physics configuration")
    if omni.timeline.get_timeline_interface().is_playing():
        raise RuntimeError("Physics configuration must be applied before timeline play")

    session = stage.GetSessionLayer()
    source_layer = stage.GetRootLayer()
    source_file = Path(stage_path)
    source_sha256 = hashlib.sha256(source_file.read_bytes()).hexdigest() if source_file.is_file() else None
    before = {}
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Scene):
            api = PhysxSchema.PhysxSceneAPI(prim)
            before[str(prim.GetPath())] = {
                "external_forces_every_iteration": api.GetEnableExternalForcesEveryIterationAttr().Get(),
                "time_steps_per_second": api.GetTimeStepsPerSecondAttr().Get(),
                "solver_type": api.GetSolverTypeAttr().Get(),
            }

    scenes = []
    # Both setup_simulation edits and the TGS force scheduling override are
    # authored into the anonymous session layer, never the original USD layer.
    with Usd.EditContext(stage, session):
        SimulationManager.setup_simulation(dt=PHYSICS_DT_S)
        for prim in stage.Traverse():
            if prim.IsA(UsdPhysics.Scene):
                api = PhysxSchema.PhysxSceneAPI.Apply(prim)
                api.CreateEnableExternalForcesEveryIterationAttr().Set(True)
                scenes.append({
                    "path": str(prim.GetPath()),
                    "before": before.get(str(prim.GetPath())),
                    "external_forces_every_iteration": api.GetEnableExternalForcesEveryIterationAttr().Get(),
                    "time_steps_per_second": api.GetTimeStepsPerSecondAttr().Get(),
                    "solver_type": api.GetSolverTypeAttr().Get(),
                })
    if not scenes or any(row["external_forces_every_iteration"] is not True for row in scenes):
        raise RuntimeError("Session-only TGS force scheduling override was not applied to every scene")

    physics_dt_s = float(SimulationManager.get_physics_dt())
    if abs(physics_dt_s - PHYSICS_DT_S) > 1e-8:
        raise RuntimeError(f"Physics timestep mismatch: {physics_dt_s} != {PHYSICS_DT_S}")
    payload = {
        "configuration_id": CONFIGURATION_ID,
        "reference": PHYSX_REFERENCE,
        "requested_physics_dt_s": PHYSICS_DT_S,
        "resolved_physics_dt_s": physics_dt_s,
        "enable_external_forces_every_iteration": True,
        "solver_iteration_counts_modified": False,
        "override_layer": "session",
        "root_layer_identifier": source_layer.identifier,
        "session_layer_identifier": session.identifier,
        "session_layer_sha256": hashlib.sha256(session.ExportToString().encode("utf-8")).hexdigest(),
        "source_stage": stage_path,
        "source_stage_sha256_at_open": source_sha256,
        "model_file_saved": False,
        "scenes": scenes,
    }
    output.mkdir(parents=True, exist_ok=True)
    provenance = output / f"{tag}_physics_config.json"
    provenance.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print("PHYSICS_CONFIG " + json.dumps(payload, ensure_ascii=False), flush=True)
    return payload
