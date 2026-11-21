#!/usr/bin/env python3
import json
import math
import os
import sys

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

from pxr import Sdf, Usd, UsdGeom, UsdPhysics


def authored(attr):
    return bool(attr and attr.HasAuthoredValueOpinion())


def main():
    if len(sys.argv) != 3:
        raise SystemExit("usage: validate_usd_stage.py INPUT_USD OUTPUT_JSON")
    input_usd, output_json = map(os.path.abspath, sys.argv[1:])
    stage = Usd.Stage.Open(input_usd)
    if not stage:
        raise RuntimeError(f"cannot open stage: {input_usd}")

    root_prim = stage.GetDefaultPrim()
    physics_variant = None
    if root_prim:
        variant_set = root_prim.GetVariantSets().GetVariantSet("Physics")
        if variant_set and "physx" in variant_set.GetVariantNames():
            variant_set.SetVariantSelection("physx")
            physics_variant = "physx"
    stage.Load()

    prims = list(stage.TraverseAll())
    prototype_prims = []
    for prototype in stage.GetPrototypes():
        prototype_prims.extend(list(Usd.PrimRange(prototype)))
    inspection_prims = prims + prototype_prims
    joints = []
    rigid_bodies = []
    collision_prims = []
    meshes = []
    mass_prims = []
    articulation_roots = []
    missing_layers = []

    for prim in inspection_prims:
        path = str(prim.GetPath())
        if prim.IsA(UsdGeom.Mesh):
            meshes.append(path)
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            rigid_bodies.append(path)
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            collision_prims.append(path)
        if prim.HasAPI(UsdPhysics.MassAPI):
            api = UsdPhysics.MassAPI(prim)
            mass_prims.append({
                "path": path,
                "mass": api.GetMassAttr().Get(),
                "center_of_mass": api.GetCenterOfMassAttr().Get(),
                "diagonal_inertia": api.GetDiagonalInertiaAttr().Get(),
                "principal_axes": api.GetPrincipalAxesAttr().Get(),
            })
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            articulation_roots.append(path)
        if prim.IsA(UsdPhysics.Joint):
            kind = prim.GetTypeName()
            row = {"path": path, "name": prim.GetName(), "type": kind}
            if prim.IsA(UsdPhysics.RevoluteJoint):
                j = UsdPhysics.RevoluteJoint(prim)
                row.update({
                    "axis": j.GetAxisAttr().Get(),
                    "lower_limit_deg": j.GetLowerLimitAttr().Get(),
                    "upper_limit_deg": j.GetUpperLimitAttr().Get(),
                })
            drive = UsdPhysics.DriveAPI.Get(prim, "angular")
            if drive:
                row.update({
                    "stiffness": drive.GetStiffnessAttr().Get(),
                    "damping": drive.GetDampingAttr().Get(),
                    "max_force": drive.GetMaxForceAttr().Get(),
                    "stiffness_authored": authored(drive.GetStiffnessAttr()),
                    "damping_authored": authored(drive.GetDampingAttr()),
                })
            joints.append(row)

    used_layers = []
    for layer in stage.GetUsedLayers():
        identifier = layer.identifier
        real_path = layer.realPath
        used_layers.append({"identifier": identifier, "real_path": real_path})
        if real_path and not os.path.exists(real_path):
            missing_layers.append({"identifier": identifier, "real_path": real_path})

    arm_joints = [j for j in joints if "_arm_joint_" in j["name"]]
    limit_groups = {}
    for j in arm_joints:
        key = (j.get("lower_limit_deg"), j.get("upper_limit_deg"))
        limit_groups[str(key)] = limit_groups.get(str(key), 0) + 1

    checks = {
        "stage_opened": True,
        "default_prim_present": bool(stage.GetDefaultPrim()),
        "exactly_14_arm_revolute_joints": len(arm_joints) == 14 and all(j["type"] == "PhysicsRevoluteJoint" for j in arm_joints),
        "meshes_present": len(meshes) > 0,
        "rigid_bodies_present": len(rigid_bodies) > 0,
        "collisions_present": len(collision_prims) > 0,
        "mass_properties_present": len(mass_prims) > 0,
        "articulation_root_present": len(articulation_roots) > 0,
        "external_references_resolve": len(missing_layers) == 0,
        "arm_gains_not_uniformly_overridden": not any(j.get("stiffness_authored") or j.get("damping_authored") for j in arm_joints),
        "arm_joint_limits_not_all_identical": len(limit_groups) > 1,
    }

    report = {
        "input_usd": input_usd,
        "physics_variant": physics_variant,
        "stage_identifier": stage.GetRootLayer().identifier,
        "default_prim": str(stage.GetDefaultPrim().GetPath()) if stage.GetDefaultPrim() else None,
        "up_axis": UsdGeom.GetStageUpAxis(stage),
        "meters_per_unit": UsdGeom.GetStageMetersPerUnit(stage),
        "counts": {
            "prims": len(prims),
            "prototype_prims": len(prototype_prims),
            "meshes": len(meshes),
            "joints_total": len(joints),
            "arm_joints": len(arm_joints),
            "rigid_bodies": len(rigid_bodies),
            "collision_prims": len(collision_prims),
            "mass_prims": len(mass_prims),
            "articulation_roots": len(articulation_roots),
        },
        "checks": checks,
        "arm_joint_limit_groups": limit_groups,
        "arm_joints": arm_joints,
        "articulation_roots": articulation_roots,
        "used_layers": used_layers,
        "missing_layers": missing_layers,
        "mass_prims": mass_prims,
    }
    os.makedirs(os.path.dirname(output_json), exist_ok=True)
    with open(output_json, "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2, default=str)
    print(json.dumps({"counts": report["counts"], "checks": checks, "output": output_json}, ensure_ascii=False, indent=2))
    simulation_app.close()


if __name__ == "__main__":
    main()
