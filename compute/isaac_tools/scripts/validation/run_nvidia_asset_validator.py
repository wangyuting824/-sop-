#!/usr/bin/env python3
import asyncio
import json
import os
import sys

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.kit.app

manager = omni.kit.app.get_app().get_extension_manager()
manager.set_extension_enabled_immediate("isaacsim.asset.validation", True)
manager.set_extension_enabled_immediate("isaacsim.asset.validation.ui", True)
for _ in range(20):
    simulation_app.update()

from isaacsim.asset.validation.ui.controller import ValidationController
from isaacsim.asset.validation.ui.profile_resolver import ProfileResolver
from usd_validation_nvidia._engine import ValidationEngine
from usd_validation_nvidia._requirements import RequirementsRegistry
from pxr import Usd


def safe_value(obj, name):
    value = getattr(obj, name, None)
    if callable(value):
        try:
            value = value()
        except Exception:
            return None
    if hasattr(value, "value"):
        value = value.value
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    return str(value)


async def validate(asset_path, output_path):
    stage = Usd.Stage.Open(asset_path)
    if not stage:
        raise RuntimeError(f"cannot open USD: {asset_path}")
    root = stage.GetDefaultPrim()
    if root:
        physics = root.GetVariantSets().GetVariantSet("Physics")
        if physics and "physx" in physics.GetVariantNames():
            physics.SetVariantSelection("physx")
    stage.Load()

    resolver = ProfileResolver()
    profile = resolver.resolve("Robot-Body-Isaac", "1.1.0")
    controller = ValidationController()
    controller.set_profile(profile)
    controller.set_asset(stage)
    registry = RequirementsRegistry()
    engine = ValidationEngine(init_rules=False, variants=False, instance_prototypes=False)
    for requirement in profile.requirements:
        if registry.is_implemented(requirement):
            engine.enable_requirement(requirement)
    validated = None

    def on_validated(results):
        nonlocal validated
        validated = results

    await engine.validate_with_callbacks(
        asset=stage,
        asset_progress_fn=lambda _progress: None,
        asset_validated_fn=on_validated,
    )
    issues = tuple(validated.issues) if validated is not None else ()
    controller.results = controller._build_results(issues)

    requirements = []
    totals = {"Pass": 0, "Fail": 0, "Warning": 0, "Error": 0, "Not implemented": 0}
    for item in controller.results:
        status = item.status.value
        totals[status] = totals.get(status, 0) + 1
        issues = []
        for issue in item.issues:
            issues.append({
                "severity": safe_value(issue, "severity"),
                "message": safe_value(issue, "message"),
                "at": safe_value(issue, "at"),
                "identifier": safe_value(issue, "identifier"),
                "rule": safe_value(issue, "rule"),
            })
        requirements.append({
            "feature_id": item.feature_id,
            "feature_name": item.feature_name,
            "optional_feature": item.optional_feature,
            "code": item.code,
            "name": item.name,
            "status": status,
            "category": item.category,
            "rule_name": item.rule_name,
            "guidance": item.guidance,
            "documentation_path": item.documentation_path,
            "issues": issues,
        })

    report = {
        "validator": "NVIDIA Isaac Sim SimReady Asset Validation UI backend",
        "profile": {"id": profile.definition.profile_id, "version": profile.definition.version},
        "asset": asset_path,
        "physics_variant": "physx",
        "totals": totals,
        "controller_error": controller.error_message,
        "requirements": requirements,
    }
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)
    print(json.dumps({"profile": report["profile"], "totals": totals, "controller_error": controller.error_message, "output": output_path}, ensure_ascii=False, indent=2))
    controller.cleanup()


def main():
    if len(sys.argv) != 3:
        raise SystemExit("usage: run_nvidia_asset_validator.py INPUT_USD OUTPUT_JSON")
    task = asyncio.ensure_future(validate(os.path.abspath(sys.argv[1]), os.path.abspath(sys.argv[2])))
    while not task.done():
        simulation_app.update()
    task.result()
    simulation_app.close()


if __name__ == "__main__":
    main()
