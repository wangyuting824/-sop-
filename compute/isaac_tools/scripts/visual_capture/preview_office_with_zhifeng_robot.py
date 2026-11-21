import os
from pathlib import Path
import asyncio

import omni.kit.app
import omni.usd
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport, next_viewport_frame_async
from pxr import Gf, Sdf, Usd, UsdGeom


ROOT = Path(os.path.join(os.getenv("CALIBRATION_SIM_HOME", os.path.expanduser("~")), "isaac_import_zjx_20261117"))
OFFICE = ROOT / "usd" / "办公室场地.usdc"
ROBOT_CALIBRATION = Path(os.path.join(os.getenv("CALIBRATION_SIM_HOME", os.path.expanduser("~")), "zhifeng_sim2real_20261113/zhifeng_robot_calibrated.usda"))
IMAGE = ROOT / "renders" / "办公室场地_智锋机器人正面验证.png"


def set_camera(stage, viewport):
    camera = UsdGeom.Camera.Define(stage, Sdf.Path("/RobotValidationCamera"))
    camera.GetFocalLengthAttr().Set(30.0)
    camera.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 10000.0))
    # Same front-facing axis used for the approved scene view.
    eye = Gf.Vec3d(3.70, 2.05, -6.85)
    target = Gf.Vec3d(3.70, 1.02, 0.14)
    view = Gf.Matrix4d().SetLookAt(eye, target, Gf.Vec3d(0.0, 1.0, 0.0))
    UsdGeom.Xformable(camera).MakeMatrixXform().Set(view.GetInverse())
    viewport.camera_path = camera.GetPath()


async def main():
    ctx = omni.usd.get_context()
    ctx.open_stage(str(OFFICE))
    for _ in range(25):
        await omni.kit.app.get_app().next_update_async()

    stage = ctx.get_stage()
    stage.SetEditTarget(Usd.EditTarget(stage.GetSessionLayer()))
    scan_prim = next(p for p in stage.GetPseudoRoot().GetChildren()
                     if p.GetTypeName() == "ParticleField3DGaussianSplat")
    UsdGeom.Xformable(scan_prim).AddRotateXOp(opSuffix="validationCorrection").Set(-90.0)

    robot = UsdGeom.Xform.Define(stage, Sdf.Path("/ZhifengRobot"))
    # The calibrated asset already sublayers the base geometry. Referencing its World root avoids a duplicate robot.
    robot.GetPrim().GetReferences().AddReference(str(ROBOT_CALIBRATION), Sdf.Path("/World"))
    xform = UsdGeom.Xformable(robot)
    # Robot bound starts 1.23 m below the scan floor after axis conversion; lift it onto the floor.
    xform.AddTranslateOp(opSuffix="placement").Set(Gf.Vec3d(5.55, 0.93, 0.05))
    xform.AddRotateXOp(opSuffix="placement").Set(-90.0)
    xform.AddRotateZOp(opSuffix="placement").Set(180.0)

    for _ in range(50):
        await omni.kit.app.get_app().next_update_async()
    # Keep only the renderable robot; collision proxies otherwise appear as oversized grey solids.
    for prim in Usd.PrimRange(robot.GetPrim()):
        if "/collisions" in prim.GetPath().pathString:
            imageable = UsdGeom.Imageable(prim)
            if imageable:
                imageable.MakeInvisible()
    for _ in range(20):
        await omni.kit.app.get_app().next_update_async()
    box = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_]).ComputeWorldBound(robot.GetPrim()).ComputeAlignedRange()
    if box.IsEmpty():
        raise RuntimeError("Zhifeng robot has no drawable bound after insertion")
    print(f"ROBOT_INSERT_BOUND|min={tuple(box.GetMin())}|max={tuple(box.GetMax())}", flush=True)

    viewport = get_active_viewport()
    set_camera(stage, viewport)
    await next_viewport_frame_async(viewport, n_frames=70)
    IMAGE.parent.mkdir(parents=True, exist_ok=True)
    helper = capture_viewport_to_file(viewport, file_path=str(IMAGE))
    await helper.wait_for_result(completion_frames=60)
    print(f"ROBOT_INSERT_CAPTURE_DONE|{IMAGE}", flush=True)
    omni.kit.app.get_app().post_quit()


asyncio.ensure_future(main())
