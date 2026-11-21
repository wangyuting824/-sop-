import os
from pathlib import Path
import asyncio

import omni.kit.app
import omni.usd
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport, next_viewport_frame_async
from pxr import Gf, Sdf, Usd, UsdGeom


ROOT = Path(os.path.join(os.getenv("CALIBRATION_SIM_HOME", os.path.expanduser("~")), "isaac_import_zjx_20261117"))
OFFICE = ROOT / "usd" / "办公室场地.usdc"
ROBOT = Path(os.path.join(os.getenv("CALIBRATION_SIM_HOME", os.path.expanduser("~")), "zhifeng_sim2real_20261113/zhifeng_robot_calibrated.usda"))
SCENE = ROOT / "办公室场地_智锋机器人验证_v2.usda"
IMAGE = ROOT / "renders" / "办公室场地_智锋机器人正面验证.png"


def set_camera(stage, viewport, eye, target):
    camera = UsdGeom.Camera.Define(stage, Sdf.Path("/World/ValidationCamera"))
    camera.GetFocalLengthAttr().Set(30.0)
    camera.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 10000.0))
    view = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0.0, 1.0, 0.0))
    UsdGeom.Xformable(camera).MakeMatrixXform().Set(view.GetInverse())
    viewport.camera_path = camera.GetPath()


async def main():
    stage = Usd.Stage.CreateNew(str(SCENE))
    world = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    stage.SetDefaultPrim(world.GetPrim())

    scene_ref = UsdGeom.Xform.Define(stage, Sdf.Path("/World/ScannedScene"))
    scene_ref.GetPrim().GetReferences().AddReference(str(OFFICE), Sdf.Path("/_______________"))
    UsdGeom.Xformable(scene_ref).AddRotateXOp().Set(-90.0)

    robot_ref = UsdGeom.Xform.Define(stage, Sdf.Path("/World/ZhifengRobot"))
    robot_ref.GetPrim().GetReferences().AddReference(str(ROBOT), Sdf.Path("/World/Robot"))
    robot_xform = UsdGeom.Xformable(robot_ref)
    # Align robot axes with the corrected scan and place it beside the green workstation.
    robot_xform.AddTranslateOp(opSuffix="placement").Set(Gf.Vec3d(4.65, -0.30, -0.45))
    robot_xform.AddRotateXOp(opSuffix="placement").Set(-90.0)
    robot_xform.AddRotateZOp(opSuffix="placement").Set(180.0)

    stage.GetRootLayer().Save()
    ctx = omni.usd.get_context()
    ctx.open_stage(str(SCENE))
    for _ in range(40):
        await omni.kit.app.get_app().next_update_async()

    stage = ctx.get_stage()
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    scene_box = cache.ComputeWorldBound(stage.GetPrimAtPath("/World/ScannedScene")).ComputeAlignedRange()
    robot_box = cache.ComputeWorldBound(stage.GetPrimAtPath("/World/ZhifengRobot")).ComputeAlignedRange()
    if scene_box.IsEmpty() or robot_box.IsEmpty():
        raise RuntimeError("Scene or robot reference has no drawable bound")
    print(f"COMPOSE_BOUNDS|scene={tuple(scene_box.GetMin())},{tuple(scene_box.GetMax())}|robot={tuple(robot_box.GetMin())},{tuple(robot_box.GetMax())}", flush=True)

    mn, mx = scene_box.GetMin(), scene_box.GetMax()
    target = ((mn[0]+mx[0])/2, (mn[1]+mx[1])/2 - 0.10, (mn[2]+mx[2])/2)
    distance = max(mx[0]-mn[0], mx[2]-mn[2]) * 1.12
    eye = (target[0], target[1] + 1.05, target[2] - distance)
    viewport = get_active_viewport()
    set_camera(stage, viewport, eye, target)
    await next_viewport_frame_async(viewport, n_frames=60)
    IMAGE.parent.mkdir(parents=True, exist_ok=True)
    helper = capture_viewport_to_file(viewport, file_path=str(IMAGE))
    await helper.wait_for_result(completion_frames=60)
    print(f"COMPOSE_CAPTURE_DONE|{SCENE}|{IMAGE}", flush=True)
    omni.kit.app.get_app().post_quit()


asyncio.ensure_future(main())
