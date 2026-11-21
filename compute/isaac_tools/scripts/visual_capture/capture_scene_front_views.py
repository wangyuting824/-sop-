import os
from pathlib import Path
import asyncio

import omni.kit.app
import omni.usd
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport, next_viewport_frame_async
from pxr import Gf, Sdf, UsdGeom


ROOT = Path(os.path.join(os.getenv("CALIBRATION_SIM_HOME", os.path.expanduser("~")), "isaac_import_zjx_20261117"))
ASSET = ROOT / "usd" / "办公室场地.usdc"
OUT = ROOT / "renders"


def set_camera(stage, viewport, eye, target):
    camera = UsdGeom.Camera.Define(stage, Sdf.Path("/ValidationFrontCamera"))
    camera.GetFocalLengthAttr().Set(28.0)
    camera.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 10000.0))
    view = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0.0, 1.0, 0.0))
    UsdGeom.Xformable(camera).MakeMatrixXform().Set(view.GetInverse())
    viewport.camera_path = camera.GetPath()


async def main():
    ctx = omni.usd.get_context()
    ctx.open_stage(str(ASSET))
    for _ in range(20):
        await omni.kit.app.get_app().next_update_async()

    stage = ctx.get_stage()
    prims = [p for p in stage.GetPseudoRoot().GetChildren() if p.GetTypeName() == "ParticleField3DGaussianSplat"]
    if not prims:
        raise RuntimeError("Gaussian Splat prim not found")
    for prim in prims:
        UsdGeom.Xformable(prim).AddRotateXOp().Set(-90.0)

    cache = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_])
    box = cache.ComputeWorldBound(prims[0]).ComputeAlignedRange()
    mn, mx = box.GetMin(), box.GetMax()
    cx, cy, cz = ((mn[0]+mx[0])/2, (mn[1]+mx[1])/2, (mn[2]+mx[2])/2)
    xspan, zspan = mx[0]-mn[0], mx[2]-mn[2]
    distance = max(xspan, zspan) * 1.05
    target = (cx, cy - 0.15, cz)
    viewport = get_active_viewport()
    OUT.mkdir(parents=True, exist_ok=True)

    for label, sign in (("正面", -1.0), ("背面", 1.0)):
        eye = (cx, cy + 1.0, cz + sign * distance)
        set_camera(stage, viewport, eye, target)
        await next_viewport_frame_async(viewport, n_frames=50)
        output = OUT / f"办公室场地_{label}近景.png"
        helper = capture_viewport_to_file(viewport, file_path=str(output))
        await helper.wait_for_result(completion_frames=60)
        print(f"FRONT_CAPTURE_DONE|{label}|{output}", flush=True)
    omni.kit.app.get_app().post_quit()


asyncio.ensure_future(main())
