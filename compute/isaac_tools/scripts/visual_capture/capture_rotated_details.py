import os
from pathlib import Path
import asyncio

import omni.kit.app
import omni.usd
from omni.kit.viewport.utility import (
    capture_viewport_to_file,
    frame_viewport_prims,
    get_active_viewport,
    next_viewport_frame_async,
)
from pxr import UsdGeom


ROOT = Path(os.path.join(os.getenv("CALIBRATION_SIM_HOME", os.path.expanduser("~")), "isaac_import_zjx_20261117"))
OUTPUT = ROOT / "renders"
ASSETS = [
    ("办公室场地.usdc", "办公室场地_旋转90度远景.png"),
    ("桌子01.usdc", "桌子01_旋转90度近景.png"),
    ("桌子02.usdc", "桌子02_旋转90度近景.png"),
    ("托盘.usdc", "托盘_旋转90度近景.png"),
]


async def capture_asset(asset_name, image_name):
    ctx = omni.usd.get_context()
    ctx.open_stage(str(ROOT / "usd" / asset_name))
    for _ in range(20):
        await omni.kit.app.get_app().next_update_async()

    stage = ctx.get_stage()
    prims = [p for p in stage.GetPseudoRoot().GetChildren()
             if p.GetTypeName() == "ParticleField3DGaussianSplat"]
    if not prims:
        raise RuntimeError(f"Gaussian Splat prim not found: {asset_name}")

    # Temporary in-memory correction only: rotate the scanned asset 90° around the Z (up) axis.
    for prim in prims:
        xform = UsdGeom.Xformable(prim)
        xform.AddRotateZOp().Set(90.0)

    viewport = get_active_viewport()
    frame_viewport_prims(viewport, prims=[p.GetPath().pathString for p in prims])
    await next_viewport_frame_async(viewport, n_frames=40)
    output = OUTPUT / image_name
    helper = capture_viewport_to_file(viewport, file_path=str(output))
    await helper.wait_for_result(completion_frames=60)
    print(f"CAPTURE_DONE|{asset_name}|{output}", flush=True)


async def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    for asset_name, image_name in ASSETS:
        await capture_asset(asset_name, image_name)
    omni.kit.app.get_app().post_quit()


asyncio.ensure_future(main())
