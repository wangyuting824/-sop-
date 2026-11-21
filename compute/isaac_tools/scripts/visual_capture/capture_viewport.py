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


ROOT = Path(os.path.join(os.getenv("CALIBRATION_SIM_HOME", os.path.expanduser("~")), "isaac_import_zjx_20261117"))
ASSET = ROOT / "usd" / "办公室场地.usdc"
OUTPUT = ROOT / "renders" / "办公室场地_视口验证.png"


async def capture():
    ctx = omni.usd.get_context()
    ctx.open_stage(str(ASSET))
    for _ in range(20):
        await omni.kit.app.get_app().next_update_async()

    stage = ctx.get_stage()
    prims = [p.GetPath().pathString for p in stage.GetPseudoRoot().GetChildren()
             if p.GetTypeName() == "ParticleField3DGaussianSplat"]
    if not prims:
        raise RuntimeError("Gaussian Splat prim was not found after opening the USD")

    viewport = get_active_viewport()
    frame_viewport_prims(viewport, prims=prims)
    await next_viewport_frame_async(viewport, n_frames=40)
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    helper = capture_viewport_to_file(viewport, file_path=str(OUTPUT))
    await helper.wait_for_result(completion_frames=60)
    print(f"VIEWPORT_CAPTURE_DONE|{OUTPUT}", flush=True)
    omni.kit.app.get_app().post_quit()


asyncio.ensure_future(capture())
