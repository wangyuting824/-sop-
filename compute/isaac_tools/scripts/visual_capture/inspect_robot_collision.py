import os
import asyncio
import omni.kit.app
import omni.usd
from pxr import UsdPhysics

ASSET = os.path.join(os.getenv("CALIBRATION_SIM_HOME", os.path.expanduser("~")), "zhifeng_sim2real_20261113/zhifeng_robot_calibrated.usda")

async def main():
    ctx = omni.usd.get_context()
    ctx.open_stage(ASSET)
    for _ in range(40):
        await omni.kit.app.get_app().next_update_async()
    stage = ctx.get_stage()
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            print(f"COLLISION|{prim.GetPath()}|{prim.GetTypeName()}", flush=True)
    omni.kit.app.get_app().post_quit()

asyncio.ensure_future(main())
