import os
from pathlib import Path
import asyncio

import omni.kit.app
import omni.usd
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport, next_viewport_frame_async
from pxr import Sdf

ASSET = os.path.join(os.getenv("CALIBRATION_SIM_HOME", os.path.expanduser("~")), "zhifeng_sim2real_20261113/zhifeng_robot_calibrated.usda")
OUT = os.path.join(os.getenv("CALIBRATION_SIM_HOME", os.path.expanduser("~")), "isaac_import_zjx_20261117/renders/智锋机器人_源场景视角.png")

async def main():
    ctx = omni.usd.get_context()
    ctx.open_stage(ASSET)
    for _ in range(50):
        await omni.kit.app.get_app().next_update_async()
    viewport = get_active_viewport()
    viewport.camera_path = Sdf.Path("/World/OverviewCamera")
    await next_viewport_frame_async(viewport, n_frames=50)
    helper = capture_viewport_to_file(viewport, file_path=OUT)
    await helper.wait_for_result(completion_frames=60)
    print(f"SOURCE_ROBOT_CAPTURE_DONE|{OUT}", flush=True)
    omni.kit.app.get_app().post_quit()

asyncio.ensure_future(main())
