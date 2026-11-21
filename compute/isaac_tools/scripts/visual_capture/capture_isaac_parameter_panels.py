from pathlib import Path
import os
import subprocess
import struct
import time

try:
    from isaacsim import SimulationApp
except ImportError:
    from omni.isaac.kit import SimulationApp


OUT = Path(os.environ.get("ZHIFENG_CAPTURE_OUT", os.path.join(os.getenv("CALIBRATION_SIM_HOME", os.path.expanduser("~")), "zhifeng_parameter_screenshots_20261116")))
OUT.mkdir(parents=True, exist_ok=True)

app = SimulationApp(
    {
        "headless": False,
        "width": 2560,
        "height": 1440,
        "window_width": 3840,
        "window_height": 2160,
        "window_title": "Isaac Sim 6.1.0 - Zhifeng Robot Parameters",
    }
)

import omni.ui as ui
import omni.usd
import carb
from omni.kit.viewport.utility import get_active_viewport
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade


STAGE = os.path.join(os.getenv("CALIBRATION_SIM_HOME", os.path.expanduser("~")), "xuchuang_sysid_20261113/compute_workspace/xuchuang_response_work/sysid_20261113/zhifeng_selected_pd_20261114_v7.usda")
omni.usd.get_context().open_stage(STAGE)
for _ in range(240):
    app.update()


def maximize_isaac_window():
    """Maximize the real Isaac window so the root screenshot contains no desktop padding."""
    try:
        ids = subprocess.check_output(
            ["xdotool", "search", "--onlyvisible", "--name", "Isaac Sim"],
            text=True,
        ).split()
        if ids:
            window_id = ids[-1]
            subprocess.run(["xdotool", "windowactivate", "--sync", window_id], check=True)
            subprocess.run(["xdotool", "key", "--window", window_id, "alt+F10"], check=True)
            time.sleep(2)
    except Exception as exc:
        print(f"Window maximize warning: {exc}", flush=True)


def position_report_camera():
    """Frame the complete robot from a natural three-quarter view."""
    stage = omni.usd.get_context().get_stage()
    robot = stage.GetPrimAtPath("/World/Robot")
    if not robot.IsValid():
        raise RuntimeError("Robot prim /World/Robot was not found")

    bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
        useExtentsHint=True,
    )
    box = bbox_cache.ComputeWorldBound(robot).ComputeAlignedBox()
    minimum = box.GetMin()
    maximum = box.GetMax()
    center = (minimum + maximum) * 0.5
    size = maximum - minimum
    radius = max(float(size[0]), float(size[1]), float(size[2]), 1.0)

    # Close three-quarter view: prioritize the upper body and both arms for the report.
    eye = Gf.Vec3d(
        float(center[0]) + radius * 0.75,
        float(center[1]) - radius * 1.35,
        float(center[2]) + radius * 0.75,
    )
    target = Gf.Vec3d(float(center[0]), float(center[1]), float(center[2]) + radius * 0.18)

    camera_path = Sdf.Path("/World/ReportCamera")
    camera = UsdGeom.Camera.Define(stage, camera_path)
    camera.GetFocalLengthAttr().Set(42.0)
    camera.GetHorizontalApertureAttr().Set(36.0)
    camera.GetVerticalApertureAttr().Set(20.25)
    xformable = UsdGeom.Xformable(camera.GetPrim())
    xformable.ClearXformOpOrder()
    world_from_camera = Gf.Matrix4d().SetLookAt(eye, target, Gf.Vec3d(0.0, 0.0, 1.0)).GetInverse()
    xformable.AddTransformOp().Set(world_from_camera)

    viewport = get_active_viewport()
    viewport.camera_path = camera_path
    print(
        f"Report camera framed robot: min={tuple(minimum)}, max={tuple(maximum)}, "
        f"eye={tuple(eye)}, target={tuple(target)}",
        flush=True,
    )
    return center, radius, minimum


def create_preview_material(stage, path, color, metallic, roughness, emissive=(0.0, 0.0, 0.0)):
    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, f"{path}/PreviewSurface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(metallic))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(roughness))
    shader.CreateInput("specularColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.55, 0.62, 0.75))
    shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*emissive))
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def style_report_scene(center, radius, minimum):
    """Give the report view a cleaner blue robot while preserving the stable Isaac viewport."""
    stage = omni.usd.get_context().get_stage()
    robot = stage.GetPrimAtPath("/World/Robot")

    robot_material = create_preview_material(
        stage,
        "/World/ReportMaterials/RobotPearlBlue",
        (0.16, 0.46, 0.98),
        0.08,
        0.34,
        (0.055, 0.16, 0.38),
    )

    for prim in Usd.PrimRange(robot):
        if prim.IsA(UsdGeom.Mesh):
            binding_api = UsdShade.MaterialBindingAPI.Apply(prim)
            binding_api.UnbindAllBindings()
            binding_api.Bind(
                robot_material,
                UsdShade.Tokens.strongerThanDescendants,
            )

    # Keep Isaac Sim's stable neutral viewport; the blue emissive tint supplies contrast.


maximize_isaac_window()
report_center, report_radius, report_minimum = position_report_camera()
for _ in range(180):
    app.update()


PANELS = [
    (
        "01_mass_center_inertia.png",
        "Mass and Inertia - part_021",
        "/World/Robot/part_021",
        [
            ("Physics Mass [kg]", "0.500000"),
            ("Center of Mass [m]", "(0.00004015, -0.00000021, 0.15461245)"),
            ("Diagonal Inertia [kg*m^2]", "(0.00139018, 0.00151560, 0.00094930)"),
            ("Principal Axes [wxyz]", "(1.0, 0.0, 0.0, 0.0)"),
        ],
        "Rigid-body values read from the composed robot asset.",
    ),
    (
        "02_joint_drive.png",
        "Angular Drive - right_arm_joint_1",
        "/World/Robot/joints/right_arm_joint_1",
        [
            ("Drive Type", "acceleration"),
            ("Stiffness", "1396.9261"),
            ("Damping", "10.39681"),
            ("Max Force", "20.0"),
        ],
        "Final effective gains selected by the SysID replay fit.",
    ),
    (
        "03_joint_axis_limits.png",
        "Joint Axis and Limits - right_arm_joint_7",
        "/World/Robot/joints/right_arm_joint_7",
        [
            ("Joint Type", "Revolute"),
            ("Axis", "Z"),
            ("Lower Limit [degree]", "-19.999998"),
            ("Upper Limit [degree]", "19.999998"),
        ],
        "Axis and travel limits inherited from the robot model.",
    ),
    (
        "04_camera_parameters.png",
        "Camera Parameters - left_wrist_camera",
        "/World/Robot/part_027/left_flange/left_wrist_camera",
        [
            ("Translate [m]", "(0.00340652, -0.06573009, 0.10893155)"),
            ("Orientation [wxyz]", "(0.40079019, 0.91604829, -0.01217046, -0.00863892)"),
            ("Focal Length [mm]", "13.265205"),
            ("Clipping Range [m]", "(0.01, 1000.0)"),
            ("Resolution", "640 x 480"),
        ],
        "Calibrated transform and camera model written to the USD camera prim.",
    ),
    (
        "05_physics_scene.png",
        "Physics Scene",
        "/World/PhysicsScene",
        [
            ("Gravity Direction", "(0.0, 0.0, -1.0)"),
            ("Gravity Magnitude [m/s^2]", "9.81"),
            ("Time Steps Per Second", "60"),
            ("Physics Time Step [s]", "0.016666667"),
            ("Solver Type", "TGS"),
            ("External Forces Every Iteration", "True"),
        ],
        "Physics settings used for the final command replay and validation.",
    ),
]


def build_panel(title, prim_path, rows, note):
    with ui.VStack(spacing=16, height=0):
        ui.Spacer(height=16)
        ui.Label(title, height=62, style={"font_size": 38, "color": 0xFFF4C95D})
        ui.Label(prim_path, height=34, style={"font_size": 20, "color": 0xFFB8C2CC})
        ui.Separator(height=2)
        ui.Spacer(height=12)
        with ui.HStack(height=34):
            ui.Label("PARAMETER", width=520, style={"font_size": 17, "color": 0xFF8996A5})
            ui.Label("VALUE", style={"font_size": 17, "color": 0xFF8996A5})
        ui.Separator(height=1)
        for label, value in rows:
            with ui.HStack(height=76):
                ui.Label(label, width=520, style={"font_size": 24, "color": 0xFFD7DEE7})
                ui.Label(value, style={"font_size": 29, "color": 0xFF6BD6FF})
        ui.Spacer(height=20)
        ui.Separator(height=2)
        ui.Label(note, height=54, word_wrap=True, style={"font_size": 17, "color": 0xFFAEB7C2})


def capture_root(path):
    xwd = path.with_suffix(".xwd")
    subprocess.run(["xwd", "-root", "-silent", "-out", str(xwd)], check=True)
    from PIL import Image

    raw = xwd.read_bytes()
    header = struct.unpack(">25I", raw[:100])
    header_size = header[0]
    width, height = header[4], header[5]
    byte_order = header[7]
    bytes_per_line = header[12]
    ncolors = header[19]
    pixel_data = raw[header_size + ncolors * 12 : header_size + ncolors * 12 + bytes_per_line * height]
    if byte_order != 0 or bytes_per_line != width * 4:
        raise RuntimeError(f"Unexpected XWD layout: byte_order={byte_order}, stride={bytes_per_line}, width={width}")
    Image.frombytes("RGB", (width, height), pixel_data, "raw", "BGRX", bytes_per_line, 1).save(path)
    xwd.unlink(missing_ok=True)


for filename, title, prim_path, rows, note in PANELS:
    window = ui.Window(
        "Zhifeng Robot - Parameter Inspector",
        width=1420,
        height=1080,
        position_x=2320,
        position_y=240,
        flags=ui.WINDOW_FLAGS_NO_COLLAPSE,
    )
    with window.frame:
        build_panel(title, prim_path, rows, note)
    window.visible = True
    for _ in range(120):
        app.update()
    capture_root(OUT / filename)
    window.visible = False
    for _ in range(30):
        app.update()

(OUT / "CAPTURE_COMPLETE.txt").write_text("5 parameter screenshots captured in Isaac Sim 6.1.0\n", encoding="utf-8")
app.close()
