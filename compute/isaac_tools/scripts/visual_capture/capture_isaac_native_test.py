from pathlib import Path
import ctypes
import os
import struct
import subprocess
import time

try:
    from isaacsim import SimulationApp
except ImportError:
    from omni.isaac.kit import SimulationApp


OUT = Path(os.environ.get("ZHIFENG_CAPTURE_OUT", os.path.join(os.getenv("CALIBRATION_SIM_HOME", os.path.expanduser("~")), "zhifeng_native_test_20261116")))
OUT.mkdir(parents=True, exist_ok=True)

app = SimulationApp(
    {
        "headless": False,
        "width": 1920,
        "height": 1200,
        "window_width": 2560,
        "window_height": 1600,
        "window_title": "Isaac Sim 6.1.0 - Zhifeng Robot",
        "extra_args": ["--/app/window/dpiScaleOverride=2.0"],
    }
)

import omni.ui as ui
import omni.usd
import omni.kit.app
from omni.kit.viewport.utility import get_active_viewport
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom


STAGE = os.path.join(os.getenv("CALIBRATION_SIM_HOME", os.path.expanduser("~")), "xuchuang_sysid_20261113/compute_workspace/xuchuang_response_work/sysid_20261113/zhifeng_selected_pd_20261114_v7.usda")
CAMERA_STAGE = os.path.join(os.getenv("CALIBRATION_SIM_HOME", os.path.expanduser("~")), "zhifeng_sim2real_20261113/zhifeng_robot_calibrated.usda")
omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate(
    "omni.kit.property.physics", True
)
for _ in range(120):
    app.update()
omni.usd.get_context().open_stage(STAGE)
for _ in range(240):
    app.update()


def maximize_isaac_window():
    """Request EWMH maximize without external xdotool/wmctrl dependencies."""

    class ClientMessageData(ctypes.Union):
        _fields_ = [
            ("b", ctypes.c_char * 20),
            ("s", ctypes.c_short * 10),
            ("l", ctypes.c_long * 5),
        ]

    class ClientMessageEvent(ctypes.Structure):
        _fields_ = [
            ("type", ctypes.c_int),
            ("serial", ctypes.c_ulong),
            ("send_event", ctypes.c_int),
            ("display", ctypes.c_void_p),
            ("window", ctypes.c_ulong),
            ("message_type", ctypes.c_ulong),
            ("format", ctypes.c_int),
            ("data", ClientMessageData),
        ]

    class XEvent(ctypes.Union):
        _fields_ = [("xclient", ClientMessageEvent), ("pad", ctypes.c_long * 24)]

    x11 = ctypes.cdll.LoadLibrary("libX11.so.6")
    x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
    x11.XOpenDisplay.restype = ctypes.c_void_p
    x11.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
    x11.XDefaultRootWindow.restype = ctypes.c_ulong
    x11.XInternAtom.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
    x11.XInternAtom.restype = ctypes.c_ulong
    x11.XGetWindowProperty.argtypes = [
        ctypes.c_void_p,
        ctypes.c_ulong,
        ctypes.c_ulong,
        ctypes.c_long,
        ctypes.c_long,
        ctypes.c_int,
        ctypes.c_ulong,
        ctypes.POINTER(ctypes.c_ulong),
        ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_ulong),
        ctypes.POINTER(ctypes.c_ulong),
        ctypes.POINTER(ctypes.POINTER(ctypes.c_ubyte)),
    ]
    x11.XFree.argtypes = [ctypes.c_void_p]
    x11.XSendEvent.argtypes = [
        ctypes.c_void_p,
        ctypes.c_ulong,
        ctypes.c_int,
        ctypes.c_long,
        ctypes.POINTER(XEvent),
    ]
    x11.XFlush.argtypes = [ctypes.c_void_p]
    x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
    display = x11.XOpenDisplay(None)
    if not display:
        return

    root = x11.XDefaultRootWindow(display)
    active_atom = x11.XInternAtom(display, b"_NET_ACTIVE_WINDOW", False)
    actual_type = ctypes.c_ulong()
    actual_format = ctypes.c_int()
    nitems = ctypes.c_ulong()
    bytes_after = ctypes.c_ulong()
    prop = ctypes.POINTER(ctypes.c_ubyte)()
    status = x11.XGetWindowProperty(
        display,
        root,
        active_atom,
        0,
        1,
        False,
        0,
        ctypes.byref(actual_type),
        ctypes.byref(actual_format),
        ctypes.byref(nitems),
        ctypes.byref(bytes_after),
        ctypes.byref(prop),
    )
    if status != 0 or not prop or nitems.value == 0:
        x11.XCloseDisplay(display)
        return

    window = ctypes.cast(prop, ctypes.POINTER(ctypes.c_ulong))[0]
    x11.XFree(prop)
    state_atom = x11.XInternAtom(display, b"_NET_WM_STATE", False)
    max_vert = x11.XInternAtom(display, b"_NET_WM_STATE_MAXIMIZED_VERT", False)
    max_horz = x11.XInternAtom(display, b"_NET_WM_STATE_MAXIMIZED_HORZ", False)
    event = XEvent()
    event.xclient.type = 33  # ClientMessage
    event.xclient.send_event = True
    event.xclient.display = display
    event.xclient.window = window
    event.xclient.message_type = state_atom
    event.xclient.format = 32
    event.xclient.data.l[0] = 1  # _NET_WM_STATE_ADD
    event.xclient.data.l[1] = max_vert
    event.xclient.data.l[2] = max_horz
    mask = (1 << 20) | (1 << 19)  # SubstructureRedirectMask | SubstructureNotifyMask
    x11.XSendEvent(display, root, False, mask, ctypes.byref(event))
    x11.XFlush(display)
    x11.XCloseDisplay(display)
    time.sleep(2)


def arrange_native_windows():
    viewport = ui.Workspace.get_window("Viewport")
    stage_window = ui.Workspace.get_window("Stage")
    property_window = ui.Workspace.get_window("Property")
    content_window = ui.Workspace.get_window("Content")
    console_window = ui.Workspace.get_window("Console")
    render_settings_window = ui.Workspace.get_window("Render Settings")
    layer_window = ui.Workspace.get_window("Layer")
    status_bar_window = ui.Workspace.get_window("Status Bar")

    if stage_window:
        stage_window.visible = True
        stage_window.dock_in(viewport, ui.DockPosition.RIGHT, 0.36)
    if property_window:
        property_window.visible = True
        if stage_window:
            property_window.dock_in(stage_window, ui.DockPosition.BOTTOM, 0.62)
        else:
            property_window.dock_in(viewport, ui.DockPosition.RIGHT, 0.30)
    if content_window:
        content_window.visible = True
        content_window.dock_in(viewport, ui.DockPosition.BOTTOM, 0.28)
    if console_window:
        console_window.visible = False
    if render_settings_window:
        render_settings_window.visible = False
    if layer_window:
        layer_window.visible = False
    if status_bar_window:
        status_bar_window.visible = False


def position_report_camera():
    stage = omni.usd.get_context().get_stage()
    robot = stage.GetPrimAtPath("/World/Robot")
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

    # Reference-like framing: whole robot, about two thirds of viewport height.
    eye = Gf.Vec3d(
        float(center[0]) + radius * 0.76,
        float(center[1]) - radius * 1.20,
        float(center[2]) + radius * 0.63,
    )
    target = Gf.Vec3d(
        float(center[0]),
        float(center[1]),
        float(center[2]) - radius * 0.02,
    )

    camera_path = Sdf.Path("/World/ReportCamera")
    camera = UsdGeom.Camera.Define(stage, camera_path)
    camera.GetFocalLengthAttr().Set(32.0)
    camera.GetHorizontalApertureAttr().Set(36.0)
    camera.GetVerticalApertureAttr().Set(20.25)
    xformable = UsdGeom.Xformable(camera.GetPrim())
    xformable.ClearXformOpOrder()
    xformable.AddTransformOp().Set(
        Gf.Matrix4d().SetLookAt(eye, target, Gf.Vec3d(0.0, 0.0, 1.0)).GetInverse()
    )
    get_active_viewport().camera_path = camera_path


def capture_root(path):
    xwd = path.with_suffix(".xwd")
    tree = subprocess.check_output(["xwininfo", "-root", "-tree"], text=True)
    candidates = []
    for line in tree.splitlines():
        if '"Isaac Sim Python 6.1.0"' in line:
            candidates.append(line.strip().split()[0])
    if not candidates:
        raise RuntimeError("Isaac Sim top-level window was not found")
    subprocess.run(
        ["xwd", "-id", candidates[-1], "-silent", "-out", str(xwd)], check=True
    )
    from PIL import Image

    raw = xwd.read_bytes()
    header = struct.unpack(">25I", raw[:100])
    header_size = header[0]
    width, height = header[4], header[5]
    byte_order = header[7]
    bytes_per_line = header[12]
    ncolors = header[19]
    pixel_data = raw[
        header_size + ncolors * 12 : header_size + ncolors * 12 + bytes_per_line * height
    ]
    if byte_order != 0 or bytes_per_line != width * 4:
        raise RuntimeError(
            f"Unexpected XWD layout: byte_order={byte_order}, stride={bytes_per_line}, width={width}"
        )
    Image.frombytes(
        "RGB", (width, height), pixel_data, "raw", "BGRX", bytes_per_line, 1
    ).save(path)
    xwd.unlink(missing_ok=True)


import omni.kit.window.property as property_window_api
import omni.kit.window.property.managed_frame as managed_frame
from omni.kit.property.physics import Manager
from omni.kit.property.physics.widgets import MainFrameWidget


def open_stage_and_frame(stage_path):
    omni.usd.get_context().open_stage(stage_path)
    for _ in range(240):
        app.update()
    arrange_native_windows()
    position_report_camera()
    for _ in range(120):
        app.update()


def select_and_scroll(prim_path, physics_title=None, camera=False, physics_main=False):
    try:
        window = property_window_api.get_window()
        window._on_search(None)
        window._rebuild_window()
    except Exception:
        pass
    omni.usd.get_context().get_selection().set_selected_prim_paths([prim_path], False)
    property_panel = ui.Workspace.get_window("Property")
    if property_panel:
        property_panel.focus()
    for _ in range(180):
        app.update()

    try:
        window = property_window_api.get_window()
        window._window_frame.scroll_y = 0
        if physics_title or physics_main:
            physics_widget = window._widgets_top[Manager.scheme][MainFrameWidget.name]
            physics_widget._collapsable_frame.collapsed = False
            if physics_title:
                target_widget = physics_widget.get_subwidget_by_title(physics_title)
                if target_widget is None:
                    raise RuntimeError(f"Physics widget not found: {physics_title}")
                target_widget._collapsable_frame.collapsed = False
                target_widget._collapsable_frame.scroll_here_y()
            else:
                physics_widget._collapsable_frame.scroll_here_y()
        elif camera:
            camera_widget = window._widgets_top["prim"]["camera"]
            camera_widget._collapsable_frame.collapsed = False
            camera_widget._collapsable_frame.scroll_here_y()
    except Exception as exc:
        print(f"Property scroll warning for {prim_path}: {exc}", flush=True)

    for _ in range(180):
        app.update()


def scroll_property_to_bottom():
    try:
        property_window_api.get_window()._window_frame.scroll_y = 100000
    except Exception as exc:
        print(f"Property bottom scroll warning: {exc}", flush=True)
    for _ in range(180):
        app.update()


def filter_property(search_words, expand_raw=False):
    try:
        window = property_window_api.get_window()
        window._on_search(search_words)
        if expand_raw:
            managed_frame.set_collapsed_state("Property/Raw USD Properties", False)
        window._rebuild_window()
        window._window_frame.scroll_y = 100000
    except Exception as exc:
        print(f"Property filter warning for {search_words}: {exc}", flush=True)
    for _ in range(240):
        app.update()


def scroll_to_last_enabled_physics_widget():
    try:
        window = property_window_api.get_window()
        physics_widget = window._widgets_top[Manager.scheme][MainFrameWidget.name]
        enabled = [sw.widget for sw in physics_widget.get_subwidgets() if sw.enabled]
        print(
            "Enabled Physics Scene widgets: "
            + ", ".join(getattr(widget, "_title", "?") for widget in enabled),
            flush=True,
        )
        if enabled:
            target = enabled[-1]
            target._collapsable_frame.collapsed = False
            target._collapsable_frame.scroll_here_y()
    except Exception as exc:
        print(f"Physics Scene detailed scroll warning: {exc}", flush=True)
    for _ in range(180):
        app.update()


arrange_native_windows()
position_report_camera()

select_and_scroll("/World/Robot/part_021", physics_title="Mass")
capture_root(OUT / "01_mass_center_inertia.png")

select_and_scroll(
    "/World/Robot/joints/right_arm_joint_1", physics_title="Revolute Joint"
)
capture_root(OUT / "02_joint_stiffness_damping_max_force.png")

select_and_scroll(
    "/World/Robot/joints/right_arm_joint_7", physics_title="Revolute Joint"
)
capture_root(OUT / "03_joint_axis_lower_upper_limits.png")

open_stage_and_frame(CAMERA_STAGE)
camera_path = "/World/Robot/part_027/left_flange/left_wrist_camera"
select_and_scroll(camera_path)
capture_root(OUT / "04a_camera_transform.png")
select_and_scroll(camera_path, camera=True)
capture_root(OUT / "04b_camera_focal_length.png")
filter_property(["Clipping"], expand_raw=True)
capture_root(OUT / "04c_camera_clipping_range.png")

open_stage_and_frame(STAGE)
scene_prim = omni.usd.get_context().get_stage().GetPrimAtPath("/World/PhysicsScene")
scene_api = PhysxSchema.PhysxSceneAPI.Apply(scene_prim)
scene_api.CreateSolverTypeAttr().Set("TGS")
scene_api.CreateTimeStepsPerSecondAttr().Set(60)
scene_api.CreateEnableExternalForcesEveryIterationAttr().Set(True)
select_and_scroll("/World/PhysicsScene", physics_main=True)
capture_root(OUT / "05a_physics_scene_gravity.png")
filter_property(["Solver"], expand_raw=True)
capture_root(OUT / "05b_physics_scene_solver.png")
filter_property(["Time", "Steps"], expand_raw=True)
capture_root(OUT / "05c_physics_scene_timestep.png")

(OUT / "CAPTURE_COMPLETE.txt").write_text(
    "8 native Isaac Sim parameter screenshots captured.\n", encoding="utf-8"
)
app.close()
