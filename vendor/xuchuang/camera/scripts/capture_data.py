"""Pure image/feedback helpers; no ROS connection and no robot commands."""
from __future__ import annotations
import math
import numpy as np


def stamp_ns(header):
    stamp = header.get("stamp", {})
    return int(stamp.get("sec", 0)) * 1_000_000_000 + int(stamp.get("nanosec", 0))


def decode_color(message):
    """Respect ROS Image row stride. The original bytes are also saved separately."""
    import cv2
    channels = {"bgr8": 3, "rgb8": 3, "mono8": 1, "bgra8": 4, "rgba8": 4}
    encoding = message.encoding.lower()
    if encoding not in channels:
        raise ValueError(f"Unsupported color Image encoding: {encoding}")
    n = channels[encoding]
    if message.step < message.width * n or len(message.data) != message.height * message.step:
        raise ValueError("Image byte count/row stride does not match dimensions")
    rows = np.frombuffer(bytes(message.data), dtype=np.uint8).reshape(message.height, message.step)
    img = rows[:, :message.width * n].reshape(message.height, message.width, n).copy()
    if encoding == "bgr8":
        return img
    return cv2.cvtColor(img, {"rgb8": cv2.COLOR_RGB2BGR, "mono8": cv2.COLOR_GRAY2BGR,
                             "bgra8": cv2.COLOR_BGRA2BGR, "rgba8": cv2.COLOR_RGBA2BGR}[encoding])


def camera_matrix(image_header, width, height, info):
    if (width, height) != (info["width"], info["height"]):
        raise ValueError("Image and CameraInfo dimensions differ")
    if not image_header.get("frame_id") or image_header["frame_id"] != info["header"]["frame_id"]:
        raise ValueError("Image and CameraInfo frame IDs differ or are empty")
    if info["distortion_model"] not in ("plumb_bob", "rational_polynomial"):
        raise ValueError(f"Unsupported distortion model: {info['distortion_model']}; raw data retained")
    if info.get("binning_x", 0) > 1 or info.get("binning_y", 0) > 1 or any(info.get("roi", {}).get(k, 0) for k in ("x_offset", "y_offset")):
        raise ValueError("Cropped/binned CameraInfo needs an explicit intrinsic conversion")
    K, D = np.asarray(info["k"], dtype=float).reshape(3, 3), np.asarray(info["d"], dtype=float)
    if not np.isfinite(K).all() or not np.isfinite(D).all() or K[0, 0] <= 0 or K[1, 1] <= 0:
        raise ValueError("CameraInfo has invalid intrinsics")
    return K, D


def normalized_positions(message, unit):
    names, values = message["name"], message["position"]
    if len(names) != len(values) or len(set(names)) != len(names):
        raise ValueError("JointState names/positions invalid")
    scale = math.pi / 180 if unit == "deg" else 1.0
    result = {name: float(value) * scale for name, value in zip(names, values)}
    if not all(math.isfinite(v) for v in result.values()):
        raise ValueError("Non-finite joint positions")
    return result


def pair_feedback(records, image_ns, arm, unit, sync_s=0.1, stable_s=1.0):
    """Pair original ROS timestamps. Never estimate a clock offset from receipt time."""
    if image_ns <= 0:
        raise ValueError("Image timestamp is zero/invalid")
    names = [f"{arm}_joint_{i}" for i in range(7)]
    usable = []
    for record in records:
        msg = record["message"]
        if stamp_ns(msg["header"]) <= 0:
            continue
        q = normalized_positions(msg, unit)
        if all(name in q for name in names):
            usable.append((record, q))
    if not usable:
        raise ValueError("Waiting for complete /manipulator/joint_states")
    nearest, q = min(usable, key=lambda item: abs(stamp_ns(item[0]["message"]["header"]) - image_ns))
    error_s = abs(stamp_ns(nearest["message"]["header"]) - image_ns) / 1e9
    if error_s > sync_s:
        raise ValueError(f"Image/joint stamp difference {error_s:.3f}s > {sync_s}s; no clock offset invented")
    history = [(r, qv) for r, qv in usable if image_ns - (stable_s + .15)*1e9 <= stamp_ns(r["message"]["header"]) <= image_ns + sync_s*1e9]
    stamps = [stamp_ns(r["message"]["header"]) for r, _ in history]
    if len(stamps) < 5 or (max(stamps) - min(stamps)) / 1e9 < stable_s * .8:
        raise ValueError(f"Need {stable_s}s of stopped joint feedback before saving")
    span = float(np.max(np.ptp([[qv[name] for name in names] for _, qv in history], axis=0)))
    if span > .003:
        raise ValueError(f"Robot still moving: joint span {span:.5f}rad; stop then save")
    velocities = []
    scale = math.pi/180 if unit == "deg" else 1.
    for record, _ in history:
        msg = record["message"]
        if len(msg["velocity"]) == len(msg["name"]):
            values = dict(zip(msg["name"], msg["velocity"]))
            velocities.extend(float(values[name])*scale for name in names)
    peak = max(map(abs, velocities)) if velocities and all(math.isfinite(v) for v in velocities) else None
    if peak is not None and peak > .01:
        raise ValueError(f"Robot still moving: velocity {peak:.5f}rad/s; stop then save")
    return {"raw": nearest, "q_rad": q, "sync_error_s": error_s, "clock_offset_applied_s": 0,
            "stability": {"window_s": stable_s, "position_span_rad": span, "peak_velocity_rad_s": peak}}
