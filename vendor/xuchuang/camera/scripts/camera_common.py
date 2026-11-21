#!/usr/bin/env python3
"""Shared read-only helpers for serial-chain robot RealSense external-calibration capture."""
from __future__ import annotations

from pathlib import Path
import hashlib
import json
import math
import time
import xml.etree.ElementTree as ET

import cv2
import numpy as np


def stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def inverse(T: np.ndarray) -> np.ndarray:
    answer = np.eye(4)
    answer[:3, :3] = T[:3, :3].T
    answer[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return answer


def rpy_matrix(values: list[float]) -> np.ndarray:
    roll, pitch, yaw = values
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.asarray([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def axis_angle(axis: list[float], q: float) -> np.ndarray:
    vector = np.asarray(axis, dtype=float)
    norm = float(np.linalg.norm(vector))
    if norm <= 0:
        raise ValueError("URDF joint axis has zero length")
    x, y, z = vector / norm
    c, s, C = math.cos(q), math.sin(q), 1.0 - math.cos(q)
    return np.asarray([
        [c + x*x*C, x*y*C - z*s, x*z*C + y*s],
        [y*x*C + z*s, c + y*y*C, y*z*C - x*s],
        [z*x*C - y*s, z*y*C + x*s, c + z*z*C],
    ])


class UrdfKinematics:
    """Minimal serial-chain URDF FK for one configured robot chain; no TF dependency."""

    def __init__(self, urdf: Path, joint_names: list[str], base_frame: str, tool_frame: str):
        self.urdf = urdf.resolve()
        self.base_frame = base_frame
        self.tool_frame = tool_frame
        self.joint_names = joint_names
        self.chain = self._load_chain()

    def _load_chain(self):
        root = ET.parse(self.urdf).getroot()
        by_parent: dict[str, list[tuple[ET.Element, str]]] = {}
        for joint in root.findall("joint"):
            parent, child = joint.find("parent"), joint.find("child")
            if parent is not None and child is not None:
                by_parent.setdefault(parent.attrib["link"], []).append((joint, child.attrib["link"]))

        def reaches(link: str, visited: frozenset[str] = frozenset()) -> bool:
            return link == self.tool_frame or (
                link not in visited and any(reaches(child, visited | {link}) for _, child in by_parent.get(link, []))
            )

        chain: list[ET.Element] = []
        here = self.base_frame
        while here != self.tool_frame:
            found = next(((joint, child) for joint, child in by_parent.get(here, []) if reaches(child)), None)
            if found is None:
                raise ValueError(f"URDF cannot connect {self.base_frame} to {self.tool_frame}: {self.urdf}")
            chain.append(found[0])
            here = found[1]
        return chain

    def fk(self, q: np.ndarray) -> np.ndarray:
        if q.shape != (len(self.joint_names),) or not np.isfinite(q).all():
            raise ValueError(f"URDF FK requires {len(self.joint_names)} finite configured joint positions")
        qmap = dict(zip(self.joint_names, q.tolist()))
        T = np.eye(4)
        for joint in self.chain:
            if joint.find("mimic") is not None:
                raise ValueError("Mimic joint in robot chain requires a full URDF FK implementation")
            origin = joint.find("origin")
            xyz = [float(x) for x in origin.attrib.get("xyz", "0 0 0").split()] if origin is not None else [0, 0, 0]
            rpy = [float(x) for x in origin.attrib.get("rpy", "0 0 0").split()] if origin is not None else [0, 0, 0]
            A = np.eye(4)
            A[:3, :3] = rpy_matrix(rpy)
            A[:3, 3] = xyz
            T = T @ A
            kind = joint.attrib.get("type", "fixed")
            if kind in ("revolute", "continuous"):
                name = joint.attrib["name"]
                axis_element = joint.find("axis")
                axis = [float(x) for x in axis_element.attrib.get("xyz", "1 0 0").split()] if axis_element is not None else [1, 0, 0]
                R = np.eye(4)
                if name not in qmap:
                    raise ValueError(f"URDF chain contains variable joint not listed in config: {name}")
                R[:3, :3] = axis_angle(axis, qmap[name])
                T = T @ R
            elif kind == "prismatic":
                name = joint.attrib["name"]
                axis_element = joint.find("axis")
                axis = np.asarray([float(x) for x in axis_element.attrib.get("xyz", "1 0 0").split()] if axis_element is not None else [1, 0, 0])
                if np.linalg.norm(axis) <= 0:
                    raise ValueError("URDF prismatic axis has zero length")
                axis = axis / np.linalg.norm(axis)
                if name not in qmap:
                    raise ValueError(f"URDF chain contains variable joint not listed in config: {name}")
                T[:3, 3] += T[:3, :3] @ (axis * qmap[name])
            elif kind != "fixed":
                raise ValueError(f"Unsupported URDF joint type: {kind}")
        return T


def load_grid_board(path: Path):
    spec = json.loads(path.read_text(encoding="utf-8"))
    if spec.get("family") != "tag36h11":
        raise ValueError(f"only tag36h11 board is supported, got {spec.get('family')}")
    tag_size = float(spec["tag_size_m"])
    gap = float(spec["tag_spacing_m"])
    if not np.isfinite([tag_size, gap]).all() or tag_size <= 0 or gap < 0:
        raise ValueError("Invalid board dimensions")
    rotation = int(spec.get("opencv_corner_rotation_steps", 0)) % 4
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    object_points, ids = [], []
    for row, values in enumerate(spec["id_grid"]):
        for column, marker_id in enumerate(values):
            x, y = column * (tag_size + gap), row * (tag_size + gap)
            corners = np.asarray([[x, y, 0], [x + tag_size, y, 0], [x + tag_size, y + tag_size, 0], [x, y + tag_size, 0]], dtype=np.float32)
            object_points.append(np.roll(corners, rotation, axis=0))
            ids.append(int(marker_id))
    if len(ids) != len(set(ids)):
        raise ValueError("Duplicate board marker IDs")
    # OpenCV 4.6 exposes both names, but Board(...) can create an object that
    # estimatePoseBoard later dereferences incorrectly. Prefer its legacy,
    # stable factory when it is available.
    if hasattr(cv2.aruco, "Board_create"):
        board = cv2.aruco.Board_create(object_points, dictionary, np.asarray(ids, dtype=np.int32))
    else:
        board = cv2.aruco.Board(object_points, dictionary, np.asarray(ids, dtype=np.int32))
    return spec, board, dictionary


def _detector(dictionary):
    params = cv2.aruco.DetectorParameters_create() if hasattr(cv2.aruco, "DetectorParameters_create") else cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return params


def detect_gridboard(image: np.ndarray, K: np.ndarray, D: np.ndarray, board, dictionary, min_tags: int):
    params = _detector(dictionary)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    if hasattr(cv2.aruco, "ArucoDetector"):
        corners, ids, _ = cv2.aruco.ArucoDetector(dictionary, params).detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
    board_ids = board.getIds() if hasattr(board, "getIds") else board.ids
    valid_ids = set(int(value) for value in np.asarray(board_ids).reshape(-1))
    matches = [] if ids is None else [i for i, value in enumerate(ids.reshape(-1)) if int(value) in valid_ids]
    corners = [corners[i] for i in matches]
    ids = None if not matches else np.asarray([ids[i] for i in matches], dtype=np.int32).reshape(-1, 1)
    count = len(matches)
    if count < min_tags:
        raise ValueError(f"only {count} grid tags detected; need at least {min_tags}")
    board_points = board.getObjPoints() if hasattr(board, "getObjPoints") else board.objPoints
    mapping = dict(zip(np.asarray(board_ids).reshape(-1).tolist(), board_points))
    obj = np.concatenate([mapping[int(value)] for value in ids.reshape(-1)]).astype(np.float64)
    pixels = np.concatenate(corners).reshape(-1, 2).astype(np.float64)
    valid, rvec, tvec = cv2.solvePnP(obj, pixels, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
    if not valid:
        raise ValueError("OpenCV could not estimate GridBoard pose")
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, np.asarray(tvec, dtype=float).reshape(3)
    if np.min((obj @ R.T + T[:3, 3])[:, 2]) <= 0:
        raise ValueError("Grid board pose has points behind camera")
    projected, _ = cv2.projectPoints(obj, rvec, tvec, K, D)
    rms = float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2) - pixels)**2, axis=1))))
    return T, count, corners, ids, rms


def detect_single_tag(image: np.ndarray, K: np.ndarray, D: np.ndarray, tag_id: int, tag_size_m: float):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    params = _detector(dictionary)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    if hasattr(cv2.aruco, "ArucoDetector"):
        corners, ids, _ = cv2.aruco.ArucoDetector(dictionary, params).detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
    if ids is None:
        raise ValueError(f"Tag{tag_id} is not visible")
    matches = [index for index, value in enumerate(ids.reshape(-1)) if int(value) == tag_id]
    if len(matches) != 1:
        raise ValueError(f"expected exactly one Tag{tag_id}, found {len(matches)}")
    pixels = corners[matches[0]].reshape(4, 2).astype(np.float64)
    half = tag_size_m / 2.0
    object_points = np.asarray([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]], dtype=np.float64)
    ok, rvecs, tvecs, _ = cv2.solvePnPGeneric(object_points, pixels, K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    candidates = []
    for rvec, tvec in zip(rvecs, tvecs):
        R, _ = cv2.Rodrigues(rvec)
        depths = (object_points @ R.T + np.asarray(tvec).reshape(3))[..., 2]
        if np.min(depths) <= 0:
            continue
        reprojection, _ = cv2.projectPoints(object_points, rvec, tvec, K, D)
        error = float(np.sqrt(np.mean(np.sum((reprojection.reshape(4, 2) - pixels) ** 2, axis=1))))
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = R, np.asarray(tvec, dtype=float).reshape(3)
        candidates.append((error, T, pixels))
    if not ok or not candidates:
        raise ValueError("Tag PnP has no positive-depth solution")
    candidates.sort(key=lambda item: item[0])
    return candidates[0], len(ids), [dict(reprojection_error_px=e, matrix=t.tolist()) for e, t, _ in candidates]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def write_json_atomic(path: Path, document: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(document, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)
