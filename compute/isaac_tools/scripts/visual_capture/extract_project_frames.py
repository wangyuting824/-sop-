import os
from pathlib import Path
import re
import sys

import cv2
import numpy as np

ROOT = (Path(os.getenv("CALIBRATION_OUTPUT", "artifacts/inspection")))
DOWNLOADS = (Path(os.getenv("CALIBRATION_MEDIA_DIR", Path.home() / "Downloads")))
OUT = ROOT / "project_video_frames"
OUT.mkdir(parents=True, exist_ok=True)

VIDEOS = [
    "3.mp4",
    "hezi_环绕.mp4",
    "table_orbit_360_18s.mp4",
    "4.mp4",
    "2.mp4",
    "las_rgb_same_view_smooth.mp4",
    "las_rgb_same_view_smooth (3).mp4",
    "las_rgb_same_view_smooth (1).mp4",
    "las_rgb_same_view_smooth (2).mp4",
]


def safe_name(name):
    return re.sub(r"[^A-Za-z0-9._-]+", "_", name.replace(".mp4", ""))


def write_png(path, image):
    ok, encoded = cv2.imencode(".png", image)
    if not ok:
        raise RuntimeError(f"Could not encode {path}")
    encoded.tofile(str(path))


def extract(video, index):
    cap = cv2.VideoCapture(str(video))
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    duration = frame_count / fps if frame_count else 0
    target = OUT / f"{index:02d}_{safe_name(video.name)}"
    target.mkdir(parents=True, exist_ok=True)
    print(f"{video.name}: {duration:.2f}s; {fps:.1f} fps")
    times = np.linspace(max(0.2, duration * 0.08), max(0.2, duration * 0.92), 6)
    thumbs = []
    for n, seconds in enumerate(times, 1):
        cap.set(cv2.CAP_PROP_POS_MSEC, seconds * 1000)
        ok, frame = cap.read()
        if not ok:
            continue
        output = target / f"frame_{n:02d}_{seconds:05.2f}s.png"
        write_png(output, frame)
        thumb_w = 400
        thumb_h = int(frame.shape[0] * thumb_w / frame.shape[1])
        thumb = cv2.resize(frame, (thumb_w, thumb_h), interpolation=cv2.INTER_AREA)
        cv2.rectangle(thumb, (0, 0), (thumb_w, 38), (0, 0, 0), -1)
        label = f"{video.name}  {seconds:.1f}s"
        cv2.putText(thumb, label, (10, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
        thumbs.append(thumb)
    cap.release()
    if not thumbs:
        return
    cols = 3
    rows = (len(thumbs) + cols - 1) // cols
    blank = np.full_like(thumbs[0], 245)
    while len(thumbs) < rows * cols:
        thumbs.append(blank.copy())
    sheet = np.vstack([np.hstack(thumbs[row * cols:(row + 1) * cols]) for row in range(rows)])
    write_png(target / "contact_sheet.png", sheet)


for idx, name in enumerate(VIDEOS, 1):
    video = DOWNLOADS / name
    if video.exists():
        extract(video, idx)
    else:
        print(f"missing: {video}")
