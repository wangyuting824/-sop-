import os
from pathlib import Path
import sys

import cv2
import numpy as np

VIDEO = (Path(os.getenv("CALIBRATION_MEDIA_DIR", Path.home() / "Downloads")) / "LCC重建与导出.mp4")
OUT = (Path(os.getenv("CALIBRATION_OUTPUT", "artifacts/inspection")) / "lcc_video_frames")
OUT.mkdir(parents=True, exist_ok=True)


def write_png(path, image):
    """Write PNG files reliably when the Windows path contains Chinese characters."""
    ok, encoded = cv2.imencode(".png", image)
    if not ok:
        raise RuntimeError(f"Could not encode {path}")
    encoded.tofile(str(path))

cap = cv2.VideoCapture(str(VIDEO))
fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
duration = count / fps if count else 0.0
print(f"fps={fps:.3f}; frames={count}; duration={duration:.2f}s")

# Extract an evenly spaced overview for visual selection.
times = np.linspace(0.5, max(0.5, duration - 0.5), 20)
thumbs = []
for index, seconds in enumerate(times, 1):
    cap.set(cv2.CAP_PROP_POS_MSEC, float(seconds) * 1000)
    ok, frame = cap.read()
    if not ok:
        continue
    target_w = 960
    target_h = int(frame.shape[0] * target_w / frame.shape[1])
    full = cv2.resize(frame, (target_w, target_h), interpolation=cv2.INTER_AREA)
    path = OUT / f"sample_{index:02d}_{seconds:06.2f}s.png"
    write_png(path, full)

    thumb_w = 360
    thumb_h = int(full.shape[0] * thumb_w / full.shape[1])
    thumb = cv2.resize(full, (thumb_w, thumb_h), interpolation=cv2.INTER_AREA)
    cv2.rectangle(thumb, (0, 0), (thumb_w, 38), (0, 0, 0), -1)
    cv2.putText(thumb, f"{index:02d}  {seconds:05.1f}s", (10, 27), cv2.FONT_HERSHEY_SIMPLEX,
                0.75, (255, 255, 255), 2, cv2.LINE_AA)
    thumbs.append(thumb)

cap.release()
if not thumbs:
    raise RuntimeError("No video frames could be extracted")

cols = 4
rows = (len(thumbs) + cols - 1) // cols
blank = np.full_like(thumbs[0], 245)
while len(thumbs) < rows * cols:
    thumbs.append(blank.copy())
rows_img = [np.hstack(thumbs[r * cols:(r + 1) * cols]) for r in range(rows)]
sheet = np.vstack(rows_img)
write_png(OUT / "contact_sheet.png", sheet)

# Full-resolution candidates for the SOP figures.
cap = cv2.VideoCapture(str(VIDEO))
selected = {
    "01_reconstruction_parameters": 8.3,
    "02_model_list": 54.8,
    "03_export_settings": 85.8,
    "04_export_target": 101.3,
    "05_export_save_dialog": 124.6,
    "06_export_complete": 147.9,
}
for label, seconds in selected.items():
    cap.set(cv2.CAP_PROP_POS_MSEC, seconds * 1000)
    ok, frame = cap.read()
    if ok:
        write_png(OUT / f"{label}_{seconds:05.1f}s.png", frame)
cap.release()
