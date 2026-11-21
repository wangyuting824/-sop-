"""Documented camera_driver stream sessions and timestamp-preserving H264 decode."""
from __future__ import annotations

from pathlib import Path
import re
import tempfile
import time


class CameraStreamSession:
    def __init__(self, node, slot, consumer, event, timeout=10.0, priority=0):
        from camera_driver.srv import AcquireStream, ReleaseStream
        self.acquire_type, self.release_type = AcquireStream, ReleaseStream
        self.acquire_client = node.create_client(AcquireStream, "/camera_driver/acquire_stream")
        self.release_client = node.create_client(ReleaseStream, "/camera_driver/release_stream")
        self.slot, self.consumer, self.event = slot, consumer, event
        self.timeout, self.priority = timeout, priority
        self.session_id = None
        self.acquire_future = None

    def wait(self, future):
        deadline = time.monotonic() + self.timeout
        while not future.done():
            if time.monotonic() >= deadline:
                raise RuntimeError("相机流服务响应超时；不重复申请会话")
            time.sleep(.02)  # The capture executor is already spinning in its own thread.
        response = future.result()
        if response is None:
            raise RuntimeError("相机流服务未返回响应")
        return response

    def acquire(self):
        if not self.acquire_client.wait_for_service(timeout_sec=self.timeout):
            raise RuntimeError("/camera_driver/acquire_stream 服务不可用")
        request = self.acquire_type.Request()
        request.consumer_id = self.consumer
        request.slot_id = self.slot
        request.stream_mask = 1  # COLOR; no depth session is needed for this calibration.
        request.codec = "h264"
        request.priority = self.priority
        self.event("acquire_request", consumer_id=request.consumer_id, slot_id=request.slot_id,
                   stream_mask=1, codec=request.codec, priority=request.priority)
        self.acquire_future = self.acquire_client.call_async(request)
        response = self.wait(self.acquire_future)
        self.event("acquire_response", success=response.success, failure_reason=response.failure_reason,
                   session_id=response.session_id)
        if not response.success or not response.session_id:
            raise RuntimeError("申请相机彩色流失败：" + response.failure_reason)
        self.session_id = response.session_id
        return self.session_id

    def release(self):
        # A reply can arrive after acquire() timed out. Release it if it is now known.
        if self.session_id is None and self.acquire_future is not None and self.acquire_future.done():
            response = self.acquire_future.result()
            if response is not None and response.success and response.session_id:
                self.session_id = response.session_id
        if self.session_id is None:
            if self.acquire_future is not None and not self.acquire_future.done():
                self.event("release_unconfirmed", consumer_id=self.consumer,
                           reason="acquire response still pending; session_id unknown")
                return False
            return True
        if not self.release_client.wait_for_service(timeout_sec=self.timeout):
            raise RuntimeError("释放相机流失败：release_stream 服务不可用，session_id=" + self.session_id)
        request = self.release_type.Request()
        request.session_id = self.session_id
        self.event("release_request", session_id=self.session_id)
        response = self.wait(self.release_client.call_async(request))
        self.event("release_response", session_id=self.session_id, success=response.success,
                   failure_reason=response.failure_reason)
        if not response.success:
            raise RuntimeError("释放相机流失败：" + response.failure_reason)
        self.session_id = None
        self.acquire_future = None
        return True


def h264_nals(payload):
    """Split a complete access unit: Annex B or structurally valid AVCC4."""
    # Check AVCC lengths first: length bytes can themselves resemble a start code.
    units, offset = [], 0
    while offset + 4 <= len(payload):
        size = int.from_bytes(payload[offset:offset + 4], "big")
        offset += 4
        if size == 0 or offset + size > len(payload):
            break
        unit = payload[offset:offset + size]
        if unit[0] & 0x80 or not 1 <= unit[0] & 31 <= 23:
            break
        units.append(unit)
        offset += size
    else:
        if units and offset == len(payload):
            return units
    starts = list(re.finditer(b"\x00\x00(?:\x00)?\x01", payload))
    if not starts or payload[:starts[0].start()].strip(b"\x00"):
        raise ValueError("H264 包不是完整 Annex B/AVCC4 帧；原始包已保留")
    units = [payload[start.end():starts[i + 1].start() if i + 1 < len(starts) else len(payload)]
             for i, start in enumerate(starts)]
    if any(not unit or unit[0] & 0x80 or not 1 <= unit[0] & 31 <= 23 for unit in units):
        raise ValueError("H264 NAL 结构无效；原始包已保留")
    return units


class H264FrameDecoder:
    """Decode independent IDR frames using only the installed OpenCV backend.

    VideoCapture does not portably expose the original ROS packet timestamp.
    Decoding exactly one IDR access unit avoids guessing P/B frame association.
    All incoming packets are still recorded by the caller.
    """
    def __init__(self):
        import cv2
        self.cv2 = cv2
        self.sps, self.pps = [], []

    def feed(self, payload, metadata):
        if metadata["codec"].lower() not in ("h264", "h.264", "avc", "avc1"):
            raise ValueError("收到非 H264 编码：" + metadata["codec"])
        if not payload:
            return []
        units = h264_nals(payload)
        sps = [unit for unit in units if unit[0] & 31 == 7]
        pps = [unit for unit in units if unit[0] & 31 == 8]
        if sps:
            self.sps = sps
        if pps:
            self.pps = pps
        if not any(unit[0] & 31 == 5 for unit in units):
            return []
        if not self.sps or not self.pps:
            raise ValueError("已收到 IDR 关键帧，但缺少 SPS/PPS；继续等待完整关键帧")
        independent = self.sps + self.pps + [unit for unit in units if unit[0] & 31 not in (7, 8)]
        with tempfile.TemporaryDirectory(prefix="robotcal_h264_") as directory:
            path = Path(directory) / "frame.h264"
            path.write_bytes(b"".join(b"\x00\x00\x00\x01" + unit for unit in independent))
            capture = self.cv2.VideoCapture(str(path), self.cv2.CAP_ANY)
            try:
                if not capture.isOpened():
                    raise RuntimeError("现有 OpenCV 无法打开 H264 关键帧；需要其已有视频后端支持 H264，未调用其他解码包")
                ok, frame = capture.read()
                if not ok or frame is None:
                    raise ValueError("OpenCV 未解出 H264 关键帧；原始包已保留，继续等待下一帧")
                if capture.read()[0]:
                    raise ValueError("一个 H264 包解出多帧，无法唯一关联原始 Header；仅保留原始数据")
                backend = capture.getBackendName() if hasattr(capture, "getBackendName") else "unknown"
            finally:
                capture.release()
        if frame.shape[:2] != (metadata["height"], metadata["width"]):
            raise ValueError("H264 解码分辨率与包内元数据不一致")
        original = dict(metadata)
        original["decoder"] = dict(library="OpenCV", version=self.cv2.__version__, backend=backend,
                                   selection="single_IDR_access_unit", timestamp_source="original_ROS_Header")
        return [(frame, original)]
