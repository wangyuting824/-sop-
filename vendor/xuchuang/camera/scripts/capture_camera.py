#!/usr/bin/env python3
"""Acquire camera streams, read Image/H264 and robot feedback; no motion commands."""
from __future__ import annotations
import argparse
from collections import deque
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import shutil
import sys
import threading
import time
import uuid
import cv2
import numpy as np
from camera_common import load_grid_board, detect_gridboard, detect_single_tag, sha256, write_json_atomic
from capture_data import camera_matrix, decode_color, pair_feedback, stamp_ns
from camera_stream import CameraStreamSession, H264FrameDecoder

ROOT = Path(__file__).resolve().parents[1]
VERSION = "20261111-camera-opencv-v3"


def clean(value):
    if isinstance(value, dict):
        return {key: clean(item) for key, item in value.items()}
    if isinstance(value, (str, bool, int)) or value is None:
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else str(value)
    return [clean(item) for item in value]


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--version", action="version", version=VERSION)
    parser.add_argument("camera", choices=("left_wrist", "right_wrist", "head"))
    parser.add_argument("--tag-arm", choices=("left", "right"), help="For head: arm carrying rigidly attached ID0")
    parser.add_argument("--joint-unit", choices=("deg", "rad"), default="deg", help="Actual site feedback is degrees")
    parser.add_argument("--board", type=Path, default=ROOT / "config/board_definition.json")
    parser.add_argument("--tag-size-m", type=float, default=.05)
    parser.add_argument("--min-tags", type=int, default=25)
    parser.add_argument("--max-reprojection-px", type=float, default=1.5)
    parser.add_argument("--sync-limit-s", type=float, default=.1)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--preview", action="store_true", help="Requires a working graphical/X11 session")
    parser.add_argument("--image-source", choices=("auto", "image", "h264"), default="auto")
    parser.add_argument("--no-acquire-stream", action="store_true", help="Only subscribe to a stream already opened elsewhere")
    parser.add_argument("--stream-priority", type=int, default=0, help="AcquireStream priority (uint8), default 0")
    parser.add_argument("--stream-timeout-s", type=float, default=10.0)
    parser.add_argument("--frame-wait-s", type=float, default=10.0, help="Wait for a fresh H264 keyframe when saving")
    args = parser.parse_args()
    if args.camera == "head" and not args.tag_arm:
        parser.error("head requires --tag-arm left or right; identify the arm holding ID0")
    if args.camera != "head" and args.tag_arm:
        parser.error("--tag-arm is only for head")
    for value in (args.tag_size_m, args.max_reprojection_px, args.sync_limit_s, args.stream_timeout_s, args.frame_wait_s):
        if not math.isfinite(value) or value <= 0:
            parser.error("Sizes, reprojection and synchronization thresholds must be positive and finite")
    if args.min_tags < 4:
        parser.error("--min-tags must be >=4")
    if not 0 <= args.stream_priority <= 255:
        parser.error("--stream-priority must be 0..255")
    if args.preview and not os.environ.get("DISPLAY"):
        parser.error("--preview needs DISPLAY/X11; omit it to use the original camera viewer")
    return args


def run(args):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs.msg import Image, CameraInfo, JointState

    out = args.output or Path.home() / "robotcal_data/cameras" / (args.camera + "_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    out = out.resolve()
    out.mkdir(parents=True, exist_ok=False)
    (out / "samples").mkdir()
    shutil.copyfile(args.board, out / "board_definition.json")
    board_spec, board, dictionary = load_grid_board(out / "board_definition.json")
    arm = args.tag_arm if args.camera == "head" else args.camera.split("_")[0]
    image_topic = f"/camera_driver/slots/{args.camera}/color"
    h264_topic = f"/camera_driver/slots/{args.camera}/h264"
    info_topic = image_topic + "/camera_info"
    document = {"schema_version": 2, "mode": "eye_to_hand" if args.camera == "head" else "eye_in_hand",
                "camera_slot": args.camera, "arm": arm, "created_utc": datetime.now(timezone.utc).isoformat(),
                "joint_raw_unit": args.joint_unit, "T_base_tool_source": "pending_real_robot_URDF",
                "board_definition": board_spec, "board_sha256": sha256(out / "board_definition.json"),
                "tag": {"family": "tag36h11", "id": 0, "black_border_m": args.tag_size_m},
                "topics": {"image": image_topic, "h264": h264_topic, "info": info_topic, "joints": "/manipulator/joint_states"},
                "software": {"python": sys.version, "opencv": cv2.__version__, "ROS_DISTRO": os.environ.get("ROS_DISTRO"),
                             "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID"), "version": VERSION},
                "settings": {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
                "samples": [], "accepted_count": 0, "holdout_count": 0,
                "commands_sent": False, "motion_commands_sent": False, "status": "capturing"}
    write_json_atomic(out / "samples.json", document)
    log = (out / "feedback_raw.jsonl").open("w", encoding="utf-8", buffering=1024*1024)
    stream_log = (out / "stream_events.jsonl").open("w", encoding="utf-8")
    h264_bytes = (out / "h264_packets.bin").open("wb")
    h264_log = (out / "h264_packets.jsonl").open("w", encoding="utf-8")
    lock = threading.RLock()
    store = {"image": None, "info": None, "image_rx_ns": 0, "joint_rx_ns": 0,
             "joints": deque(maxlen=8000), "latest": {}, "counts": {}, "last_saved_stamp": None, "error": None,
             "raw_image_rx_ns": 0, "image_source": None, "image_packet": None, "h264_count": 0,
             "decoder": None, "decode_error": None}
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("camera_extrinsic_capture_" + args.camera)
    volatile = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def stream_event(kind, **fields):
        with lock:
            stream_log.write(json.dumps(dict(event=kind, received_unix_ns=time.time_ns(), **fields), ensure_ascii=False) + "\n")
            stream_log.flush()

    def record(topic, message):
        record = {"topic": topic, "received_unix_ns": time.time_ns(), "received_monotonic_ns": time.monotonic_ns(),
                  "message": clean(message_to_ordereddict(message))}
        with lock:
            log.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
            store["latest"][topic] = record
            store["counts"][topic] = store["counts"].get(topic, 0) + 1
            if topic == "/manipulator/joint_states":
                store["joints"].append(record)
                store["joint_rx_ns"] = record["received_monotonic_ns"]
            elif topic == info_topic:
                store["info"] = record["message"]

    def on_image(message):
        with lock:
            store["raw_image_rx_ns"] = time.monotonic_ns()
            if args.image_source == "h264":
                return
            first = store["image"] is None
            store["image"], store["image_rx_ns"] = message, time.monotonic_ns()
            store["image_source"], store["image_packet"] = "sensor_msgs/Image", None
        if first:
            print(f"已收到 color Image：{message.width}x{message.height}。", flush=True)

    def on_h264(message):
        if message.slot_id != args.camera:
            return
        payload = bytes(message.data)
        with lock:
            store["h264_count"] += 1
            meta = dict(packet_index=store["h264_count"], topic=h264_topic,
                        header=clean(message_to_ordereddict(message.header)), slot_id=message.slot_id,
                        width=message.width, height=message.height, codec=message.codec,
                        is_keyframe=message.is_keyframe, pts_us=message.pts_us,
                        received_monotonic_ns=time.monotonic_ns(), received_unix_ns=time.time_ns(),
                        bytes_offset=h264_bytes.tell(), bytes_length=len(payload), bytes_file="h264_packets.bin")
            h264_bytes.write(payload)
            h264_bytes.flush()
            h264_log.write(json.dumps(meta, ensure_ascii=False) + "\n")
            h264_log.flush()
            if args.image_source == "image":
                return
            raw_fresh = time.monotonic_ns() - store["raw_image_rx_ns"] < 500_000_000
            if args.image_source == "auto" and raw_fresh:
                return
        try:
            if store["decoder"] is None:
                store["decoder"] = H264FrameDecoder()
            for bgr, original in store["decoder"].feed(payload, meta):
                image = Image()
                image.header.frame_id = original["header"]["frame_id"]
                image.header.stamp.sec = original["header"]["stamp"]["sec"]
                image.header.stamp.nanosec = original["header"]["stamp"]["nanosec"]
                image.height, image.width = bgr.shape[:2]
                image.encoding, image.is_bigendian, image.step = "bgr8", 0, image.width * 3
                image.data = bgr.tobytes()
                with lock:
                    first = store["image"] is None
                    store["image"], store["image_rx_ns"] = image, original["received_monotonic_ns"]
                    store["image_source"], store["image_packet"] = "OpenCV H264 IDR decoded to bgr8", original
                    store["decode_error"] = None
                if first:
                    print(f"OpenCV 已解码 H264 关键帧：{image.width}x{image.height}。停稳后按 s 保存。", flush=True)
        except Exception as exc:
            message = str(exc)
            if message != store["decode_error"]:
                stream_event("decode_error", error=message, packet_index=meta["packet_index"])
            store["decode_error"] = message
            # Preserve cached SPS/PPS; an invalid packet must not erase them.

    subscriptions = [node.create_subscription(Image, image_topic, on_image, volatile),
                     node.create_subscription(CameraInfo, info_topic, lambda msg: record(info_topic, msg), latched),
                     node.create_subscription(JointState, "/manipulator/joint_states", lambda msg: record("/manipulator/joint_states", msg), volatile)]
    document["optional_topics"] = {}
    if args.image_source != "image":
        try:
            subscriptions.append(node.create_subscription(get_message("camera_driver/msg/H264Packet"), h264_topic, on_h264,
                                 QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)))
        except (ImportError, AttributeError, ModuleNotFoundError) as exc:
            document["optional_topics"][h264_topic] = str(exc)
    for topic, msg_type, qos in (("/manipulator/arm_state", "manipulator/msg/ArmStateArray", volatile),
                                 ("/camera_driver/module_status", "camera_driver/msg/CameraModuleStatus", latched)):
        try:
            subscriptions.append(node.create_subscription(get_message(msg_type), topic, lambda msg, t=topic: record(t, msg), qos))
            document["optional_topics"][topic] = "subscribed"
        except (ImportError, AttributeError, ModuleNotFoundError) as exc:
            document["optional_topics"][topic] = str(exc)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def spin():
        try:
            executor.spin()
        except Exception as exc:
            store["error"] = str(exc)
    worker = threading.Thread(target=spin, daemon=True)
    worker.start()

    def save():
        if args.image_source != "image":
            deadline = time.monotonic() + args.frame_wait_s
            announced = False
            while True:
                with lock:
                    candidate = store["image"]
                    fresh = candidate is not None and time.monotonic_ns() - store["image_rx_ns"] <= 500_000_000
                    new = candidate is not None and stamp_ns(clean(message_to_ordereddict(candidate.header))) != store["last_saved_stamp"]
                if fresh and new or time.monotonic() >= deadline or store["error"]:
                    break
                if not announced:
                    print("等待新的彩色图像/独立关键帧，请保持当前姿态停稳……", flush=True)
                    announced = True
                time.sleep(.02)
        snapshot_ns = time.monotonic_ns()
        with lock:
            image, info = store["image"], store["info"]
            records, latest = list(store["joints"]), dict(store["latest"])
            image_rx, joint_rx = store["image_rx_ns"], store["joint_rx_ns"]
            image_source, image_packet = store["image_source"], store["image_packet"]
        if image is None:
            print(f"尚未收到可用彩色图像；已收到 H264 包 {store['h264_count']} 个。继续等待后按 s。", flush=True)
            if store["decode_error"]:
                print("解码信息：" + store["decode_error"], flush=True)
            module = latest.get("/camera_driver/module_status", {}).get("message", {})
            for slot in module.get("slots", []):
                if slot.get("slot_id") == args.camera:
                    print("驱动机位状态：" + json.dumps({key: slot.get(key) for key in (
                        "device_open", "color_ready", "color_stream_running", "color_session_count", "active_fault_codes")},
                        ensure_ascii=False), flush=True)
            return
        header = clean(message_to_ordereddict(image.header))
        image_ns = stamp_ns(header)
        if store["last_saved_stamp"] == image_ns:
            print("没有新图像；本帧已保存。", flush=True)
            return
        index = len(document["samples"]) + 1
        folder = out / "samples" / f"{index:04d}"
        folder.mkdir()
        raw_image = folder / "image_data.bin"
        raw_image.write_bytes(bytes(image.data))
        sample = {"index": index, "split": "fit", "use_for_calibration": False,
                  "image": {"header": header, "width": image.width, "height": image.height, "step": image.step,
                            "source": image_source, "h264_packet": image_packet,
                            "encoding": image.encoding, "is_bigendian": image.is_bigendian,
                            "received_monotonic_ns": image_rx, "bytes_file": str(raw_image.relative_to(out)).replace("\\", "/"),
                            "bytes_sha256": sha256(raw_image)},
                  "camera_info": info, "raw_state_at_save": latest,
                  "joint_window_raw": [r for r in records if r["received_monotonic_ns"] >= image_rx - 2_000_000_000],
                  "notes": "Keep camera mounts/waist/head and board attachment unchanged throughout this session"}
        reasons = []
        bgr = None
        try:
            bgr = decode_color(image)
            if not cv2.imwrite(str(folder / "image.png"), bgr):
                raise OSError("Could not write image.png")
            sample["image"]["png_file"] = str((folder / "image.png").relative_to(out)).replace("\\", "/")
            sample["image"]["png_sha256"] = sha256(folder / "image.png")
            if snapshot_ns - image_rx > 500_000_000:
                raise ValueError("Image is stale (>0.5s since receipt)")
            if info is None:
                raise ValueError("Waiting for latched CameraInfo")
            K, D = camera_matrix(header, image.width, image.height, info)
            if args.camera == "head":
                (rms, T, pixels), tag_count, alternatives = detect_single_tag(bgr, K, D, 0, args.tag_size_m)
                sample["pnp_alternatives"] = alternatives
                sample["marker_ids"] = [0]
                sample["marker_corners_px"] = [pixels.tolist()]
                if len(alternatives) > 1 and alternatives[1]["reprojection_error_px"] - rms < .05:
                    raise ValueError("ID0 planar pose is ambiguous; tilt the tag and save a new pose")
                key, target = "T_camera_tag", "tag36h11_id0"
            else:
                T, tag_count, corners, ids, rms = detect_gridboard(bgr, K, D, board, dictionary, args.min_tags)
                sample["marker_ids"] = ids.reshape(-1).tolist()
                sample["marker_corners_px"] = [c.reshape(4, 2).tolist() for c in corners]
                key, target = "T_camera_board", board_spec["board_id"]
            sample["reprojection_error_px"] = rms
            sample["detected_tags"] = tag_count
            sample[key] = {"from_frame": target, "to_frame": header["frame_id"], "matrix": T.tolist()}
            if not math.isfinite(rms) or rms > args.max_reprojection_px:
                raise ValueError(f"Board reprojection error {rms:.3f}px > {args.max_reprojection_px}px")
        except (ValueError, cv2.error) as exc:
            reasons.append(str(exc))
        try:
            if snapshot_ns - joint_rx > 500_000_000:
                raise ValueError("JointState feedback is stale/missing")
            sample["robot_feedback"] = pair_feedback(records, image_ns, arm, args.joint_unit, args.sync_limit_s)
        except (ValueError, KeyError) as exc:
            reasons.append(str(exc))
        sample["exclusion_reasons"] = reasons
        sample["use_for_calibration"] = not reasons
        document["samples"].append(sample)
        store["last_saved_stamp"] = image_ns
        write_json_atomic(folder / "sample.json", sample)
        flush_document()
        if bgr is not None:
            marked = bgr.copy()
            if "marker_corners_px" in sample:
                cs = [np.asarray(c, dtype=np.float32).reshape(1, 4, 2) for c in sample["marker_corners_px"]]
                cv2.aruco.drawDetectedMarkers(marked, cs, np.asarray(sample["marker_ids"], dtype=np.int32).reshape(-1, 1))
            cv2.imwrite(str(out / "latest_preview.jpg"), marked)
            if args.preview:
                cv2.imshow("Saved camera sample", marked)
                cv2.waitKey(1)
        usable = sum(s["use_for_calibration"] for s in document["samples"])
        print(f"已保存第 {index} 帧；可用于手眼计算 {usable} 帧。" + ("请换新姿态。" if not reasons else " 本帧仅保留原始数据：" + "; ".join(reasons)), flush=True)

    def flush_document():
        with lock:
            document["received_message_counts"] = dict(store["counts"])
            log.flush()
        good = [s for s in document["samples"] if s["use_for_calibration"]]
        holdout = {s["index"] for s in good[-5:]} if len(good) >= 15 else set()
        for s in document["samples"]:
            s["split"] = "holdout" if s["index"] in holdout else "fit"
        document["accepted_count"] = len(good)
        document["holdout_count"] = len(holdout)
        document["h264_packet_count"] = store["h264_count"]
        document["last_decode_error"] = store["decode_error"]
        write_json_atomic(out / "samples.json", document)

    print(f"版本：{VERSION}\n数据目录：{out}\n相机：{args.camera}，反馈按 {args.joint_unit} 读取；不发送运动命令。\n"
          "每个新位置/朝向停稳至少 1 秒，输入 s 回车保存；15 个有效姿态自动结束；输入 q 可提前结束。\n"
          "原始数据会逐帧保存；内参、时间配对或板识别不合格的帧会注明原因。", flush=True)
    session = None
    try:
        if not args.no_acquire_stream:
            consumer = "robotcal_" + args.camera + "_" + uuid.uuid4().hex[:12]
            document["stream_consumer_id"] = consumer
            session = CameraStreamSession(node, args.camera, consumer, stream_event,
                                          args.stream_timeout_s, args.stream_priority)
            document["stream_session_id"] = session.acquire()
            write_json_atomic(out / "samples.json", document)
            print("已申请 COLOR/H264 流，会话：" + document["stream_session_id"], flush=True)
        while rclpy.ok():
            if store["error"]:
                raise RuntimeError("ROS callback failed: " + store["error"])
            line = input("s 保存 / q 结束 > ").strip().lower()
            if line == "q":
                break
            if line == "s":
                save()
                if document["accepted_count"] >= 15:
                    print("15 个有效姿态已采满，结束本路采集。", flush=True)
                    break
    except (KeyboardInterrupt, EOFError):
        pass
    except Exception as exc:
        document["capture_error"] = str(exc)
        raise
    finally:
        if session is not None:
            try:
                document["stream_released"] = session.release()
                if not document["stream_released"]:
                    print("流会话释放未确认，详见 stream_events.jsonl。", flush=True)
            except Exception as exc:
                document["stream_released"] = False
                document["stream_release_error"] = str(exc)
                print(str(exc), flush=True)
        executor.shutdown(timeout_sec=3)
        worker.join(timeout=3)
        document["status"] = "raw_capture_complete_model_pending"
        if store["error"]:
            document["callback_error"] = store["error"]
            document["status"] = "capture_interrupted"
        if "capture_error" in document:
            document["status"] = "capture_interrupted"
        flush_document()
        for sample in document["samples"]:
            write_json_atomic(out / "samples" / f"{sample['index']:04d}" / "sample.json", sample)
        log.close()
        h264_bytes.close()
        h264_log.close()
        stream_log.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if args.preview:
            cv2.destroyAllWindows()
        print(f"已保存：{out}\n有效姿态 {document['accepted_count']}，留出验证 {document['holdout_count']}。", flush=True)
        if document["accepted_count"] < 15:
            print("本场提前结束，尚不足 15 个有效姿态，当前数据已保留。", flush=True)
    if store["error"]:
        raise RuntimeError(store["error"])


if __name__ == "__main__":
    try:
        run(arguments())
    except Exception as exc:
        print(f"采集停止：{exc}", file=sys.stderr)
        raise SystemExit(1)
