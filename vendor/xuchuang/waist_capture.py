#!/usr/bin/env python3
"""Read-only torso/waist capture using the site's ROS Python and rosbag2."""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import time

VERSION = "20261110-waist-capture-v1"
AXES = ("HEAD_YAW", "HEAD_PITCH", "WAIST_YAW", "WAIST_PITCH_1", "WAIST_PITCH_2", "WAIST_PITCH_3")
TOPICS = (
    "/MotorState", "/motor_state/array", "/WaistCtrl", "/business_manager/waist_ctrl",
    "/BalanceCmd", "/MotorCmd", "/MotorCmdResult", "/motor_cmd_result",
    "/AttitudeCtrl", "/AttitudeCtrlResult", "/business_manager/attitude_ctrl",
    "/HeadCtrl", "/business_manager/head_ctrl", "/TorsoPose", "/TorsoTeleoperate",
    "/TorsoHeadCtrlExtend", "/actuator_state", "/business_manager/system_state_snapshots",
    "/parameter_events", "/rosout", "/tf", "/tf_static",
)
COMMAND_TOPICS = {
    "/WaistCtrl", "/business_manager/waist_ctrl", "/BalanceCmd", "/MotorCmd",
    "/AttitudeCtrl", "/business_manager/attitude_ctrl", "/HeadCtrl",
    "/business_manager/head_ctrl", "/TorsoTeleoperate", "/TorsoHeadCtrlExtend",
}
FIELDS = [
    "received_unix_ns", "received_monotonic_ns", "subscriber_ros_time_ns", "source_topic",
    "message_index", "joint_index", "joint_name", "part", "position_deg", "velocity_deg_s",
    "position_rad", "velocity_rad_s", "current_mA", "temperature_reserved_raw",
    "fault_code_raw", "enabled_raw",
]


def save_json(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def state_rows(payload, clocks, source, sequence):
    """Decode the documented 6 x 6 Float32MultiArray, without inventing time."""
    values = payload.get("data")
    offset = payload.get("layout", {}).get("data_offset", 0)
    if type(offset) is not int or offset < 0 or not isinstance(values, (list, tuple)):
        raise ValueError("MotorState 数据或 data_offset 格式错误")
    if len(values) - offset != 36:
        raise ValueError(f"MotorState 需要 36 项有效数据，收到 {len(values) - offset} 项")
    values = values[offset:]
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
        raise ValueError("MotorState 含非数值或非有限值；原始消息仍保留")
    rows = []
    for index, name in enumerate(AXES):
        q, dq, current, temperature, fault, enabled = values[index * 6:index * 6 + 6]
        rows.append([*clocks, source, sequence, index, name, "head" if index < 2 else "waist",
                     q, dq, math.radians(q), math.radians(dq), current, temperature, fault, enabled])
    return rows


def select_state_topic(topic_types, requested=None):
    candidates = [requested] if requested else ["/MotorState", "/motor_state/array"]
    for topic in candidates:
        if "std_msgs/msg/Float32MultiArray" in topic_types.get(topic, []):
            return topic
    raise RuntimeError("没有找到 Float32MultiArray 类型的腰部反馈 /MotorState 或 /motor_state/array")


def bag_command(help_text, output, topics):
    # Humble supports positional topics; newer releases also provide --topics.
    command = ["ros2", "bag", "record", "--output", str(output)]
    if "--topics" in help_text:
        command.append("--topics")
    return command + list(topics)


def stop_bag(process):
    """Finish only our recorder; never call any robot service or publisher."""
    forced = False
    if process.poll() is None:
        try:
            process.send_signal(signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=30)
        except subprocess.TimeoutExpired:
            forced = True
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
    return process.returncode, forced


def snapshot(run, node, topic_types):
    """Archive definitions/configuration alongside data, using read-only calls."""
    directory = run / "interfaces"
    directory.mkdir()
    save_json(directory / "topic_types.json", topic_types)
    commands = [("torso_node.txt", ["node", "info", "/robot_torso_control_node"]),
                ("torso_parameters.yaml", ["param", "dump", "/robot_torso_control_node"])]
    packages = set()
    for topic in TOPICS:
        for typ in topic_types.get(topic, []):
            if re.fullmatch(r"[A-Za-z0-9_]+/msg/[A-Za-z0-9_]+", typ):
                packages.add(typ.split("/")[0])
                commands.append((typ.replace("/", "__") + ".txt", ["interface", "show", typ]))
        if topic in topic_types:
            endpoints = node.get_publishers_info_by_topic(topic)
            save_json(directory / (topic.replace("/", "_") + "_publishers.json"), [
                {"node_name": e.node_name, "node_namespace": e.node_namespace,
                 "topic_type": e.topic_type, "qos": str(e.qos_profile)} for e in endpoints])
    # Repeated types only need one export. Timeouts/errors remain as evidence.
    for name, args in dict(commands).items():
        try:
            completed = subprocess.run(["ros2", *args], capture_output=True, text=True, timeout=5)
            text = completed.stdout
            if completed.returncode or completed.stderr:
                text += f"\n# exit={completed.returncode}\n# stderr:\n{completed.stderr}"
        except Exception as exc:
            text = f"# 读取失败：{exc}\n"
        (directory / name).write_text(text, encoding="utf-8")
    # Preserve custom interface sources for later bag decoding on a workstation.
    for package in sorted(packages):
        try:
            prefix = subprocess.run(["ros2", "pkg", "prefix", package], capture_output=True,
                                    text=True, timeout=5, check=True).stdout.strip()
            shared = Path(prefix) / "share" / package
            for folder in ("msg", "srv", "action"):
                if (shared / folder).is_dir():
                    shutil.copytree(shared / folder, directory / "packages" / package / folder)
            if (shared / "package.xml").is_file():
                dest = directory / "packages" / package
                dest.mkdir(parents=True, exist_ok=True)
                shutil.copy2(shared / "package.xml", dest / "package.xml")
        except Exception as exc:
            (directory / (package + "_export_error.txt")).write_text(str(exc), encoding="utf-8")


def capture(args):
    # No dependency on the offline bundle's x86 runtime, openpyxl or PyYAML.
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from rclpy.signals import SignalHandlerOptions
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rosidl_runtime_py.utilities import get_message

    help_result = subprocess.run(["ros2", "bag", "record", "--help"], capture_output=True,
                                 text=True, timeout=15, check=True)
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("robotcal_waist_capture_" + str(os.getpid()))
    run = args.output.expanduser() / datetime.now().strftime("waist_%Y%m%d_%H%M%S_%f")
    run.mkdir(parents=True, exist_ok=False)
    print(f"脚本版本：{VERSION}\n腰部采集保存到：{run}", flush=True)
    subscriptions, topic_counts, decode_errors = {}, {}, {}
    bag = None
    bag_log = None
    stop_reason = "error"
    failure = None
    source = None
    valid_frames = 0
    first_feedback = None
    metadata = {
        "version": VERSION, "started_utc": datetime.now(timezone.utc).isoformat(),
        "commands_sent": False, "planned_duration_s": args.duration, "joint_order": list(AXES),
        "waist_indices": [2, 3, 4, 5], "topics_requested": list(TOPICS),
        "time_semantics": "MotorState has no header/device timestamp. Saved times are subscriber receive clocks. No device synchronization claimed.",
        "units": {"MotorState.position": "deg", "MotorState.velocity": "deg/s",
                  "MotorState.current": "mA", "MotorState.temperature": "reserved; documented fixed zero"},
        "command_semantics": "Observed published requests, not proof of controller acceptance or interpolated servo targets. MotorCmd/Result raw scaling is not changed.",
        "source_reference": "User-provided torso interface document, 2026-11-10",
        "ros_distro": os.environ.get("ROS_DISTRO"), "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"),
    }
    save_json(run / "capture_metadata.json", metadata)

    with (run / "feedback_actual.csv").open("w", encoding="utf-8", newline="") as feedback, \
         (run / "messages_raw.jsonl").open("w", encoding="utf-8") as raw, \
         (run / "commands_raw.jsonl").open("w", encoding="utf-8") as commands:
        writer = csv.writer(feedback)
        writer.writerow(FIELDS)
        feedback.flush()

        def callback(topic, typ, message):
            nonlocal valid_frames, first_feedback
            clocks = (time.time_ns(), time.monotonic_ns(), node.get_clock().now().nanoseconds)
            payload = message_to_ordereddict(message)
            topic_counts[topic] = topic_counts.get(topic, 0) + 1
            event = dict(zip(FIELDS[:3], clocks))
            event.update(topic=topic, type=typ, message_index=topic_counts[topic], message=payload)
            line = json.dumps(event, ensure_ascii=False) + "\n"
            raw.write(line)
            if topic in COMMAND_TOPICS:
                commands.write(line)
                commands.flush()
            if topic == source:
                try:
                    rows = state_rows(payload, clocks, topic, topic_counts[topic])
                except ValueError as exc:
                    key = str(exc)
                    if key not in decode_errors:
                        print(f"反馈格式提示：{key}；完整原始消息继续保存。", flush=True)
                    decode_errors[key] = decode_errors.get(key, 0) + 1
                else:
                    writer.writerows(rows)
                    feedback.flush()
                    valid_frames += 1
                    if first_feedback is None:
                        first_feedback = time.monotonic()
                        print(f"已收到并保存六轴反馈，腰部为索引 2～5；反馈话题 {source}。", flush=True)
                        print("采集正在运行，可在另一终端使用原控制软件动作。Ctrl+C 结束采集，不控制机器人停止。", flush=True)
            raw.flush()

        def discover():
            topics = dict(node.get_topic_names_and_types())
            for topic in TOPICS:
                if topic in subscriptions or len(topics.get(topic, [])) != 1:
                    continue
                typ = topics[topic][0]
                try:
                    cls = get_message(typ)
                except (ImportError, AttributeError, ModuleNotFoundError) as exc:
                    decode_errors.setdefault(f"无法加载消息定义 {topic} {typ}: {exc}", 1)
                    continue
                endpoints = node.get_publishers_info_by_topic(topic)
                if not endpoints:
                    continue
                best_effort = topic in ("/MotorState", "/motor_state/array") or any(
                    e.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT for e in endpoints)
                transient = all(e.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL for e in endpoints)
                qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1000,
                                 reliability=ReliabilityPolicy.BEST_EFFORT if best_effort else ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL if transient else DurabilityPolicy.VOLATILE)
                subscriptions[topic] = node.create_subscription(
                    cls, topic, lambda message, t=topic, ty=typ: callback(t, ty, message), qos)
            return topics

        try:
            deadline = time.monotonic() + 15
            while True:
                topic_types = dict(node.get_topic_names_and_types())
                try:
                    source = select_state_topic(topic_types, args.state_topic)
                    break
                except RuntimeError:
                    if time.monotonic() >= deadline:
                        raise
                    rclpy.spin_once(node, timeout_sec=0.2)
            print("正在归档消息定义和腰部节点参数，完成后开始计时采集。", flush=True)
            snapshot(run, node, topic_types)
            metadata["feedback_topic"] = source
            command = bag_command(help_result.stdout, run / "rosbag", TOPICS)
            metadata["rosbag_command"] = command
            save_json(run / "capture_metadata.json", metadata)
            bag_log = (run / "rosbag_record.log").open("w", encoding="utf-8")
            bag = subprocess.Popen(command, stdout=bag_log, stderr=subprocess.STDOUT,
                                   stdin=subprocess.DEVNULL, start_new_session=True)
            discover()
            wait_started = time.monotonic()
            next_discovery, next_status = wait_started + 1, wait_started + 5
            while rclpy.ok():
                bag_code = bag.poll()
                if bag_code is not None:
                    raise RuntimeError(f"rosbag 提前退出（{bag_code}），请查看 rosbag_record.log")
                rclpy.spin_once(node, timeout_sec=0.1)
                now = time.monotonic()
                if first_feedback is None and now - wait_started > 15:
                    raise RuntimeError("15 秒内没有收到有效的 36 项 MotorState；已保存原始消息和错误信息")
                if first_feedback is not None and args.duration and now - first_feedback >= args.duration:
                    stop_reason = "duration_reached"
                    break
                if now >= next_discovery:
                    discover()
                    next_discovery = now + 1
                if now >= next_status:
                    count = sum(topic_counts.get(t, 0) for t in COMMAND_TOPICS)
                    print(f"已保存反馈 {valid_frames} 帧；命令话题消息 {count} 条。", flush=True)
                    next_status = now + 5
            else:
                stop_reason = "ros_shutdown"
        except KeyboardInterrupt:
            stop_reason = "operator_ctrl_c"
        except Exception as exc:
            failure = str(exc)
        finally:
            raw.flush()
            commands.flush()
            feedback.flush()
            if bag is not None:
                code, forced = stop_bag(bag)
                metadata.update(rosbag_exit_code=code, rosbag_forced_stop=forced)
                if forced or code not in (0, -signal.SIGINT, 128 + signal.SIGINT):
                    failure = failure or f"rosbag 未正常结束，退出码 {code}；原始文件已保留"
            if bag_log is not None:
                bag_log.close()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    if bag is not None:
        try:
            info = subprocess.run(["ros2", "bag", "info", str(run / "rosbag")], capture_output=True,
                                  text=True, timeout=30)
            (run / "rosbag_info.txt").write_text(info.stdout + info.stderr, encoding="utf-8")
            if info.returncode or not (run / "rosbag" / "metadata.yaml").is_file():
                failure = failure or "rosbag 元数据未正常生成；查看 rosbag_info.txt，保留全部文件"
        except Exception as exc:
            failure = failure or f"无法读取 rosbag 元数据：{exc}"
    command_count = sum(topic_counts.get(t, 0) for t in COMMAND_TOPICS)
    metadata.update(ended_utc=datetime.now(timezone.utc).isoformat(), stop_reason=stop_reason,
                    feedback_frames=valid_frames, observed_topic_counts=topic_counts,
                    decode_errors=decode_errors, error=failure,
                    acquisition_status="error" if failure or not valid_frames else "recorded",
                    parameter_readiness="not_assessed", command_messages_observed=command_count)
    save_json(run / "capture_metadata.json", metadata)
    # A complete directory manifest, including bag files and interface definitions.
    with (run / "SHA256SUMS").open("w", encoding="utf-8") as manifest:
        for path in sorted(run.rglob("*")):
            if path.is_file() and path.name != "SHA256SUMS":
                with path.open("rb") as file:
                    digest = hashlib.sha256()
                    for block in iter(lambda: file.read(1024 * 1024), b""):
                        digest.update(block)
                manifest.write(f"{digest.hexdigest()}  {path.relative_to(run).as_posix()}\n")
    print(f"采集文件已保存：{run}\n反馈：{valid_frames} 帧；命令：{command_count} 条。", flush=True)
    if not command_count:
        print("本次未收到所列命令话题的消息；不能把实际位置当成下发目标。", flush=True)
    if failure:
        print(f"采集未完整完成：{failure}", file=sys.stderr, flush=True)
        return 2
    return 0 if valid_frames else 2


def main():
    parser = argparse.ArgumentParser(description="只读采集腰部：原始 rosbag、六轴反馈 CSV、命令与状态 JSONL。")
    parser.add_argument("duration", nargs="?", type=int, default=1200, help="记录秒数；0 表示持续记录直到 Ctrl+C")
    parser.add_argument("--output", type=Path, default=Path.home() / "robotcal_data", help="采集根目录，每次新建 waist_时间戳 子目录")
    parser.add_argument("--state-topic", choices=("/MotorState", "/motor_state/array"), help="指定主反馈话题；默认优先 /MotorState")
    args = parser.parse_args()
    if args.duration < 0:
        parser.error("时长须为非负整数")
    if not shutil.which("ros2"):
        parser.error("请先加载现场厂家 ROS 2 环境")
    try:
        return capture(args)
    except ImportError as exc:
        print(f"现场 Python 缺少 ROS 依赖：{exc}。请用厂家 ROS 环境的 Python，不要使用离线包 runtime。", file=sys.stderr)
        return 2
    except Exception as exc:
        print(f"采集未完成：{exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

