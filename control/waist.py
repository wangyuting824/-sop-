#!/usr/bin/env python3
"""Yaw-only position trials through the documented /WaistCtrl interface."""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time

VERSION = "20261111-waist-yaw-v3"
COMMAND_TOPIC = "/WaistCtrl"
COMMAND_TYPE = "torso_chassis_interfaces/msg/TorsoWaistCtrl"
STATE_TYPE = "std_msgs/msg/Float32MultiArray"
AXES = ("HEAD_YAW", "HEAD_PITCH", "WAIST_YAW", "WAIST_PITCH_1", "WAIST_PITCH_2", "WAIST_PITCH_3")
YAW = 2
WAIST_INDICES = (2, 3, 4, 5)


def parse_state(payload):
    values = payload.get("data")
    offset = payload.get("layout", {}).get("data_offset", 0)
    if type(offset) is not int or offset < 0 or not isinstance(values, (list, tuple)) or len(values) - offset != 36:
        raise ValueError("MotorState 需要 36 项有效数据：六轴各六项")
    values = values[offset:]
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
        raise ValueError("MotorState 含无效数值")
    return [list(values[i * 6:i * 6 + 6]) for i in range(6)]


def amplitude(value, unit):
    if not math.isfinite(value) or value <= 0:
        raise ValueError("摆幅须为正有限数")
    result = math.degrees(value) if unit == "rad" else value
    if not math.isfinite(result):
        raise ValueError("摆幅换算无效")
    return result


def targets(q0, delta_deg, cycles):
    result = []
    for cycle in range(1, cycles + 1):
        for label, offset in (("positive", delta_deg), ("center_after_positive", 0),
                              ("negative", -delta_deg), ("center_after_negative", 0)):
            result.append((f"cycle_{cycle}_{label}", q0 + offset))
    return result


def check_state(state, received_at, now, max_age):
    if state is None or received_at is None:
        raise RuntimeError("尚未收到有效腰部反馈；不发送目标")
    age = now - received_at
    if age > max_age:
        raise RuntimeError(f"腰部反馈中断或过期：距上一帧 {age:.3f} 秒，阈值 {max_age:g} 秒；不再发送目标")
    # This trial controls waist yaw. Keep raw head feedback in the recording,
    # but do not use head fault/enabled fields as a waist motion precondition.
    # The other waist joints support the torso and remain part of this check.
    for i in WAIST_INDICES:
        axis = state[i]
        if axis[4] != 0 or axis[5] != 1:
            raise RuntimeError(f"{AXES[i]} fault={axis[4]} enabled={axis[5]}；未发送后续目标")


def run_sequence(args, io):
    q0 = io.position()
    plan = targets(q0, args.delta_deg, args.cycles)
    if any(not math.isfinite(value) or not -90.0 <= value <= 90.0 for _, value in plan):
        raise ValueError("偏航目标超出文档的 [-90, 90] deg 范围；未发送目标")
    io.event("plan", q0_deg=q0, delta_deg=args.delta_deg, targets_deg=plan,
             execute_requested=args.execute, scope="WAIST_YAW only")
    print(f"偏航起点 q0={q0:.6f} deg；正负摆幅 {args.delta:g} {args.delta_unit} = {args.delta_deg:.6f} deg。", flush=True)
    if not args.execute:
        for name, value in plan:
            print(f"计划 {name}: waist_yaw={value:.6f} deg")
        return "planned"
    io.connect()
    io.hold(args.dwell_s)
    if abs(io.position() - q0) > args.tolerance_deg:
        raise RuntimeError("准备期间偏航实际位置已变化；未发送目标，请在原控制器停稳后重试")
    for name, target in plan:
        print(f"{name}: waist_yaw={target:.6f} deg", flush=True)
        io.move(name, target)
        io.hold(args.dwell_s)
    return "completed_by_feedback"


def capture_and_move(args):
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from rosidl_runtime_py.convert import message_to_ordereddict
    from std_msgs.msg import Float32MultiArray

    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("robotcal_waist_yaw_motion")
    run = args.output.expanduser() / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    run.mkdir(parents=True, exist_ok=False)
    print(f"脚本版本：{VERSION}\n范围：仅腰部偏航 WAIST_YAW\n日志：{run}", flush=True)
    print("WaistCtrl 没有速度/加速度字段，运动使用原控制器设置。其他轴目标由原控制器部分更新逻辑保留。", flush=True)
    print("状态检查范围：仅腰部四轴。头部故障/使能原样记录，不作为腰部动作的停止条件。", flush=True)
    state, received_at, receive_count, parse_error = None, None, 0, None
    source, published_targets = None, 0
    status, error = "interrupted", None
    subscriptions = []
    with (run / "commands_sent.jsonl").open("w", encoding="utf-8") as command_log, \
         (run / "events.jsonl").open("w", encoding="utf-8") as events, \
         (run / "feedback_raw.jsonl").open("w", encoding="utf-8") as raw, \
         (run / "feedback_actual.csv").open("w", encoding="utf-8", newline="") as feedback:
        writer = csv.writer(feedback)
        writer.writerow(["received_unix_ns", "received_monotonic_ns", "source_topic", "message_index",
                         "joint_index", "joint_name", "position_deg", "velocity_deg_s", "current_mA",
                         "temperature_reserved_raw", "fault_code_raw", "enabled_raw"])

        def event(kind, **fields):
            events.write(json.dumps(dict(event=kind, received_unix_ns=time.time_ns(),
                                         received_monotonic_ns=time.monotonic_ns(), **fields), ensure_ascii=False) + "\n")
            events.flush()

        def feedback_callback(message):
            nonlocal state, received_at, receive_count, parse_error
            wall, monotonic = time.time_ns(), time.monotonic_ns()
            payload = message_to_ordereddict(message)
            raw.write(json.dumps(dict(received_unix_ns=wall, received_monotonic_ns=monotonic,
                                      topic=source, message=payload), ensure_ascii=False) + "\n")
            raw.flush()
            try:
                decoded = parse_state(payload)
            except ValueError as exc:
                parse_error = str(exc)
                return
            state, received_at = decoded, time.monotonic()
            receive_count += 1
            for i, axis in enumerate(decoded):
                writer.writerow([wall, monotonic, source, receive_count, i, AXES[i], *axis])
            feedback.flush()

        def result_callback(topic, message):
            event("controller_result_unmatched", topic=topic, message=message_to_ordereddict(message))

        class Interface:
            publisher = None
            def event(self, kind, **fields):
                event(kind, **fields)

            def pump(self, seconds=0.05):
                rclpy.spin_once(node, timeout_sec=seconds)
                if not rclpy.ok():
                    raise RuntimeError("ROS 上下文已退出；不再发送目标")
                if parse_error:
                    raise RuntimeError(parse_error)

            def require_state(self):
                check_state(state, received_at, time.monotonic(), args.feedback_timeout_s)

            def position(self):
                self.require_state()
                return state[YAW][0]

            def prepare(self):
                # First message import / publisher creation can block. Finish
                # them before sampling q0, so setup time cannot age that sample.
                from torso_chassis_interfaces.msg import TorsoWaistCtrl
                self.message_class = TorsoWaistCtrl
                self.publisher = node.create_publisher(
                    TorsoWaistCtrl, COMMAND_TOPIC,
                    QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE))
                event("publisher_prepared", topic=COMMAND_TOPIC)

            def connect(self):
                until = time.monotonic() + 10
                while not self.publisher.get_subscription_count():
                    self.pump()
                    self.require_state()
                    if time.monotonic() >= until:
                        raise RuntimeError("/WaistCtrl 没有匹配的订阅者；未发送目标")

            def hold(self, seconds):
                until = time.monotonic() + seconds
                while time.monotonic() < until:
                    self.pump()
                    self.require_state()

            def move(self, label, target):
                nonlocal published_targets
                self.require_state()
                message = self.message_class()
                stamp = node.get_clock().now().to_msg()
                message.header.stamp = stamp
                message.node_header.stamp = stamp
                message.node_header.node_name = node.get_name()
                message.waist_yaw = float(target)
                payload = message_to_ordereddict(message)
                command_log.write(json.dumps(dict(event="publish_attempt", label=label,
                                                   host_unix_ns=time.time_ns(), host_monotonic_ns=time.monotonic_ns(),
                                                   topic=COMMAND_TOPIC, type=COMMAND_TYPE, message=payload), ensure_ascii=False) + "\n")
                command_log.flush()
                before = receive_count
                self.publisher.publish(message)
                published_targets += 1
                event("published", label=label, target_deg=target)
                deadline, stable_since, last_sample = time.monotonic() + args.timeout_s, None, before
                while True:
                    self.pump()
                    self.require_state()
                    now = time.monotonic()
                    # Only newly received feedback can satisfy arrival. A topic
                    # result without a request ID is logged, not treated as an Action success.
                    if receive_count != last_sample:
                        last_sample = receive_count
                        arrived = abs(state[YAW][0] - target) <= args.tolerance_deg and abs(state[YAW][1]) <= args.settle_velocity_deg_s
                        stable_since = (stable_since if stable_since is not None else now) if arrived else None
                        if stable_since is not None and now - stable_since >= args.settle_s:
                            event("feedback_arrived", label=label, target_deg=target, actual_deg=state[YAW][0])
                            return
                    if now >= deadline:
                        raise RuntimeError(f"{label} 未在 {args.timeout_s:g} 秒内由反馈确认到位；不再发送后续目标")

        try:
            deadline = time.monotonic() + 10
            while True:
                graph = dict(node.get_topic_names_and_types())
                source = next((t for t in (args.state_topic,) if t and STATE_TYPE in graph.get(t, [])), None) if args.state_topic else next(
                    (t for t in ("/MotorState", "/motor_state/array") if STATE_TYPE in graph.get(t, [])), None)
                if source:
                    break
                if time.monotonic() >= deadline:
                    raise RuntimeError("找不到 /MotorState 或 /motor_state/array 的标准六轴反馈")
                rclpy.spin_once(node, timeout_sec=0.1)
            interface = Interface()
            if args.execute:
                interface.prepare()
            subscriptions.append(node.create_subscription(Float32MultiArray, source, feedback_callback,
                                 QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)))
            from rosidl_runtime_py.utilities import get_message
            for topic in ("/MotorCmdResult", "/motor_cmd_result"):
                if len(graph.get(topic, [])) == 1:
                    subscriptions.append(node.create_subscription(get_message(graph[topic][0]), topic,
                                         lambda msg, t=topic: result_callback(t, msg), 10))
            deadline = time.monotonic() + 10
            while state is None:
                interface.pump()
                if time.monotonic() >= deadline:
                    raise RuntimeError("10 秒内未收到有效腰部反馈")
            event("feedback_ready", topic=source, feedback_frames=receive_count,
                  feedback_timeout_s=args.feedback_timeout_s)
            status = run_sequence(args, interface)
        except KeyboardInterrupt:
            error = "用户中断；停止后续下发。已发目标是否停止由原控制系统决定。"
        except Exception as exc:
            status, error = "failed", str(exc)
        finally:
            event("finished", status=status, error=error, published_targets=published_targets,
                  feedback_frames=receive_count,
                  feedback_age_s=None if received_at is None else time.monotonic() - received_at)
            (run / "run.json").write_text(json.dumps({
                "version": VERSION, "status": status, "error": error, "execute": args.execute,
                "scope": "WAIST_YAW only", "command_topic": COMMAND_TOPIC,
                "checked_axes": [AXES[i] for i in WAIST_INDICES],
                "head_status_policy": "record_only",
                "delta": args.delta, "delta_unit": args.delta_unit, "delta_deg": args.delta_deg,
                "cycles": args.cycles, "feedback_frames": receive_count,
                "published_targets": published_targets, "feedback_topic": source,
                "feedback_timeout_s": args.feedback_timeout_s,
                "feedback_age_s": None if received_at is None else time.monotonic() - received_at,
                "time_semantics": "MotorState has no header; timestamps are host receive times",
                "result_semantics": "Arrival judged from fresh feedback, not a controller Action acceptance/result",
            }, ensure_ascii=False, indent=2), encoding="utf-8")
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    if error:
        print(f"停止后续目标：{error}\n日志：{run}", file=sys.stderr, flush=True)
        return 2
    print(f"{'偏航往返完成（按反馈判定）' if args.execute else '仅生成计划，未发运动目标'}；日志：{run}", flush=True)
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("delta", nargs="?", type=float, default=0.05, help="正负摆幅，默认 0.05 rad")
    parser.add_argument("--delta-unit", choices=("rad", "deg"), default="rad")
    parser.add_argument("--cycles", type=int, default=3)
    parser.add_argument("--dwell-s", type=float, default=2.0, help="初始和每次反馈到位后的记录等待时间")
    parser.add_argument("--timeout-s", type=float, default=8.0, help="每段等待反馈到位的超时")
    parser.add_argument("--feedback-timeout-s", type=float, default=1.0)
    parser.add_argument("--tolerance-deg", type=float, default=0.1)
    parser.add_argument("--settle-s", type=float, default=0.3)
    parser.add_argument("--settle-velocity-deg-s", type=float, default=0.5)
    parser.add_argument("--state-topic", choices=("/MotorState", "/motor_state/array"))
    parser.add_argument("--output", type=Path, default=Path.home() / "robotcal_data" / "waist_motion_logs")
    parser.add_argument("--execute", action="store_true", help="实际发送 /WaistCtrl；默认只读取反馈和保存计划")
    args = parser.parse_args()
    try:
        args.delta_deg = amplitude(args.delta, args.delta_unit)
        if args.cycles <= 0:
            raise ValueError("循环次数须为正整数")
        for key in ("dwell_s", "timeout_s", "feedback_timeout_s", "tolerance_deg", "settle_s", "settle_velocity_deg_s"):
            value = getattr(args, key)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{key} 须为正有限数")
        import fcntl
        with (Path.home() / ".robotcal_waist_motion.lock").open("a") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            return capture_and_move(args)
    except (ImportError, OSError, ValueError, RuntimeError) as exc:
        print(f"未执行完成：{exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

