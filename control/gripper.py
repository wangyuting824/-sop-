#!/usr/bin/env python3
"""ROS gripper opening trials and capture. The documented position unit is mm."""
from __future__ import annotations

import argparse
from contextlib import ExitStack
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys
import time

VERSION = "20261111-gripper-opening-ltr-v4"
STATUS_TOPIC = "/end_effector/status"
SET_SERVICE = "/end_effector/set_position"
PROPERTIES_SERVICE = "/end_effector/get_properties"
ENDPOINT_SERVICES = {"open": "/end_effector/open", "close": "/end_effector/close"}
FIXED_POINTS_MM = (10.0, 20.0, 40.0, 60.0, 80.0)
FEEDBACK_FIELDS = ["received_unix_ns", "received_monotonic_ns", "subscriber_ros_time_ns",
                   "message_index", "header_stamp_sec", "header_stamp_nanosec", "slot_id",
                   "jaw_mm", "velocity_raw_json", "effort_raw_json", "moving", "connected",
                   "estopped", "operational", "backend", "kind", "detail_json"]
POINT_FIELDS = ["point_id", "slot_id", "cycle", "direction", "command", "target_mm", "feedback_mm",
                "message_index", "feedback_received_unix_ns", "manual_recorded_unix_ns",
                "external_opening_mm", "left_finger_angle_deg", "right_finger_angle_deg"]


def numeric(value):
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)


def slots_for(selection):
    return ("left", "right") if selection == "both" else (selection,)


def fixed_protocol(args):
    return not args.record_only and args.positions_mm is None and args.delta_mm is None


def validate_fixed_support(properties, slots):
    if tuple(slots) != ("left", "right"):
        raise ValueError("固定测量含无 slot 参数的 open/close 服务，请用 both；不能把该服务伪装为单侧命令")
    if set(properties) != set(slots):
        raise ValueError("末端模块配置的 slot 不止 left/right；open/close 的作用范围不能按两侧假定")
    for slot in slots:
        if "open_close" not in properties[slot].get("capabilities", []):
            raise ValueError(f"{slot} 未声明 open_close 能力；未调用开合服务")


def validate_properties(response, slots):
    if not response.get("success"):
        raise RuntimeError("读取夹爪属性失败：" + str(response.get("message")))
    properties = {}
    for unit in response.get("units", []):
        slot = unit.get("slot_id")
        if slot in properties:
            raise ValueError("属性中存在重复 slot_id")
        properties[slot] = unit
    for slot in slots:
        unit = properties.get(slot, {})
        if (unit.get("category") != "gripper" or unit.get("position_dim") != 1
                or unit.get("position_unit") != "mm"):
            raise ValueError(f"{slot} 必须为 position_dim=1、position_unit=mm 的 gripper；实际属性：{unit}")
    return properties


def status_units(payload, slots):
    units = {}
    for unit in payload.get("units", []):
        slot = unit.get("slot_id")
        if slot in units:
            raise ValueError("状态中存在重复 slot_id")
        units[slot] = unit
    for slot in slots:
        unit = units.get(slot)
        if unit is None:
            raise ValueError(f"状态缺少 {slot} 夹爪")
        position = unit.get("position", [])
        if len(position) != 1 or not numeric(position[0]) or position[0] < 0:
            raise ValueError(f"{slot} 开口反馈不是单一非负有限毫米值")
        if unit.get("category") != "gripper":
            raise ValueError(f"{slot} 状态 category 不是 gripper")
        if type(unit.get("connected")) is not bool or type(unit.get("moving")) is not bool:
            raise ValueError(f"{slot} connected/moving 字段不完整")
    return units


def make_plan(q0, args):
    if args.positions_mm is not None:
        points = args.positions_mm
        if len(points) < 2 or any(not numeric(x) or x < 0 for x in points):
            raise ValueError("--positions-mm 需要至少两个非负有限毫米值")
        if any(b <= a for a, b in zip(points, points[1:])):
            raise ValueError("--positions-mm 请按开口从小到大填写，不要重复；脚本自动生成反向过程")
        cycle = [("opening", x) for x in points] + [("closing", x) for x in reversed(points[:-1])]
    else:
        if q0 < args.delta_mm:
            raise ValueError(f"当前开口 {q0:g} mm 不足以缩小 {args.delta_mm:g} mm；请用 --positions-mm 指定已确认行程内的测量点")
        cycle = [("closing", q0 - args.delta_mm), ("opening", q0)]
    plan = [(i, direction, value) for i in range(1, args.cycles + 1) for direction, value in cycle]
    if abs(plan[-1][2] - q0) > 1e-9:
        plan.append((0, "return_start", q0))
    return plan


def parse_measurement(line):
    if not line.strip():
        return ["", "", ""]
    parts = [p.strip() for p in line.split(",")]
    if len(parts) == 1:
        parts += ["", ""]
    if len(parts) != 3:
        raise ValueError("输入卡尺读数 mm；也可输入三列：开口mm,左指角度deg,右指角度deg")
    result = [float(p) if p else "" for p in parts]
    if any(v != "" and not math.isfinite(v) for v in result) or (result[0] != "" and result[0] < 0):
        raise ValueError("外测值须为有限数，开口不能为负")
    return result


def run(args):
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from rosidl_runtime_py.convert import message_to_ordereddict
    from end_effector.msg import EndEffectorStatus, EndEffectorCommand
    from end_effector.srv import GetProperties, SetPosition
    from std_srvs.srv import Trigger

    selected = slots_for(args.slot)
    fixed = fixed_protocol(args)
    measure = args.measure or fixed
    folder = args.output.expanduser() / datetime.now().strftime("gripper_%Y%m%d_%H%M%S_%f")
    folder.mkdir(parents=True, exist_ok=False)
    print(f"版本：{VERSION}\n数据：{folder}\n单位：开口 mm；没有手指角度反馈。", flush=True)
    print("speed=0、max_effort=0 使用原节点设置；speed 非零时按厂商原单位，不统一换算。", flush=True)
    metadata = {"version": VERSION, "started_utc": datetime.now(timezone.utc).isoformat(),
                "arguments": {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
                "status": "starting", "angle_calibration_computed": False,
                "fixed_protocol": fixed, "fixed_points_mm": list(FIXED_POINTS_MM) if fixed else None,
                "command_semantics": "SetPosition timeout=0 acknowledges dispatch. Trigger open/close has no slot argument; endpoints are observations, not assumed 0/85mm. Fresh status checks follow replies.",
                "time_semantics": "Host receive clocks and original status header; hardware synchronization not assumed",
                "velocity_semantics": "Preserved as raw array; supplied interface does not specify velocity unit",
                "effort_semantics": "Reported gripper scalar force N may be [0]; not assumed to be measured force"}
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("robotcal_gripper_" + ("capture" if args.record_only else "motion"))
    latest, received, frame, malformed = None, None, 0, None
    points, history, subscriptions = [], [], []
    collecting_point = False
    command_count = 0
    status, failure = "interrupted", None

    with ExitStack() as stack:
        def open_log(name):
            return stack.enter_context((folder / name).open("w", encoding="utf-8", newline=""))
        raw, commands, events = open_log("messages_raw.jsonl"), open_log("commands_sent.jsonl"), open_log("events.jsonl")
        feedback = open_log("feedback_actual.csv")
        csv_writer = csv.writer(feedback)
        csv_writer.writerow(FEEDBACK_FIELDS)
        point_stream = open_log("measurement_points.csv")
        point_writer = csv.DictWriter(point_stream, fieldnames=POINT_FIELDS)
        point_writer.writeheader()
        tty = None

        def clocks():
            return dict(received_unix_ns=time.time_ns(), received_monotonic_ns=time.monotonic_ns(),
                        subscriber_ros_time_ns=node.get_clock().now().nanoseconds)

        def log(stream, obj):
            stream.write(json.dumps(obj, ensure_ascii=False) + "\n")
            stream.flush()

        def event(kind, **fields):
            log(events, dict(event=kind, **clocks(), **fields))

        def receive_status(message):
            nonlocal latest, received, frame, malformed
            data, clock = message_to_ordereddict(message), clocks()
            log(raw, dict(topic=STATUS_TOPIC, type="end_effector/msg/EndEffectorStatus", message=data, **clock))
            try:
                units = status_units(data, selected)
            except (ValueError, TypeError) as exc:
                malformed = str(exc)
                event("invalid_status", error=malformed)
                return
            frame += 1
            latest, received = data, time.monotonic()
            stamp = data.get("header", {}).get("stamp", {})
            for slot in selected:
                unit = units[slot]
                csv_writer.writerow([*clock.values(), frame, stamp.get("sec", ""), stamp.get("nanosec", ""), slot,
                                     unit["position"][0], json.dumps(unit.get("velocity", [])), json.dumps(unit.get("effort", [])),
                                     unit["moving"], unit["connected"], data.get("estopped"), data.get("operational"),
                                     unit.get("backend", ""), unit.get("kind", ""), unit.get("detail_json", "")])
                if collecting_point:
                    history.append((frame, clock["received_unix_ns"], slot, unit["position"][0], unit["moving"]))
            feedback.flush()

        def receive_command(topic, message):
            log(raw, dict(topic=topic, type="end_effector/msg/EndEffectorCommand",
                          message=message_to_ordereddict(message), **clocks()))

        def pump():
            rclpy.spin_once(node, timeout_sec=0.05)
            if not rclpy.ok():
                raise RuntimeError("ROS 已退出")
            if malformed:
                raise RuntimeError(malformed)

        def ready(for_motion=True):
            if latest is None or received is None or time.monotonic() - received > args.feedback_timeout_s:
                raise RuntimeError("夹爪反馈中断或过期")
            units = status_units(latest, selected)
            if for_motion:
                if latest.get("estopped") is not False or latest.get("operational") is not True:
                    raise RuntimeError("末端处于软急停或不可操作状态；不再下发")
                for slot in selected:
                    if not units[slot]["connected"]:
                        raise RuntimeError(f"{slot} 夹爪未连接；不再下发")
            return units

        def hold(seconds, motion=True):
            until = time.monotonic() + seconds
            while time.monotonic() < until:
                pump()
                ready(motion)

        def wait_service(client, name, motion=False):
            until = time.monotonic() + args.timeout_s
            while not client.service_is_ready():
                pump()
                if motion:
                    ready()
                if time.monotonic() >= until:
                    raise RuntimeError(f"服务不可用：{name}")

        def wait_future(future, motion=False):
            until = time.monotonic() + args.timeout_s
            while not future.done():
                pump()
                if motion:
                    ready()
                if time.monotonic() >= until:
                    raise RuntimeError("服务响应超时；不重试、不再下发。超时不表示原命令已撤销")
            result = future.result()
            if result is None:
                raise RuntimeError("服务未返回有效结果")
            return message_to_ordereddict(result)

        def move(client, slot, target, point_id):
            nonlocal command_count
            units = ready()
            if any(units[s]["moving"] for s in selected):
                raise RuntimeError("发送前检测到夹爪仍在运动；不再下发")
            request = SetPosition.Request()
            request.slot_id, request.position = slot, [float(target)]
            request.max_effort, request.speed, request.timeout = args.max_effort, args.speed, 0.0
            # Zero timeout uses documented dispatch mode, supporting backends
            # without blocking_actuate. A positive service reply alone is not arrival.
            command_count += 1
            log(commands, dict(event="request", command_id=command_count, point_id=point_id,
                               service=SET_SERVICE, request=message_to_ordereddict(request), **clocks()))
            before = frame
            response = wait_future(client.call_async(request), motion=True)
            log(commands, dict(event="response", command_id=command_count, point_id=point_id,
                               response=response, **clocks()))
            if response.get("success") is not True:
                raise RuntimeError("夹爪拒绝命令：" + str(response.get("message")))
            deadline, stable_since, last_frame = time.monotonic() + args.timeout_s, None, before
            while True:
                pump()
                unit = ready()[slot]
                now = time.monotonic()
                if frame != last_frame:
                    last_frame = frame
                    arrived = not unit["moving"] and abs(unit["position"][0] - target) <= args.tolerance_mm
                    stable_since = (stable_since if stable_since is not None else now) if arrived else None
                    if stable_since is not None and now - stable_since >= args.settle_s:
                        event("feedback_arrived", point_id=point_id, slot_id=slot, target_mm=target,
                              actual_mm=unit["position"][0])
                        return
                if now >= deadline:
                    raise RuntimeError(f"{slot} 未在 {args.timeout_s:g}s 内到达 {target:g}mm；不再下发")

        def endpoint(client, action, expected=None):
            nonlocal command_count
            units = ready()
            if any(units[s]["moving"] for s in selected):
                raise RuntimeError("调用端点服务前夹爪仍在运动；不再下发")
            request = Trigger.Request()
            command_count += 1
            log(commands, dict(event="request", command_id=command_count, service=ENDPOINT_SERVICES[action],
                               scope="module_service_without_slot_argument", request=message_to_ordereddict(request), **clocks()))
            before, sent_at = frame, time.monotonic()
            response = wait_future(client.call_async(request), motion=True)
            log(commands, dict(event="response", command_id=command_count, service=ENDPOINT_SERVICES[action],
                               response=response, **clocks()))
            if response.get("success") is not True:
                raise RuntimeError(f"{action} 服务拒绝：{response.get('message')}")
            deadline, stable_since, last_frame = time.monotonic()+args.timeout_s, None, before
            anchor = None
            while True:
                pump()
                units, now = ready(), time.monotonic()
                if frame != last_frame:
                    last_frame = frame
                    values = {s: units[s]["position"][0] for s in selected}
                    still = not any(units[s]["moving"] for s in selected)
                    matches = expected is None or all(abs(values[s]-expected[s]) <= args.tolerance_mm for s in selected)
                    if not still or not matches or now-sent_at < 1.0:
                        anchor, stable_since = None, None
                    elif anchor is None or any(abs(values[s]-anchor[s]) > args.tolerance_mm for s in selected):
                        anchor, stable_since = values, now
                    elif now-stable_since >= max(0.5, args.settle_s):
                        event("endpoint_observed_stopped", command=action, feedback_mm=values,
                              note="Observed settled positions after Trigger; not a verified mechanical limit")
                        return values
                if now >= deadline:
                    raise RuntimeError(f"{action} 后双侧未在超时内保持停稳或未回到已记录端点；不再下发")

        def measured_point(slot, cycle, direction, target, point_id, command="set_position"):
            nonlocal collecting_point
            reference = ready()[slot]["position"][0] if target is None else target
            baseline = frame
            history.clear()
            collecting_point = True
            try:
                hold(args.dwell_s)
            finally:
                collecting_point = False
            samples = [s for s in history if s[0] > baseline and s[2] == slot]
            if not samples or any(s[4] or abs(s[3] - reference) > args.tolerance_mm for s in samples):
                raise RuntimeError("测量停留期间反馈偏离目标或仍在运动；保留原始记录，不生成有效测量点")
            last = samples[-1]
            point = dict(zip(POINT_FIELDS, [point_id, slot, cycle, direction, command,
                         "" if target is None else target, last[3], last[0], last[1], "", "", "", ""]))
            # Write the point before optional manual entry so interruption never loses it.
            points.append(point)
            point_writer.writerow(point)
            point_stream.flush()
            if tty is not None:
                import select
                label = command.upper() if target is None else f"{target:g} mm"
                print(f"测量点 {point_id}：{slot}，指令 {label}，反馈 {last[3]:.4f} mm。量完并取下卡尺后输入读数(mm)，回车进入下一测点；无法测量输入 s 标记未测：", flush=True)
                while True:
                    pump()
                    unit = ready()[slot]
                    if unit["moving"] or abs(unit["position"][0]-reference) > args.tolerance_mm:
                        raise RuntimeError("手动测量期间夹爪位置改变；不再下发")
                    if select.select([tty], [], [], 0)[0]:
                        line = tty.readline()
                        if line == "":
                            raise RuntimeError("测量终端已关闭")
                        if fixed and not line.strip():
                            print("请填实际卡尺读数，或输入 s 标记未测；空回车不会移动。", flush=True)
                            continue
                        try:
                            external = parse_measurement("") if line.strip().lower() == "s" else parse_measurement(line)
                        except ValueError as exc:
                            print(str(exc), flush=True)
                            continue
                        for key, value in zip(POINT_FIELDS[-3:], external):
                            point[key] = value
                        point["manual_recorded_unix_ns"] = time.time_ns()
                        event("external_measurement", point=point)
                        # Persist manual readings immediately without duplicate point rows.
                        point_stream.seek(0)
                        point_stream.truncate()
                        point_writer.writeheader()
                        point_writer.writerows(points)
                        point_stream.flush()
                        break

        def run_fixed(properties):
            validate_fixed_support(properties, selected)
            if args.cycles != 1:
                raise ValueError("固定测量现在每侧只运行一遍；请去掉 --cycles 或设为 1")
            sequence = ["close", *FIXED_POINTS_MM, "open"]
            metadata["fixed_sequence"] = sequence
            metadata["fixed_slot_order"] = list(selected)
            metadata["endpoint_scope"] = "Trigger has no slot_id. Each side is measured once in left/right order; global endpoints may also move the other side."
            print("固定流程：先左夹爪，再右夹爪；各测 CLOSE -> 10 -> 20 -> 40 -> 60 -> 80 mm -> OPEN 一遍。", flush=True)
            print("OPEN/CLOSE 使用原模块 Trigger 服务，没有左右参数，另一侧也可能跟随端点动作。", flush=True)
            event("fixed_plan", sequence=sequence, slot_order=selected, cycles=1, execute=args.execute)
            if not args.execute:
                return "planned_no_motion"
            clients = {name: node.create_client(Trigger, service) for name, service in ENDPOINT_SERVICES.items()}
            position_client = node.create_client(SetPosition, SET_SERVICE)
            for name, client in clients.items():
                wait_service(client, ENDPOINT_SERVICES[name], motion=True)
            wait_service(position_client, SET_SERVICE, motion=True)
            hold(args.dwell_s)
            observed = []
            metadata["observed_endpoint_feedback_mm"] = observed
            for slot in selected:
                print(f"开始测量 {slot}：CLOSE -> OPEN，单次。", flush=True)
                closed = endpoint(clients["close"], "close")
                observed.append(dict(measured_slot=slot, command="close", feedback_mm=closed))
                measured_point(slot, 1, "opening_start", None, f"{slot}_close", "close")
                if closed[slot] > FIXED_POINTS_MM[0] + args.tolerance_mm:
                    raise RuntimeError(f"{slot} CLOSE 后反馈 {closed[slot]:g} mm 高于第一个 10 mm 测点；不能按单向张开继续")
                for target in FIXED_POINTS_MM:
                    point_id = f"{slot}_opening_{target:g}"
                    unit = ready()[slot]
                    if unit["moving"] or abs(unit["position"][0]-target) > args.tolerance_mm:
                        move(position_client, slot, target, point_id)
                    else:
                        event("already_at_target_no_command", point_id=point_id, target_mm=target)
                    measured_point(slot, 1, "opening", target, point_id)
                opened = endpoint(clients["open"], "open")
                observed.append(dict(measured_slot=slot, command="open", feedback_mm=opened))
                if opened[slot] < FIXED_POINTS_MM[-1] - args.tolerance_mm:
                    raise RuntimeError(f"{slot} OPEN 后反馈 {opened[slot]:g} mm 小于上一个 80 mm 测点；保留原始记录并停止")
                measured_point(slot, 1, "opening", None, f"{slot}_open", "open")
            return "completed_by_feedback"

        try:
            if measure and args.execute:
                # Only read manual measurements here; prompts use stdout.
                # r+ requires a seekable BufferedRandom stream, unlike a TTY.
                tty = stack.enter_context(open("/dev/tty", "r", encoding="utf-8"))
            subscriptions.append(node.create_subscription(EndEffectorStatus, STATUS_TOPIC, receive_status,
                                 QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.VOLATILE)))
            for topic in ("/end_effector/command", "/business_manager/end_effector/command"):
                subscriptions.append(node.create_subscription(EndEffectorCommand, topic,
                                     lambda msg, t=topic: receive_command(t, msg),
                                     QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)))
            properties_client = node.create_client(GetProperties, PROPERTIES_SERVICE)
            wait_service(properties_client, PROPERTIES_SERVICE)
            request = GetProperties.Request()
            request.slot_id = "all"
            response = wait_future(properties_client.call_async(request))
            metadata["properties_response"] = response
            properties = validate_properties(response, selected)
            deadline = time.monotonic() + args.timeout_s
            while latest is None:
                pump()
                if time.monotonic() >= deadline:
                    raise RuntimeError("未收到夹爪状态")
            event("capture_started", selected_slots=selected)
            print("已开始保存夹爪反馈。", flush=True)
            if args.record_only:
                started = time.monotonic()
                while args.duration_s == 0 or time.monotonic() - started < args.duration_s:
                    pump()
                    ready(False)
                status = "recorded"
            elif fixed:
                ready()
                status = run_fixed(properties)
            else:
                initial = ready()
                if any(initial[s]["moving"] for s in selected):
                    raise RuntimeError("夹爪尚未停稳；未发送目标")
                origins = {slot: initial[slot]["position"][0] for slot in selected}
                plans = {slot: make_plan(origins[slot], args) for slot in selected}
                metadata.update(origins_mm=origins, plans=plans)
                event("plan", origins_mm=origins, plans=plans, execute=args.execute)
                for slot, plan in plans.items():
                    print(f"{slot} 起点 {origins[slot]:g} mm；计划：" + " -> ".join(f"{v:g}" for _, _, v in plan), flush=True)
                if args.execute:
                    client = node.create_client(SetPosition, SET_SERVICE)
                    wait_service(client, SET_SERVICE, motion=True)
                    hold(args.dwell_s)
                    for slot, plan in plans.items():
                        if abs(ready()[slot]["position"][0]-origins[slot]) > args.tolerance_mm:
                            raise RuntimeError(f"{slot} 起点在准备期间发生变化；未发送该夹爪后续目标")
                        for index, (cycle, direction, target) in enumerate(plan, 1):
                            point_id = f"{slot}_{index:03d}"
                            print(f"{point_id}: {target:g} mm ({direction})", flush=True)
                            unit = ready()[slot]
                            if unit["moving"] or abs(unit["position"][0]-target) > args.tolerance_mm:
                                move(client, slot, target, point_id)
                            else:
                                event("already_at_target_no_command", point_id=point_id, target_mm=target)
                            measured_point(slot, cycle, direction, target, point_id)
                    status = "completed_by_feedback"
                else:
                    status = "planned_no_motion"
        except KeyboardInterrupt:
            status = "recorded_interrupted" if args.record_only else "interrupted"
            failure = None if args.record_only else "用户中断；不再下发。已经发出的目标由原控制系统处理。"
        except Exception as exc:
            status, failure = "failed", str(exc)
        finally:
            event("finished", status=status, error=failure)
            metadata.update(status=status, error=failure, feedback_frames=frame,
                            requests_sent=command_count, measured_points=len(points),
                            ended_utc=datetime.now(timezone.utc).isoformat())
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    (folder / "run.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")
    report = ["夹爪开口原始记录", f"运行状态：{status}", f"错误：{failure or '无'}",
              f"反馈帧：{frame}；下发请求：{command_count}；停稳测量点：{len(points)}",
              "只保存命令、原始反馈和手工读数；不计算均值、标准差、误差、角度或标定参数。",
              "单位：mm。外测列为空时表示未测，不表示零。",
              "原始状态保存在 messages_raw.jsonl，完整请求/响应保存在 commands_sent.jsonl。"]
    (folder / "report.txt").write_text("\n".join(report) + "\n", encoding="utf-8")
    with (folder / "SHA256SUMS").open("w", encoding="utf-8") as manifest:
        for path in sorted(folder.iterdir()):
            if path.is_file() and path.name != "SHA256SUMS":
                digest = hashlib.sha256()
                with path.open("rb") as stream:
                    for block in iter(lambda: stream.read(1024 * 1024), b""):
                        digest.update(block)
                manifest.write(f"{digest.hexdigest()}  {path.name}\n")
    print(f"状态：{status}；数据和 report.txt：{folder}", flush=True)
    if failure:
        print(failure, file=sys.stderr)
    return 2 if failure else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("slot", nargs="?", choices=("left", "right", "both"), default="both")
    targets = parser.add_mutually_exclusive_group()
    targets.add_argument("--positions-mm", type=float, nargs="+", help="递增开口测量点，自动生成反向及返回起点")
    targets.add_argument("--delta-mm", type=float, help="兼容局部往返；默认不填时运行 OPEN/CLOSE + 固定 10/20/40/60/80 mm 测量")
    parser.add_argument("--cycles", type=int, default=1, help="固定测量仅允许 1；自定义往返模式可指定次数")
    parser.add_argument("--speed", type=float, default=0.0, help="厂商原单位；0 沿用节点 vmax，不代表停止")
    parser.add_argument("--max-effort", type=float, default=0.0, help="0 沿用节点 fmax")
    parser.add_argument("--timeout-s", type=float, default=10.0)
    parser.add_argument("--feedback-timeout-s", type=float, default=1.0)
    parser.add_argument("--dwell-s", type=float, default=2.0)
    parser.add_argument("--settle-s", type=float, default=0.3)
    parser.add_argument("--tolerance-mm", type=float, default=0.2)
    parser.add_argument("--measure", action="store_true", help="自定义模式手动测量；默认固定流程自动在每点等待卡尺读数")
    parser.add_argument("--measurement-note", default="", help="外测工具、角度零位及正方向说明")
    parser.add_argument("--record-only", action="store_true")
    parser.add_argument("--duration-s", type=float, default=1200.0, help="只采集时的时长，0 持续到 Ctrl+C")
    parser.add_argument("--output", type=Path, default=Path.home() / "robotcal_data")
    parser.add_argument("--execute", action="store_true", help="实际发动作；默认只读属性/状态并打印计划")
    args = parser.parse_args()
    for key in ("timeout_s", "feedback_timeout_s", "dwell_s", "settle_s", "tolerance_mm"):
        value = getattr(args, key)
        if not math.isfinite(value) or value <= 0:
            parser.error(f"{key} 须为正有限数")
    if args.delta_mm is not None and (not math.isfinite(args.delta_mm) or args.delta_mm <= 0):
        parser.error("delta_mm 须为正有限数")
    for key in ("speed", "max_effort", "duration_s"):
        value = getattr(args, key)
        if not math.isfinite(value) or value < 0:
            parser.error(f"{key} 须为非负有限数")
    if args.cycles < 1 or (args.record_only and (args.execute or args.measure)):
        parser.error("循环次数须为正；record-only 不能同时 execute/measure")
    try:
        with ExitStack() as stack:
            if args.execute:
                import fcntl
                lock = stack.enter_context((Path.home() / ".robotcal_gripper_motion.lock").open("a"))
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            return run(args)
    except (ImportError, OSError) as exc:
        print(f"启动失败：{exc}。请使用现场已加载 ROS 2 的系统 Python。", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

