#!/usr/bin/env python3
"""Use the site's MoveJoint action, with fresh native state for every axis.

No goal is sent without --execute. CLI speed/acceleration values are passed
through unchanged: this tool does not assume whether the ROS wrapper uses
0..1 ratios or 0..100 percentages. Command units must also be explicit.
"""
from __future__ import annotations

import argparse
import csv
from contextlib import nullcontext
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import shlex
import signal
import subprocess
import threading
import time

import yaml

ACTION = "/manipulator/move_joint"
TYPE = "manipulator/action/MoveJoint"
ARMS = {"left": 0, "right": 1}
SCRIPT_VERSION = "20261111-arm-motion-step-v5"


def vector(value):
    if not isinstance(value, (list, tuple)) or len(value) != 7:
        raise ValueError("需要七轴数值")
    if any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in value):
        raise ValueError("七轴值必须是有限数")
    return list(map(float, value))


def targets(q0, index, delta, cycles):
    q0 = vector(q0)
    result = [("initial_hold", q0.copy())]
    for cycle in range(1, cycles + 1):
        for label, offset in (("positive", delta), ("center_after_positive", 0.0),
                              ("negative", -delta), ("center_after_negative", 0.0)):
            q = q0.copy()
            q[index] += offset
            result.append((f"cycle_{cycle}_{label}", q))
    return result


def parse_arm_state(text):
    messages = []
    for document in yaml.safe_load_all(text):
        if document is None:
            continue
        if not isinstance(document, dict) or not isinstance(document.get("arms"), list):
            raise ValueError("ArmStateArray 格式不正确")
        messages.append(document)
    if not messages:
        raise ValueError("未收到非空 ArmStateArray")
    # ROS 2 echo emits YAML separators and may produce several samples.
    # Use the last received sample; never silently use the first stale one.
    return messages[-1]


def delta_in_degrees(value, unit):
    if not math.isfinite(value) or value <= 0:
        raise ValueError("摆幅须为正有限数")
    if unit not in ("deg", "rad"):
        raise ValueError("摆幅单位须为 deg 或 rad")
    converted = math.degrees(value) if unit == "rad" else value
    if not math.isfinite(converted):
        raise ValueError("换算后摆幅不是有限数")
    return converted


def command_positions(target_deg, unit):
    target_deg = vector(target_deg)
    if unit == "rad":
        return [math.radians(x) for x in target_deg]
    if unit not in (None, "deg"):
        raise ValueError("目标单位须为 deg 或 rad")
    return target_deg


def read_state():
    p = subprocess.run(["ros2", "topic", "echo", "/manipulator/arm_state", "--once"],
                       capture_output=True, text=True, timeout=10, check=True)
    data = parse_arm_state(p.stdout)
    ids = [a["arm_id"] for a in data["arms"]]
    if len(ids) != len(set(ids)) or not set(ARMS.values()).issubset(ids):
        raise ValueError("双臂 ID 缺失或重复")
    states = {a["arm_id"]: a for a in data["arms"]}
    for arm_id in ARMS.values():
        arm = states[arm_id]
        vector(arm["joint_pos_act"])
    return data, states


CSV_FIELDS = ["received_unix_s", "stamp_sec", "stamp_nanosec", "arm_id", "frame_seq",
              "joint_index", "joint_name", "joint_pos_act_raw", "joint_vel_act_raw",
              "position_rad_from_deg", "velocity_rad_s_from_deg_s"]


def feedback_rows(data, received):
    rows = []
    for arm in data["arms"]:
        arm_id = arm["arm_id"]
        q, dq = vector(arm["joint_pos_act"]), vector(arm["joint_vel_act"])
        role = next((role for role, value in ARMS.items() if value == arm_id), str(arm_id))
        for index, (position, velocity) in enumerate(zip(q, dq)):
            rows.append([received, data["stamp"]["sec"], data["stamp"]["nanosec"],
                         arm_id, arm["frame_seq"], index, f"{role}_joint_{index}", position,
                         velocity, math.radians(position), math.radians(velocity)])
    return rows


class FeedbackRecorder:
    """Continuously archive received native state; never publishes a command."""
    def __init__(self, directory):
        self.directory = directory
        self.ready = threading.Event()
        self.stopping = threading.Event()
        self.failure = None
        self.samples = 0
        self.process = None
        self.thread = None
        self.stderr = None

    def __enter__(self):
        self.stderr = (self.directory / "feedback_ros2_stderr.log").open("w", encoding="utf-8")
        try:
            self.process = subprocess.Popen(["ros2", "topic", "echo", "/manipulator/arm_state"],
                                            stdout=subprocess.PIPE, stderr=self.stderr, text=True, bufsize=1)
            self.thread = threading.Thread(target=self._read, daemon=True)
            self.thread.start()
            if not self.ready.wait(10):
                raise RuntimeError("连续采集尚未收到实际位置/速度；未发送动作")
            self.check()
            print(f"已开始连续保存实际位置/速度：{self.directory / 'feedback_actual.csv'}", flush=True)
            return self
        except BaseException:
            self.close()
            raise

    def _read(self):
        try:
            with (self.directory / "feedback_actual.csv").open("w", encoding="utf-8", newline="") as output, \
                    (self.directory / "feedback_raw.yaml").open("w", encoding="utf-8") as raw:
                writer = csv.writer(output)
                writer.writerow(CSV_FIELDS)
                buffer = []
                for line in self.process.stdout:
                    raw.write(line)
                    if line.strip() != "---":
                        buffer.append(line)
                        continue
                    if not any(line.strip() for line in buffer):
                        buffer.clear()
                        continue
                    data = parse_arm_state("".join(buffer))
                    buffer.clear()
                    writer.writerows(feedback_rows(data, time.time()))
                    self.samples += 1
                    output.flush()
                    raw.flush()
                    self.ready.set()
                # A process may end after a final message without a separator.
                if any(line.strip() for line in buffer):
                    data = parse_arm_state("".join(buffer))
                    writer.writerows(feedback_rows(data, time.time()))
                    self.samples += 1
                    self.ready.set()
                if not self.stopping.is_set():
                    raise RuntimeError("ROS 反馈采集进程意外退出")
        except Exception as exc:
            self.failure = exc
            self.ready.set()

    def check(self):
        if self.failure is not None:
            raise RuntimeError(f"连续位置/速度记录失败：{self.failure}")

    def close(self):
        self.stopping.set()
        if self.process is not None and self.process.poll() is None:
            try:
                self.process.send_signal(signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        if self.thread is not None:
            self.thread.join(timeout=10)
        if self.process is not None and self.process.stdout is not None:
            self.process.stdout.close()
        if self.stderr is not None:
            self.stderr.close()

    def __exit__(self, *unused):
        self.close()
        print(f"连续反馈已保存：{self.samples} 条消息；每条包含现场双臂关节位置/速度。", flush=True)


def result_ok(text, success_code):
    statuses = list(re.finditer(r"(?m)^Goal finished with status:\s*([A-Z_]+)\s*$", text))
    if not statuses:
        reason = "动作被控制器拒绝" if "Goal was rejected." in text else "未收到动作最终状态"
        raise RuntimeError(f"{reason}；不再发送下一目标")
    status_match = statuses[-1]
    status = status_match.group(1)
    headers = list(re.finditer(r"(?m)^Result:\s*\n", text[:status_match.start()]))
    if not headers:
        raise RuntimeError(f"动作状态 {status}，但缺少厂家结果；不再发送下一目标")
    # ros2action indents only the first YAML line with four spaces.
    # Removing that prefix restores the actual message_to_yaml indentation.
    result_text = text[headers[-1].end():status_match.start()].lstrip()
    documents = [doc for doc in yaml.safe_load_all(result_text) if doc is not None]
    if len(documents) != 1:
        raise RuntimeError(f"动作状态 {status}，厂家结果不是单一有效消息；停止后续下发")
    document = documents[0]
    if not isinstance(document, dict) or not isinstance(document.get("result"), dict):
        raise RuntimeError("无法解析厂家动作结果；停止后续下发")
    result = document["result"]
    if type(result.get("code")) is not int or type(result.get("vendor_code")) is not int:
        raise RuntimeError("厂家动作结果缺少整数 code/vendor_code；停止后续下发")
    if status != "SUCCEEDED" or result["code"] != success_code or result["vendor_code"] != 0:
        raise RuntimeError(f"厂家动作 {status}：code={result['code']}，vendor_code={result['vendor_code']}，"
                           f"message={result.get('message', '')}；不再发送下一目标")
    return result


def send(goal, log_path, timeout, success_code):
    command = ["ros2", "action", "send_goal", ACTION, TYPE, json.dumps(goal), "--feedback"]
    print(shlex.join(command), flush=True)
    with log_path.open("w", encoding="utf-8") as log:
        log.write(shlex.join(command) + "\n")
        log.flush()
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                   env={**os.environ, "PYTHONUNBUFFERED": "1"}, bufsize=1)
        received = []

        def stream():
            for line in process.stdout:
                received.append(line)
                log.write(line)
                log.flush()
                print(line, end="", flush=True)

        reader = threading.Thread(target=stream, daemon=True)
        reader.start()
        try:
            process.wait(timeout=timeout)
        except (KeyboardInterrupt, subprocess.TimeoutExpired):
            try:
                process.send_signal(signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            raise RuntimeError("试验中断，已请求 CLI 中断并停止后续下发。取消是否生效须由现场确认；必要时使用原控制系统停止。")
        finally:
            reader.join(timeout=10)
            if reader.is_alive():
                raise RuntimeError("动作 CLI 输出未能结束；停止后续下发")
            process.stdout.close()
    text = "".join(received)
    if process.returncode != 0:
        raise RuntimeError(f"ros2 命令退出码 {process.returncode}")
    return result_ok(text, success_code)


def main():
    p = argparse.ArgumentParser(prog="arm_motion.sh", allow_abbrev=False, description="逐轴往返并记录反馈；摆幅输入统一使用弧度。默认 ±0.05 rad。")
    p.add_argument("--version", action="version", version=SCRIPT_VERSION)
    p.add_argument("arm", nargs="?", default="both", choices=("left", "right", "both"))
    p.add_argument("joint", nargs="?", default="all", help="现场轴号 1..7，分别对应 *_joint_0..6；或 all")
    amplitude = p.add_mutually_exclusive_group()
    amplitude.add_argument("--delta-rad", type=float, default=0.05, help="往返摆幅，单位始终为 rad，默认 0.05")
    amplitude.add_argument("--step-rad", type=float, help="仅单轴移动一次：相对当前实际位置的有符号增量(rad)，例如 0.05 或 -0.05")
    p.add_argument("--cycles", type=int, default=3)
    p.add_argument("--dwell-s", type=float, default=2.0)
    p.add_argument("--vel-percent", type=float, help="原样传入 MoveJoint；沿用现场已验证的值")
    p.add_argument("--acc-percent", type=float, help="原样传入 MoveJoint；沿用现场已验证的值")
    p.add_argument("--command-unit", choices=("deg", "rad"), help="按 MoveJoint 封装源码确认的目标单位填写")
    p.add_argument("--timeout-s", type=float, help="可选通信等待超时；默认一直等控制器结果")
    p.add_argument("--success-code", type=int, default=0, help="保守默认仅接受 result.code=0 且 vendor_code=0")
    p.add_argument("--output", type=Path, default=Path.home() / "robotcal_data" / "motion_logs")
    p.add_argument("--execute", action="store_true")
    a = p.parse_args()
    if a.step_rad is not None and (a.arm == "both" or a.joint == "all"):
        p.error("单次移动请指定 left/right 和轴号 1..7，不能使用 both/all")
    a.delta, a.delta_unit = (abs(a.step_rad) if a.step_rad is not None else a.delta_rad), "rad"
    try:
        a.delta_deg = delta_in_degrees(a.delta, a.delta_unit)
    except ValueError as exc:
        p.error(str(exc))
    try:
        joints = list(range(7)) if a.joint == "all" else [int(a.joint) - 1]
    except ValueError:
        p.error("轴号须为 1..7 或 all")
    if any(j not in range(7) for j in joints):
        p.error("轴号须为 1..7 或 all")
    for name in ("delta_deg",):
        if not math.isfinite(getattr(a, name)) or getattr(a, name) <= 0:
            p.error(f"{name} 须为正有限数")
    if a.cycles < 1:
        p.error("循环次数须为正整数")
    if not math.isfinite(a.dwell_s) or a.dwell_s < 0:
        p.error("间隔须为非负有限数")
    if a.timeout_s is not None and (not math.isfinite(a.timeout_s) or a.timeout_s <= 0):
        p.error("可选超时须为正有限数")
    if a.execute and (a.command_unit is None or a.vel_percent is None or a.acc_percent is None):
        p.error("执行需要显式指定 --command-unit、--vel-percent 和 --acc-percent")
    for value in (a.vel_percent, a.acc_percent):
        if value is not None and (not math.isfinite(value) or value <= 0):
            p.error("速度和加速度必须为正有限数；数值量纲沿用厂家 ROS 封装")
    # Linux-only advisory lock prevents concurrent copies of this test runner.
    import fcntl
    with open(Path.home() / ".robotcal_move_joint.lock", "a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError("已有逐轴动作程序占用运行锁，可能是 SSH 断开前的任务。请先在原控制系统确认并结束旧任务；本次不重复下发。") from exc
        run_trials(a, joints)


def run_trials(a, joints):
    run = a.output / (datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    run.mkdir(parents=True, exist_ok=False)
    print(f"脚本版本：{SCRIPT_VERSION}", flush=True)
    print(f"模式：{'执行' if a.execute else '仅生成计划，不发送动作'}；记录：{run}", flush=True)
    print("ArmState 原始反馈按现场度制读取；不会按 JointState 的常规弧度定义再次换算。", flush=True)
    if a.step_rad is None:
        print(f"每轴正负摆幅：{a.delta:g} {a.delta_unit} = {a.delta_deg:.6f} deg；循环：{a.cycles} 次。", flush=True)
    else:
        print(f"单次移动：{a.arm} J{joints[0]+1}，相对当前实际位置 {a.step_rad:+g} rad = {math.degrees(a.step_rad):+.6f} deg。只发送一个目标，不自动返回。", flush=True)
    if a.execute:
        print(f"目标单位：{a.command_unit}；vel_percent 原始值：{a.vel_percent}；acc_percent 原始值：{a.acc_percent}。请按现场 ROS 封装的定义使用。", flush=True)
    with FeedbackRecorder(run) if a.execute else nullcontext(None) as recorder:
        run_axes(a, joints, run, recorder)


def run_axes(a, joints, run, recorder):
    arms = ("left", "right") if a.arm == "both" else (a.arm,)
    for arm in arms:
        arm_id = ARMS[arm]
        for index in joints:
            directory = run / f"{arm}_J{index + 1}"
            directory.mkdir()
            raw, states = read_state()
            (directory / "state_before.json").write_text(json.dumps(raw, ensure_ascii=False, indent=2), encoding="utf-8")
            if a.execute:
                trajectory = states[arm_id].get("traj_plan_state")
                if type(trajectory) is not int or trajectory != 0:
                    raise RuntimeError(f"{arm} 当前 traj_plan_state={trajectory}，本次未下发目标。该臂须先结束/清理未完成轨迹；另一个终端的采集不受此脚本控制。")
            q0 = vector(states[arm_id]["joint_pos_act"])
            if a.step_rad is None:
                sequence = targets(q0, index, a.delta_deg, a.cycles)
            else:
                single_target = q0.copy()
                single_target[index] += math.degrees(a.step_rad)
                sequence = [("initial_hold", q0.copy()), ("single_step", single_target)]
            for step, (label, target_deg) in enumerate(sequence):
                target = command_positions(target_deg, a.command_unit)
                initial_hold = label == "initial_hold"
                goal = None if initial_hold else dict(arm_id=arm_id, target_joint=target, vel_percent=a.vel_percent,
                                                       acc_percent=a.acc_percent, planned=True)
                path = directory / f"{step:02d}_{label}.json"
                path.write_text(json.dumps({"goal": goal, "target_deg": target_deg,
                                           "dispatch": "record_only" if initial_hold else "move_joint",
                                           "command_unit": a.command_unit, "delta": a.delta, "delta_unit": a.delta_unit,
                                           "delta_deg": a.delta_deg, "step_rad": a.step_rad,
                                           "motion_mode": "single_step" if a.step_rad is not None else "oscillation",
                                           "created_utc": datetime.now(timezone.utc).isoformat()},
                                          ensure_ascii=False, indent=2), encoding="utf-8")
                if not a.execute:
                    print(f"计划 {arm} J{index + 1} {label}: {target_deg}")
                    continue
                recorder.check()
                if initial_hold:
                    print(f"{arm} J{index + 1}：起始静止记录 {a.dwell_s:g} 秒，不发送零位移规划。", flush=True)
                    time.sleep(a.dwell_s)
                    recorder.check()
                    after, _ = read_state()
                    path.with_suffix(".state_after.json").write_text(json.dumps(after, ensure_ascii=False, indent=2), encoding="utf-8")
                    continue
                send(goal, path.with_suffix(".log"), a.timeout_s, a.success_code)
                recorder.check()
                after, _ = read_state()
                path.with_suffix(".state_after.json").write_text(json.dumps(after, ensure_ascii=False, indent=2), encoding="utf-8")
                time.sleep(a.dwell_s)
    print(f"{'动作序列完成' if a.execute else '计划已生成'}；原始目标与日志：{run}")


if __name__ == "__main__":
    try:
        main()
    except (Exception, KeyboardInterrupt) as exc:
        raise SystemExit(f"停止本次程序，不再下发后续目标：{exc}")

