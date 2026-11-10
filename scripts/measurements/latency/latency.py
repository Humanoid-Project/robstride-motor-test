#!/usr/bin/env python3
import argparse
import csv
import math
import os
import select
import socket
import struct
import sys
import time
from datetime import datetime

import can

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from common import (
    finite_float,
    shutdown_motor,
    resolve_model,
    HOST_ID, DEFAULT_INTERFACE, SPECS, RATED_TORQUE,
    RUN_MODE_INDEX, RUN_MODE_OPERATION,
    Motor, channel_for_id, active_brake, PLACEHOLDER_ARMATURE,
    validate_args, report_invalid_args,
    DEFAULT_LIMIT_MARGIN_RAD, joint_limit_for, exceeds_joint_limit,
    add_can_timeout_option, can_timeout_problem, arm_can_watchdog, install_stop_signals,
)
from robonex_common.can import drain
from robonex_common.protocol import COMM_FEEDBACK, COMM_OPERATION, float_to_uint, parse_arbitration_id

MODE_RUNNING = 2
TORQUE_CAP_FRACTION = 0.25
DEFAULT_TORQUE_STEPS = [0.5, -0.5, 1.0, -1.0]
DEFAULT_VEL_STEPS = [0.25, -0.25, 0.5, -0.5]
DEFAULT_POS_STEPS = [1.0, -1.0, 2.0, -2.0]
MAX_POS_STEP_DEG = 5.0
POS_DEVIATION_MARGIN_RAD = math.radians(1.0)
POS_SPEED_FACTOR = 2.0
POS_SPEED_MARGIN = 1.0
POS_MIN_ARMATURE_FRACTION = 0.5
FALLBACK_MIN_ARMATURE = 0.001
MAX_SHIFT_RAD = 0.15
MAX_HOLD_TIME = 2.0
MIN_REST_TIME = 0.1
MAX_REST_TIME = 2.0
MAX_REPEATS = 50
MAX_PHASE_STEPS = 20
MAX_PHASE_SPAN_MS = 5.0
REPLY_TIMEOUT = 0.05
ECHO_GRACE = 0.002
ECHO_CLOCK_SLACK = 0.0002
WARMUP_TIME = 0.2

SO_TIMESTAMPNS = 35
SO_TIMESTAMPING = 37
SOF_TIMESTAMPING_RX_HARDWARE = 1 << 2
SOF_TIMESTAMPING_RX_SOFTWARE = 1 << 3
SOF_TIMESTAMPING_SOFTWARE = 1 << 4
SOF_TIMESTAMPING_RAW_HARDWARE = 1 << 6
CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_FRAME_SIZE = 72
ANCILLARY_SIZE = 256

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
ARG_CHECKS = [("kp", "positive"), ("kd", "positive"), ("hold-time", "positive"),
              ("rest-time", "positive"), ("feedback-timeout", "positive")]
COLUMNS = ["trial", "repeat", "step_nm", "phase", "edge", "gap_s", "status",
           "t_send_mono", "t_send_wall", "tx_ts", "tx_hw_ts", "t_rx_mono", "rx_ts", "rx_hw_ts",
           "cmd_pos", "cmd_vel", "cmd_kp", "cmd_kd", "cmd_torque",
           "pos_rad", "vel_rad_s", "torque_nm", "temp_c", "mode", "fault"]


class Abort(Exception):
    pass


class Frame:
    __slots__ = ("timestamp", "hw_timestamp", "arbitration_id", "is_extended_id", "is_error_frame",
                 "is_remote_frame", "is_rx", "dlc", "data")


def parse_frame(raw, ancillary, msg_flags):
    can_id, dlc = struct.unpack_from("=IB", raw)
    frame = Frame()
    frame.timestamp = None
    frame.hw_timestamp = None
    for level, kind, payload in ancillary:
        if level != socket.SOL_SOCKET:
            continue
        if kind == SO_TIMESTAMPNS and len(payload) >= struct.calcsize("@ll"):
            seconds, nanos = struct.unpack_from("@ll", payload)
            frame.timestamp = seconds + nanos * 1e-9
        elif kind == SO_TIMESTAMPING and len(payload) >= struct.calcsize("@6l"):
            stamps = struct.unpack_from("@6l", payload)
            if frame.timestamp is None and (stamps[0] or stamps[1]):
                frame.timestamp = stamps[0] + stamps[1] * 1e-9
            if stamps[4] or stamps[5]:
                frame.hw_timestamp = stamps[4] * 1_000_000_000 + stamps[5]
    frame.is_extended_id = bool(can_id & CAN_EFF_FLAG)
    frame.is_remote_frame = bool(can_id & CAN_RTR_FLAG)
    frame.is_error_frame = bool(can_id & CAN_ERR_FLAG)
    frame.arbitration_id = can_id & (0x1FFFFFFF if frame.is_extended_id else 0x7FF)
    frame.is_rx = not bool(msg_flags & socket.MSG_DONTROUTE)
    frame.dlc = dlc
    frame.data = bytes(raw[8:8 + min(dlc, 8)])
    return frame


class StampedBus:
    def __init__(self, bus):
        self.bus = bus
        self.socket = bus.socket
        flags = (SOF_TIMESTAMPING_RX_HARDWARE | SOF_TIMESTAMPING_RX_SOFTWARE
                 | SOF_TIMESTAMPING_SOFTWARE | SOF_TIMESTAMPING_RAW_HARDWARE)
        self.socket.setsockopt(socket.SOL_SOCKET, SO_TIMESTAMPING, flags)

    def send(self, msg, timeout=None):
        self.bus.send(msg, timeout)

    def recv(self, timeout=None):
        ready, _, _ = select.select([self.socket], [], [], timeout)
        if not ready:
            return None
        raw, ancillary, msg_flags, _ = self.socket.recvmsg(CAN_FRAME_SIZE, ANCILLARY_SIZE)
        frame = parse_frame(raw, ancillary, msg_flags)
        if frame.timestamp is None:
            frame.timestamp = time.time()
        return frame

    def shutdown(self):
        self.bus.shutdown()


def open_bus(args):
    bus = can.interface.Bus(channel=args.channel, interface=args.interface, receive_own_messages=True)
    if not args.hw_timestamps:
        return bus
    try:
        return StampedBus(bus)
    except OSError as error:
        bus.shutdown()
        print(f"ERROR: SO_TIMESTAMPING refused on {args.channel} ({error}); run without --hw-timestamps.")
        return None


def torque_cap(model):
    return TORQUE_CAP_FRACTION * RATED_TORQUE[model]


def step_kind(args):
    return getattr(args, "step_kind", "torque")


def steps(args):
    kind = step_kind(args)
    if kind == "velocity":
        return args.vel_steps
    if kind == "position":
        return args.pos_steps
    return args.torque_steps


def step_torque(args, step):
    kind = step_kind(args)
    if kind == "velocity":
        return args.kd * step
    if kind == "position":
        return args.kp * math.radians(step)
    return step


def step_command(args, step):
    kind = step_kind(args)
    if kind == "velocity":
        return (0.0, step)
    if kind == "position":
        return (0.0, 0.0)
    return (step, 0.0)


def step_offset(args, step):
    return math.radians(step) if step_kind(args) == "position" else 0.0


def min_armature(model):
    known = PLACEHOLDER_ARMATURE.get(model)
    if known is None:
        return FALLBACK_MIN_ARMATURE
    return POS_MIN_ARMATURE_FRACTION * known


def runtime_limits(args):
    largest = max(abs(step_torque(args, s)) for s in steps(args))
    if step_kind(args) == "position":
        step = max(abs(math.radians(s)) for s in steps(args))
        natural = math.sqrt(args.kp / min_armature(args.model))
        return {"deviation": 2.0 * step + POS_DEVIATION_MARGIN_RAD,
                "speed": POS_SPEED_FACTOR * natural * step + POS_SPEED_MARGIN}
    return {"deviation": 2.0 * largest / args.kp + 0.1, "speed": 2.0 * largest / args.kd + 1.0}


def parse_args(argv=None):
    p = argparse.ArgumentParser(description="Measure command-to-torque latency with kernel CAN timestamps.")
    p.add_argument("--motor-id", type=lambda v: int(v, 0), required=True)
    p.add_argument("--model", choices=list(SPECS.keys()), default=None)
    p.add_argument("--kp", type=float, required=True, help="Position gain to hold with")
    p.add_argument("--kd", type=float, required=True, help="Velocity gain to hold with")
    p.add_argument("--step-kind", choices=["torque", "velocity", "position"], default="torque",
                   help="Step the feed-forward torque, the velocity target (through kd) or the position target "
                        "(through kp)")
    p.add_argument("--torque-steps", type=finite_float, nargs="+", default=None,
                   help=f"Feed-forward torque steps in N*m, signed; each at most "
                        f"{TORQUE_CAP_FRACTION:g} x the model's rated torque (default "
                        f"{' '.join(f'{v:g}' for v in DEFAULT_TORQUE_STEPS)})")
    p.add_argument("--vel-steps", type=finite_float, nargs="+", default=None,
                   help=f"With --step-kind velocity: velocity-target steps in rad/s, signed; kd x |step| at most "
                        f"{TORQUE_CAP_FRACTION:g} x the model's rated torque (default "
                        f"{' '.join(f'{v:g}' for v in DEFAULT_VEL_STEPS)})")
    p.add_argument("--pos-steps", type=finite_float, nargs="+", default=None,
                   help=f"With --step-kind position: position-target steps in degrees, signed; each at most "
                        f"{MAX_POS_STEP_DEG:g} deg and kp x |step| at most {TORQUE_CAP_FRACTION:g} x the model's "
                        f"rated torque (default {' '.join(f'{v:g}' for v in DEFAULT_POS_STEPS)})")
    p.add_argument("--repeats", type=int, default=8)
    p.add_argument("--hold-time", type=float, default=0.3,
                   help=f"Seconds each torque step is held, at most {MAX_HOLD_TIME:g}")
    p.add_argument("--rest-time", type=float, default=0.3,
                   help="Seconds at zero feed-forward torque before and after each step")
    p.add_argument("--phase-steps", type=int, default=4,
                   help="Number of different gaps inserted after a step frame, cycled over the repeats")
    p.add_argument("--phase-span-ms", type=float, default=0.6,
                   help="The gaps are spread evenly over this many milliseconds")
    p.add_argument("--hw-timestamps", action="store_true",
                   help="Also record the adapter's own timestamps (SO_TIMESTAMPING)")
    p.add_argument("--ignore-joint-limit", action="store_true")
    add_can_timeout_option(p)
    p.set_defaults(interface=DEFAULT_INTERFACE, host_id=HOST_ID,
                   feedback_timeout=0.3, limit_margin=DEFAULT_LIMIT_MARGIN_RAD, out=None)
    args = resolve_model(p, p.parse_args(argv))
    args.step_options = [name for name in ("torque_steps", "vel_steps", "pos_steps")
                         if getattr(args, name) is not None]
    if args.torque_steps is None:
        args.torque_steps = list(DEFAULT_TORQUE_STEPS)
    if args.vel_steps is None:
        args.vel_steps = list(DEFAULT_VEL_STEPS)
    if args.pos_steps is None:
        args.pos_steps = list(DEFAULT_POS_STEPS)
    return args


def argument_problems(args):
    problems = validate_args(args, args.model, ARG_CHECKS)
    problem = can_timeout_problem(args)
    if problem:
        problems.append(problem)
    spec = SPECS[args.model]
    cap = torque_cap(args.model)
    if args.motor_id == args.host_id:
        problems.append(f"--motor-id must differ from the host ID {args.host_id:#x}")
    if math.isfinite(args.kp) and args.kp > spec.kp_max:
        problems.append(f"--kp {args.kp} exceeds the {args.model.upper()} frame range {spec.kp_max}")
    if math.isfinite(args.kd) and args.kd > spec.kd_max:
        problems.append(f"--kd {args.kd} exceeds the {args.model.upper()} frame range {spec.kd_max}")
    if step_kind(args) == "velocity":
        problems.extend(velocity_problems(args, spec, cap))
    elif step_kind(args) == "position":
        problems.extend(position_problems(args, cap))
    else:
        if "vel_steps" in getattr(args, "step_options", ()):
            problems.append("--vel-steps needs --step-kind velocity")
        if "pos_steps" in getattr(args, "step_options", ()):
            problems.append("--pos-steps needs --step-kind position")
        for step in args.torque_steps:
            if not math.isfinite(step):
                problems.append(f"--torque-steps must be finite ({step})")
            elif step == 0.0:
                problems.append("--torque-steps must not contain 0")
            elif abs(step) > cap + 1e-9:
                problems.append(f"--torque-steps: |{step}| exceeds {cap:.2f} N*m "
                                f"({TORQUE_CAP_FRACTION:g} x the {args.model.upper()} rated torque "
                                f"{RATED_TORQUE[args.model]} N*m)")
        finite_steps = [abs(s) for s in args.torque_steps if math.isfinite(s)]
        if finite_steps and math.isfinite(args.kp) and args.kp > 0:
            shift = max(finite_steps) / args.kp
            if shift > MAX_SHIFT_RAD:
                problems.append(f"expected shift tau/kp = {math.degrees(shift):.1f} deg exceeds "
                                f"{math.degrees(MAX_SHIFT_RAD):.1f} deg; raise --kp or lower --torque-steps")
    if not 1 <= args.repeats <= MAX_REPEATS:
        problems.append(f"--repeats must be between 1 and {MAX_REPEATS} ({args.repeats})")
    if math.isfinite(args.hold_time) and args.hold_time > MAX_HOLD_TIME:
        problems.append(f"--hold-time must be at most {MAX_HOLD_TIME:g} ({args.hold_time})")
    if math.isfinite(args.rest_time) and not MIN_REST_TIME <= args.rest_time <= MAX_REST_TIME:
        problems.append(f"--rest-time must be between {MIN_REST_TIME:g} and {MAX_REST_TIME:g} ({args.rest_time})")
    if not 1 <= args.phase_steps <= MAX_PHASE_STEPS:
        problems.append(f"--phase-steps must be between 1 and {MAX_PHASE_STEPS} ({args.phase_steps})")
    if not (math.isfinite(args.phase_span_ms) and 0.0 <= args.phase_span_ms <= MAX_PHASE_SPAN_MS):
        problems.append(f"--phase-span-ms must be between 0 and {MAX_PHASE_SPAN_MS:g} ({args.phase_span_ms})")
    return problems


def position_problems(args, cap):
    problems = []
    options = getattr(args, "step_options", ())
    if "torque_steps" in options:
        problems.append("--torque-steps needs --step-kind torque; use --pos-steps")
    if "vel_steps" in options:
        problems.append("--vel-steps needs --step-kind velocity; use --pos-steps")
    kp_ok = math.isfinite(args.kp) and args.kp > 0
    for step in args.pos_steps:
        if not math.isfinite(step):
            problems.append(f"--pos-steps must be finite ({step})")
        elif step == 0.0:
            problems.append("--pos-steps must not contain 0")
        else:
            if abs(step) > MAX_POS_STEP_DEG + 1e-9:
                problems.append(f"--pos-steps: |{step}| deg exceeds {MAX_POS_STEP_DEG:g} deg")
            if kp_ok and args.kp * math.radians(abs(step)) > cap + 1e-9:
                problems.append(f"--pos-steps: kp x |{step}| deg = {args.kp * math.radians(abs(step)):.2f} N*m "
                                f"exceeds {cap:.2f} N*m ({TORQUE_CAP_FRACTION:g} x the {args.model.upper()} "
                                f"rated torque {RATED_TORQUE[args.model]} N*m); largest step at this kp "
                                f"{math.degrees(cap / args.kp):.2f} deg")
    return problems


def velocity_problems(args, spec, cap):
    problems = []
    if "torque_steps" in getattr(args, "step_options", ()):
        problems.append("--torque-steps needs --step-kind torque; use --vel-steps")
    if "pos_steps" in getattr(args, "step_options", ()):
        problems.append("--pos-steps needs --step-kind position")
    kd_ok = math.isfinite(args.kd) and args.kd > 0
    for step in args.vel_steps:
        if not math.isfinite(step):
            problems.append(f"--vel-steps must be finite ({step})")
        elif step == 0.0:
            problems.append("--vel-steps must not contain 0")
        else:
            if abs(step) > spec.v_max:
                problems.append(f"--vel-steps: |{step}| exceeds the {args.model.upper()} frame range "
                                f"{spec.v_max} rad/s")
            if kd_ok and args.kd * abs(step) > cap + 1e-9:
                problems.append(f"--vel-steps: kd x |{step}| = {args.kd * abs(step):.2f} N*m exceeds "
                                f"{cap:.2f} N*m ({TORQUE_CAP_FRACTION:g} x the {args.model.upper()} rated "
                                f"torque {RATED_TORQUE[args.model]} N*m)")
    finite_steps = [abs(s) for s in args.vel_steps if math.isfinite(s)]
    if finite_steps and kd_ok and math.isfinite(args.kp) and args.kp > 0:
        shift = args.kd * max(finite_steps) / args.kp
        if shift > MAX_SHIFT_RAD:
            problems.append(f"expected shift kd*v/kp = {math.degrees(shift):.1f} deg exceeds "
                            f"{math.degrees(MAX_SHIFT_RAD):.1f} deg; raise --kp or lower --vel-steps")
    return problems


def run_duration(args):
    return args.repeats * len(steps(args)) * (2 * args.rest_time + args.hold_time)


def confirm(args):
    largest = max(abs(step_torque(args, s)) for s in steps(args))
    print(f"Motor: ID {args.motor_id}, {args.model.upper()}, {args.channel}")
    if step_kind(args) == "position":
        limits = runtime_limits(args)
        print(f"Holding kp {args.kp}, kd {args.kd}; position target steps away from the position read after "
              f"enable and back")
        print(f"Position-target steps: {args.pos_steps} deg x {args.repeats}, initial torque kp*step up to "
              f"{largest:.2f} N*m (cap {torque_cap(args.model):.2f} N*m); velocity target and feed-forward "
              f"torque stay 0")
        print(f"Each step: {args.rest_time:g} s at the reference, {args.hold_time:g} s at reference + step, "
              f"{args.rest_time:g} s at the reference; about {run_duration(args):.0f} s in total")
        print(f"Abort if the joint moves more than {math.degrees(limits['deviation']):.2f} deg from the reference "
              f"or faster than {limits['speed']:.2f} rad/s")
        print()
        print("The joint moves by the step and back; it is not commanded to run.")
        print("WARNING: Verify zero calibration, secure the robot, and prepare the E-stop.")
        if not sys.stdin.isatty():
            print("ERROR: Run this command in an interactive terminal.")
            return False
        try:
            input("Press Enter to continue or Ctrl-C to cancel: ")
        except (KeyboardInterrupt, EOFError):
            print("\nCancelled.")
            return False
        return True
    print(f"Holding kp {args.kp}, kd {args.kd}; position target fixed at the position read after enable")
    if step_kind(args) == "velocity":
        print(f"Velocity-target steps: {args.vel_steps} rad/s x {args.repeats}, torque kd*v up to "
              f"{largest:.2f} N*m (cap {torque_cap(args.model):.2f} N*m); feed-forward torque stays 0")
    else:
        print(f"Feed-forward torque steps: {args.torque_steps} N*m x {args.repeats} "
              f"(cap {torque_cap(args.model):.2f} N*m)")
    print(f"Each step: {args.rest_time:g} s at 0, {args.hold_time:g} s at the step, {args.rest_time:g} s at 0; "
          f"about {run_duration(args):.0f} s in total")
    if step_kind(args) == "velocity":
        print(f"Expected position shift per step: kd*v/kp = {math.degrees(largest / args.kp):.2f} deg at most")
    else:
        print(f"Expected position shift per step: tau/kp = {math.degrees(largest / args.kp):.2f} deg at most")
    print()
    print("The joint is pushed against its own position hold; it is not commanded to run.")
    print("WARNING: Verify zero calibration, secure the robot, and prepare the E-stop.")
    if not sys.stdin.isatty():
        print("ERROR: Run this command in an interactive terminal.")
        return False
    try:
        input("Press Enter to continue or Ctrl-C to cancel: ")
    except (KeyboardInterrupt, EOFError):
        print("\nCancelled.")
        return False
    return True


def hw_seconds(value):
    if value is None:
        return ""
    return f"{value // 1_000_000_000}.{value % 1_000_000_000:09d}"


def stamp(value):
    return "" if value is None else repr(float(value))


class Link:
    def __init__(self, bus, motor, args):
        self.bus = bus
        self.motor = motor
        self.args = args
        self.counts = {"cycles": 0, "no_reply": 0, "no_echo": 0, "stale_reply": 0,
                       "stray_echo": 0, "other": 0, "error_frames": 0}

    def exchange(self, pos, torque, vel=0.0):
        motor = self.motor
        args = self.args
        spec = motor.spec
        raw_torque = float_to_uint(torque, spec.t_min, spec.t_max, 16)
        raw_vel = float_to_uint(vel, spec.v_min, spec.v_max, 16)
        vel_bytes = bytes([(raw_vel >> 8) & 0xFF, raw_vel & 0xFF])
        raw_pos = float_to_uint(pos, spec.p_min, spec.p_max, 16)
        pos_bytes = bytes([(raw_pos >> 8) & 0xFF, raw_pos & 0xFF])
        t_wall = time.time()
        t_mono = time.monotonic()
        motor.control(pos=pos, vel=vel, kp=args.kp, kd=args.kd, torque=torque)
        deadline = t_mono + REPLY_TIMEOUT
        echo = None
        reply = None
        early = None
        while True:
            now = time.monotonic()
            if reply is not None and echo is not None:
                break
            limit = deadline
            if echo is None and early is not None:
                limit = min(deadline, early[2] + ECHO_GRACE)
            if now >= limit:
                break
            msg = self.bus.recv(timeout=limit - now)
            if msg is None:
                continue
            seen = time.monotonic()
            if getattr(msg, "is_error_frame", False):
                self.counts["error_frames"] += 1
                continue
            if not msg.is_extended_id:
                self.counts["other"] += 1
                continue
            comm, data16, destination = parse_arbitration_id(msg.arbitration_id)
            if not getattr(msg, "is_rx", True):
                fresh = msg.timestamp >= t_wall - ECHO_CLOCK_SLACK
                if (echo is None and fresh and comm == COMM_OPERATION and destination == motor.motor_id
                        and data16 == raw_torque and bytes(msg.data[2:4]) == vel_bytes
                        and bytes(msg.data[0:2]) == pos_bytes):
                    echo = (msg.timestamp, getattr(msg, "hw_timestamp", None))
                    if early is not None:
                        self.counts["stale_reply"] += 1
                        early = None
                else:
                    self.counts["stray_echo"] += 1
                continue
            if comm != COMM_FEEDBACK or destination != args.host_id or (data16 & 0xFF) != motor.motor_id:
                self.counts["other"] += 1
                continue
            item = (msg.timestamp, getattr(msg, "hw_timestamp", None), seen, bytes(msg.data), data16)
            if echo is None:
                if early is not None:
                    self.counts["stale_reply"] += 1
                early = item
            else:
                reply = item
        status = "ok"
        if reply is None and early is not None:
            reply = early
            status = "no_echo"
        if reply is None:
            status = "no_reply"
        elif echo is None:
            status = "no_echo"
        self.counts["cycles"] += 1
        if status != "ok":
            self.counts[status] += 1
        feedback = None
        mode = ""
        fault = ""
        rx = ("", "", "")
        if reply is not None:
            feedback = motor.ingest_feedback(reply[3], data16=reply[4], now=reply[2])
            mode = motor.last_mode_status
            fault = motor.last_fault
            rx = (repr(reply[2]), stamp(reply[0]), hw_seconds(reply[1]))
        tx = (stamp(echo[0]), hw_seconds(echo[1])) if echo is not None else ("", "")
        measured = feedback[1:5] if feedback is not None else ("", "", "", "")
        record = (status, repr(t_mono), repr(t_wall), tx[0], tx[1], rx[0], rx[1], rx[2],
                  pos, vel, args.kp, args.kd, torque, *measured, mode, fault)
        return record, feedback


def check_feedback(motor, feedback, reference, args, limits):
    _, pos, vel, _, _, fault = feedback
    if motor.last_mode_status != MODE_RUNNING:
        raise Abort(f"motor left run mode (mode {motor.last_mode_status})")
    if fault:
        raise Abort(f"feedback fault bits {fault:#04x}")
    if not args.ignore_joint_limit and exceeds_joint_limit(pos, args.motor_id, args.limit_margin):
        raise Abort("joint_limit")
    if abs(pos - reference) > limits["deviation"]:
        raise Abort(f"position moved {math.degrees(pos - reference):+.2f} deg from the reference "
                    f"(limit {math.degrees(limits['deviation']):.2f})")
    if abs(vel) > limits["speed"]:
        raise Abort(f"speed {vel:+.2f} rad/s (limit {limits['speed']:.2f})")


def run_phase(link, reference, torque, duration, gap, tag, rows, limits, vel=0.0, target=None):
    args = link.args
    motor = link.motor
    began = time.monotonic()
    end = began + duration
    last_ok = began
    first = True
    peak = 0.0
    while time.monotonic() < end:
        record, feedback = link.exchange(reference if target is None else target, torque, vel)
        edge = 1 if first and tag is not None and tag[3] != "pre" else 0
        if tag is not None:
            rows.append((tag[0], tag[1], tag[2], tag[3], edge, gap if edge else 0.0, *record))
        now = time.monotonic()
        if feedback is None:
            if now - last_ok > args.feedback_timeout:
                raise Abort("feedback_lost")
        else:
            last_ok = now
            check_feedback(motor, feedback, reference, args, limits)
            peak = max(peak, abs(feedback[3]))
        if first and gap > 0.0 and tag is not None and tag[3] != "pre":
            resume = time.monotonic() + gap
            while time.monotonic() < resume:
                pass
        first = False
    return peak


def limit_problems(args, reference):
    lo, hi = joint_limit_for(args.motor_id, args.limit_margin)

    def shift_for(step):
        return 2.0 * abs(step_torque(args, step)) / args.kp

    up = max((shift_for(s) for s in steps(args) if s > 0), default=0.0)
    down = max((shift_for(s) for s in steps(args) if s < 0), default=0.0)
    bad = []
    if not lo < reference < hi:
        bad.append(f"the reference {math.degrees(reference):+.2f} deg is outside the joint limits "
                   f"{math.degrees(lo):+.1f} to {math.degrees(hi):+.1f}")
    if up and reference + up > hi:
        bad.append(f"+{math.degrees(up):.2f} deg exceeds the upper limit "
                   f"{math.degrees(hi):+.1f} from {math.degrees(reference):+.2f}")
    if down and reference - down < lo:
        bad.append(f"-{math.degrees(down):.2f} deg exceeds the lower limit "
                   f"{math.degrees(lo):+.1f} from {math.degrees(reference):+.2f}")
    if bad:
        room_up = max(0.0, hi - reference)
        room_down = max(0.0, reference - lo)
        if step_kind(args) == "position":
            bad.append(f"largest safe steps from here: {-math.degrees(room_down) / 2.0:+.2f} to "
                       f"{math.degrees(room_up) / 2.0:+.2f} deg (one-sided --pos-steps are fine)")
        elif step_kind(args) == "velocity":
            bad.append(f"largest safe steps from here: {-room_down * args.kp / 2.0 / args.kd:+.2f} to "
                       f"{room_up * args.kp / 2.0 / args.kd:+.2f} rad/s (one-sided --vel-steps are fine)")
        else:
            bad.append(f"largest safe steps from here: {-room_down * args.kp / 2.0:+.2f} to "
                       f"{room_up * args.kp / 2.0:+.2f} N*m (one-sided --torque-steps are fine)")
    return bad


def capture(args, out):
    bus = open_bus(args)
    if bus is None:
        return 1
    motor = Motor(bus, args.motor_id, SPECS[args.model], host_id=args.host_id)
    link = Link(bus, motor, args)
    rows = []
    reason = ""
    code = 0
    try:
        motor.stop()
        time.sleep(0.02)
        drain(bus)
        if not arm_can_watchdog(motor, args.can_timeout_ms):
            return 1
        motor.write_param_u8(RUN_MODE_INDEX, RUN_MODE_OPERATION)
        time.sleep(0.01)
        motor.enable()
        time.sleep(0.2)
        fb = None
        while True:
            latest = motor.poll_feedback(timeout=0.0 if fb else 1.0)
            if latest is None:
                break
            fb = latest
        if fb is None:
            print("ERROR: no feedback from the motor.")
            return 1
        reference = fb[1]
        print(f"Reference position: {math.degrees(reference):+.2f} deg")
        deadline = time.monotonic() + 0.3
        while time.monotonic() < deadline:
            motor.control(pos=reference, vel=0.0, kp=args.kp, kd=args.kd, torque=0.0)
            if motor.poll_feedback(timeout=0.02) is not None and motor.last_mode_status == MODE_RUNNING:
                break
        if motor.last_mode_status != MODE_RUNNING:
            print(f"ERROR: motor is not running after enable (mode {motor.last_mode_status}, "
                  f"expected {MODE_RUNNING}); nothing was measured.")
            return 1
        print(f"Run mode confirmed (mode {MODE_RUNNING}).")
        if not args.ignore_joint_limit:
            bad = limit_problems(args, reference)
            if bad:
                for line in bad:
                    print(f"ERROR: {line}")
                return 1
        limits = runtime_limits(args)
        time.sleep(0.005)
        drain(bus)
        run_phase(link, reference, 0.0, WARMUP_TIME, 0.0, None, rows, limits)
        for name in link.counts:
            link.counts[name] = 0
        print()
        trial = 0
        for rep in range(args.repeats):
            gap = (rep % args.phase_steps) / args.phase_steps * args.phase_span_ms * 1e-3
            for step in steps(args):
                torque, vel = step_command(args, step)
                run_phase(link, reference, 0.0, args.rest_time, 0.0, (trial, rep, step, "pre"), rows, limits)
                peak = run_phase(link, reference, torque, args.hold_time, gap, (trial, rep, step, "on"), rows,
                                 limits, vel, reference + step_offset(args, step))
                run_phase(link, reference, 0.0, args.rest_time, gap, (trial, rep, step, "off"), rows, limits)
                if step_kind(args) == "position":
                    label = f"{step:+.2f} deg (kp*step {step_torque(args, step):+.2f} N*m)"
                elif step_kind(args) == "velocity":
                    label = f"{step:+.2f} rad/s (kd*v {args.kd * step:+.2f} N*m)"
                else:
                    label = f"{step:+.2f} N*m"
                print(f"[rep {rep + 1}/{args.repeats}] step {label}  gap {gap * 1e3:.2f} ms  "
                      f"peak |torque| {peak:.2f} N*m  pos {math.degrees(motor.last_position - reference):+.2f} deg  "
                      f"T {motor.last_temp:.0f} C")
                trial += 1
    except Abort as error:
        reason = str(error)
        code = 1
        print(f"\nStopped: {reason}")
    except KeyboardInterrupt:
        reason = "interrupted"
        code = 1
        print("\nStop requested.")
    finally:
        shutdown_motor(motor, bus, (active_brake,))

    if rows:
        with open(out, "x", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["# motor_id", args.motor_id])
            w.writerow(["# model", args.model])
            w.writerow(["# channel", args.channel])
            w.writerow(["# kp", args.kp])
            w.writerow(["# kd", args.kd])
            if step_kind(args) == "velocity":
                w.writerow(["# step_kind", "velocity"])
                w.writerow(["# vel_steps", *args.vel_steps])
            elif step_kind(args) == "position":
                w.writerow(["# step_kind", "position"])
                w.writerow(["# pos_steps", *args.pos_steps])
            else:
                w.writerow(["# torque_steps", *args.torque_steps])
            w.writerow(["# repeats", args.repeats])
            w.writerow(["# hold_time", args.hold_time])
            w.writerow(["# rest_time", args.rest_time])
            w.writerow(["# phase_steps", args.phase_steps])
            w.writerow(["# phase_span_ms", args.phase_span_ms])
            w.writerow(["# hw_timestamps", int(args.hw_timestamps)])
            w.writerow(["# can_timeout_ms", args.can_timeout_ms])
            w.writerow(["# python_can", getattr(can, "__version__", "")])
            w.writerow(["# aborted", reason])
            for name, value in link.counts.items():
                w.writerow([f"# {name}", value])
            w.writerow(COLUMNS)
            w.writerows(rows)
        print(f"\nWrote {len(rows)} cycles to {out}")
        bad = {k: v for k, v in link.counts.items() if v and k != "cycles"}
        if bad:
            print(f"Frame bookkeeping: {bad}")
        print(f"Analyze: python3 {os.path.join(os.path.dirname(os.path.abspath(__file__)), 'analyze_latency.py')} {out}")
    return code


def main(argv=None):
    args = parse_args(argv)
    install_stop_signals()
    problems = argument_problems(args)
    if problems:
        report_invalid_args(problems)
        return 1
    args.channel = channel_for_id(args.motor_id)
    if not confirm(args):
        return 1

    os.makedirs(DATA_DIR, exist_ok=True)
    stamp_text = datetime.now().strftime("%Y%m%dT%H%M%S")
    out = args.out or os.path.join(DATA_DIR, f"latency_id{args.motor_id}_{args.model}_{stamp_text}.csv")
    if os.path.exists(out):
        print(f"ERROR: {out} already exists.")
        return 1
    return capture(args, out)


if __name__ == "__main__":
    sys.exit(main())
