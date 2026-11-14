#!/usr/bin/env python3
import argparse
import csv
import math
import os
import signal
import sys
import time
from datetime import datetime

import can

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)
from common import (
    HOST_ID, DEFAULT_INTERFACE, SPECS,
    RUN_MODE_INDEX, RUN_MODE_OPERATION,
    Motor,
    DEFAULT_LIMIT_MARGIN_RAD, joint_limit_for, exceeds_joint_limit,
    add_can_timeout_option, can_timeout_problem, arm_can_watchdog, disarm_can_watchdog,
    install_stop_signals, report_invalid_args,
)
from motor_selection import SELECTOR_HELP, SelectionError, attached_variant, format_ids, resolve_motors
from latency import StampedBus, hw_seconds, stamp
from analyze_bus_timing import frame_bits
from robonex_common.can import FeedbackHub, drain
from robonex_common.protocol import COMM_FEEDBACK, COMM_OPERATION, float_to_uint, parse_arbitration_id

MODE_RESET = 0
MODE_RUNNING = 2
PATTERNS = {"burst": ("burst",), "sequential": ("sequential",), "both": ("burst", "sequential")}
DEFAULT_IDS = ["left_leg"]
DEFAULT_PERIOD_MS = 20.0
MAX_PERIOD_MS = 1000.0
DEFAULT_DURATION = 10.0
MAX_DURATION = 120.0
DEFAULT_REPLY_TIMEOUT_MS = 10.0
MIN_REPLY_TIMEOUT_MS = 1.0
MAX_REPLY_TIMEOUT_MS = 50.0
LATE_WAIT = 0.005
ECHO_CLOCK_SLACK = 0.0002
QUIET_TIME = 0.2
PROBE_KD = 1.0
PROBE_TIMEOUT = 0.05
FEEDBACK_TIMEOUT = 0.25
HOLD_DEVIATION_RAD = math.radians(5.0)
HOLD_SPEED = 1.5
ENABLE_WAIT = 0.3
KEEPALIVE = 0.01
BRAKE_KD = 3.0
BRAKE_TIME = 0.3

DATA_DIR = os.path.join(HERE, "data")
COLUMNS = ["cycle", "pattern", "slot", "motor_id", "status",
           "t_cycle_mono", "t_send_mono", "t_send_wall", "tx_ts", "tx_hw_ts", "t_rx_mono", "rx_ts", "rx_hw_ts",
           "tx_id", "tx_data", "rx_id", "rx_data", "cmd_pos", "cmd_kp", "cmd_kd",
           "pos_rad", "vel_rad_s", "torque_nm", "temp_c", "mode", "fault"]
COUNT_NAMES = ("cycles", "exchanges", "no_reply", "no_echo", "stale_reply", "dup_reply", "stray_echo",
               "other", "error_frames", "late", "overrun")


class Abort(Exception):
    pass


class EnabledMotor(Abort):
    pass


class LoadBus:
    """Count observed traffic before exchange matching can discard it."""

    def __init__(self, bus):
        self.bus = bus
        self.load_start_mono = None
        self.load_end_mono = None
        self.load_frames = 0
        self.load_bits = 0
        self.load_error_records = 0
        self.load_unsupported_records = 0

    def send(self, msg, timeout=None):
        return self.bus.send(msg, timeout)

    def recv(self, timeout=None):
        msg = self.bus.recv(timeout)
        if msg is None or self.load_start_mono is None or self.load_end_mono is not None:
            return msg
        if msg.is_error_frame:
            self.load_error_records += 1
            return msg
        try:
            if getattr(msg, "is_fd", False):
                raise ValueError("CAN FD is not supported by this Classical CAN bench")
            bits = frame_bits(msg.arbitration_id, msg.data, extended=msg.is_extended_id,
                              remote=msg.is_remote_frame, dlc=msg.dlc)
        except ValueError:
            self.load_unsupported_records += 1
        else:
            self.load_frames += 1
            self.load_bits += bits
        return msg

    def shutdown(self):
        self.bus.shutdown()


class Slot:
    __slots__ = ("index", "motor", "command", "data16", "data", "arbitration_id", "t_send_mono", "t_send_wall",
                 "echo", "reply", "early")

    def __init__(self, index, motor, command):
        self.index = index
        self.motor = motor
        self.command = command
        self.echo = None
        self.reply = None
        self.early = None


def encode_control(spec, pos, vel, kp, kd, torque):
    data16 = float_to_uint(torque, spec.t_min, spec.t_max, 16)
    raw = (float_to_uint(pos, spec.p_min, spec.p_max, 16), float_to_uint(vel, spec.v_min, spec.v_max, 16),
           float_to_uint(kp, 0.0, spec.kp_max, 16), float_to_uint(kd, 0.0, spec.kd_max, 16))
    return data16, bytes(b for value in raw for b in ((value >> 8) & 0xFF, value & 0xFF))


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


def select_motors(tokens):
    joints = resolve_motors(tokens, attached_variant())
    channels = sorted({joint.channel for joint in joints})
    if len(channels) != 1:
        raise SelectionError(f"the selected motors use {len(channels)} channels ({', '.join(channels)}); "
                             f"select motors of one CAN line")
    return joints, channels[0]


def parse_args(argv=None):
    p = argparse.ArgumentParser(description="Time command/feedback exchanges with several motors on one CAN line.")
    p.add_argument("--ids", nargs="+", default=list(DEFAULT_IDS), metavar="SEL",
                   help=f"Motors on one channel: {SELECTOR_HELP} (default: left_leg)")
    p.add_argument("--mode", choices=["passive", "hold"], default="passive",
                   help="passive: motors stay disabled, zero-gain frames; hold: enable and hold the current position")
    p.add_argument("--pattern", choices=list(PATTERNS), default="both",
                   help="burst: all frames back to back, then collect; sequential: one exchange at a time; "
                        "both: alternate cycle by cycle")
    p.add_argument("--period-ms", type=float, default=DEFAULT_PERIOD_MS,
                   help="Cycle period in ms; 0 starts the next cycle as soon as the previous one ends")
    p.add_argument("--duration", type=float, default=DEFAULT_DURATION, help=f"Seconds, at most {MAX_DURATION:g}")
    p.add_argument("--reply-timeout-ms", type=float, default=DEFAULT_REPLY_TIMEOUT_MS,
                   help="Wait this long after the last frame of a burst (or after each frame) for replies")
    p.add_argument("--kp", type=float, default=None, help="hold: position gain")
    p.add_argument("--kd", type=float, default=None, help="hold: velocity gain")
    p.add_argument("--hw-timestamps", action="store_true",
                   help="Also record the adapter's own timestamps (SO_TIMESTAMPING)")
    p.add_argument("--ignore-joint-limit", action="store_true")
    p.add_argument("--out", default=None, help="CSV path (default: data/bus_timing_<channel>_<mode>_<time>.csv)")
    add_can_timeout_option(p)
    p.set_defaults(interface=DEFAULT_INTERFACE, host_id=HOST_ID, limit_margin=DEFAULT_LIMIT_MARGIN_RAD)
    args = p.parse_args(argv)
    try:
        args.joints, args.channel = select_motors(args.ids)
    except SelectionError as error:
        p.error(str(error))
    return args


def argument_problems(args):
    problems = []
    problem = can_timeout_problem(args)
    if problem:
        problems.append(problem)
    for joint in args.joints:
        if joint.motor_id == args.host_id:
            problems.append(f"ID {joint.motor_id} must differ from the host ID {args.host_id:#x}")
    if not (math.isfinite(args.period_ms) and (args.period_ms == 0.0 or 1.0 <= args.period_ms <= MAX_PERIOD_MS)):
        problems.append(f"--period-ms must be 0 or between 1 and {MAX_PERIOD_MS:g} ({args.period_ms})")
    if not (math.isfinite(args.duration) and 0.0 < args.duration <= MAX_DURATION):
        problems.append(f"--duration must be above 0 and at most {MAX_DURATION:g} ({args.duration})")
    if not (math.isfinite(args.reply_timeout_ms)
            and MIN_REPLY_TIMEOUT_MS <= args.reply_timeout_ms <= MAX_REPLY_TIMEOUT_MS):
        problems.append(f"--reply-timeout-ms must be between {MIN_REPLY_TIMEOUT_MS:g} and "
                        f"{MAX_REPLY_TIMEOUT_MS:g} ({args.reply_timeout_ms})")
    if args.mode == "passive":
        for name in ("kp", "kd"):
            if getattr(args, name) is not None:
                problems.append(f"--{name} is for --mode hold; passive frames carry kp 0 and kd 0")
        if args.ignore_joint_limit:
            problems.append("--ignore-joint-limit is for --mode hold")
        return problems
    for name in ("kp", "kd"):
        value = getattr(args, name)
        if value is None:
            problems.append(f"--mode hold needs --{name}")
        elif not math.isfinite(value) or value <= 0.0:
            problems.append(f"--{name} must be positive and finite ({value})")
        else:
            for joint in args.joints:
                spec = SPECS[joint.motor_model]
                limit = spec.kp_max if name == "kp" else spec.kd_max
                if value > limit:
                    problems.append(f"--{name} {value} exceeds the {joint.motor_model.upper()} frame range {limit} "
                                    f"(ID {joint.motor_id})")
    return problems


def confirm(args):
    ids = [joint.motor_id for joint in args.joints]
    cycles = "as many cycles as fit" if args.period_ms == 0 else f"about {args.duration * 1e3 / args.period_ms:.0f} cycles"
    print(f"Motors: IDs {format_ids(ids)} on {args.channel}")
    for joint in args.joints:
        print(f"  {joint.motor_id:>2}  {joint.hardware_name:<20} {joint.motor_model.upper()}")
    period = "as fast as possible" if args.period_ms == 0 else f"every {args.period_ms:g} ms"
    print(f"Pattern: {args.pattern}, one cycle {period}, {args.duration:g} s ({cycles})")
    if args.mode == "passive":
        print("Mode: passive. The motors are NOT enabled. Each motor first gets one probe frame "
              f"(kp 0, kd {PROBE_KD:g}, torque 0); the run stops before any further frame if a motor reports run mode.")
        print("Then every frame is a type-1 command with kp 0, kd 0, torque 0, velocity 0; a disabled motor only "
              "answers with feedback. The joints stay limp and unpowered as they are now.")
    else:
        print(f"Mode: hold. Each motor is stopped, given a {args.can_timeout_ms} ms CAN watchdog, enabled and held at "
              f"the position read after enable with kp {args.kp:g}, kd {args.kd:g}, torque 0.")
        print(f"The run stops (brake {BRAKE_TIME:g} s, then disable) if a joint moves more than "
              f"{math.degrees(HOLD_DEVIATION_RAD):g} deg or faster than {HOLD_SPEED:g} rad/s, leaves run mode, "
              f"reports a fault or loses feedback for {FEEDBACK_TIMEOUT * 1e3:.0f} ms.")
        print("WARNING: The joints hold torque. Hang or support the robot and keep the E-stop in reach.")
    print("Nothing else may use this CAN line during the run (the run refuses a busy line).")
    if not sys.stdin.isatty():
        print("ERROR: Run this command in an interactive terminal.")
        return False
    try:
        input("Press Enter to continue or Ctrl-C to cancel: ")
    except (KeyboardInterrupt, EOFError):
        print("\nCancelled.")
        return False
    return True


def hex_id(value):
    return f"{value:08x}"


class Bench:
    def __init__(self, bus, motors, args):
        self.bus = bus
        self.motors = list(motors)
        self.by_id = {motor.motor_id: motor for motor in self.motors}
        self.hub = FeedbackHub(bus, self.motors, host_id=args.host_id)
        self.args = args
        self.reply_timeout = args.reply_timeout_ms * 1e-3
        self.commands = {}
        self.counts = {name: 0 for name in COUNT_NAMES}

    def set_command(self, motor, pos=0.0, vel=0.0, kp=0.0, kd=0.0, torque=0.0):
        self.commands[motor.motor_id] = (pos, vel, kp, kd, torque)

    def send(self, slot):
        motor = slot.motor
        slot.data16, slot.data = encode_control(motor.spec, *slot.command)
        slot.t_send_wall = time.time()
        slot.t_send_mono = time.monotonic()
        motor._send(COMM_OPERATION, slot.data16, slot.data)

    def classify(self, msg, seen, pending):
        if getattr(msg, "is_error_frame", False):
            self.counts["error_frames"] += 1
            return
        if not msg.is_extended_id:
            self.counts["other"] += 1
            return
        comm, data16, destination = parse_arbitration_id(msg.arbitration_id)
        if not getattr(msg, "is_rx", True):
            slot = pending.get(destination) if comm == COMM_OPERATION else None
            if (slot is not None and slot.echo is None and msg.timestamp >= slot.t_send_wall - ECHO_CLOCK_SLACK
                    and data16 == slot.data16 and bytes(msg.data) == slot.data):
                slot.echo = (msg.timestamp, getattr(msg, "hw_timestamp", None))
                if slot.early is not None:
                    self.counts["stale_reply"] += 1
                    slot.early = None
            else:
                self.counts["stray_echo"] += 1
            return
        if comm != COMM_FEEDBACK or destination != self.args.host_id:
            self.counts["other"] += 1
            return
        slot = pending.get(data16 & 0xFF)
        if slot is None:
            self.counts["other"] += 1
            return
        item = (msg.timestamp, getattr(msg, "hw_timestamp", None), seen, bytes(msg.data), data16,
                msg.arbitration_id)
        if slot.echo is None:
            if slot.early is not None:
                self.counts["stale_reply"] += 1
            slot.early = item
        elif slot.reply is None:
            slot.reply = item
        else:
            self.counts["dup_reply"] += 1

    def collect(self, slots, deadline):
        pending = {slot.motor.motor_id: slot for slot in slots}
        while True:
            if all(slot.echo is not None and slot.reply is not None for slot in slots):
                return
            now = time.monotonic()
            if now >= deadline:
                return
            msg = self.bus.recv(timeout=deadline - now)
            if msg is None:
                continue
            self.classify(msg, time.monotonic(), pending)

    def drain_late(self):
        deadline = time.monotonic() + LATE_WAIT
        while True:
            now = time.monotonic()
            if now >= deadline:
                break
            if self.bus.recv(timeout=deadline - now) is not None:
                self.counts["late"] += 1
        self.counts["late"] += drain(self.bus)

    def exchange(self, pattern, motors=None):
        motors = self.motors if motors is None else motors
        slots = [Slot(k, motor, self.commands[motor.motor_id]) for k, motor in enumerate(motors)]
        if pattern == "burst":
            for slot in slots:
                self.send(slot)
            self.collect(slots, time.monotonic() + self.reply_timeout)
        else:
            for slot in slots:
                self.send(slot)
                self.collect([slot], time.monotonic() + self.reply_timeout)
        if any(slot.reply is None and slot.early is None for slot in slots):
            self.drain_late()
        return slots

    def finish(self, slot, cycle, pattern, t_cycle):
        motor = slot.motor
        reply = slot.reply
        status = "ok"
        if reply is None and slot.early is not None:
            reply = slot.early
            status = "no_echo"
        if reply is None:
            status = "no_reply"
        elif slot.echo is None:
            status = "no_echo"
        self.counts["exchanges"] += 1
        if status != "ok":
            self.counts[status] += 1
        feedback = None
        rx = ("", "", "", "", "")
        mode = ""
        fault = ""
        if reply is not None:
            feedback = motor.ingest_feedback(reply[3], data16=reply[4], now=reply[2])
            mode = motor.last_mode_status
            fault = motor.last_fault
            rx = (repr(reply[2]), stamp(reply[0]), hw_seconds(reply[1]), hex_id(reply[5]), reply[3].hex())
        tx = (stamp(slot.echo[0]), hw_seconds(slot.echo[1])) if slot.echo is not None else ("", "")
        tx_id = hex_id((COMM_OPERATION << 24) | (slot.data16 << 8) | motor.motor_id)
        measured = feedback[1:5] if feedback is not None else ("", "", "", "")
        pos, _, kp, kd, _ = slot.command
        row = (cycle, pattern, slot.index, motor.motor_id, status, repr(t_cycle), repr(slot.t_send_mono),
               repr(slot.t_send_wall), tx[0], tx[1], rx[0], rx[1], rx[2], tx_id, slot.data.hex(), rx[3], rx[4],
               pos, kp, kd, *measured, mode, fault)
        return row, feedback


def quiet_frames(bus, duration):
    seen = 0
    deadline = time.monotonic() + duration
    while True:
        now = time.monotonic()
        if now >= deadline:
            return seen
        if bus.recv(timeout=deadline - now) is not None:
            seen += 1


def check_passive(motor):
    if motor.last_mode_status != MODE_RESET:
        raise EnabledMotor(f"ID {motor.motor_id} reports mode {motor.last_mode_status} (enabled); passive frames carry "
                    f"zero gains, so nothing more is sent")


def check_hold(motor, feedback, reference, args):
    _, pos, vel, _, _, fault = feedback
    if motor.last_mode_status != MODE_RUNNING:
        raise Abort(f"ID {motor.motor_id} left run mode (mode {motor.last_mode_status})")
    if fault:
        raise Abort(f"ID {motor.motor_id} feedback fault bits {fault:#04x}")
    if not args.ignore_joint_limit and exceeds_joint_limit(pos, motor.motor_id, args.limit_margin):
        raise Abort(f"ID {motor.motor_id} joint_limit")
    if abs(pos - reference) > HOLD_DEVIATION_RAD:
        raise Abort(f"ID {motor.motor_id} moved {math.degrees(pos - reference):+.2f} deg from its hold position")
    if abs(vel) > HOLD_SPEED:
        raise Abort(f"ID {motor.motor_id} speed {vel:+.2f} rad/s")


def probe(bench):
    for motor in bench.motors:
        bench.set_command(motor, kd=PROBE_KD)
        slot = Slot(0, motor, bench.commands[motor.motor_id])
        bench.send(slot)
        bench.collect([slot], time.monotonic() + PROBE_TIMEOUT)
        reply = slot.reply or slot.early
        if reply is None:
            raise Abort(f"no feedback from ID {motor.motor_id} to the probe frame")
        motor.ingest_feedback(reply[3], data16=reply[4], now=reply[2])
        check_passive(motor)
        bench.set_command(motor)
        print(f"ID {motor.motor_id}: disabled (mode {motor.last_mode_status}), "
              f"{math.degrees(motor.last_position):+.2f} deg")


def keep_holding(bench, held, references, until, target=None):
    next_keep = time.monotonic()
    while True:
        now = time.monotonic()
        if now >= until:
            return None
        if held and now >= next_keep:
            for motor in held:
                bench.set_command(motor, pos=references[motor.motor_id], kp=bench.args.kp, kd=bench.args.kd)
                pos, vel, kp, kd, torque = bench.commands[motor.motor_id]
                motor.control(pos=pos, vel=vel, kp=kp, kd=kd, torque=torque)
            next_keep = now + KEEPALIVE
        wait = min(until, next_keep) - now if held else until - now
        msg = bench.bus.recv(timeout=max(0.0, wait))
        if msg is None:
            continue
        routed = bench.hub.route(msg, now=time.monotonic())
        if target is not None and routed is target:
            return target


def start_hold(bench):
    args = bench.args
    for motor in bench.motors:
        motor.stop()
    time.sleep(0.02)
    drain(bench.bus)
    for motor in bench.motors:
        if not arm_can_watchdog(motor, args.can_timeout_ms):
            raise Abort(f"CAN watchdog not armed on ID {motor.motor_id}")
    held = []
    references = {}
    for motor in bench.motors:
        motor.write_param_u8(RUN_MODE_INDEX, RUN_MODE_OPERATION)
        keep_holding(bench, held, references, time.monotonic() + 0.01)
        motor.enable()
        if keep_holding(bench, held, references, time.monotonic() + ENABLE_WAIT, target=motor) is None:
            raise Abort(f"no feedback from ID {motor.motor_id} after enable")
        reference = motor.last_position
        if not args.ignore_joint_limit:
            lo, hi = joint_limit_for(motor.motor_id, args.limit_margin)
            if not lo < reference < hi:
                raise Abort(f"ID {motor.motor_id} is at {math.degrees(reference):+.2f} deg, outside its joint limits "
                            f"{math.degrees(lo):+.1f} to {math.degrees(hi):+.1f}")
        references[motor.motor_id] = reference
        held.append(motor)
        deadline = time.monotonic() + ENABLE_WAIT
        while motor.last_mode_status != MODE_RUNNING and time.monotonic() < deadline:
            keep_holding(bench, held, references, min(deadline, time.monotonic() + KEEPALIVE), target=motor)
        if motor.last_mode_status != MODE_RUNNING:
            raise Abort(f"ID {motor.motor_id} is not running after enable (mode {motor.last_mode_status}, "
                        f"expected {MODE_RUNNING})")
        print(f"ID {motor.motor_id}: running, holding {math.degrees(reference):+.2f} deg")
    return references


def release_hold(bench):
    try:
        deadline = time.monotonic() + BRAKE_TIME
        while time.monotonic() < deadline:
            for motor in bench.motors:
                motor.control(pos=0.0, vel=0.0, kp=0.0, kd=BRAKE_KD, torque=0.0)
            stop_at = time.monotonic() + KEEPALIVE
            while time.monotonic() < stop_at:
                if bench.bus.recv(timeout=max(0.0, stop_at - time.monotonic())) is None:
                    break
    except KeyboardInterrupt:
        print("\nBrake interrupted; disabling now.")
    except Exception as error:
        print(f"\nBrake failed ({error}); disabling now.")
    finally:
        previous = signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            for motor in bench.motors:
                try:
                    motor.stop()
                except Exception as error:
                    print(f"Stop frame to ID {motor.motor_id} failed: {error}")
            for motor in bench.motors:
                disarm_can_watchdog(motor)
            bench.bus.shutdown()
        finally:
            signal.signal(signal.SIGINT, previous)


def run_cycles(bench, rows, references):
    args = bench.args
    patterns = PATTERNS[args.pattern]
    period = args.period_ms * 1e-3
    last_ok = {motor.motor_id: time.monotonic() for motor in bench.motors}
    start = time.monotonic()
    end = start + args.duration
    next_tick = start
    cycle = 0
    while time.monotonic() < end:
        if period > 0.0:
            wait = next_tick - time.monotonic()
            while wait > 0.0:
                if bench.bus.recv(timeout=wait) is not None:
                    bench.counts["late"] += 1
                wait = next_tick - time.monotonic()
        t_cycle = time.monotonic()
        if t_cycle >= end:
            break
        if period > 0.0:
            next_tick += period
            if next_tick < t_cycle:
                bench.counts["overrun"] += 1
                next_tick = t_cycle + period
        pattern = patterns[cycle % len(patterns)]
        slots = bench.exchange(pattern)
        bench.counts["cycles"] += 1
        now = time.monotonic()
        results = [bench.finish(slot, cycle, pattern, t_cycle) for slot in slots]
        rows.extend(row for row, _ in results)
        for slot, (_, feedback) in zip(slots, results):
            motor = slot.motor
            if feedback is None:
                if now - last_ok[motor.motor_id] > FEEDBACK_TIMEOUT:
                    raise Abort(f"feedback_lost from ID {motor.motor_id}")
                continue
            last_ok[motor.motor_id] = now
            if args.mode == "passive":
                check_passive(motor)
            else:
                check_hold(motor, feedback, references[motor.motor_id], args)
        cycle += 1


def capture(args, out):
    bus = open_bus(args)
    if bus is None:
        return 1
    bus = LoadBus(bus)
    motors = [Motor(bus, joint.motor_id, SPECS[joint.motor_model], host_id=args.host_id) for joint in args.joints]
    bench = Bench(bus, motors, args)
    rows = []
    reason = ""
    code = 0
    references = {}
    started = False
    try:
        busy = quiet_frames(bus, QUIET_TIME)
        if busy:
            print(f"ERROR: {busy} frames seen on {args.channel} in {QUIET_TIME:g} s before sending anything; "
                  f"another program is using this line. Nothing was sent.")
            return 1
        if args.mode == "passive":
            probe(bench)
            for motor in motors:
                bench.set_command(motor)
        else:
            started = True
            references = start_hold(bench)
            for motor in motors:
                bench.set_command(motor, pos=references[motor.motor_id], kp=args.kp, kd=args.kd)
        drain(bus)
        print()
        bus.load_start_mono = time.monotonic()
        try:
            run_cycles(bench, rows, references)
        finally:
            bus.load_end_mono = time.monotonic()
    except Abort as error:
        reason = str(error)
        code = 1
        print(f"\nStopped: {reason}")
        if isinstance(error, EnabledMotor):
            ids = " ".join(str(joint.motor_id) for joint in args.joints)
            print(f"A motor on this line is enabled. Disable it before a passive run:\n"
                  f"  python3 scripts/measurements/check/shutdown.py --ids {ids}")
    except KeyboardInterrupt:
        reason = "interrupted"
        code = 1
        print("\nStop requested.")
    except (can.CanError, OSError) as error:
        reason = f"{type(error).__name__}: {error}"
        code = 1
        print(f"\nStopped: {reason}")
    finally:
        if started:
            release_hold(bench)
        else:
            bus.shutdown()

    if rows or bus.load_start_mono is not None:
        write_csv(out, args, bench, rows, reason, references)
        print(f"\nWrote {len(rows)} exchanges ({bench.counts['cycles']} cycles) to {out}")
        bad = {k: v for k, v in bench.counts.items() if v and k not in ("cycles", "exchanges")}
        if bad:
            print(f"Frame bookkeeping: {bad}")
        print(f"Analyze: python3 {os.path.join(HERE, 'analyze_bus_timing.py')} {out}")
    return code


def write_csv(out, args, bench, rows, reason, references):
    with open(out, "x", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["# mode", args.mode])
        w.writerow(["# channel", args.channel])
        w.writerow(["# motor_ids", *[joint.motor_id for joint in args.joints]])
        w.writerow(["# models", *[joint.motor_model for joint in args.joints]])
        w.writerow(["# pattern", args.pattern])
        w.writerow(["# period_ms", args.period_ms])
        w.writerow(["# duration", args.duration])
        w.writerow(["# reply_timeout_ms", args.reply_timeout_ms])
        if args.mode == "hold":
            w.writerow(["# kp", args.kp])
            w.writerow(["# kd", args.kd])
            w.writerow(["# can_timeout_ms", args.can_timeout_ms])
            w.writerow(["# references", *[references.get(joint.motor_id, "") for joint in args.joints]])
        w.writerow(["# hw_timestamps", int(args.hw_timestamps)])
        w.writerow(["# python_can", getattr(can, "__version__", "")])
        w.writerow(["# aborted", reason])
        if bench.bus.load_start_mono is not None:
            for name in ("load_start_mono", "load_end_mono", "load_frames", "load_bits",
                         "load_error_records", "load_unsupported_records"):
                w.writerow([f"# {name}", getattr(bench.bus, name)])
        for name, value in bench.counts.items():
            w.writerow([f"# {name}", value])
        w.writerow(COLUMNS)
        w.writerows(rows)


def main(argv=None):
    args = parse_args(argv)
    install_stop_signals()
    problems = argument_problems(args)
    if problems:
        report_invalid_args(problems)
        return 1
    if not confirm(args):
        return 1
    os.makedirs(DATA_DIR, exist_ok=True)
    stamp_text = datetime.now().strftime("%Y%m%dT%H%M%S")
    out = args.out or os.path.join(DATA_DIR, f"bus_timing_{args.channel}_{args.mode}_{stamp_text}.csv")
    if os.path.exists(out):
        print(f"ERROR: {out} already exists.")
        return 1
    return capture(args, out)


if __name__ == "__main__":
    sys.exit(main())
