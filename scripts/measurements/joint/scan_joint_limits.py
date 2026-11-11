#!/usr/bin/env python3
import argparse
import csv
import math
import os
import signal
import sys
import threading
import time
from datetime import datetime

import can

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from common import (
    DEFAULT_INTERFACE, DEFAULT_LIMIT_MARGIN_RAD, HOST_ID, JOINT_MAP, JOINT_LIMITS_RAD, MECH_POS_INDEX, MOTOR_MODEL,
    RUN_MODE_INDEX, RUN_MODE_OPERATION, SPECS, Motor, add_can_timeout_option, arm_can_watchdog, can_timeout_problem,
    channel_for_id, disarm_can_watchdog, exceeds_joint_limit, install_stop_signals, joint_limit_for,
)
from robonex_common.can import FeedbackHub
from robonex_common.foot_roll import foot_roll
from robonex_common.joints import JOINT_BY_MODEL_NAME, MOTOR_BY_ID
from robonex_common.models import VER2_EDU
from robonex_common.motors import JOINT_CONTROL_GAINS

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "joint_limit")

DEFAULT_TIMEOUT = 0.1
DEFAULT_INTERVAL = 0.1

LEG_IDS = tuple(range(1, 13))
HOLD_TARGET = 0.0
APPROACH_SPEED = 0.3
APPROACH_KP = 15.0
APPROACH_KD = 3.0
POSE_TOL = math.radians(2.0)
HOLD_TOL = 0.10
APPROACH_TOL = 0.20
ROLL_MARGIN = math.radians(1.0)
LIMIT_SLACK = math.radians(2.0)
CONTROL_PERIOD = 0.005
REPLY_TIMEOUT = 0.01
RENDER_INTERVAL = 0.1
COUNTDOWN_S = 2
BRAKE_S = 0.3
FEEDBACK_FAULT_NAMES = (
    "undervoltage", "overcurrent", "overtemperature", "magnetic encoder fault", "stall overload", "uncalibrated",
)
CSV_COLUMNS = [
    "motor_id", "joint", "samples",
    "min_rad", "max_rad", "min_deg", "max_deg", "span_deg",
    "current_limit_lo_deg", "current_limit_hi_deg",
]


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Measure joint ranges by hand. Default: one motor at a time while every other motor of the set is "
                    "held at 0 with its deploy gains. --passive: the old read-only scan (no motor is enabled).")
    parser.add_argument("--motor-id", "--motor-ids", dest="motor_id", nargs="+",
                        type=lambda v: int(v, 0), default=None,
                        help="Motor IDs to scan, measured in ascending order (default: legs 1-12; "
                             "with --passive every motor)")
    parser.add_argument("--passive", action="store_true",
                        help="Read-only scan of all selected motors at once (MECH_POS); never enables a motor")
    add_can_timeout_option(parser)
    parser.set_defaults(
        interface=DEFAULT_INTERFACE, host_id=HOST_ID,
        timeout=DEFAULT_TIMEOUT, interval=DEFAULT_INTERVAL,
        feedback_timeout=0.3, max_vel=3.0, limit_margin=DEFAULT_LIMIT_MARGIN_RAD,
    )
    return parser.parse_args(argv)


def format_deg(value):
    if value is None or not math.isfinite(value):
        return "--"
    return f"{math.degrees(value):+7.2f}"


def poll_worker(channel, motor_ids, interface, host_id, timeout,
                 state, minmax, samples, lock, notes, stop):
    try:
        bus = can.Bus(channel=channel, interface=interface)
    except OSError as error:
        with lock:
            notes.append(f"[{channel}] open failed: {error}  "
                         f"(sudo ip link set {channel} up type can bitrate 1000000)")
        return
    motors = {
        motor_id: Motor(bus, motor_id, SPECS[MOTOR_MODEL[motor_id]], host_id=host_id)
        for motor_id in motor_ids
    }
    try:
        while not stop.is_set():
            for motor_id, motor in motors.items():
                position = motor.read_param_f32(MECH_POS_INDEX, timeout=timeout)
                if position is None or not math.isfinite(position):
                    continue
                with lock:
                    state[motor_id] = position
                    samples[motor_id] += 1
                    lo, hi = minmax[motor_id]
                    minmax[motor_id] = (min(lo, position), max(hi, position))
    except can.CanError as error:
        with lock:
            notes.append(f"[{channel}] CAN error: {error}")
    finally:
        bus.shutdown()


def wait_for_enter(stop):
    try:
        input()
    except EOFError:
        pass
    stop.set()


def render(motor_ids, state, minmax, samples, notes):
    lines = ["\033[2J\033[3J\033[H"]
    lines.append(f"joint limit scan   {time.strftime('%H:%M:%S')}   "
                 "Move each joint slowly through its range; press Enter to stop.\n")
    lines.append(f"  {'ID':>3}  {'joint':<18}  {'current':>8}  {'min':>8}  {'max':>8}  "
                 f"{'span':>7}  {'defined limit':>18}  {'n':>6}")
    lines.append("  " + "-" * 88)
    for motor_id in motor_ids:
        lo, hi = minmax[motor_id]
        span = f"{math.degrees(hi - lo):7.2f}" if math.isfinite(lo) and math.isfinite(hi) else f"{'--':>7}"
        lim_lo, lim_hi = JOINT_LIMITS_RAD[motor_id]
        lim_txt = f"{math.degrees(lim_lo):+.1f}~{math.degrees(lim_hi):+.1f}"
        lines.append(
            f"  {motor_id:>3}  {JOINT_MAP.get(motor_id, '?'):<18}  "
            f"{format_deg(state[motor_id]):>8}  {format_deg(lo):>8}  {format_deg(hi):>8}  "
            f"{span}  {lim_txt:>18}  {samples[motor_id]:>6}"
        )
    if notes:
        lines.append("")
        lines.extend(notes)
    sys.stdout.write("\n".join(lines) + "\n")
    sys.stdout.flush()


def summary_line(motor_id, lo, hi, samples):
    lim_lo, lim_hi = JOINT_LIMITS_RAD[motor_id]
    lim_txt = f"{math.degrees(lim_lo):+.1f}~{math.degrees(lim_hi):+.1f}"
    span = f"{math.degrees(hi - lo):7.2f}" if math.isfinite(lo) and math.isfinite(hi) else f"{'--':>7}"
    return (f"{motor_id:>3}  {JOINT_MAP.get(motor_id, '?'):<18}  "
            f"{format_deg(lo):>8}  {format_deg(hi):>8}  {span}  {lim_txt:>18}  {samples:>6}")


def print_summary(motor_ids, minmax, samples):
    print(f"\n{'ID':>3}  {'joint':<18}  {'min':>8}  {'max':>8}  {'span':>7}  {'defined limit':>18}  {'n':>6}")
    print("-" * 88)
    for motor_id in motor_ids:
        lo, hi = minmax[motor_id]
        print(summary_line(motor_id, lo, hi, samples[motor_id]))


def csv_row(motor_id, lo, hi, samples):
    lim_lo, lim_hi = JOINT_LIMITS_RAD[motor_id]
    row = [motor_id, JOINT_MAP.get(motor_id, "?"), samples]
    if math.isfinite(lo) and math.isfinite(hi):
        row += [
            f"{lo:.6f}", f"{hi:.6f}",
            f"{math.degrees(lo):.4f}", f"{math.degrees(hi):.4f}",
            f"{math.degrees(hi - lo):.4f}",
        ]
    else:
        row += ["", "", "", "", ""]
    row += [f"{math.degrees(lim_lo):.4f}", f"{math.degrees(lim_hi):.4f}"]
    return row


def new_csv_path():
    os.makedirs(DATA_DIR, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%dT%H%M%S")
    return os.path.join(DATA_DIR, f"joint_limit_{ts}.csv")


def save_csv(motor_ids, minmax, samples):
    path = new_csv_path()
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(CSV_COLUMNS)
        for motor_id in motor_ids:
            lo, hi = minmax[motor_id]
            writer.writerow(csv_row(motor_id, lo, hi, samples[motor_id]))
    return path


def run_passive(args, motor_ids):
    channels = sorted({channel_for_id(motor_id) for motor_id in motor_ids})
    state = {motor_id: None for motor_id in motor_ids}
    minmax = {motor_id: (float("inf"), float("-inf")) for motor_id in motor_ids}
    samples = {motor_id: 0 for motor_id in motor_ids}
    notes = []
    lock = threading.Lock()
    stop = threading.Event()

    threads = [
        threading.Thread(
            target=poll_worker,
            args=(
                channel,
                [motor_id for motor_id in motor_ids if channel_for_id(motor_id) == channel],
                args.interface, args.host_id, args.timeout,
                state, minmax, samples, lock, notes, stop,
            ),
            daemon=True,
        )
        for channel in channels
    ]
    for thread in threads:
        thread.start()

    print("Move each joint slowly through its full range. Press Enter to save or Ctrl-C to stop.")
    enter_thread = threading.Thread(target=wait_for_enter, args=(stop,), daemon=True)
    enter_thread.start()

    try:
        while not stop.is_set():
            with lock:
                state_snapshot = dict(state)
                minmax_snapshot = dict(minmax)
                samples_snapshot = dict(samples)
                notes_snapshot = list(notes)
            render(motor_ids, state_snapshot, minmax_snapshot, samples_snapshot, notes_snapshot)
            time.sleep(args.interval)
    except KeyboardInterrupt:
        stop.set()

    for thread in threads:
        thread.join(timeout=2.0)

    with lock:
        final_minmax = dict(minmax)
        final_samples = dict(samples)

    print_summary(motor_ids, final_minmax, final_samples)
    missing = [motor_id for motor_id in motor_ids if final_samples[motor_id] == 0]
    if missing:
        print(f"WARNING: No samples from motor IDs {missing}.")

    path = save_csv(motor_ids, final_minmax, final_samples)
    print(f"Saved: {path}")
    return 0


def deploy_gains(motor_id):
    name = MOTOR_BY_ID[motor_id].model_name[:-len("_joint")]
    if name[:2] in ("l_", "r_"):
        name = name[2:]
    return JOINT_CONTROL_GAINS[name]


def partner_ankle(motor_id):
    for upper_name, lower_name, _ in VER2_EDU.foot_roll.pairs:
        ids = (JOINT_BY_MODEL_NAME[upper_name].motor_id, JOINT_BY_MODEL_NAME[lower_name].motor_id)
        if motor_id in ids:
            return ids[1] if ids[0] == motor_id else ids[0]
    return None


def roll_status(positions, measured):
    clip = VER2_EDU.foot_roll
    problems, warnings = [], []
    for upper_name, lower_name, sign in clip.pairs:
        upper = JOINT_BY_MODEL_NAME[upper_name].motor_id
        lower = JOINT_BY_MODEL_NAME[lower_name].motor_id
        if upper not in positions or lower not in positions:
            continue
        roll = foot_roll(sign * positions[upper], sign * positions[lower], clip.coeffs)
        if abs(roll) <= clip.limit - ROLL_MARGIN:
            continue
        text = (f"foot roll {math.degrees(roll):+.1f} deg of IDs {upper}/{lower} exceeds "
                f"{math.degrees(clip.limit - ROLL_MARGIN):.0f} deg")
        if measured in (upper, lower):
            warnings.append(f"WARNING: {text} (ankle ID {measured} is being measured; no abort)")
        else:
            problems.append(text)
    return problems, warnings


def limit_excess(position, motor_id, margin):
    lo, hi = joint_limit_for(motor_id, margin)
    return max(0.0, lo - position, position - hi)


def ramp_problem(motor_id, start, target, margin):
    if start is None or not math.isfinite(start) or abs(start) > math.pi:
        return f"ID {motor_id} reads {format_deg(start).strip()} deg, outside +-180 deg (check zero_sta)"
    if exceeds_joint_limit(target, motor_id, margin):
        return f"ID {motor_id} target {math.degrees(target):+.1f} deg is outside its joint limit"
    return None


def feedback_fault_names(flags):
    return [name for bit, name in enumerate(FEEDBACK_FAULT_NAMES) if flags & (1 << bit)]


class Channel:
    def __init__(self, name, bus, motors, host_id):
        self.name = name
        self.bus = bus
        self.motors = motors
        self.hub = FeedbackHub(bus, motors, host_id=host_id)

    def collect(self, sent_at, timeout=REPLY_TIMEOUT):
        deadline = time.monotonic() + timeout
        for motor in self.motors:
            while motor.last_feedback_time < sent_at:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0 or self.hub.wait_for(motor.motor_id, remaining) is None:
                    break


class Ramp:
    def __init__(self, motor_id, start, end, margin):
        self.start = start
        self.end = end
        self.t0 = time.monotonic()
        self.duration = max(0.5, abs(end - start) / APPROACH_SPEED)
        self.start_excess = limit_excess(start, motor_id, margin)

    def goal(self, now):
        progress = min(1.0, (now - self.t0) / self.duration)
        smooth = progress * progress * (3.0 - 2.0 * progress)
        return self.start + (self.end - self.start) * smooth, progress


class Abort(Exception):
    pass


class HoldScan:
    def __init__(self, channels, order, args, enter):
        self.channels = channels
        self.order = order
        self.args = args
        self.enter = enter
        self.motors = {m.motor_id: m for ch in channels for m in ch.motors}
        self.mode = {i: "brake" for i in self.motors}
        self.target = {i: HOLD_TARGET for i in self.motors}
        self.gains = {i: deploy_gains(i) for i in self.motors}
        self.ramps = {}
        self.enabled = False
        self.measured = None
        self.phase = 0
        self.tracking = False
        self.lo = self.hi = None
        self.samples = 0
        self.last_sample_time = 0.0
        self.done = []
        self.status = ""
        self.warnings = []
        self.csv_path = None
        self.last_render = 0.0

    def command(self, motor, now):
        i = motor.motor_id
        mode = self.mode[i]
        if mode == "free":
            motor.control(pos=0.0, vel=0.0, kp=0.0, kd=0.0, torque=0.0)
        elif mode == "ramp":
            goal, _ = self.ramps[i].goal(now)
            self.target[i] = goal
            motor.control(pos=goal, vel=0.0, kp=APPROACH_KP, kd=APPROACH_KD, torque=0.0)
        elif mode == "hold":
            kp, kd = self.gains[i]
            motor.control(pos=self.target[i], vel=0.0, kp=kp, kd=kd, torque=0.0)
        else:
            motor.control(pos=0.0, vel=0.0, kp=0.0, kd=APPROACH_KD, torque=0.0)

    def cycle(self, guard=True):
        t0 = time.monotonic()
        for ch in self.channels:
            for motor in ch.motors:
                self.command(motor, t0)
        for ch in self.channels:
            ch.collect(t0)
        if self.tracking:
            m = self.motors[self.measured]
            if m.last_position is not None and m.last_feedback_time > self.last_sample_time:
                self.last_sample_time = m.last_feedback_time
                self.samples += 1
                self.lo = m.last_position if self.lo is None else min(self.lo, m.last_position)
                self.hi = m.last_position if self.hi is None else max(self.hi, m.last_position)
        if guard:
            problem = self.guard()
            if problem:
                raise Abort(problem)
            self.finish_ramps()
            self.maybe_render()
        rest = CONTROL_PERIOD - (time.monotonic() - t0)
        if rest > 0:
            time.sleep(rest)

    def guard(self):
        now = time.monotonic()
        positions = {}
        for i, m in self.motors.items():
            if m.last_position is None or now - m.last_feedback_time > self.args.feedback_timeout:
                return f"ID {i} feedback lost"
            positions[i] = m.last_position
            if m.last_fault:
                names = ", ".join(feedback_fault_names(m.last_fault))
                return f"ID {i} reports fault flags 0x{m.last_fault:02X} ({names})"
            mode = self.mode[i]
            if mode not in ("hold", "ramp"):
                continue
            if abs(m.last_velocity) >= self.args.max_vel:
                return f"ID {i} speed {m.last_velocity:+.2f} rad/s"
            tol = APPROACH_TOL if mode == "ramp" else HOLD_TOL
            if abs(m.last_position - self.target[i]) > tol:
                return f"ID {i} left its target by {math.degrees(m.last_position - self.target[i]):+.1f} deg"
            if exceeds_joint_limit(m.last_position, i, self.args.limit_margin):
                ramp = self.ramps.get(i) if mode == "ramp" else None
                excess = limit_excess(m.last_position, i, self.args.limit_margin)
                if ramp is None or ramp.start_excess == 0.0 or excess > ramp.start_excess + LIMIT_SLACK:
                    return f"ID {i} at {math.degrees(m.last_position):+.1f} deg is outside its joint limit"
        problems, self.warnings = roll_status(positions, self.measured)
        return problems[0] if problems else None

    def start_ramp(self, motor_id):
        start = self.motors[motor_id].last_position
        problem = ramp_problem(motor_id, start, HOLD_TARGET, self.args.limit_margin)
        if problem:
            raise Abort(f"ramp refused: {problem}")
        self.ramps[motor_id] = Ramp(motor_id, start, HOLD_TARGET, self.args.limit_margin)
        self.target[motor_id] = start
        self.mode[motor_id] = "ramp"

    def finish_ramps(self):
        now = time.monotonic()
        for i, ramp in list(self.ramps.items()):
            _, progress = ramp.goal(now)
            error = self.motors[i].last_position - ramp.end
            late = now - ramp.t0 > ramp.duration + 1.5
            if progress < 1.0 or (abs(error) > POSE_TOL and not late):
                continue
            if abs(error) > POSE_TOL:
                print(f"\nWARNING: ID {i} is {math.degrees(error):+.1f} deg from 0 after its ramp.")
            del self.ramps[i]
            self.target[i] = ramp.end
            self.mode[i] = "hold"

    def startup(self):
        deadline = time.monotonic() + 0.3
        while time.monotonic() < deadline:
            self.cycle(guard=False)
            if all(m.last_position is not None for m in self.motors.values()):
                break
        missing = [i for i, m in self.motors.items() if m.last_position is None]
        if missing:
            raise Abort(f"no type 0x02 feedback from IDs {missing}; nothing moves")
        starts = {i: m.last_position for i, m in self.motors.items()}
        problems = [ramp_problem(i, q, HOLD_TARGET, self.args.limit_margin) for i, q in starts.items()]
        problems += roll_status(starts, self.order[0])[0]
        problems = [p for p in problems if p]
        if problems:
            raise Abort("; ".join(problems) + "; nothing moves")

    def measure(self, k, motor_id):
        self.phase = k
        self.measured = motor_id
        self.ramps.pop(motor_id, None)
        self.mode[motor_id] = "free"
        for i in self.motors:
            if i != motor_id and self.mode[i] not in ("hold", "ramp"):
                self.start_ramp(i)
        self.lo = self.hi = None
        self.samples = 0
        self.last_sample_time = 0.0
        self.status = f"Move ID {motor_id} by hand through its range, then press Enter."
        self.enter.clear()
        self.tracking = True
        while not self.enter.is_set():
            if getattr(self.enter, "closed", False):
                raise Abort("stdin closed")
            self.cycle()
        self.tracking = False
        self.record(k, motor_id)

    def record(self, k, motor_id):
        lo = self.lo if self.lo is not None else float("inf")
        hi = self.hi if self.hi is not None else float("-inf")
        held = sorted(i for i in self.motors if i != motor_id)
        row = csv_row(motor_id, lo, hi, self.samples) + [k + 1, " ".join(str(i) for i in held)]
        new = self.csv_path is None
        if new:
            self.csv_path = new_csv_path()
        with open(self.csv_path, "a", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            if new:
                writer.writerow(CSV_COLUMNS + ["order", "held_ids"])
            writer.writerow(row)
        self.done.append((motor_id, lo, hi, self.samples))
        print("\n" + summary_line(motor_id, lo, hi, self.samples) + f"   saved to {self.csv_path}")

    def give_back(self, motor_id):
        end = time.monotonic() + COUNTDOWN_S
        while time.monotonic() < end:
            self.status = f"Returning ID {motor_id} to 0 in {math.ceil(end - time.monotonic())}..."
            self.cycle()
        self.status = f"Returning ID {motor_id} to 0."
        self.start_ramp(motor_id)
        while self.mode[motor_id] == "ramp":
            self.cycle()

    def run(self):
        self.startup()
        for k, motor_id in enumerate(self.order):
            self.measure(k, motor_id)
            self.give_back(motor_id)
        self.status = "All motors measured."
        self.maybe_render(force=True)

    def brake(self):
        self.ramps.clear()
        self.tracking = False
        for i in self.motors:
            self.mode[i] = "brake"
        end = time.monotonic() + BRAKE_S
        while time.monotonic() < end:
            self.cycle(guard=False)

    def maybe_render(self, force=False):
        now = time.monotonic()
        if not force and now - self.last_render < RENDER_INTERVAL:
            return
        self.last_render = now
        sys.stdout.write("\n".join(self.screen()) + "\n")
        sys.stdout.flush()

    def screen(self):
        m = self.measured
        lines = ["\033[2J\033[3J\033[H"]
        lines.append(f"joint limit scan (hold)   {time.strftime('%H:%M:%S')}   phase {self.phase + 1}/{len(self.order)}"
                     f"   measuring ID {m} ({JOINT_MAP.get(m, '?')})")
        lines.append("Motor angles in deg (knee and ankle: crank angles). Ctrl-C releases, brakes and stops.\n")
        lines.append(f"  {'ID':>3}  {'joint':<18}  {'current':>8}  {'min':>8}  {'max':>8}  "
                     f"{'span':>7}  {'defined limit':>18}  {'n':>6}")
        lines.append("  " + "-" * 88)
        if m is not None:
            lo = self.lo if self.lo is not None else float("inf")
            hi = self.hi if self.hi is not None else float("-inf")
            row = summary_line(m, lo, hi, self.samples)
            lines.append(f"  {row[:23]}  {format_deg(self.motors[m].last_position):>8}{row[23:]}")
        lines.append("")
        lines.append(f"  {'ID':>3}  {'held joint':<18}  {'mode':<5}  {'target':>8}  {'pos':>8}  {'dev':>7}")
        for i, motor in sorted(self.motors.items()):
            if i == m:
                continue
            pos = motor.last_position
            dev = f"{math.degrees(pos - self.target[i]):+7.2f}" if pos is not None else f"{'--':>7}"
            lines.append(f"  {i:>3}  {JOINT_MAP.get(i, '?'):<18}  {self.mode[i]:<5}  "
                         f"{format_deg(self.target[i]):>8}  {format_deg(pos):>8}  {dev}")
        if self.done:
            lines.append("\nmeasured:")
            lines.extend("  " + summary_line(*row) for row in self.done)
        lines.append("")
        lines.extend(self.warnings)
        lines.append(self.status)
        return lines


class EnterKey:
    def __init__(self):
        self.event = threading.Event()
        self.closed = False
        threading.Thread(target=self.read, daemon=True).start()

    def read(self):
        while True:
            try:
                input()
            except (EOFError, OSError):
                self.closed = True
                return
            self.event.set()

    def is_set(self):
        return self.event.is_set()

    def clear(self):
        self.event.clear()


def confirm_hold(args, order):
    print(f"Motors: {order}, measured one at a time in this order")
    print("Channels: " + ", ".join(f"{name} {[i for i in order if channel_for_id(i) == name]}"
                                   for name in sorted({channel_for_id(i) for i in order})))
    print(f"Hold: every motor of the set except the measured one is held at {math.degrees(HOLD_TARGET):+.1f} deg "
          "with its deploy gains:")
    for i in order:
        kp, kd = deploy_gains(i)
        lo, hi = JOINT_LIMITS_RAD[i]
        print(f"  ID {i:>2}  {JOINT_MAP.get(i, '?'):<18}  kp={kp:g} kd={kd:g}  "
              f"limit {math.degrees(lo):+.1f}~{math.degrees(hi):+.1f} deg")
    print(f"Ramps: from the current position to 0 at {APPROACH_SPEED} rad/s (kp={APPROACH_KP:g} kd={APPROACH_KD:g}), "
          "then the deploy gains.")
    print("The measured motor applies no torque (enabled, kp=kd=torque=0). Press Enter when it is done; after a "
          f"{COUNTDOWN_S} s countdown it ramps back to 0 and the next motor is released.")
    for i in order:
        partner = partner_ankle(i)
        if partner is not None and partner not in order:
            print(f"NOTE: ankle ID {i}'s partner {partner} is not in the set; it stays unpowered.")
    print("Angles are motor angles (knee and ankle: crank angles).")
    print(f"CAN watchdog: {args.can_timeout_ms} ms on every motor.")
    print("WARNING: The robot must hang on its stand with the feet off the ground, hands clear; "
          "keep the E-stop in reach.")
    if not sys.stdin.isatty():
        print("ERROR: Run this command in an interactive terminal.")
        return False
    try:
        input("Press Enter to start or Ctrl-C to cancel: ")
    except (KeyboardInterrupt, EOFError):
        print("\nCancelled.")
        return False
    return True


def open_channels(args, order):
    channels = []
    try:
        for name in sorted({channel_for_id(i) for i in order}):
            bus = can.Bus(channel=name, interface=args.interface)
            motors = [Motor(bus, i, SPECS[MOTOR_MODEL[i]], host_id=args.host_id)
                      for i in order if channel_for_id(i) == name]
            channels.append(Channel(name, bus, motors, args.host_id))
    except BaseException:
        for ch in channels:
            ch.bus.shutdown()
        raise
    return channels


def enable_all(scan, args):
    motors = list(scan.motors.values())
    for m in motors:
        m.stop()
    time.sleep(0.02)
    for m in motors:
        if not arm_can_watchdog(m, args.can_timeout_ms):
            raise Abort(f"CAN watchdog not confirmed on ID {m.motor_id}; nothing moves")
    scan.enabled = True
    for m in motors:
        m.write_param_u8(RUN_MODE_INDEX, RUN_MODE_OPERATION)
        time.sleep(0.01)
        m.enable()
        time.sleep(0.01)


def shutdown_all(channels):
    previous = signal.signal(signal.SIGINT, signal.SIG_IGN)
    try:
        for ch in channels:
            for m in ch.motors:
                try:
                    m.stop()
                except Exception as error:
                    print(f"Stop frame for ID {m.motor_id} failed: {error}")
                disarm_can_watchdog(m)
            try:
                ch.bus.shutdown()
            except Exception as error:
                print(f"[{ch.name}] shutdown failed: {error}")
    finally:
        signal.signal(signal.SIGINT, previous)


def print_hold_summary(scan, aborted):
    print(f"\n{'ID':>3}  {'joint':<18}  {'min':>8}  {'max':>8}  {'span':>7}  {'defined limit':>18}  {'n':>6}")
    print("-" * 88)
    for row in scan.done:
        print(summary_line(*row))
    finished = [row[0] for row in scan.done]
    if aborted and scan.measured is not None and scan.measured not in finished:
        print(f"ID {scan.measured} was being measured when the run stopped (not saved): "
              f"min {format_deg(scan.lo).strip()} max {format_deg(scan.hi).strip()} deg, n={scan.samples}")
    left = [i for i in scan.order if i not in finished]
    if left:
        print(f"Not measured: {left}")
    print(f"Saved: {scan.csv_path}" if scan.csv_path else "No CSV written (no motor finished).")


def run_hold(args, order, enter=None):
    channels = open_channels(args, order)
    aborted = True
    try:
        scan = HoldScan(channels, order, args, enter if enter is not None else EnterKey())
        try:
            enable_all(scan, args)
            scan.run()
            aborted = False
        except Abort as error:
            print(f"\nSTOP: {error}.")
        except KeyboardInterrupt:
            print("\nInterrupted.")
        except Exception as error:
            print(f"\nSTOP: {type(error).__name__}: {error}.")
        if scan.enabled:
            if aborted:
                print("Releasing holds and braking; no return move.")
            try:
                scan.brake()
            except KeyboardInterrupt:
                print("Brake interrupted; sending stop frames now.")
            except Exception as error:
                print(f"Brake failed ({error}); sending stop frames now.")
    finally:
        shutdown_all(channels)
    print_hold_summary(scan, aborted)
    return 1 if aborted else 0


def default_ids(passive):
    if passive:
        return sorted(JOINT_MAP)
    return list(LEG_IDS)


def main(argv=None):
    args = parse_args(argv)
    requested = args.motor_id if args.motor_id is not None else default_ids(args.passive)
    motor_ids = sorted(set(requested))
    unknown = [motor_id for motor_id in motor_ids if motor_id not in JOINT_MAP]
    if unknown:
        print(f"ERROR: Unsupported motor IDs: {unknown}")
        return 1
    if args.passive:
        return run_passive(args, motor_ids)

    install_stop_signals()
    problems = [p for p in [can_timeout_problem(args)] if p]
    problems += [f"ID {i}: hold target {math.degrees(HOLD_TARGET):+.1f} deg is outside its joint limit"
                 for i in motor_ids if exceeds_joint_limit(HOLD_TARGET, i, args.limit_margin)]
    if problems:
        print("Invalid arguments; nothing will run:")
        for problem in problems:
            print(f"  {problem}")
        return 1
    if not confirm_hold(args, motor_ids):
        return 1
    return run_hold(args, motor_ids)


if __name__ == "__main__":
    sys.exit(main())
