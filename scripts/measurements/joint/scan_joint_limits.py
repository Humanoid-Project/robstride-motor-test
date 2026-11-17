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
    channel_for_id, disarm_can_watchdog, install_stop_signals, joint_limit_for,
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
LEGS = {"left": tuple(range(1, 7)), "right": tuple(range(7, 13))}
HOLD_TARGET = 0.0
EDGE_SLACK = math.radians(4.0)
APPROACH_SPEED = 0.3
APPROACH_KP = 40.0
APPROACH_KD = 3.0
RAMP_KD = 2.0
POSE_TOL = math.radians(2.0)
HOLD_TOL = 0.10
APPROACH_TOL = 0.20
ROLL_MARGIN = math.radians(1.0)
LIMIT_SLACK = math.radians(2.0)
SETTLE_S = 2.0
CONTROL_PERIOD = 0.005
REPLY_TIMEOUT = 0.01
RENDER_INTERVAL = 0.1
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
        description="Measure joint ranges by hand, one leg at a time: the six motors of the measured leg are "
                    "released together while the other leg is held at 0 with its deploy gains. --passive: "
                    "read-only scan (no motor is enabled).")
    parser.add_argument("--leg", choices=sorted(LEGS), default=None,
                        help="Leg to measure; the other leg is held at 0 for the whole run")
    parser.add_argument("--passive", action="store_true",
                        help="Read-only scan (MECH_POS); never enables a motor")
    parser.add_argument("--motor-id", "--motor-ids", dest="motor_id", nargs="+",
                        type=lambda v: int(v, 0), default=None,
                        help="With --passive only: motor IDs to read (default: every motor)")
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


def roll_status(positions, measured, ramping=()):
    clip = VER2_EDU.foot_roll
    problems, warnings = [], []
    for upper_name, lower_name, sign in clip.pairs:
        upper = JOINT_BY_MODEL_NAME[upper_name].motor_id
        lower = JOINT_BY_MODEL_NAME[lower_name].motor_id
        if upper not in positions or lower not in positions:
            continue
        roll = foot_roll(sign * positions[upper], sign * positions[lower], clip.coeffs)
        limit = clip.limit if upper in ramping or lower in ramping else clip.limit - ROLL_MARGIN
        if abs(roll) <= limit:
            continue
        text = (f"foot roll {math.degrees(roll):+.1f} deg of IDs {upper}/{lower} exceeds "
                f"{math.degrees(limit):.0f} deg")
        if upper in measured or lower in measured:
            warnings.append(f"WARNING: {text} (IDs {upper}/{lower} are being measured; no abort)")
        else:
            problems.append(text)
    return problems, warnings


def hold_band(motor_id, margin):
    lo, hi = joint_limit_for(motor_id, margin)
    edge_lo, edge_hi = joint_limit_for(motor_id, 0.0)
    if HOLD_TARGET <= lo and edge_lo <= HOLD_TARGET:
        lo = edge_lo - EDGE_SLACK
    if HOLD_TARGET >= hi and HOLD_TARGET <= edge_hi:
        hi = edge_hi + EDGE_SLACK
    return lo, hi


def limit_excess(position, motor_id, margin):
    lo, hi = hold_band(motor_id, margin)
    return max(0.0, lo - position, position - hi)


def hold_target(motor_id, margin):
    lo, hi = hold_band(motor_id, margin)
    return min(max(HOLD_TARGET, lo), hi)


def ramp_problem(motor_id, start, target, margin):
    if start is None or not math.isfinite(start) or abs(start) > math.pi:
        return f"ID {motor_id} reads {format_deg(start).strip()} deg, outside +-180 deg (check zero_sta)"
    if limit_excess(target, motor_id, margin) > 0.0:
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
    def __init__(self, motor_id, start, end, margin, duration=None):
        self.start = start
        self.end = end
        self.t0 = time.monotonic()
        self.duration = duration if duration is not None else max(0.5, abs(end - start) / APPROACH_SPEED)
        self.start_excess = limit_excess(start, motor_id, margin)

    def goal(self, now):
        progress = min(1.0, (now - self.t0) / self.duration)
        smooth = progress * progress * (3.0 - 2.0 * progress)
        return self.start + (self.end - self.start) * smooth, progress


class Abort(Exception):
    pass

class LegScan:
    def __init__(self, channels, measured, held, args, enter):
        self.channels = channels
        self.measured = tuple(measured)
        self.held = tuple(held)
        self.args = args
        self.enter = enter
        self.motors = {m.motor_id: m for ch in channels for m in ch.motors}
        self.mode = {i: "brake" for i in self.motors}
        self.hold = {i: hold_target(i, args.limit_margin) for i in self.held}
        self.target = dict(self.hold)
        self.gains = {i: deploy_gains(i) for i in self.held}
        self.ramps = {}
        self.settle = {}
        self.enabled = False
        self.tracking = False
        self.minmax = {i: (float("inf"), float("-inf")) for i in self.measured}
        self.samples = {i: 0 for i in self.measured}
        self.last_sample = {i: 0.0 for i in self.measured}
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
            motor.control(pos=goal, vel=0.0, kp=APPROACH_KP, kd=RAMP_KD, torque=0.0)
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
            for i in self.measured:
                m = self.motors[i]
                if m.last_position is not None and m.last_feedback_time > self.last_sample[i]:
                    self.last_sample[i] = m.last_feedback_time
                    self.samples[i] += 1
                    lo, hi = self.minmax[i]
                    self.minmax[i] = (min(lo, m.last_position), max(hi, m.last_position))
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
            excess = limit_excess(m.last_position, i, self.args.limit_margin)
            if excess > 0.0:
                ramp = self.ramps.get(i) if mode == "ramp" else None
                if ramp is not None:
                    allowed = ramp.start_excess
                elif mode == "hold" and i in self.settle and now < self.settle[i][1]:
                    allowed = self.settle[i][0]
                else:
                    allowed = 0.0
                if allowed == 0.0 or excess > allowed + LIMIT_SLACK:
                    return f"ID {i} at {math.degrees(m.last_position):+.1f} deg is outside its joint limit"
            else:
                self.settle.pop(i, None)
        ramping = {i for i, mode in self.mode.items() if mode == "ramp"}
        problems, self.warnings = roll_status(positions, self.measured, ramping)
        return problems[0] if problems else None

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
            if ramp.start_excess > 0.0 and limit_excess(self.motors[i].last_position, i, self.args.limit_margin) > 0.0:
                self.settle[i] = (ramp.start_excess, now + SETTLE_S)
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
        problems = [ramp_problem(i, starts[i], self.hold[i], self.args.limit_margin) for i in self.held]
        problems += roll_status(starts, self.measured)[0]
        problems = [p for p in problems if p]
        if problems:
            raise Abort("; ".join(problems) + "; nothing moves")
        for i in self.measured:
            self.mode[i] = "free"
        duration = max(0.5, max(abs(self.hold[i] - starts[i]) for i in self.held) / APPROACH_SPEED)
        for i in self.held:
            self.ramps[i] = Ramp(i, starts[i], self.hold[i], self.args.limit_margin, duration)
            self.target[i] = starts[i]
            self.mode[i] = "ramp"

    def run(self):
        self.startup()
        self.status = "Move every joint of the measured leg through its full range, then press Enter."
        self.enter.clear()
        self.tracking = True
        while not self.enter.is_set():
            if getattr(self.enter, "closed", False):
                raise Abort("stdin closed")
            self.cycle()
        self.tracking = False
        self.record()
        self.status = "Saved; braking and stopping every motor. Support the legs."
        self.maybe_render(force=True)

    def record(self):
        self.csv_path = new_csv_path()
        leg = next(name for name, ids in LEGS.items() if ids == self.measured)
        held = " ".join(str(i) for i in self.held)
        with open(self.csv_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(CSV_COLUMNS + ["measured_leg", "held_ids"])
            for i in self.measured:
                lo, hi = self.minmax[i]
                writer.writerow(csv_row(i, lo, hi, self.samples[i]) + [leg, held])

    def brake(self):
        self.ramps.clear()
        self.settle.clear()
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
        lines = ["\033[2J\033[3J\033[H"]
        lines.append(f"joint limit scan   {time.strftime('%H:%M:%S')}   measuring IDs {list(self.measured)}   "
                     f"holding IDs {list(self.held)} at 0")
        lines.append("Motor angles in deg (knee and ankle: crank angles). Ctrl-C brakes and stops.\n")
        lines.append(f"  {'ID':>3}  {'joint':<18}  {'current':>8}  {'min':>8}  {'max':>8}  "
                     f"{'span':>7}  {'defined limit':>18}  {'n':>6}")
        lines.append("  " + "-" * 88)
        for i in self.measured:
            lo, hi = self.minmax[i]
            row = summary_line(i, lo, hi, self.samples[i])
            lines.append(f"  {row[:23]}  {format_deg(self.motors[i].last_position):>8}{row[23:]}")
        lines.append("")
        lines.append(f"  {'ID':>3}  {'held joint':<18}  {'mode':<5}  {'target':>8}  {'pos':>8}  {'dev':>7}")
        for i in self.held:
            pos = self.motors[i].last_position
            dev = f"{math.degrees(pos - self.target[i]):+7.2f}" if pos is not None else f"{'--':>7}"
            lines.append(f"  {i:>3}  {JOINT_MAP.get(i, '?'):<18}  {self.mode[i]:<5}  "
                         f"{format_deg(self.target[i]):>8}  {format_deg(pos):>8}  {dev}")
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


def confirm_hold(args, measured, held):
    print(f"Measure: {args.leg} leg {list(measured)}, all six released together (kp=kd=torque=0)")
    print(f"Hold: {list(held)} at 0 for the whole run with the deploy gains; all ramp to 0 together, the farthest at {APPROACH_SPEED} rad/s "
          f"(kp={APPROACH_KP:g} kd={RAMP_KD:g}):")
    for i in held:
        kp, kd = deploy_gains(i)
        lo, hi = JOINT_LIMITS_RAD[i]
        band_lo, band_hi = hold_band(i, args.limit_margin)
        print(f"  ID {i:>2}  {JOINT_MAP.get(i, '?'):<18}  kp={kp:g} kd={kd:g}  "
              f"limit {math.degrees(lo):+.1f}~{math.degrees(hi):+.1f} deg  "
              f"stop outside {math.degrees(band_lo):+.1f}~{math.degrees(band_hi):+.1f} deg")
    print(f"Stops on: feedback loss, fault flags, a held motor faster than {args.max_vel:g} rad/s, "
          f"{math.degrees(HOLD_TOL):.1f} deg off its target, outside its limit, or a held foot rolling past "
          f"{math.degrees(VER2_EDU.foot_roll.limit - ROLL_MARGIN):.0f} deg.")
    print("Press Enter when every joint of the measured leg has been moved through its range; the CSV is saved "
          "and every motor brakes and stops.")
    print("Angles are motor angles (knee and ankle: crank angles).")
    print(f"CAN watchdog: {args.can_timeout_ms} ms on every motor.")
    print("WARNING: The robot must hang on its stand with the feet off the ground; keep the E-stop in reach.")
    if not sys.stdin.isatty():
        print("ERROR: Run this command in an interactive terminal.")
        return False
    try:
        input("Press Enter to start or Ctrl-C to cancel: ")
    except (KeyboardInterrupt, EOFError):
        print("\nCancelled.")
        return False
    return True


def open_channels(args, motor_ids):
    channels = []
    try:
        for name in sorted({channel_for_id(i) for i in motor_ids}):
            bus = can.Bus(channel=name, interface=args.interface)
            motors = [Motor(bus, i, SPECS[MOTOR_MODEL[i]], host_id=args.host_id)
                      for i in motor_ids if channel_for_id(i) == name]
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


def print_leg_summary(scan, aborted):
    print(f"\n{'ID':>3}  {'joint':<18}  {'min':>8}  {'max':>8}  {'span':>7}  {'defined limit':>18}  {'n':>6}")
    for i in scan.measured:
        lo, hi = scan.minmax[i]
        print(summary_line(i, lo, hi, scan.samples[i]))
    if scan.csv_path:
        print(f"Saved: {scan.csv_path}")
    elif aborted:
        print("Not saved (the run stopped before Enter).")


def run_hold(args, measured, held, enter=None):
    channels = open_channels(args, sorted(measured + held))
    aborted = True
    try:
        scan = LegScan(channels, measured, held, args, enter if enter is not None else EnterKey())
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
            try:
                scan.brake()
            except KeyboardInterrupt:
                print("Brake interrupted; sending stop frames now.")
            except Exception as error:
                print(f"Brake failed ({error}); sending stop frames now.")
    finally:
        shutdown_all(channels)
    print_leg_summary(scan, aborted)
    return 1 if aborted else 0


def main(argv=None):
    args = parse_args(argv)
    if args.passive:
        if args.leg:
            print("ERROR: --leg enables motors and cannot be combined with --passive")
            return 1
        motor_ids = sorted(set(args.motor_id)) if args.motor_id is not None else sorted(JOINT_MAP)
        unknown = [i for i in motor_ids if i not in JOINT_MAP]
        if unknown:
            print(f"ERROR: Unsupported motor IDs: {unknown}")
            return 1
        return run_passive(args, motor_ids)
    if args.motor_id is not None:
        print("ERROR: --motor-id is for --passive only; use --leg left|right")
        return 1
    if not args.leg:
        print("ERROR: choose the leg to measure with --leg left|right (or --passive)")
        return 1
    install_stop_signals()
    measured = LEGS[args.leg]
    held = next(ids for name, ids in LEGS.items() if name != args.leg)
    problems = [p for p in [can_timeout_problem(args)] if p]
    problems += [f"ID {i}: hold target {math.degrees(hold_target(i, args.limit_margin)):+.1f} deg is outside its joint limit"
                 for i in held if limit_excess(hold_target(i, args.limit_margin), i, args.limit_margin) > 0.0]
    if problems:
        print("Invalid arguments; nothing will run:")
        for problem in problems:
            print(f"  {problem}")
        return 1
    if not confirm_hold(args, measured, held):
        return 1
    return run_hold(args, measured, held)


if __name__ == "__main__":
    sys.exit(main())
