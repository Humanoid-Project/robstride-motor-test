#!/usr/bin/env python3





import argparse
import math
import os
import signal
import subprocess
import sys
import time
from datetime import datetime

import can

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from common import (
    shutdown_motor,
    resolve_model,
    HOST_ID, DEFAULT_INTERFACE, SPECS, RATED_TORQUE, PEAK_TORQUE,
    RUN_MODE_INDEX, RUN_MODE_OPERATION, MECH_POS_INDEX, FAULT_STA_INDEX,
    Motor, channel_for_id, decode_fault_bits, active_brake,
    validate_args, report_invalid_args,
    JOINT_LIMITS_RAD, DEFAULT_LIMIT_MARGIN_RAD, joint_limit_for, exceeds_joint_limit, MOTOR_MODEL,
)
from robonex_common.can import FeedbackHub
from robonex_common.foot_roll import foot_roll
from robonex_common.joints import DEFAULT_JOINT_POS, JOINT_BY_ID, JOINT_BY_MODEL_NAME
from robonex_common.models import VER2_EDU
from robonex_common.motors import JOINT_CONTROL_GAINS

ARG_CHECKS = [
    ("start-torque", "torque"), ("step", "positive"), ("max-torque", "torque"),
    ("probe-time", "positive"), ("move-threshold", "positive"),
    ("max-vel", "speed"), ("max-time", "positive"),
    ("settle-pos-tol", "positive"), ("settle-time", "positive"),
    ("settle-timeout", "positive"), ("settle-max-vel", "speed"),
    ("feedback-timeout", "positive"),
]

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
ANALYZE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "analyze_friction.py")
PAUSE_S = 1.0
APPROACH_SPEED = 0.3
APPROACH_KP = 15.0
APPROACH_KD = 3.0
POSE_TOL = math.radians(2.0)
HOLD_TOL = 0.10
APPROACH_TOL = 0.20
ROLL_MARGIN = math.radians(1.0)


def positive_int(value):
    value = int(value)
    if value < 1:
        raise argparse.ArgumentTypeError("Must be an integer of 1 or greater")
    return value


def parse_args():
    p = argparse.ArgumentParser(description="Measure and analyze motor breakaway friction.")
    p.add_argument("--motor-id", type=lambda v: int(v, 0), required=True)
    p.add_argument("--model", choices=list(SPECS.keys()), default=None)
    p.add_argument("--signs", type=int, nargs="+", choices=[1, -1], default=[1, -1],
                   help="Motor torque directions")
    p.add_argument("--repeats", type=positive_int, default=1, help="Repeats per direction")
    p.add_argument("--max-torque", type=float, default=1.0,
                   help="Torque search upper bound in N*m; a run ends without a breakaway if no movement by then")
    p.add_argument("--ignore-joint-limit", action="store_true",
                   help="Disable joint-limit checks for unloaded bench tests only")
    p.add_argument("--pose", choices=["current", "zero", "stand"], default="current",
                   help="Start pose: current (as found), zero, or stand (the policy default pose); the measured and held "
                        "motors move there slowly before each search")
    p.add_argument("--hold-ids", type=lambda v: int(v, 0), nargs="+", default=[],
                   help="Motor IDs on the same CAN channel held at the start pose with their deploy gains during the "
                        "search (e.g. the other ankle motor of the same leg)")
    p.set_defaults(
        interface=DEFAULT_INTERFACE, host_id=HOST_ID,
        start_torque=0.02, step=0.02, probe_time=0.3,
        move_threshold=0.03, max_vel=3.0, max_time=30.0,
        settle_pos_tol=0.01, settle_time=0.3, settle_timeout=5.0,
        settle_max_vel=3.0, feedback_timeout=0.3, out=None,
        limit_margin=DEFAULT_LIMIT_MARGIN_RAD,
    )
    args = resolve_model(p, p.parse_args())
    if args.max_torque <= args.start_torque:
        p.error(f"--max-torque must be above the start torque {args.start_torque} N*m")
    search_s = (math.ceil((args.max_torque - args.start_torque) / args.step) + 1) * args.probe_time
    args.max_time = max(args.max_time, search_s + 5.0)
    return args


def confirm(args, model):
    rated = RATED_TORQUE[model]
    n_steps_est = int(math.ceil((args.max_torque - args.start_torque) / args.step)) + 1
    print(f"Motor: ID {args.motor_id}, {model.upper()}, {args.channel}")
    print(f"Directions: {args.signs} x {args.repeats}")
    print(f"Torque search: {args.start_torque} to {args.max_torque} N*m, step={args.step} N*m")
    if args.ignore_joint_limit:
        print("WARNING: Joint-limit checks are disabled.")
    else:
        lo, hi = JOINT_LIMITS_RAD.get(args.motor_id, (None, None))
        if lo is None:
            print(f"ERROR: No joint limit is defined for motor ID {args.motor_id}.")
        else:
            print(f"Joint limit: {math.degrees(lo):+.1f} to {math.degrees(hi):+.1f} deg")
    print(f"Start pose: {args.pose}" + (f"; hold IDs {args.hold_ids} at their deploy gains" if args.hold_ids else ""))
    if args.pose != "current":
        for i in [args.motor_id] + args.hold_ids:
            print(f"  ID {i}: moves slowly to {math.degrees(pose_target(i, args.pose)):+.2f} deg before each search")
    partner = partner_ankle(args.motor_id)
    if partner is not None and partner not in args.hold_ids:
        print(f"NOTE: ID {args.motor_id} is an ankle motor; add --hold-ids {partner} so the linkage does not back-drive "
              f"the free partner motor.")
    print("WARNING: Verify zero calibration, hang the robot with the feet off the ground, and prepare the E-stop.")
    if args.max_torque > rated:
        print(f"WARNING: Maximum torque exceeds the {rated} N*m rated torque.")
    if args.max_torque > PEAK_TORQUE[model]:
        print(f"ERROR: Maximum torque exceeds the {PEAK_TORQUE[model]} N*m peak torque.")
        return False
    if not sys.stdin.isatty():
        print("ERROR: Run this command in an interactive terminal.")
        return False
    try:
        input("Press Enter to continue or Ctrl-C to cancel: ")
    except (KeyboardInterrupt, EOFError):
        print("\nCancelled.")
        return False
    return True


def pose_target(motor_id, pose):
    if pose == "zero":
        return 0.0
    return float(DEFAULT_JOINT_POS[JOINT_BY_ID[motor_id].model_name])


def deploy_gains(motor_id):
    name = JOINT_BY_ID[motor_id].model_name
    return JOINT_CONTROL_GAINS[name[2:-len("_joint")]]


def roll_problems(positions):
    clip = VER2_EDU.foot_roll
    problems = []
    for upper_name, lower_name, sign in clip.pairs:
        upper = JOINT_BY_MODEL_NAME[upper_name].motor_id
        lower = JOINT_BY_MODEL_NAME[lower_name].motor_id
        if upper in positions and lower in positions:
            roll = foot_roll(sign * positions[upper], sign * positions[lower], clip.coeffs)
            if abs(roll) > clip.limit - ROLL_MARGIN:
                problems.append(f"foot roll {math.degrees(roll):+.1f} deg of IDs {upper}/{lower} exceeds "
                                f"{math.degrees(clip.limit - ROLL_MARGIN):.0f} deg")
    return problems


def limit_problems(positions, args):
    if args.ignore_joint_limit:
        return []
    return [f"ID {motor_id} target {math.degrees(pos):+.1f} deg is outside its joint limit"
            for motor_id, pos in positions.items() if exceeds_joint_limit(pos, motor_id, args.limit_margin)]


def partner_ankle(motor_id):
    for upper_name, lower_name, _ in VER2_EDU.foot_roll.pairs:
        ids = (JOINT_BY_MODEL_NAME[upper_name].motor_id, JOINT_BY_MODEL_NAME[lower_name].motor_id)
        if motor_id in ids:
            return ids[1] if ids[0] == motor_id else ids[0]
    return None


class Hold:
    def __init__(self, motor, target, kp, kd):
        self.motor = motor
        self.target = target
        self.kp = kp
        self.kd = kd


class HeldMotor:
    def __init__(self, motor, hub, holds, args):
        self.motor = motor
        self.hub = hub
        self.holds = holds
        self.args = args

    def __getattr__(self, name):
        return getattr(self.motor, name)

    def control(self, pos, vel, kp, kd, torque=0.0):
        self.motor.control(pos=pos, vel=vel, kp=kp, kd=kd, torque=torque)
        for hold in self.holds:
            hold.motor.control(pos=hold.target, vel=0.0, kp=hold.kp, kd=hold.kd, torque=0.0)

    def poll_feedback(self, timeout=0.05):
        if self.hub.wait_for(self.motor.motor_id, timeout) is None:
            return None
        m = self.motor
        return (m.last_feedback_time, m.last_position, m.last_velocity, m.last_torque, m.last_temp, m.last_fault)

    def hold_problem(self, tol=HOLD_TOL):
        now = time.monotonic()
        positions = {}
        if self.motor.last_position is not None:
            positions[self.motor.motor_id] = self.motor.last_position
        for hold in self.holds:
            m = hold.motor
            if m.last_position is None or now - m.last_feedback_time > self.args.feedback_timeout:
                return f"hold ID {m.motor_id} feedback lost"
            positions[m.motor_id] = m.last_position
            if abs(m.last_position - hold.target) > tol:
                return f"hold ID {m.motor_id} left its target by {math.degrees(m.last_position - hold.target):+.1f} deg"
            if abs(m.last_velocity) >= self.args.max_vel:
                return f"hold ID {m.motor_id} speed {m.last_velocity:+.2f} rad/s"
            if exceeds_joint_limit(m.last_position, m.motor_id, self.args.limit_margin):
                return f"hold ID {m.motor_id} reached its joint limit"
        problems = roll_problems(positions)
        return problems[0] if problems else None

    def stop(self, clear_fault=False):
        for hold in self.holds:
            try:
                hold.motor.stop()
            except Exception as error:
                print(f"Stop frame for hold ID {hold.motor.motor_id} failed: {error}")
        self.motor.stop()


def feedback_position(motor, hub, kd, tries=3):
    for _ in range(tries):
        motor.control(pos=0.0, vel=0.0, kp=0.0, kd=kd, torque=0.0)
        pos = hub.wait_for(motor.motor_id, 0.1)
        if pos is not None:
            return pos
    return None


def release_holds(held):
    for hold in held.holds:
        m = hold.motor
        if m.last_position is not None:
            hold.target = m.last_position
        hold.kp, hold.kd = 0.0, APPROACH_KD


def approach(held, targets, starts, args):
    span = max(abs(targets[i] - starts[i]) for i in targets)
    ramp_time = max(0.5, span / APPROACH_SPEED)
    measured = held.motor.motor_id
    gains = {hold.motor.motor_id: (hold.kp, hold.kd) for hold in held.holds}
    for hold in held.holds:
        hold.kp, hold.kd = APPROACH_KP, APPROACH_KD
    t0 = time.monotonic()
    deadline = t0 + ramp_time + 1.5
    last_ok = t0
    last_print = 0.0
    ok = False
    try:
        while time.monotonic() < deadline:
            progress = min(1.0, (time.monotonic() - t0) / ramp_time)
            smooth = progress * progress * (3.0 - 2.0 * progress)
            for hold in held.holds:
                i = hold.motor.motor_id
                hold.target = starts[i] + (targets[i] - starts[i]) * smooth
            goal = starts[measured] + (targets[measured] - starts[measured]) * smooth
            held.control(pos=goal, vel=0.0, kp=APPROACH_KP, kd=APPROACH_KD, torque=0.0)
            fb = held.poll_feedback(timeout=0.05)
            now = time.monotonic()
            if fb is None:
                if now - last_ok > args.feedback_timeout:
                    print(f"\nEMERGENCY STOP: no feedback from ID {measured} for {args.feedback_timeout}s during the approach.")
                    return False
                continue
            last_ok = now
            _, pos, vel, _, _, _ = fb
            if abs(vel) >= args.max_vel:
                print(f"\nEMERGENCY STOP: |vel|={vel:+.3f} rad/s during the approach.")
                return False
            if exceeds_joint_limit(pos, measured, args.limit_margin):
                print(f"\nJoint limit reached during the approach at {math.degrees(pos):+.1f} deg.")
                return False
            problem = held.hold_problem(tol=APPROACH_TOL)
            if problem:
                print(f"\nEMERGENCY STOP: {problem} during the approach.")
                return False
            if now - last_print >= 0.1:
                print(f"\r  approach {progress * 100:5.1f}%  pos={math.degrees(pos):+7.2f} deg  "
                      f"target={math.degrees(targets[measured]):+7.2f} deg    ", end="", flush=True)
                last_print = now
        print()
        error = abs((held.motor.last_position or 0.0) - targets[measured])
        if error > POSE_TOL:
            print(f"WARNING: Measured motor is {math.degrees(error):.1f} deg from the start pose after the approach.")
        ok = True
        return True
    finally:
        if ok:
            for hold in held.holds:
                i = hold.motor.motor_id
                hold.target = targets[i]
                hold.kp, hold.kd = gains[i]
        else:
            release_holds(held)


def wait_settled(motor, hold_pos, args):

    deadline = time.monotonic() + args.settle_timeout
    ref_pos = None
    ref_time = time.monotonic()
    last_pos = hold_pos
    misses = 0
    last_print = 0.0
    while time.monotonic() < deadline:
        motor.control(pos=0.0, vel=0.0, kp=0.0, kd=2.0, torque=0.0)
        fb = motor.poll_feedback(timeout=0.05)
        now = time.monotonic()
        if fb is None:
            misses += 1
            status = f"No reply (miss {misses})"
        else:
            _, pos, vel, tq, temp, fault = fb
            last_pos = pos
            if abs(vel) >= args.settle_max_vel:
                print()
                print(f"EMERGENCY STOP: |vel|={vel:+.3f} rad/s exceeds {args.settle_max_vel}.")
                motor.stop()
                return None
            if ref_pos is None or abs(pos - ref_pos) > args.settle_pos_tol:
                ref_pos = pos
                ref_time = now
            elif now - ref_time >= args.settle_time:
                print()
                return last_pos
            status = f"pos={pos:+.4f} rad  quiet={now - ref_time:4.2f}/{args.settle_time:.2f}s"
        if now - last_print >= 0.1:
            print(f"\r  {status}    ", end="", flush=True)
            last_print = now
    print()
    return None


def return_to_zero(motor, timeout=2.5, kp=15.0, kd=3.0, pos_tol=0.02, max_speed=0.4, target=0.0):
    fb = None
    for _ in range(3):
        motor.control(pos=0.0, vel=0.0, kp=0.0, kd=kd, torque=0.0)
        fb = motor.poll_feedback(timeout=0.1)
        if fb is not None:
            break
    if fb is None:
        print("WARNING: Return position unavailable; sending stop only.")
        return None
    _, start_pos, _, _, _, _ = fb
    ramp_time = max(0.5, abs(start_pos - target) / max_speed)
    deadline = time.monotonic() + max(timeout, ramp_time + 0.5)
    ramp_start = time.monotonic()
    last_pos = start_pos
    while time.monotonic() < deadline:
        progress = min(1.0, (time.monotonic() - ramp_start) / ramp_time)
        smooth = progress * progress * (3.0 - 2.0 * progress)
        goal = start_pos + (target - start_pos) * smooth
        motor.control(pos=goal, vel=0.0, kp=kp, kd=kd, torque=0.0)
        fb = motor.poll_feedback(timeout=0.05)
        if fb is None:
            continue
        _, pos, vel, tq, temp, fault = fb
        last_pos = pos
        if progress >= 1.0 and abs(pos - target) <= pos_tol and abs(vel) < 0.05:
            return last_pos
    print(f"WARNING: Failed to return to {target:+.4f} rad within {timeout}s; position={last_pos:+.4f} rad.")
    return last_pos


def run_search(motor, pos_start, args):
    rows = []
    misses = 0
    t0 = time.monotonic()
    current_torque = args.sign * args.start_torque
    step = args.sign * args.step
    max_torque_signed = args.sign * args.max_torque
    step_deadline = t0 + args.probe_time
    stop_reason = "max_time"
    last_print = 0.0
    step_start_index = 0
    mismatch_streak = 0
    steps_taken = 0
    last_ok = t0
    try:
        while True:
            now = time.monotonic()
            elapsed = now - t0
            if elapsed >= args.max_time:
                stop_reason = "max_time"
                break
            if now >= step_deadline:
                finished_rows = rows[step_start_index:]
                if finished_rows:
                    mean_tq = sum(r[4] for r in finished_rows) / len(finished_rows)
                    if abs(mean_tq - current_torque) > max(0.08, 0.5 * abs(current_torque)):
                        mismatch_streak += 1
                    else:
                        mismatch_streak = 0
                    if mismatch_streak >= 2:
                        stop_reason = "comm_mismatch"
                        break
                step_start_index = len(rows)
                current_torque += step
                steps_taken += 1
                step_deadline = now + args.probe_time
                print()
                print(f"Torque: {current_torque:+.3f} N*m")
                if abs(current_torque) > abs(max_torque_signed):
                    stop_reason = "max_torque_no_movement"
                    break
            motor.control(pos=0.0, vel=0.0, kp=0.0, kd=0.0, torque=current_torque)
            fb = motor.poll_feedback(timeout=0.05)
            if fb is None:
                misses += 1
                if time.monotonic() - last_ok > args.feedback_timeout:
                    stop_reason = "feedback_lost"
                    break
                continue
            last_ok = time.monotonic()
            t_recv, pos, vel, tq, temp, fault = fb
            rows.append((t_recv - t0, pos - pos_start, vel, current_torque, tq, temp, fault))
            if now - last_print >= 0.1:
                print(f"\r  torque={current_torque:+.3f} N*m  pos={pos - pos_start:+.4f} rad  "
                      f"vel={vel:+.3f} rad/s  tq={tq:+.3f} N*m    ", end="", flush=True)
                last_print = now
            if abs(vel) >= args.max_vel:
                stop_reason = "max_vel"
                break
            if not args.ignore_joint_limit and exceeds_joint_limit(pos, args.motor_id, args.limit_margin):
                print(f"\nJoint limit reached at {pos:+.4f} rad ({math.degrees(pos):+.1f} deg).")
                stop_reason = "joint_limit"
                break
            hold_problem = getattr(motor, "hold_problem", None)
            problem = hold_problem() if hold_problem is not None else None
            if problem:
                print(f"\nSTOP: {problem}.")
                stop_reason = "hold_lost"
                break
            if abs(pos - pos_start) >= args.move_threshold:
                if steps_taken == 0:
                    stop_reason = "movement_at_start_suspicious"
                else:
                    stop_reason = "movement_detected"
                break
    except KeyboardInterrupt:
        stop_reason = "keyboard_interrupt"
    print()
    return rows, stop_reason, current_torque


def save_csv(path, rows, meta):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        for key, value in meta.items():
            f.write(f"# {key}: {value}\n")
        f.write("t_s,pos_rel_rad,vel_rad_s,torque_cmd_Nm,torque_meas_Nm,temp_C,fault_hex\n")
        for t_s, pos, vel, tcmd, tq, temp, fault in rows:
            f.write(f"{t_s:.6f},{pos:.6f},{vel:.6f},{tcmd:.4f},{tq:.4f},{temp:.1f},{fault:02X}\n")


def capture_once(args, sign, run_index):
    args.sign = sign
    spec = SPECS[args.model]
    bus = can.Bus(channel=args.channel, interface=args.interface)
    motor = None
    held = None
    hold_motors = []
    rows, stop_reason, torque_at_stop = [], "not_started", 0.0
    pos_start = None
    targets = {}
    plain = not args.hold_ids and args.pose == "current"
    return_target = 0.0
    may_return = plain
    try:
        motor = Motor(bus, args.motor_id, spec, host_id=args.host_id)
        hold_motors = [Motor(bus, i, SPECS[MOTOR_MODEL[i]], host_id=args.host_id) for i in args.hold_ids]
        hub = FeedbackHub(bus, [motor] + hold_motors, host_id=args.host_id)

        motor.stop()
        for m in hold_motors:
            m.stop()
        time.sleep(0.02)
        initial_pos = motor.read_param_f32(MECH_POS_INDEX, timeout=0.3)
        if initial_pos is None:
            print(f"ERROR: Motor ID {args.motor_id} did not reply on {args.channel}.")
            return 1, None, "no_response"
        print(f"Position: {initial_pos:+.4f} rad ({math.degrees(initial_pos):+.2f} deg)")

        fault_reg = motor.read_param(FAULT_STA_INDEX, fmt="<I", timeout=0.3)
        if fault_reg is None:
            print("WARNING: Fault register 0x3022 could not be read.")
        else:
            bits = decode_fault_bits(fault_reg)
            print(f"Fault register: 0x{fault_reg:08X}" + (f" ({', '.join(bits)})" if bits else " (OK)"))

        for m in [motor] + hold_motors:
            m.write_param_u8(RUN_MODE_INDEX, RUN_MODE_OPERATION)
            time.sleep(0.01)
            m.enable()
            time.sleep(0.01)

        active = motor
        if not plain:
            starts = {}
            for m in [motor] + hold_motors:
                pos = feedback_position(m, hub, kd=APPROACH_KD)
                if pos is None:
                    print(f"ERROR: No type 0x02 feedback from ID {m.motor_id}.")
                    return 1, None, "no_response"
                starts[m.motor_id] = pos
            bad = [f"ID {i} reads {math.degrees(q):+.1f} deg, outside its joint limit or +-180 deg (check zero_sta)"
                   for i, q in starts.items() if abs(q) > math.pi or exceeds_joint_limit(q, i, args.limit_margin)]
            if bad:
                for problem in bad:
                    print(f"ERROR: {problem}; nothing moves.")
                return 1, None, "start_refused"
            if args.pose == "current":
                targets = dict(starts)
            else:
                targets = {i: pose_target(i, args.pose) for i in starts}
            problems = limit_problems(targets, args) + roll_problems(targets)
            if problems:
                for problem in problems:
                    print(f"ERROR: {problem}; nothing moves.")
                return 1, None, "pose_refused"
            holds = [Hold(m, starts[m.motor_id], *deploy_gains(m.motor_id)) for m in hold_motors]
            held = HeldMotor(motor, hub, holds, args)
            active = held
            return_target = targets[args.motor_id]
            print("Start pose: " + ", ".join(f"ID {i} {math.degrees(targets[i]):+.2f} deg" for i in targets)
                  + (f"; holding {args.hold_ids}" if holds else ""))
            if not approach(held, targets, starts, args):
                return 1, None, "approach_failed"
            may_return = True

        print("Waiting for the motor to settle...")
        pos_start = wait_settled(active, initial_pos, args)
        if pos_start is None:
            print("ERROR: The motor did not settle before the safety timeout.")
            may_return = False
            active.stop()
            return 1, None, "not_settled"
        print(f"Settled at {pos_start:+.4f} rad. Starting torque search...")
        if held is not None:
            problem = held.hold_problem()
            if problem:
                print(f"ERROR: {problem} before the search.")
                may_return = False
                return 1, None, "hold_lost"
            if abs(pos_start - targets[args.motor_id]) > POSE_TOL:
                print(f"WARNING: The measured joint settled {math.degrees(pos_start - targets[args.motor_id]):+.1f} deg "
                      f"from the start pose (gravity or friction); the result is for {math.degrees(pos_start):+.1f} deg.")

        if not args.ignore_joint_limit and exceeds_joint_limit(pos_start, args.motor_id, args.limit_margin):
            print(f"ERROR: Start position {pos_start:+.4f} rad is outside the joint limit.")
            may_return = False
            active.stop()
            return 1, None, "initial_joint_limit"

        rows, stop_reason, torque_at_stop = run_search(active, pos_start, args)
        if stop_reason in ("hold_lost", "keyboard_interrupt") and not plain:
            may_return = False
    except KeyboardInterrupt:
        print("\nInterrupted.")
        stop_reason = "keyboard_interrupt"
        if not plain:
            may_return = False
    finally:
        def return_home(m):
            return return_to_zero(m, target=return_target)
        steps = (active_brake, return_home) if may_return else (active_brake,)
        if not may_return and not plain:
            print("Braking only; no return move.")
        if held is not None:
            if not may_return:
                release_holds(held)
            shutdown_motor(held, bus, steps)
        else:
            previous = signal.signal(signal.SIGINT, signal.SIG_IGN)
            try:
                for m in hold_motors:
                    try:
                        m.stop()
                    except Exception as error:
                        print(f"Stop frame for ID {m.motor_id} failed: {error}")
            finally:
                signal.signal(signal.SIGINT, previous)
            shutdown_motor(motor, bus, steps)

    if not rows:
        print(f"ERROR: No samples recorded ({stop_reason}).")
        return 1, None, stop_reason

    duration = rows[-1][0] - rows[0][0]
    achieved_hz = (len(rows) - 1) / duration if duration > 0 else 0.0

    print(f"Search: {stop_reason}, samples={len(rows)}, duration={duration*1000:.1f} ms, rate={achieved_hz:.0f} Hz")

    breakaway_low = breakaway_high = breakaway_mid = None
    if stop_reason == "movement_detected":
        breakaway_low, breakaway_high = sorted((torque_at_stop - args.sign * args.step, torque_at_stop))
        breakaway_mid = (breakaway_low + breakaway_high) / 2.0
        print(f"Breakaway bracket: [{min(breakaway_low, breakaway_high):.3f}, "
              f"{max(breakaway_low, breakaway_high):.3f}] N*m")
        print(f"Breakaway estimate: {breakaway_mid:+.3f} N*m (±{abs(args.step)/2:.3f})")
    elif stop_reason == "movement_at_start_suspicious":
        print(f"WARNING: Movement started at the first torque step ({torque_at_stop:+.3f} N*m); discard and repeat.")
    elif stop_reason == "max_torque_no_movement":
        print(f"WARNING: No movement up to {args.max_torque} N*m; check the mechanism.")
    elif stop_reason == "max_vel":
        print("WARNING: Safety speed limit reached before movement detection.")
    elif stop_reason == "feedback_lost":
        print(f"ERROR: Feedback was lost for more than {args.feedback_timeout}s; discard this result.")
    elif stop_reason == "comm_mismatch":
        print("ERROR: Commanded and measured torque disagree; discard this result and check CAN.")
    elif stop_reason == "joint_limit":
        print(f"WARNING: Joint limit reached at {torque_at_stop:+.3f} N*m before breakaway.")

    ts = datetime.now().strftime("%Y%m%dT%H%M%S_%f")
    sign_tag = "pos" if args.sign > 0 else "neg"
    fname = f"id{args.motor_id}_{args.model}_{sign_tag}_{ts}_r{run_index:02d}.csv"
    out_path = os.path.join(DATA_DIR, fname)

    meta = {
        "motor_id": args.motor_id,
        "model": args.model,
        "channel": args.channel,
        "host_id": f"0x{args.host_id:02X}",
        "sign": args.sign,
        "start_torque_Nm": f"{args.start_torque:.4f}",
        "step_Nm": f"{args.step:.4f}",
        "probe_time_s": f"{args.probe_time:.3f}",
        "move_threshold_rad": f"{args.move_threshold:.4f}",
        "max_torque_Nm": f"{args.max_torque:.4f}",
        "started_at": datetime.now().isoformat(timespec="seconds"),
        "pos_start_rad": f"{pos_start:.6f}" if pos_start is not None else "",
        "pose": args.pose,
        "start_target_rad": f"{targets[args.motor_id]:.6f}" if args.motor_id in targets else "",
        "hold_ids": " ".join(str(i) for i in args.hold_ids),
        "hold_targets_rad": " ".join(f"{targets[i]:.6f}" for i in args.hold_ids if i in targets),
        "stop_reason": stop_reason,
        "breakaway_low_Nm": f"{breakaway_low:.4f}" if breakaway_low is not None else "",
        "breakaway_high_Nm": f"{breakaway_high:.4f}" if breakaway_high is not None else "",
        "breakaway_mid_Nm": f"{breakaway_mid:.4f}" if breakaway_mid is not None else "",
        "samples": len(rows),
        "duration_s": f"{duration:.4f}",
    }
    save_csv(out_path, rows, meta)
    print(f"Saved: {out_path}")
    return (0 if stop_reason == "movement_detected" else 1), out_path, stop_reason


def main():
    args = parse_args()
    args.channel = channel_for_id(args.motor_id)
    problems = []
    for i in args.hold_ids:
        if i == args.motor_id:
            problems.append(f"--hold-ids must not contain the measured ID {i}")
        elif i not in JOINT_BY_ID:
            problems.append(f"--hold-ids {i} is not a RoboNex leg joint")
        elif channel_for_id(i) != args.channel:
            problems.append(f"--hold-ids {i} is on {channel_for_id(i)}, not {args.channel}")
    if len(set(args.hold_ids)) != len(args.hold_ids):
        problems.append("--hold-ids has duplicates")
    if args.pose != "current" and args.motor_id not in JOINT_BY_ID:
        problems.append(f"--pose {args.pose} needs a RoboNex leg joint ID")
    partner = partner_ankle(args.motor_id)
    if args.pose != "current" and partner is not None and partner not in args.hold_ids:
        problems.append(f"--pose {args.pose} on ankle ID {args.motor_id} needs --hold-ids {partner}")
    if args.ignore_joint_limit and (args.hold_ids or args.pose != "current"):
        problems.append("--ignore-joint-limit cannot be combined with --pose or --hold-ids")
    if not problems and args.pose != "current":
        targets = {i: pose_target(i, args.pose) for i in [args.motor_id] + args.hold_ids}
        problems += limit_problems(targets, args) + roll_problems(targets)
    if report_invalid_args(problems):
        return 1

    for sign in args.signs:
        args.sign = sign
        if report_invalid_args(validate_args(args, args.model, ARG_CHECKS)):
            return 1
    if args.start_torque > args.max_torque:
        print("ERROR: Internal start torque exceeds maximum torque.")
        return 1
    if not confirm(args, args.model):
        return 1

    plan = [sign for sign in args.signs for _ in range(args.repeats)]
    paths = []
    failed = False
    for index, sign in enumerate(plan, 1):
        print(f"\n[{index}/{len(plan)}] sign={sign:+d}")
        rc, path, stop_reason = capture_once(args, sign, index)
        if rc == 0 and path is not None:
            paths.append(path)
        if rc != 0 or stop_reason == "keyboard_interrupt":
            failed = True
            print("Stopping the remaining runs for safety.")
            break
        if index < len(plan):
            time.sleep(PAUSE_S)

    if paths:
        print("\nAnalyzing captured CSV files...")
        analysis = subprocess.run([sys.executable, ANALYZE, *paths])
        failed = failed or analysis.returncode != 0
    return 1 if failed or not paths else 0


if __name__ == "__main__":
    sys.exit(main())
