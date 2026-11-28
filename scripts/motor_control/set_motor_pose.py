#!/usr/bin/env python3
import argparse
import math
import os
import signal
import sys
import time

import can

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "measurements"))
from common import (
    add_can_timeout_option, arm_can_watchdog, can_timeout_problem, disarm_can_watchdog, install_stop_signals,
)
from motor_selection import (
    SELECTOR_HELP,
    BusOpenError,
    SelectionError,
    add_placeholder_option,
    motion_problems,
    open_bus,
    required_variant,
    resolve_motors,
    variant_motors,
    variant_summary,
)
from robonex_common.can import FeedbackHub, Motor
from robonex_common.foot_roll import foot_roll
from robonex_common.joints import ALL_MOTORS, DEFAULT_JOINT_POS, JOINT_BY_MODEL_NAME
from robonex_common.models import VER2_EDU
from robonex_common.motors import MOTOR_SPECS
from robonex_common.protocol import DEFAULT_INTERFACE, HOST_ID, clamp
from robonex_common.joints import channel_for_motor_id as channel_for_id

MOTORS = {
    joint.motor_id: {
        "target_rad": DEFAULT_JOINT_POS.get(joint.model_name, 0.0),
        "model": joint.motor_model,
    }
    for joint in ALL_MOTORS
}

MOVE_SPEED = 0.4
MIN_MOVE_TIME = 3.0
RATE = 100.0
HOLD_KP = 40.0
HOLD_KD = 2.0
OVERSPEED_STOP = 2.0
TRACKING_STOP = math.radians(25.0)
FEEDBACK_TIMEOUT = 0.3
SPECS = MOTOR_SPECS
JOINT_MAP = {joint.motor_id: joint.hardware_name for joint in ALL_MOTORS}
ROLL_MARGIN = math.radians(1.0)
ROLL_PATH_STEPS = 20
FEEDBACK_FAULT_NAMES = (
    "undervoltage", "overcurrent", "overtemperature", "magnetic encoder fault", "stall overload", "uncalibrated",
)


def foot_rolls(positions):
    clip = VER2_EDU.foot_roll
    rolls = {}
    for upper_name, lower_name, sign in clip.pairs:
        upper = JOINT_BY_MODEL_NAME[upper_name].motor_id
        lower = JOINT_BY_MODEL_NAME[lower_name].motor_id
        if upper in positions and lower in positions:
            rolls[(upper, lower)] = foot_roll(sign * positions[upper], sign * positions[lower], clip.coeffs)
    return rolls


def roll_path_problems(starts, targets):
    limit = VER2_EDU.foot_roll.limit
    start_rolls = foot_rolls(starts)
    problems = []
    for pair, roll in foot_rolls(targets).items():
        if abs(roll) > limit - ROLL_MARGIN:
            problems.append(f"target foot roll {math.degrees(roll):+.1f} deg of IDs {pair[0]}/{pair[1]} exceeds "
                            f"{math.degrees(limit - ROLL_MARGIN):.0f} deg")
    for step in range(1, ROLL_PATH_STEPS):
        frac = step / ROLL_PATH_STEPS
        path = {mid: starts[mid] + (targets[mid] - starts[mid]) * frac for mid in targets}
        for pair, roll in foot_rolls(path).items():
            bound = max(limit, abs(start_rolls.get(pair, 0.0)))
            if abs(roll) > bound:
                problems.append(f"foot roll of IDs {pair[0]}/{pair[1]} reaches {math.degrees(roll):+.1f} deg on "
                                f"the way to the targets (bound {math.degrees(bound):.0f} deg)")
                return problems
    return problems


def feedback_fault_names(flags):
    return [name for bit, name in enumerate(FEEDBACK_FAULT_NAMES) if flags & (1 << bit)]




def fmt(rad):
    return f"{rad:+8.4f} rad ({math.degrees(rad):+8.2f} deg)"


def main(zero_pose=False):
    parser = argparse.ArgumentParser(
        description="Move motors slowly to 0 rad and hold until stopped." if zero_pose
        else "Move motors slowly to target angles and hold until stopped.")
    parser.add_argument("--ids", nargs="+", default=None, metavar="SEL",
                        help=f"Motors to move: {SELECTOR_HELP}. Default: legs (the 12 leg motors, "
                             + ("0 rad)" if zero_pose else "policy default pose); head and arm motors go to 0 rad "
                                "when listed"))
    parser.add_argument("--target-deg", nargs="+", default=[], metavar="ID=DEG",
                        help="Override the target of a selected motor in degrees, e.g. 26=-1.18 (must lie inside "
                             "that joint's limits)")
    add_placeholder_option(parser)
    add_can_timeout_option(parser)
    parser.set_defaults(
        interface=DEFAULT_INTERFACE,
        host_id=HOST_ID,
    )
    args = parser.parse_args()
    problem = can_timeout_problem(args)
    if problem:
        parser.error(problem)
    install_stop_signals()

    try:
        variant = required_variant()
        selected = resolve_motors(args.ids or ["legs"], variant)
    except SelectionError as error:
        parser.error(str(error))
    print(variant_summary(variant, variant_motors(variant)))
    problems = motion_problems([joint.motor_id for joint in selected], args.allow_placeholder_limits)
    if problems:
        for problem in problems:
            print(f"ERROR: {problem}")
        print("Nothing moved. For head and arm motors, check the zero and the free range by hand first.")
        return 1
    active_motor_ids = [joint.motor_id for joint in selected]
    overrides = {}
    for item in args.target_deg:
        try:
            key, value = item.split("=", 1)
            motor_id, degrees = int(key), float(value)
        except ValueError:
            parser.error(f"--target-deg expects ID=DEG, got {item!r}")
        if not math.isfinite(degrees):
            parser.error(f"--target-deg {item}: the angle is not finite")
        joint = next((j for j in selected if j.motor_id == motor_id), None)
        if joint is None:
            parser.error(f"--target-deg {item}: ID {motor_id} is not among the selected motors")
        if not joint.lower <= math.radians(degrees) <= joint.upper:
            parser.error(f"--target-deg {item}: outside the joint limits "
                         f"{math.degrees(joint.lower):+.1f}..{math.degrees(joint.upper):+.1f} deg")
        overrides[motor_id] = math.radians(degrees)

    buses = {}
    motors = {}
    try:
        needed_channels = sorted({channel_for_id(mid) for mid in active_motor_ids})
        for channel in needed_channels:
            try:
                buses[channel] = open_bus(channel, args.interface)
            except BusOpenError as error:
                print(f"ERROR: {error}")
                return 1

        for motor_id in active_motor_ids:
            cfg = MOTORS[motor_id]
            spec = SPECS[cfg["model"]]
            bus = buses[channel_for_id(motor_id)]
            motors[motor_id] = {
                "motor": Motor(bus, motor_id, spec, host_id=args.host_id),
                "target_cfg": overrides.get(motor_id, 0.0 if zero_pose else cfg["target_rad"]),
                "target": None,
                "start": None,
            }

        hubs = {
            channel: FeedbackHub(
                bus,
                [m["motor"] for mid, m in motors.items() if channel_for_id(mid) == channel],
                args.host_id,
            )
            for channel, bus in buses.items()
        }
        hub_of = {mid: hubs[channel_for_id(mid)] for mid in motors}

        print(f"{'ID':>3}  {'ch':<5}  {'joint':<18}  {'model':<5}  {'current':>26}  {'target':>26}")
        move_time = MIN_MOVE_TIME
        for motor_id, m in motors.items():
            current = m["motor"].read_mech_position(timeout=0.3)
            channel = channel_for_id(motor_id)
            if current is None:
                print(f"{motor_id:>3}  {channel:<5}  {JOINT_MAP.get(motor_id, '?'):<18}  "
                      f"{m['motor'].spec.name:<5}  {'no position':>26}")
                print(f"ERROR: Motor ID {motor_id} returned no position (no reply, or the read failed). "
                      "Check power, wiring, and ID; nothing moved.")
                return 1
            if abs(current) > math.pi:
                print(f"{motor_id:>3}  {channel:<5}  {JOINT_MAP.get(motor_id, '?'):<18}  "
                      f"{m['motor'].spec.name:<5}  {fmt(current):>26}")
                print(f"ERROR: Motor ID {motor_id} reads outside -180..+180 deg. "
                      "Set zero_sta=1 (0x7029) on it before moving.")
                return 1
            m["current"] = current
            target = current if m["target_cfg"] is None else m["target_cfg"]
            m["target"] = clamp(target, m["motor"].spec.p_min, m["motor"].spec.p_max)
            travel = abs(m["target"] - current)
            move_time = max(move_time, travel / MOVE_SPEED)
            note = "" if m["target_cfg"] is not None else "  (no target: hold current position)"
            print(f"{motor_id:>3}  {channel:<5}  {JOINT_MAP.get(motor_id, '?'):<18}  "
                  f"{m['motor'].spec.name:<5}  {fmt(current):>26}  {fmt(m['target']):>26}{note}")

        roll_problems = roll_path_problems({mid: m["current"] for mid, m in motors.items()},
                                           {mid: m["target"] for mid, m in motors.items()})
        if roll_problems:
            for text in roll_problems:
                print(f"ERROR: {text}")
            print("Nothing moved. Choose ankle targets whose foot roll stays inside the 12 deg clip.")
            return 1

        print(f"Move time: about {move_time:.1f} s at {MOVE_SPEED} rad/s, "
              f"hold gains kp={HOLD_KP}, kd={HOLD_KD}")

        if not sys.stdin.isatty():
            print("ERROR: Run this command in an interactive terminal.")
            return 1
        try:
            answer = input("Press Enter to start, or Ctrl-C to cancel: ")
            del answer
        except (KeyboardInterrupt, EOFError):
            print("\nCancelled.")
            return 0

        def emergency_stop(reason):
            print(f"\nEMERGENCY STOP: {reason}")
            for mm in motors.values():
                try:
                    mm["motor"].stop()
                except can.CanError:
                    pass

        for m in motors.values():
            m["motor"].stop()
        time.sleep(0.02)
        for motor_id, m in motors.items():
            if not arm_can_watchdog(m["motor"], args.can_timeout_ms):
                print(f"ERROR: CAN watchdog not confirmed on ID {motor_id}; nothing moved.")
                return 1

        for motor_id, m in motors.items():
            m["motor"].write_run_mode_operation()
            time.sleep(0.005)
            m["motor"].enable()
            start = hub_of[motor_id].wait_for(motor_id, timeout=0.3)
            if start is None:
                emergency_stop(f"ID {m['motor'].motor_id} has no live feedback")
                return 1
            if abs(start) > math.pi:
                emergency_stop(f"ID {motor_id} feedback {fmt(start)} is outside -180..+180 deg (zero_sta)")
                return 1
            m["start"] = start
            if m["target_cfg"] is None:
                m["target"] = start
            m["cmd"] = start
            m["motor"].control(pos=start, vel=0.0, kp=HOLD_KP, kd=HOLD_KD)

        move_time = MIN_MOVE_TIME
        for m in motors.values():
            move_time = max(move_time, abs(m["target"] - m["start"]) / MOVE_SPEED)

        period = 1.0 / RATE
        roll_bounds = {pair: max(VER2_EDU.foot_roll.limit, abs(roll)) + ROLL_MARGIN
                       for pair, roll in foot_rolls({mid: m["start"] for mid, m in motors.items()}).items()}

        def check_safety(now):

            for mid, mm in motors.items():
                motor = mm["motor"]
                if motor.last_fault:
                    names = ", ".join(feedback_fault_names(motor.last_fault))
                    return f"ID {mid} reports fault flags 0x{motor.last_fault:02X} ({names})"
                if abs(motor.last_velocity) > OVERSPEED_STOP:
                    return f"ID {mid} overspeed ({motor.last_velocity:+.2f} rad/s)"
                if motor.last_feedback_time <= 0.0:
                    return f"ID {mid} never received feedback"
                age = now - motor.last_feedback_time
                if age > FEEDBACK_TIMEOUT:
                    return f"ID {mid} feedback timeout ({age:.2f} s > {FEEDBACK_TIMEOUT} s)"
                if motor.last_position is not None and abs(mm["cmd"] - motor.last_position) > TRACKING_STOP:
                    return (f"ID {mid} tracking error "
                            f"{math.degrees(mm['cmd'] - motor.last_position):+.1f} deg")
            measured = {mid: mm["motor"].last_position for mid, mm in motors.items()
                        if mm["motor"].last_position is not None}
            for pair, roll in foot_rolls(measured).items():
                if abs(roll) > roll_bounds[pair]:
                    return (f"foot roll {math.degrees(roll):+.1f} deg of IDs {pair[0]}/{pair[1]} exceeds "
                            f"{math.degrees(roll_bounds[pair]):.0f} deg")
            return None

        print(f"Moving ({move_time:.1f} s)")
        start_time = time.monotonic()
        while True:
            now = time.monotonic()
            progress = min(1.0, (now - start_time) / move_time)
            smooth = progress * progress * (3.0 - 2.0 * progress)
            smooth_vel = 6.0 * progress * (1.0 - progress) / move_time
            for m in motors.values():
                travel = m["target"] - m["start"]
                m["cmd"] = m["start"] + travel * smooth
                m["motor"].control(
                    pos=m["cmd"],
                    vel=clamp(travel * smooth_vel, -MOVE_SPEED, MOVE_SPEED),
                    kp=HOLD_KP, kd=HOLD_KD,
                )
            for hub in hubs.values():
                hub.pump()
            reason = check_safety(time.monotonic())
            if reason:
                emergency_stop(reason)
                return 1
            if progress >= 1.0:
                break
            sleep = period - (time.monotonic() - now)
            if sleep > 0:
                time.sleep(sleep)

        print("Targets reached. Holding position. Press Ctrl-C to stop.")
        last_print = 0.0
        drawn = 0
        while True:
            now = time.monotonic()
            for m in motors.values():
                m["cmd"] = m["target"]
                m["motor"].control(pos=m["target"], vel=0.0, kp=HOLD_KP, kd=HOLD_KD)
            for hub in hubs.values():
                hub.pump()
            reason = check_safety(time.monotonic())
            if reason:
                emergency_stop(reason)
                return 1
            if now - last_print >= 1.0:
                last_print = now
                lines = [f"[{time.strftime('%H:%M:%S')}] {'ID':>3} {'joint':<16} {'current->target deg':>20} "
                         f"{'error deg':>10} {'torque N*m':>18} {'temp':>6}"]
                for mid, m in motors.items():
                    mm = m["motor"]
                    tmax = mm.spec.t_max
                    cur = mm.last_position
                    if cur is None:
                        line = f"           {mid:>3} {JOINT_MAP.get(mid, '?'):<16} {'(no feedback)':>20}"
                    else:
                        err = math.degrees(m["target"] - cur)
                        tq_pct = abs(mm.last_torque) / tmax * 100.0 if tmax else 0.0
                        line = (f"           {mid:>3} {JOINT_MAP.get(mid, '?'):<16} "
                                f"{math.degrees(cur):+7.1f}->{math.degrees(m['target']):+7.1f}"
                                f" {err:+10.1f}"
                                f" {mm.last_torque:+7.2f}/{tmax:.0f}({tq_pct:3.0f}%)"
                                f" {mm.last_temp:5.1f}C")
                    lines.append(line)
                sys.stdout.write((f"\033[{drawn}F" if drawn else "") + "".join(f"{text}\033[K\n" for text in lines))
                sys.stdout.flush()
                drawn = len(lines)
            sleep = period - (time.monotonic() - now)
            if sleep > 0:
                time.sleep(sleep)

    except KeyboardInterrupt:
        print("\nStop requested.")
    except (can.CanError, OSError) as error:
        print(f"\nERROR: CAN send/receive failed ({error}); stopping.")
        return 1
    finally:
        masked = {sig: signal.signal(sig, signal.SIG_IGN) for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
        for m in motors.values():
            try:
                m["motor"].stop()
            except (can.CanError, OSError):
                pass
            disarm_can_watchdog(m["motor"])
        for bus in buses.values():
            bus.shutdown()
        for sig, handler in masked.items():
            signal.signal(sig, handler)
        print("Stopped.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
