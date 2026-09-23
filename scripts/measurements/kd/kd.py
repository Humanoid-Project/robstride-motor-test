#!/usr/bin/env python3
"""Measure the effective velocity gain of one joint on the real hardware.

MIT mode applies `tau = kp*(pos_t - pos) + kd*(vel_t - vel)`, so holding the position target
fixed and stepping only the velocity target adds `kd*vel_t` to the holding torque while the
joint stays essentially still. Sampling several velocity targets and regressing
`tau = kp*e + kd*vel_t + b` recovers kd without ever letting the joint run.

The joint only shifts by `kd*vel_t/kp` between steps -- about 1 deg at kd=2, vel_t=1, kp=100 --
so this works with the feet on the ground, unlike a free-spin damping test.
"""

import argparse
import math
import os
import sys
import time
from datetime import datetime

import can

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from common import (
    shutdown_motor,
    resolve_model,
    HOST_ID, DEFAULT_INTERFACE, SPECS,
    RUN_MODE_INDEX, RUN_MODE_OPERATION,
    Motor, channel_for_id, active_brake,
    validate_args, report_invalid_args,
    DEFAULT_LIMIT_MARGIN_RAD, joint_limit_for, exceeds_joint_limit,
)

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
ARG_CHECKS = [("kp", "positive"), ("kd", "positive"), ("settle-time", "positive"),
              ("ramp-time", "positive"), ("feedback-timeout", "positive")]


def parse_args():
    p = argparse.ArgumentParser(description="Measure a joint's effective velocity gain.")
    p.add_argument("--motor-id", type=lambda v: int(v, 0), required=True)
    p.add_argument("--model", choices=list(SPECS.keys()), default=None)
    p.add_argument("--kp", type=float, required=True, help="Position gain to hold with")
    p.add_argument("--kd", type=float, required=True,
                   help="Velocity gain to command; the measurement checks whether it is honoured")
    p.add_argument("--vel-targets", type=float, nargs="+", default=[-1.0, -0.5, 0.5, 1.0],
                   help="Velocity targets in rad/s; the joint does not actually run at these")
    p.add_argument("--repeats", type=int, default=2)
    p.add_argument("--ignore-joint-limit", action="store_true")
    p.add_argument("--ramp-time", type=float, default=0.5,
                   help="Seconds to ramp the velocity target in. Set near 0.02 for a step, which "
                        "turns the capture into a command-to-response latency measurement")
    p.add_argument("--sample-all", action="store_true",
                   help="Record the whole hold, not just the settled window (needed for latency)")
    p.set_defaults(interface=DEFAULT_INTERFACE, host_id=HOST_ID,
                   settle_time=1.0, sample_time=0.5,
                   feedback_timeout=0.3, limit_margin=DEFAULT_LIMIT_MARGIN_RAD, out=None)
    return resolve_model(p, p.parse_args())


def confirm(args):
    shift = max(abs(v) for v in args.vel_targets) * args.kd / args.kp
    print(f"Motor: ID {args.motor_id}, {args.model.upper()}, {args.channel}")
    print(f"Holding kp {args.kp}, commanded kd {args.kd}")
    print(f"Velocity targets: {args.vel_targets} rad/s x {args.repeats}")
    print(f"Expected position shift per step: kd*vel/kp = {math.degrees(shift):.2f} deg")
    print(f"Expected torque step: kd*vel = {args.kd*max(abs(v) for v in args.vel_targets):.2f} N.m")
    print()
    print("The joint is NOT commanded to run: the position target stays fixed the whole time.")
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


def hold(motor, pos_target, vel_target, args, duration, collect):
    """Hold `pos_target` while ramping the velocity target in; sample once it is steady."""
    began = time.monotonic()
    end = began + duration
    sample_from = end - args.sample_time
    last_ok = began
    rows = []
    while time.monotonic() < end:
        alpha = min((time.monotonic() - began) / args.ramp_time, 1.0)
        vt = vel_target * alpha
        motor.control(pos=pos_target, vel=vt, kp=args.kp, kd=args.kd, torque=0.0)
        fb = motor.poll_feedback(timeout=0.05)
        now = time.monotonic()
        if fb is None:
            if now - last_ok > args.feedback_timeout:
                return None, "feedback_lost"
            continue
        last_ok = now
        _, pos, vel, tq, temp, fault = fb
        if not args.ignore_joint_limit and exceeds_joint_limit(pos, args.motor_id, args.limit_margin):
            return None, "joint_limit"
        if collect and (args.sample_all or (now >= sample_from and alpha >= 1.0)):
            rows.append((round(now - began, 5), round(vt, 5), pos_target, vel_target,
                         pos, vel, tq, temp))
        print(f"\r  vel_t={vel_target:+5.2f}  pos_err={math.degrees(pos_target-pos):+6.2f}deg  "
              f"vel={vel:+6.3f}  tq={tq:+6.2f} N*m  T={temp:.0f}C    ", end="", flush=True)
    print()
    return rows, "ok"


def main():
    args = parse_args()
    problems = validate_args(args, args.model, ARG_CHECKS)
    if problems:
        report_invalid_args(problems)
        return 1
    args.channel = channel_for_id(args.motor_id)
    if not confirm(args):
        return 1

    os.makedirs(DATA_DIR, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%dT%H%M%S")
    out = args.out or os.path.join(DATA_DIR, f"kd_id{args.motor_id}_{args.model}_{stamp}.csv")
    if os.path.exists(out):
        print(f"ERROR: {out} already exists.")
        return 1

    bus = can.interface.Bus(channel=args.channel, interface="socketcan")
    motor = Motor(bus, args.motor_id, SPECS[args.model], host_id=args.host_id)
    rows = []
    try:
        motor.write_param_u8(RUN_MODE_INDEX, RUN_MODE_OPERATION)
        motor.enable()
        time.sleep(0.2)
        fb = motor.poll_feedback(timeout=1.0)
        if fb is None:
            print("ERROR: no feedback from the motor.")
            return 1
        reference = fb[1]
        print(f"Reference position: {math.degrees(reference):+.2f} deg")
        if not args.ignore_joint_limit:
            lo, hi = joint_limit_for(args.motor_id, args.limit_margin)
            # Twice the predicted shift: the prediction assumes the commanded kd and kp are
            # exactly honoured, which is the very thing being measured.
            # Each sign is checked against its own side. A positive velocity target pushes one
            # way and a negative one the other, so a joint near a hard stop can still be measured
            # with targets of the sign that moves it away from that stop.
            def shift_for(v):
                return 2.0 * abs(v) * args.kd / args.kp
            up = max((shift_for(v) for v in args.vel_targets if v > 0), default=0.0)
            down = max((shift_for(v) for v in args.vel_targets if v < 0), default=0.0)
            bad = []
            if up and reference + up > hi:
                bad.append(f"+{math.degrees(up):.2f} deg exceeds the upper limit "
                           f"{math.degrees(hi):+.1f} from {math.degrees(reference):+.2f}")
            if down and reference - down < lo:
                bad.append(f"-{math.degrees(down):.2f} deg exceeds the lower limit "
                           f"{math.degrees(lo):+.1f} from {math.degrees(reference):+.2f}")
            if bad:
                for line in bad:
                    print(f"ERROR: {line}")
                room_up = math.degrees(hi - reference)
                room_down = math.degrees(reference - lo)
                print(f"Room: {room_down:.2f} deg down, {room_up:.2f} deg up.")
                v_up = room_up * math.radians(1) * args.kp / (2 * args.kd)
                v_dn = room_down * math.radians(1) * args.kp / (2 * args.kd)
                print(f"Largest safe targets from here: {-v_dn:+.2f} to {v_up:+.2f} rad/s.")
                print("Use one-sided --vel-targets if only one direction has room; the fit needs "
                      "a span, not both signs.")
                return 1
        print()

        for rep in range(args.repeats):
            for vt in args.vel_targets:
                print(f"[rep {rep+1}/{args.repeats}] vel target {vt:+.2f} rad/s")
                got, why = hold(motor, reference, vt, args,
                                args.ramp_time + args.settle_time + args.sample_time, True)
                if got is None:
                    print(f"  stopped: {why}")
                    raise SystemExit(1)
                for t_s, vt_now, pt, v, p, vv, tq, temp in got:
                    rows.append((rep, vt, t_s, vt_now, pt, p, vv, tq, temp))
                back, why = hold(motor, reference, 0.0, args, args.ramp_time + 0.4, False)
                if back is None:
                    print(f"  stopped while returning: {why}")
                    raise SystemExit(1)
    except KeyboardInterrupt:
        print("\nStop requested.")
    finally:
        shutdown_motor(motor, bus, (active_brake,))

    if rows:
        import csv as _csv
        with open(out, "w", newline="") as fh:
            w = _csv.writer(fh)
            w.writerow(["# motor_id", args.motor_id, "model", args.model,
                        "commanded_kp", args.kp, "commanded_kd", args.kd])
            w.writerow(["repeat", "vel_target", "t_s", "vel_cmd_now", "pos_target",
                        "pos_rad", "vel_rad_s", "torque_nm", "temp_c"])
            w.writerows(rows)
        print(f"\nWrote {len(rows)} samples to {out}")
        print(f"Analyze: python3 {os.path.join(os.path.dirname(os.path.abspath(__file__)), 'analyze_kd.py')} {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
