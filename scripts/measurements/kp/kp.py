#!/usr/bin/env python3
"""Measure the effective position gain of one joint on the real hardware.

Walking telemetry fits `tau = kp*(cmd - pos) - kd*vel` at anywhere from 8% to 84% of the
commanded kp, and the spread does not follow the motor model, so the fit -- not the motor --
is the likely problem: it omits armature inertia, joint friction and the closed-loop
transmission, all of which matter most on the fast-swinging joints where the fit is worst.

This isolates kp instead of inferring it. The joint is held at a reference, then offset by a
sequence of small steps and left to settle. With the output unloaded and stationary, inertia
and damping drop out and the holding torque is kp times the steady-state error, so a straight
line through (error, torque) has slope kp.

Either protocol works for kp. `tau = kp*e - kd*v` is the control law, not an equilibrium
condition, so at rest the slope of (error, torque) is kp whatever load the joint carries -- the
load sets where the joint settles, not an extra torque. Feet on the ground is the safer setup and
is what the 2026-09-18 bench used.

Hanging the output free buys something else: a dangling limb's gravity torque is computable from
the URDF, so it is the only protocol here that checks the *scale* of the reported torque against
an external reference. This fit cannot -- the firmware closes its loop on the same quantity it
reports, so a common multiplicative error cancels out of the slope.
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
    HOST_ID, DEFAULT_INTERFACE, SPECS,
    RUN_MODE_INDEX, RUN_MODE_OPERATION, FAULT_STA_INDEX,
    Motor, channel_for_id, decode_fault_bits, active_brake,
    validate_args, report_invalid_args,
    JOINT_LIMITS_RAD, DEFAULT_LIMIT_MARGIN_RAD, joint_limit_for, exceeds_joint_limit,
)

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
ARG_CHECKS = [("kp", "positive"), ("kd", "positive"), ("settle-time", "positive"),
              ("ramp-rate", "positive"), ("feedback-timeout", "positive")]


def parse_args():
    p = argparse.ArgumentParser(description="Measure a joint's effective position gain.")
    p.add_argument("--motor-id", type=lambda v: int(v, 0), required=True)
    p.add_argument("--model", choices=list(SPECS.keys()), required=True)
    p.add_argument("--kp", type=float, required=True,
                   help="Position gain to command; the measurement checks whether the joint honours it")
    p.add_argument("--offsets-deg", type=float, nargs="+",
                   default=[-6, -4, -2, 2, 4, 6],
                   help="Step offsets from the reference, in degrees")
    p.add_argument("--repeats", type=int, default=2)
    p.add_argument("--ignore-joint-limit", action="store_true",
                   help="Unloaded bench tests only")
    p.add_argument("--ramp-rate", type=float, default=0.15,
                   help="Commanded position slew while moving between offsets, rad/s")
    p.set_defaults(interface=DEFAULT_INTERFACE, host_id=HOST_ID, kd=2.0,
                   settle_time=1.2, sample_time=0.5, feedback_timeout=0.3,
                   limit_margin=DEFAULT_LIMIT_MARGIN_RAD, out=None)
    return p.parse_args()


def confirm(args, model):
    print(f"Motor: ID {args.motor_id}, {model.upper()}, {args.channel}")
    print(f"Commanded kp {args.kp}, kd {args.kd}")
    print(f"Offsets: {args.offsets_deg} deg x {args.repeats}")
    print()
    print("Feet on the ground is fine: kp is the slope of (error, torque) at rest, and an")
    print("external load moves where the joint settles, not the slope. Keep it still while")
    print("sampling. Suspend the robot only if you also want an external torque-scale check.")
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


def hold(motor, target, args, duration, collect, start=None):
    """Ramp to `target`, hold, and return samples from the last `sample_time`.

    The commanded position is interpolated from `start` over `ramp_time` rather than stepped.
    A step of 6 deg at kp=150 is 15.7 N.m on the first control frame, which snaps a free-hanging
    leg; ramping keeps the peak rate near `ramp_rate` and the torque near its steady value.
    """
    end = time.monotonic() + duration
    sample_from = end - args.sample_time
    began = time.monotonic()
    if start is None:
        start = target
    ramp_s = max(abs(target - start) / args.ramp_rate, 1e-3)
    last_ok = time.monotonic()
    rows = []
    while time.monotonic() < end:
        alpha = min((time.monotonic() - began) / ramp_s, 1.0)
        commanded = start + (target - start) * alpha
        motor.control(pos=commanded, vel=0.0, kp=args.kp, kd=args.kd, torque=0.0)
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
        if collect and now >= sample_from and alpha >= 1.0:
            rows.append((target, pos, vel, tq, temp))
        print(f"\r  target={math.degrees(target):+7.2f}deg  pos={math.degrees(pos):+7.2f}  "
              f"err={math.degrees(target-pos):+6.2f}  tq={tq:+6.2f} N*m  T={temp:.0f}C    ",
              end="", flush=True)
    print()
    return rows, "ok"


def main():
    args = parse_args()
    problems = validate_args(args, args.model, ARG_CHECKS)
    if problems:
        report_invalid_args(problems)
        return 1
    args.channel = channel_for_id(args.motor_id)
    if not confirm(args, args.model):
        return 1

    os.makedirs(DATA_DIR, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%dT%H%M%S")
    out = args.out or os.path.join(DATA_DIR, f"kp_id{args.motor_id}_{args.model}_{stamp}.csv")
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
            bad = [o for o in args.offsets_deg
                   if not (lo <= reference + math.radians(o) <= hi)]
            print(f"Joint limit: {math.degrees(lo):+.1f} to {math.degrees(hi):+.1f} deg; "
                  f"targets span {math.degrees(reference)+min(args.offsets_deg):+.1f} to "
                  f"{math.degrees(reference)+max(args.offsets_deg):+.1f} deg")
            if bad:
                print(f"ERROR: offsets {bad} would leave the joint limit from this reference.")
                print("Move the joint nearer the middle of its range, or pass smaller --offsets-deg.")
                return 1
        print(f"Ramp rate {args.ramp_rate} rad/s: a {max(abs(o) for o in args.offsets_deg):.0f} deg "
              f"step takes {math.radians(max(abs(o) for o in args.offsets_deg))/args.ramp_rate:.2f} s\n")

        for rep in range(args.repeats):
            for off in args.offsets_deg:
                target = reference + math.radians(off)
                print(f"[rep {rep+1}/{args.repeats}] offset {off:+.1f} deg")
                got, why = hold(motor, target, args, args.settle_time + args.sample_time,
                                True, start=reference)
                if got is None:
                    print(f"  stopped: {why}")
                    raise SystemExit(1)
                for t, p, v, tq, temp in got:
                    rows.append((rep, off, t, p, v, tq, temp))
                back, why = hold(motor, reference, args, args.settle_time, False, start=target)
                if back is None:
                    print(f"  stopped on the way back: {why}")
                    raise SystemExit(1)
    except KeyboardInterrupt:
        print("\nStop requested.")
    finally:
        try:
            active_brake(motor)
            motor.stop()
        except Exception:
            pass
        bus.shutdown()

    if rows:
        import csv as _csv
        with open(out, "w", newline="") as fh:
            w = _csv.writer(fh)
            w.writerow(["# motor_id", args.motor_id, "model", args.model,
                        "commanded_kp", args.kp, "commanded_kd", args.kd])
            w.writerow(["repeat", "offset_deg", "target_rad", "pos_rad", "vel_rad_s",
                        "torque_nm", "temp_c"])
            w.writerows(rows)
        print(f"\nWrote {len(rows)} samples to {out}")
        print(f"Analyze: python3 {os.path.join(os.path.dirname(os.path.abspath(__file__)), 'analyze_kp.py')} {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
