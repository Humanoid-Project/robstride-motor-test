#!/usr/bin/env python3
"""Hold one motor at a fixed position under load and log its temperature until a stop rule ends the run.

The motor holds the current position (or --target-deg) with --kp/--kd and an optional feed-forward torque. Temperature,
position, velocity, reported torque, the filtered q-axis current (iqf, 0x701A) and the type-0x02 fault flags are logged
to CSV. The hold ends at --stop-temp-c, --max-minutes, a temperature plateau (--plateau-stop), any fault flag, a
feedback timeout, a tracking error, overspeed, a joint stop limit, Ctrl-C or SIGTERM/SIGHUP. The motor is then braked
and stopped, and a cool-down phase keeps logging with the motor disabled.
"""

import argparse
import bisect
import math
import os
import signal
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
    RUN_MODE_INDEX, RUN_MODE_OPERATION, FAULT_STA_INDEX,
    Motor, channel_for_id, decode_fault_bits, active_brake,
    validate_args, report_invalid_args,
    DEFAULT_LIMIT_MARGIN_RAD, exceeds_joint_limit, exceeds_stop_limit, outside_position_domain,
    add_can_timeout_option, can_timeout_problem, arm_can_watchdog, install_stop_signals,
)
from robonex_common.can import drain

MODE_RUNNING = 2
IQF_INDEX = 0x701A
IQ_READ_TIMEOUT = 0.03
CONTROL_HZ = 50.0
RAMP_SPEED = 0.3
MIN_RAMP_S = 1.0
TAU_RAMP_S = 2.0
MAX_STOP_TEMP_C = 65.0
MAX_TRACKING_DEG = 25.0
MAX_MINUTES = 120.0
MAX_RATE_HZ = 10.0
PLATEAU_WINDOW_S = 600.0
PLATEAU_RISE_C = 1.0
COOLDOWN_FEEDBACK_TIMEOUT = 10.0
FEEDBACK_FAULT_NAMES = (
    "undervoltage", "overcurrent", "overtemperature", "magnetic encoder fault", "stall overload", "uncalibrated",
)
NORMAL_ENDS = ("stop_temp", "max_minutes", "plateau")
COOLDOWN_AFTER = NORMAL_ENDS + ("fault", "mode", "tracking", "speed", "joint_limit")

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
ANALYZE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "analyze_thermal.py")
ARG_CHECKS = [("kp", "positive"), ("kd", "positive"), ("rate-hz", "positive"), ("max-minutes", "positive"),
              ("cooldown-minutes", "nonneg"), ("stop-temp-c", "positive"), ("max-error-deg", "positive"),
              ("max-vel", "speed"), ("feedback-timeout", "positive")]
COLUMNS = ["t_s", "phase", "temp_c", "pos_rad", "vel_rad_s", "torque_nm", "iq_a", "fault_flags", "mode",
           "cmd_pos_rad", "tau_ff_nm", "ambient_c"]


def parse_args(argv=None):
    p = argparse.ArgumentParser(description="Hold one motor under load and log its temperature with stop rules.")
    p.add_argument("--motor-id", type=lambda v: int(v, 0), required=True)
    p.add_argument("--model", choices=list(SPECS.keys()), default=None)
    p.add_argument("--kp", type=finite_float, required=True, help="Position gain of the hold, at most the model's kp_max")
    p.add_argument("--kd", type=finite_float, required=True, help="Velocity gain of the hold, at most the model's kd_max")
    p.add_argument("--tau-ff", type=finite_float, default=0.0,
                   help="Feed-forward torque in N*m, ramped in over 2 s; |value| at most the model's rated torque")
    p.add_argument("--target-deg", type=finite_float, default=None,
                   help="Hold this position (type-0x02 frame, deg) instead of the current one; reached at 0.3 rad/s")
    p.add_argument("--ambient-c", type=finite_float, required=True, help="Room temperature from a thermometer, deg C")
    p.add_argument("--stop-temp-c", type=finite_float, default=60.0,
                   help=f"Stop the hold at this motor temperature, at most {MAX_STOP_TEMP_C:g}")
    p.add_argument("--max-minutes", type=finite_float, default=45.0,
                   help=f"Longest hold in minutes, at most {MAX_MINUTES:g}")
    p.add_argument("--plateau-stop", action="store_true",
                   help="Also stop when the temperature rose less than 1 deg C over the last 10 min")
    p.add_argument("--cooldown-minutes", type=finite_float, default=30.0,
                   help="Keep logging this long with the motor disabled after the hold; 0 skips it")
    p.add_argument("--rate-hz", type=finite_float, default=2.0, help=f"Log rate, at most {MAX_RATE_HZ:g}")
    p.add_argument("--max-error-deg", type=finite_float, default=10.0,
                   help=f"Stop when |command - position| exceeds this, at most {MAX_TRACKING_DEG:g}")
    p.add_argument("--max-vel", type=finite_float, default=1.0, help="Stop above this speed in rad/s")
    p.add_argument("--ignore-joint-limit", action="store_true",
                   help="Disable joint-limit checks for bench tests only (not with --target-deg)")
    add_can_timeout_option(p)
    p.set_defaults(interface=DEFAULT_INTERFACE, host_id=HOST_ID, feedback_timeout=0.3,
                   limit_margin=DEFAULT_LIMIT_MARGIN_RAD, out=None)
    return resolve_model(p, p.parse_args(argv))


def argument_problems(args):
    spec = SPECS[args.model]
    problems = validate_args(args, args.model, ARG_CHECKS)
    problem = can_timeout_problem(args)
    if problem:
        problems.append(problem)
    if args.kp > spec.kp_max:
        problems.append(f"--kp {args.kp} exceeds the {args.model.upper()} kp_max {spec.kp_max}")
    if args.kd > spec.kd_max:
        problems.append(f"--kd {args.kd} exceeds the {args.model.upper()} kd_max {spec.kd_max}")
    if abs(args.tau_ff) > RATED_TORQUE[args.model]:
        problems.append(f"--tau-ff |{args.tau_ff}| exceeds the {args.model.upper()} rated torque "
                        f"{RATED_TORQUE[args.model]} N*m")
    if args.stop_temp_c > MAX_STOP_TEMP_C:
        problems.append(f"--stop-temp-c {args.stop_temp_c} is above {MAX_STOP_TEMP_C:g}")
    if args.stop_temp_c <= args.ambient_c:
        problems.append(f"--stop-temp-c {args.stop_temp_c} is not above the ambient {args.ambient_c}")
    if args.max_minutes > MAX_MINUTES:
        problems.append(f"--max-minutes {args.max_minutes} is above {MAX_MINUTES:g}")
    if args.cooldown_minutes > MAX_MINUTES:
        problems.append(f"--cooldown-minutes {args.cooldown_minutes} is above {MAX_MINUTES:g}")
    if args.rate_hz > MAX_RATE_HZ:
        problems.append(f"--rate-hz {args.rate_hz} is above {MAX_RATE_HZ:g}")
    if args.max_vel <= 0:
        problems.append(f"--max-vel must be positive ({args.max_vel})")
    if args.max_error_deg > MAX_TRACKING_DEG:
        problems.append(f"--max-error-deg {args.max_error_deg} is above {MAX_TRACKING_DEG:g}")
    if args.target_deg is not None:
        target = math.radians(args.target_deg)
        if args.ignore_joint_limit:
            problems.append("--ignore-joint-limit is allowed only without --target-deg (hold the current position)")
        elif exceeds_joint_limit(target, args.motor_id, args.limit_margin):
            problems.append(f"--target-deg {args.target_deg} is outside the command limits of ID {args.motor_id}")
        if abs(target) > math.pi:
            problems.append(f"--target-deg {args.target_deg} is outside -180..+180 deg")
    return problems


def confirm(args):
    spec = SPECS[args.model]
    print(f"Motor: ID {args.motor_id}, {args.model.upper()}, {args.channel}")
    target = "current position" if args.target_deg is None else f"{args.target_deg:+.2f} deg"
    print(f"Hold: {target}, kp {args.kp}, kd {args.kd}, tau_ff {args.tau_ff:+.2f} N*m "
          f"(rated {RATED_TORQUE[args.model]}, kp_max {spec.kp_max}, kd_max {spec.kd_max})")
    print(f"Stops: temp >= {args.stop_temp_c} C, {args.max_minutes} min"
          + (", plateau < 1 C / 10 min" if args.plateau_stop else "")
          + f", any fault flag, feedback > {args.feedback_timeout} s, error > {args.max_error_deg} deg, "
            f"speed > {args.max_vel} rad/s, Ctrl-C")
    print(f"Cool-down: {args.cooldown_minutes} min logging with the motor disabled; ambient {args.ambient_c} C")
    if args.ignore_joint_limit:
        print("WARNING: Joint-limit checks are disabled.")
    print("WARNING: Support the load so a stop cannot drop it, keep clear of the arm, and prepare the E-stop.")
    if not sys.stdin.isatty():
        print("ERROR: Run this command in an interactive terminal.")
        return False
    try:
        input("Press Enter to continue or Ctrl-C to cancel: ")
    except (KeyboardInterrupt, EOFError):
        print("\nCancelled.")
        return False
    return True


def open_motor(args):
    bus = can.interface.Bus(channel=args.channel, interface=args.interface)
    return bus, Motor(bus, args.motor_id, SPECS[args.model], host_id=args.host_id)


def feedback_fault_names(flags):
    return [name for bit, name in enumerate(FEEDBACK_FAULT_NAMES) if flags & (1 << bit)]


class CsvLog:
    def __init__(self, path, args):
        self.path = path
        self.rows = 0
        self.fh = open(path, "x")
        self.fh.write(f"# motor_id,{args.motor_id},model,{args.model},kp,{args.kp},kd,{args.kd},tau_ff_nm,"
                      f"{args.tau_ff},ambient_c,{args.ambient_c},stop_temp_c,{args.stop_temp_c},"
                      f"start,{datetime.now().isoformat(timespec='seconds')}\n")
        self.fh.write(",".join(COLUMNS) + "\n")
        self.fh.flush()

    def write(self, t_s, phase, motor, iq, cmd, tau_ff, ambient):
        pos = "" if motor.last_position is None else f"{motor.last_position:.6f}"
        iq_text = "" if iq is None else f"{iq:.4f}"
        cmd_text = "" if cmd is None else f"{cmd:.6f}"
        self.fh.write(f"{t_s:.3f},{phase},{motor.last_temp:.1f},{pos},{motor.last_velocity:.5f},"
                      f"{motor.last_torque:.4f},{iq_text},{motor.last_fault:02X},{motor.last_mode_status},"
                      f"{cmd_text},{tau_ff:.4f},{ambient:.1f}\n")
        self.fh.flush()
        self.rows += 1

    def close(self):
        self.fh.close()


def hold_problem(motor, cmd, args):
    if motor.last_fault:
        names = ", ".join(feedback_fault_names(motor.last_fault))
        return "fault", f"fault flags 0x{motor.last_fault:02X} ({names})"
    if motor.last_mode_status != MODE_RUNNING:
        return "mode", f"motor left run mode (mode {motor.last_mode_status})"
    if motor.last_temp >= args.stop_temp_c:
        return "stop_temp", f"temperature {motor.last_temp:.1f} C reached the stop {args.stop_temp_c} C"
    if abs(motor.last_velocity) > args.max_vel:
        return "speed", f"speed {motor.last_velocity:+.2f} rad/s above {args.max_vel}"
    error = cmd - motor.last_position
    if abs(error) > math.radians(args.max_error_deg):
        return "tracking", f"tracking error {math.degrees(error):+.2f} deg above {args.max_error_deg}"
    if not args.ignore_joint_limit and exceeds_stop_limit(motor.last_position, args.motor_id, args.limit_margin):
        return "joint_limit", f"position {math.degrees(motor.last_position):+.2f} deg at the stop limit"
    return None


def plateau_reached(times, temps, now):
    if not times or now - times[0] < PLATEAU_WINDOW_S:
        return False
    i = bisect.bisect_right(times, now - PLATEAU_WINDOW_S) - 1
    return temps[-1] - temps[i] < PLATEAU_RISE_C


def start_hold(motor, args):
    motor.stop()
    time.sleep(0.02)
    drain(motor.bus)
    fault_reg = motor.read_param(FAULT_STA_INDEX, fmt="<I", timeout=0.3)
    if fault_reg is None:
        print("Fault word 0x3022: unread")
    else:
        bits = decode_fault_bits(fault_reg)
        print(f"Fault word 0x3022: 0x{fault_reg:08X}" + (f" ({', '.join(bits)})" if bits else " (OK)"))
    if not arm_can_watchdog(motor, args.can_timeout_ms):
        return None, ("watchdog", "CAN watchdog not armed")
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
        return None, ("no_feedback", "no feedback from the motor after enable")
    start = fb[1]
    if outside_position_domain(start) or (args.target_deg is not None and abs(start) > math.pi):
        return None, ("start", f"start position {math.degrees(start):+.2f} deg is outside the usable range")
    if not args.ignore_joint_limit and exceeds_stop_limit(start, args.motor_id, args.limit_margin):
        return None, ("start", f"start position {math.degrees(start):+.2f} deg is at the stop limit")
    if fb[4] >= args.stop_temp_c:
        return None, ("start", f"motor already at {fb[4]:.1f} C (stop {args.stop_temp_c} C)")
    if abs(fb[4] - args.ambient_c) > 1.0:
        print(f"WARNING: motor {fb[4]:.1f} C differs from the ambient {args.ambient_c} C by more than 1 C.")
    deadline = time.monotonic() + 0.3
    while time.monotonic() < deadline:
        motor.control(pos=start, vel=0.0, kp=args.kp, kd=args.kd, torque=0.0)
        if motor.poll_feedback(timeout=0.02) is not None and motor.last_mode_status == MODE_RUNNING:
            break
    if motor.last_mode_status != MODE_RUNNING:
        return None, ("start", f"motor is not running after enable (mode {motor.last_mode_status})")
    return start, None


def hold_phase(motor, args, log):
    start, problem = start_hold(motor, args)
    if problem:
        return problem
    target = start if args.target_deg is None else math.radians(args.target_deg)
    ramp_s = max(MIN_RAMP_S, abs(target - start) / RAMP_SPEED)
    period = 1.0 / CONTROL_HZ
    log_period = 1.0 / args.rate_hz
    print(f"Start {math.degrees(start):+.2f} deg, target {math.degrees(target):+.2f} deg, ramp {ramp_s:.1f} s")
    t0 = time.monotonic()
    last_ok = t0
    next_log = t0
    times, temps = [], []
    while True:
        tick = time.monotonic()
        elapsed = tick - t0
        progress = min(1.0, elapsed / ramp_s)
        cmd = start + (target - start) * progress * progress * (3.0 - 2.0 * progress)
        tau_ff = args.tau_ff * min(1.0, elapsed / TAU_RAMP_S)
        motor.control(pos=cmd, vel=0.0, kp=args.kp, kd=args.kd, torque=tau_ff)
        fb = motor.poll_feedback(timeout=period)
        now = time.monotonic()
        if fb is None:
            if now - last_ok > args.feedback_timeout:
                print()
                return "feedback_timeout", f"no feedback for {now - last_ok:.2f} s (> {args.feedback_timeout} s)"
        else:
            last_ok = now
            problem = hold_problem(motor, cmd, args)
            if problem:
                log.write(now - t0, "hold", motor, None, cmd, tau_ff, args.ambient_c)
                print()
                return problem
        if now >= next_log:
            iq = motor.read_param(IQF_INDEX, fmt="<f", timeout=IQ_READ_TIMEOUT)
            log.write(now - t0, "hold", motor, iq, cmd, tau_ff, args.ambient_c)
            times.append(now - t0)
            temps.append(motor.last_temp)
            next_log = max(next_log + log_period, now)
            iq_text = "  -  " if iq is None else f"{iq:+5.2f}"
            print(f"\r  {elapsed / 60:5.1f} min  T={motor.last_temp:5.1f} C  "
                  f"err={math.degrees(cmd - motor.last_position):+6.2f} deg  tq={motor.last_torque:+6.2f} N*m  "
                  f"iq={iq_text} A    ", end="", flush=True)
            if args.plateau_stop and plateau_reached(times, temps, now - t0):
                print()
                return "plateau", f"temperature rose {temps[-1] - temps[0]:.1f} C in total, < 1 C over 10 min"
        if elapsed >= args.max_minutes * 60.0:
            print()
            return "max_minutes", f"{args.max_minutes} min reached"
        wait = period - (time.monotonic() - tick)
        if wait > 0:
            time.sleep(wait)


def stop_hold(motor):
    try:
        active_brake(motor)
    except KeyboardInterrupt:
        pass
    previous = signal.signal(signal.SIGINT, signal.SIG_IGN)
    try:
        motor.stop()
    finally:
        signal.signal(signal.SIGINT, previous)


def cooldown_phase(motor, args, log, t_offset):
    print(f"Cool-down: logging {args.cooldown_minutes} min with the motor disabled (Ctrl-C ends it).")
    period = 1.0 / args.rate_hz
    t0 = time.monotonic()
    last_ok = t0
    while True:
        tick = time.monotonic()
        if tick - t0 >= args.cooldown_minutes * 60.0:
            return "done"
        motor.stop()
        fb = motor.poll_feedback(timeout=min(0.2, period))
        now = time.monotonic()
        if fb is None:
            if now - last_ok > COOLDOWN_FEEDBACK_TIMEOUT:
                print(f"\nCool-down: no feedback for {COOLDOWN_FEEDBACK_TIMEOUT:g} s; ending it.")
                return "feedback_timeout"
        else:
            last_ok = now
            log.write(t_offset + now - t0, "cooldown", motor, None, None, 0.0, args.ambient_c)
            print(f"\r  cool-down {(now - t0) / 60:5.1f} min  T={motor.last_temp:5.1f} C  mode {motor.last_mode_status}"
                  "    ", end="", flush=True)
        wait = period - (time.monotonic() - tick)
        if wait > 0:
            time.sleep(wait)


def main(argv=None):
    args = parse_args(argv)
    install_stop_signals()
    if report_invalid_args(argument_problems(args)):
        return 1
    args.channel = channel_for_id(args.motor_id)
    if not confirm(args):
        return 1
    os.makedirs(DATA_DIR, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%dT%H%M%S")
    out = args.out or os.path.join(DATA_DIR, f"id{args.motor_id}_{args.model}_{stamp}.csv")
    if os.path.exists(out):
        print(f"ERROR: {out} already exists.")
        return 1
    bus, motor = open_motor(args)
    log = None
    stopped = False
    reason = ("setup", "not started")
    try:
        log = CsvLog(out, args)
        t_hold = time.monotonic()
        try:
            reason = hold_phase(motor, args, log)
        except KeyboardInterrupt:
            print()
            reason = ("interrupted", "Ctrl-C or stop signal")
        print(f"Hold ended: {reason[1]}")
        stop_hold(motor)
        stopped = True
        if reason[0] in COOLDOWN_AFTER and args.cooldown_minutes > 0:
            try:
                cooldown_phase(motor, args, log, time.monotonic() - t_hold)
            except KeyboardInterrupt:
                print("\nCool-down ended early.")
            print()
    finally:
        shutdown_motor(motor, bus, () if stopped else (active_brake,))
        if log is not None:
            log.close()
    if log is not None and log.rows:
        print(f"Saved: {out} ({log.rows} rows)")
        print(f"Analyze: python3 {ANALYZE} {out}")
    return 0 if reason[0] in NORMAL_ENDS else 1


if __name__ == "__main__":
    sys.exit(main())
