import argparse
import math
import signal
import struct
import time

from robonex_common.can import FeedbackHub, Motor
from robonex_common.joints import ALL_MOTORS, MOTOR_BY_ID, MOTOR_LIMITS_BY_ID
from robonex_common.joints import channel_for_motor_id as channel_for_id
from robonex_common.limits import DEFAULT_LIMIT_MARGIN_RAD, exceeds_joint_limit, joint_limit_for
from robonex_common.motors import MOTOR_PHYSICS, MOTOR_SPECS, PEAK_TORQUE, RATED_TORQUE
from robonex_common.protocol import (
    COMM_PARAMETER_WRITE,
    DEFAULT_INTERFACE,
    FAULT_BIT_NAMES,
    FAULT_STATUS_INDEX,
    HOST_ID,
    MECHANICAL_POSITION_INDEX,
    RUN_MODE_INDEX,
    RUN_MODE_OPERATION,
    decode_fault_bits,
)

SPECS = MOTOR_SPECS
JOINT_MAP = {joint.motor_id: joint.hardware_name for joint in ALL_MOTORS}
JOINT_LIMITS_RAD = MOTOR_LIMITS_BY_ID
MOTOR_MODEL = {joint.motor_id: joint.motor_model for joint in ALL_MOTORS}
MECH_POS_INDEX = MECHANICAL_POSITION_INDEX
FAULT_STA_INDEX = FAULT_STATUS_INDEX
PLACEHOLDER_ARMATURE = {model: values["armature"] for model, values in MOTOR_PHYSICS.items()}
PLACEHOLDER_DAMPING = {"rs02": 0.2, "rs03": 0.2}
CAN_TIMEOUT_INDEX = 0x7028
CAN_TIMEOUT_COUNTS_PER_MS = 20
DEFAULT_CAN_TIMEOUT_MS = 1000
MAX_CAN_TIMEOUT_MS = 5000
POSITION_DOMAIN_RAD = 4.0 * math.pi - 0.5


def add_can_timeout_option(parser):
    parser.add_argument("--can-timeout-ms", type=int, default=DEFAULT_CAN_TIMEOUT_MS,
                        help=f"Motor CAN watchdog for this run (0x7028, not saved; the motor stops when no frame "
                             f"arrives for this long); 0 disables, at most {MAX_CAN_TIMEOUT_MS}")


def can_timeout_problem(args):
    value = getattr(args, "can_timeout_ms", 0)
    if not 0 <= value <= MAX_CAN_TIMEOUT_MS:
        return f"--can-timeout-ms must be between 0 and {MAX_CAN_TIMEOUT_MS} ({value})"
    return None


def _write_can_timeout(motor, counts):
    data = bytearray(8)
    struct.pack_into("<H", data, 0, CAN_TIMEOUT_INDEX)
    struct.pack_into("<I", data, 4, counts)
    motor._send(COMM_PARAMETER_WRITE, motor.host_id, data)


def arm_can_watchdog(motor, timeout_ms):
    if timeout_ms <= 0:
        print("CAN watchdog: off (--can-timeout-ms 0)")
        return True
    counts = int(timeout_ms) * CAN_TIMEOUT_COUNTS_PER_MS
    _write_can_timeout(motor, counts)
    time.sleep(0.01)
    readback = motor.read_param(CAN_TIMEOUT_INDEX, fmt="<I", timeout=0.3)
    if readback != counts:
        print(f"ERROR: CAN watchdog 0x7028 read back {readback} after writing {counts}; nothing moves. "
              "Pass --can-timeout-ms 0 to run without it.")
        return False
    motor.can_watchdog_armed = True
    print(f"CAN watchdog: {timeout_ms} ms (0x7028 = {counts}, not saved)")
    return True


def disarm_can_watchdog(motor):
    target = getattr(motor, "motor", motor)
    if not getattr(target, "can_watchdog_armed", False):
        return
    try:
        _write_can_timeout(target, 0)
        target.can_watchdog_armed = False
    except Exception as error:
        print(f"CAN watchdog reset failed: {error} (it is not saved; a power cycle clears it)")


def install_stop_signals():
    def handler(signum, _frame):
        signal.signal(signal.SIGHUP, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        raise KeyboardInterrupt(f"signal {signum}")

    signal.signal(signal.SIGHUP, handler)
    signal.signal(signal.SIGTERM, handler)


def outside_position_domain(position):
    return abs(position) >= POSITION_DOMAIN_RAD


def return_to_position(motor, target, timeout=2.5, kp=15.0, kd=3.0, pos_tol=0.02, max_speed=0.4):
    fb = None
    for _ in range(3):
        motor.control(pos=0.0, vel=0.0, kp=0.0, kd=kd, torque=0.0)
        fb = motor.poll_feedback(timeout=0.1)
        if fb is not None:
            break
    if fb is None:
        return None
    start_pos = fb[1]
    ramp_time = max(0.5, abs(start_pos - target) / max_speed)
    ramp_start = time.monotonic()
    deadline = ramp_start + max(timeout, ramp_time + 0.5)
    last_pos = start_pos
    while time.monotonic() < deadline:
        progress = min(1.0, (time.monotonic() - ramp_start) / ramp_time)
        smooth = progress * progress * (3.0 - 2.0 * progress)
        goal = start_pos + (target - start_pos) * smooth
        motor.control(pos=goal, vel=0.0, kp=kp, kd=kd, torque=0.0)
        fb = motor.poll_feedback(timeout=0.05)
        if fb is None:
            continue
        _, pos, vel, _, _, _ = fb
        last_pos = pos
        if progress >= 1.0 and abs(pos - target) <= pos_tol and abs(vel) < 0.05:
            break
    return last_pos

def resolve_model(parser, args):
    joint = MOTOR_BY_ID.get(args.motor_id)
    if joint is None:
        if args.model is None:
            parser.error(f"ID {args.motor_id} is not a RoboNex joint; pass --model explicitly")
        return args
    if args.model is None:
        args.model = joint.motor_model
    elif args.model != joint.motor_model:
        parser.error(
            f"--model {args.model} does not match ID {args.motor_id} "
            f"({joint.hardware_name}), which is {joint.motor_model}"
        )
    return args


def shutdown_motor(motor, bus, steps=()):
    try:
        if motor is not None:
            for step in steps:
                try:
                    step(motor)
                except KeyboardInterrupt:
                    print("\nCleanup interrupted; stopping the motor now.")
                    break
                except Exception as error:
                    print(f"\nCleanup step {step.__name__} failed ({error}); stopping the motor now.")
                    break
    except KeyboardInterrupt:
        pass
    finally:
        previous = signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            if motor is not None:
                try:
                    motor.stop()
                except Exception as error:
                    print(f"Stop frame failed: {error}")
                disarm_can_watchdog(motor)
            bus.shutdown()
        finally:
            signal.signal(signal.SIGINT, previous)


def finite_float(text):
    value = float(text)
    if not math.isfinite(value):
        raise argparse.ArgumentTypeError(f"must be finite ({text})")
    return value


def validate_args(args, model, checks):
    spec = SPECS[model]
    problems = []
    for name, kind in checks:
        attr = name.replace("-", "_")
        if not hasattr(args, attr):
            continue
        value = getattr(args, attr)
        if value is None:
            continue
        if not math.isfinite(value):
            problems.append(f"--{name} must be finite ({value})")
            continue
        if kind == "positive" and value <= 0:
            problems.append(f"--{name} must be positive ({value})")
        elif kind == "nonneg" and value < 0:
            problems.append(f"--{name} must be non-negative ({value})")
        elif kind == "torque" and abs(value) > PEAK_TORQUE[model]:
            problems.append(
                f"--{name}: |{value}| exceeds the {model.upper()} peak torque {PEAK_TORQUE[model]} N*m"
            )
        elif kind == "speed" and abs(value) > spec.v_max:
            problems.append(f"--{name}: |{value}| exceeds the {model.upper()} speed limit {spec.v_max} rad/s")
    return problems


def report_invalid_args(problems):
    if not problems:
        return False
    print("Invalid arguments; nothing will run:")
    for problem in problems:
        print(f"  {problem}")
    return True


def active_brake(motor, duration=0.3, kd=3.0):
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        motor.control(pos=0.0, vel=0.0, kp=0.0, kd=kd, torque=0.0)
        motor.poll_feedback(timeout=0.05)
