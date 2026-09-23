import math
import signal
import time

from robonex_common.can import FeedbackHub, Motor
from robonex_common.joints import ACTUATED_JOINTS, JOINT_BY_ID, JOINT_LIMITS_BY_ID
from robonex_common.joints import channel_for_motor_id as channel_for_id
from robonex_common.limits import DEFAULT_LIMIT_MARGIN_RAD, exceeds_joint_limit, joint_limit_for
from robonex_common.motors import MOTOR_SPECS, PEAK_TORQUE, RATED_TORQUE
from robonex_common.protocol import (
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
JOINT_MAP = {joint.motor_id: joint.hardware_name for joint in ACTUATED_JOINTS}
JOINT_LIMITS_RAD = JOINT_LIMITS_BY_ID
MECH_POS_INDEX = MECHANICAL_POSITION_INDEX
FAULT_STA_INDEX = FAULT_STATUS_INDEX
PLACEHOLDER_ARMATURE = {"rs02": 0.003, "rs03": 0.017}
PLACEHOLDER_DAMPING = {"rs02": 0.2, "rs03": 0.2}

def resolve_model(parser, args):
    joint = JOINT_BY_ID.get(args.motor_id)
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
            bus.shutdown()
        finally:
            signal.signal(signal.SIGINT, previous)


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
