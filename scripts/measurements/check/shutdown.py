#!/usr/bin/env python3
import argparse
import os
import sys
import time

import can

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from motor_selection import (
    SELECTOR_HELP,
    BusOpenError,
    SelectionError,
    attached_variant,
    format_ids,
    motors_by_channel,
    open_bus,
    resolve_motors,
    variant_motors,
    variant_summary,
)
from robonex_common.can import Motor, drain
from robonex_common.motors import MOTOR_SPECS
from robonex_common.protocol import DEFAULT_INTERFACE, HOST_ID

MODE_NAMES = {0: "Reset", 1: "Calibration", 2: "Motor active"}
STATUSES = ("stopped", "active", "unconfirmed", "send_failed", "channel_unavailable", "interrupted")




def parse_args():
    parser = argparse.ArgumentParser(
        description="Brake and disable the selected motors before disconnecting power."
    )
    parser.add_argument("--ids", nargs="+", default=None, metavar="SEL",
                        help=f"Motors to stop: {SELECTOR_HELP} (default: every motor of the attached robot)")
    parser.add_argument("--brake-time", type=float, default=0.3)
    parser.add_argument("--kd", type=float, default=3.0)
    parser.set_defaults(interface=DEFAULT_INTERFACE, host_id=HOST_ID)
    args = parser.parse_args()
    try:
        args.variant = attached_variant()
        args.joints = resolve_motors(args.ids, args.variant)
    except SelectionError as error:
        parser.error(str(error))
    return args


def motor_for(joint, buses, motors, host_id):
    motor = motors.get(joint.motor_id)
    if motor is None and joint.channel in buses:
        motor = Motor(buses[joint.channel], joint.motor_id, MOTOR_SPECS[joint.motor_model], host_id)
        motors[joint.motor_id] = motor
    return motor


def brake(args, buses, motors):
    print(f"Braking... ({args.brake_time:.2f} s, kd={args.kd})")
    deadline = time.monotonic() + args.brake_time
    while time.monotonic() < deadline:
        for motor in motors.values():
            try:
                motor.control(
                    pos=0.0,
                    vel=0.0,
                    kp=0.0,
                    kd=min(args.kd, motor.spec.kd_max),
                    torque=0.0,
                )
            except can.CanError:
                pass
        for bus in buses.values():
            drain(bus)
        time.sleep(0.01)


def disable_all(args, results, buses, motors):
    for channel, joints in motors_by_channel(args.joints).items():
        try:
            buses[channel] = open_bus(channel, args.interface)
        except BusOpenError as error:
            print(f"Skipping {error}")
            for joint in joints:
                results[joint.motor_id] = ("channel_unavailable", None, "bus not open")
    for joint in args.joints:
        motor_for(joint, buses, motors, args.host_id)
    if not motors:
        print("No target motors.")
        return
    if args.brake_time > 0.0:
        brake(args, buses, motors)
    for bus in buses.values():
        drain(bus)
    print("\nDisabling...")
    for motor_id, motor in motors.items():
        try:
            motor.stop()
        except can.CanError as error:
            results[motor_id] = ("send_failed", None, str(error))
            continue
        feedback = motor.poll_feedback(timeout=0.2)
        if feedback is None:
            results[motor_id] = ("unconfirmed", None, "stop sent, no response")
            continue
        if motor.last_mode_status == 0:
            results[motor_id] = ("stopped", motor.last_velocity, "OK")
        else:
            mode = MODE_NAMES.get(motor.last_mode_status, f"?({motor.last_mode_status})")
            results[motor_id] = ("active", motor.last_velocity, f"{mode}; retry required")


def stop_remaining(args, results, buses, motors):
    for joint in args.joints:
        if results.get(joint.motor_id) is not None:
            continue
        motor = motor_for(joint, buses, motors, args.host_id)
        if motor is None:
            results[joint.motor_id] = ("interrupted", None, "bus not open, no stop sent")
            continue
        try:
            motor.stop()
            detail = "stop sent after interrupt, not confirmed"
        except can.CanError as error:
            detail = f"stop send failed after interrupt ({error})"
        results[joint.motor_id] = ("interrupted", None, detail)


def report(joints, results):
    print(f"\n{'ID':>3}  {'joint':<18}  {'speed':>10}  {'status':<19}  detail")
    for channel, channel_joints in motors_by_channel(joints).items():
        print(f"[{channel}]")
        for joint in channel_joints:
            status, speed, detail = results[joint.motor_id]
            speed_text = "-" if speed is None else f"{speed:+.3f}"
            print(f"{joint.motor_id:>3}  {joint.hardware_name:<18}  {speed_text:>10}  {status:<19}  {detail}")
    counts = {status: 0 for status in STATUSES}
    for status, _speed, _detail in results.values():
        counts[status] += 1
    print(
        f"\n{counts['stopped']}/{len(joints)} motors confirmed stopped; "
        + ", ".join(f"{status} {counts[status]}" for status in STATUSES[1:])
        + "."
    )
    failed = [joint.motor_id for joint in joints if results[joint.motor_id][0] != "stopped"]
    if failed:
        print(f"Not confirmed stopped: IDs {format_ids(failed)}. Do not disconnect power until they are checked.")
        return 1
    print("Disable check passed.")
    return 0


def main():
    args = parse_args()
    print(variant_summary(args.variant, variant_motors(args.variant)))
    results = {joint.motor_id: None for joint in args.joints}
    buses = {}
    motors = {}
    try:
        disable_all(args, results, buses, motors)
    except KeyboardInterrupt:
        print("\nInterrupted. Sending stop to every remaining motor.")
    finally:
        try:
            stop_remaining(args, results, buses, motors)
        finally:
            for bus in buses.values():
                bus.shutdown()
    return report(args.joints, results)


if __name__ == "__main__":
    sys.exit(main())
