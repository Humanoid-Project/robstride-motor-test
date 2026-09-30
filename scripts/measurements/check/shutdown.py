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
    motors_by_channel,
    open_bus,
    resolve_motors,
    variant_motors,
    variant_summary,
)
from robonex_common.can import Motor, drain
from robonex_common.joints import MOTOR_BY_ID as JOINT_BY_ID
from robonex_common.motors import MOTOR_SPECS
from robonex_common.protocol import DEFAULT_INTERFACE, HOST_ID

MODE_NAMES = {0: "Reset", 1: "Calibration", 2: "Motor active"}




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


def main():
    args = parse_args()
    print(variant_summary(args.variant, variant_motors(args.variant)))
    target_ids = [joint.motor_id for joint in args.joints]
    buses = {}
    try:
        for channel in motors_by_channel(args.joints):
            try:
                buses[channel] = open_bus(channel, args.interface)
            except BusOpenError as error:
                print(f"Skipping {error}")
        motors = {}
        for motor_id in target_ids:
            joint = JOINT_BY_ID.get(motor_id)
            if joint is None or joint.channel not in buses:
                continue
            motors[motor_id] = Motor(
                buses[joint.channel],
                motor_id,
                MOTOR_SPECS[joint.motor_model],
                args.host_id,
            )
        if not motors:
            print("No target motors.")
            return 1
        if args.brake_time > 0.0:
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
        for bus in buses.values():
            drain(bus)
        print("\nDisabling...")
        print(f"{'ID':>3}  {'ch':<5}  {'joint':<18}  {'speed':>10}  result")
        print("-" * 60)
        ok_count = 0
        silent_count = 0
        running_count = 0
        for motor_id, motor in motors.items():
            joint = JOINT_BY_ID[motor_id]
            try:
                motor.stop()
            except can.CanError as error:
                print(
                    f"{motor_id:>3}  {joint.channel:<5}  {joint.hardware_name:<18}  "
                    f"{'-':>10}  send failed ({error})"
                )
                continue
            feedback = motor.poll_feedback(timeout=0.2)
            if feedback is None:
                silent_count += 1
                print(
                    f"{motor_id:>3}  {joint.channel:<5}  {joint.hardware_name:<18}  "
                    f"{'-':>10}  no response"
                )
                continue
            mode = MODE_NAMES.get(motor.last_mode_status, f"?({motor.last_mode_status})")
            success = motor.last_mode_status == 0
            if success:
                ok_count += 1
            else:
                running_count += 1
            result = "OK" if success else f"WARNING: {mode}; retry required"
            print(
                f"{motor_id:>3}  {joint.channel:<5}  {joint.hardware_name:<18}  "
                f"{motor.last_velocity:>+9.3f}  {result}"
            )
        print(
            f"\n{ok_count}/{len(motors)} motors confirmed disabled, "
            f"silent {silent_count}, active {running_count}."
        )
        if running_count:
            print("Some motors are still active. Run this tool again before disconnecting power.")
            return 1
        print("Disable check passed. Silent motors may already be off or disconnected.")
        return 0
    except KeyboardInterrupt:
        print("\nCancelled. Stop commands already sent remain effective.")
        return 0
    finally:
        for bus in buses.values():
            bus.shutdown()


if __name__ == "__main__":
    sys.exit(main())
