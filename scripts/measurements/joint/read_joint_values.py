#!/usr/bin/env python3
import argparse
import csv
import math
import os
import struct
import sys
import threading
import time

import can

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from motor_selection import (
    SELECTOR_HELP,
    BusOpenError,
    SelectionError,
    attached_variant,
    bus_map_summary,
    channel_groups,
    has_placeholder_limits,
    motors_by_channel,
    open_bus,
    resolve_motors,
    variant_motors,
    variant_summary,
)
from robonex_common.protocol import (
    COMM_PARAMETER_READ,
    DEFAULT_INTERFACE,
    HOST_ID,
    MECHANICAL_POSITION_INDEX,
    build_arbitration_id,
    parse_arbitration_id,
)

ONESHOT_TIMEOUT = 0.1
WATCH_TIMEOUT = 0.02
CSV_FIELDS = ("time_s", "unix_time", "channel", "motor_id", "joint", "position_rad")
CLEAR_SCREEN = "\033[2J\033[3J\033[H"


def read_mech_position(bus, host_id, motor_id, timeout=0.1):
    data = bytearray(8)
    struct.pack_into("<H", data, 0, MECHANICAL_POSITION_INDEX)
    bus.send(can.Message(
        arbitration_id=build_arbitration_id(COMM_PARAMETER_READ, host_id, motor_id),
        data=bytes(data), is_extended_id=True))

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=max(0.0, deadline - time.monotonic()))
        if msg is None or not msg.is_extended_id:
            continue
        comm_type, data16, destination = parse_arbitration_id(msg.arbitration_id)
        if comm_type != COMM_PARAMETER_READ or destination != host_id or (data16 & 0xFF) != motor_id:
            continue
        payload = bytes(msg.data)
        if len(payload) < 8:
            continue
        if int.from_bytes(payload[0:2], "little") != MECHANICAL_POSITION_INDEX:
            continue
        return struct.unpack_from("<f", payload, 4)[0]
    return None


class CsvLog:
    def __init__(self, path):
        self.handle = open(path, "w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.handle)
        self.writer.writerow(CSV_FIELDS)
        self.start = time.monotonic()
        self.lock = threading.Lock()
        self.rows = 0

    def write(self, channel, joint, position):
        now = time.monotonic()
        value = "" if position is None else f"{position:.6f}"
        with self.lock:
            self.writer.writerow((f"{now - self.start:.6f}", f"{time.time():.6f}", channel,
                                  joint.motor_id, joint.hardware_name, value))
            self.rows += 1

    def close(self):
        with self.lock:
            self.handle.close()


def format_position(position):
    if position is None:
        return "-"
    return f"{position:+8.4f} rad ({math.degrees(position):+8.2f} deg)"


def position_note(joint, position):
    if position is None:
        return "no response"
    if abs(position) > math.pi:
        return "outside +-180 deg (set zero_sta=1)"
    if not joint.lower <= position <= joint.upper:
        kind = "PLACEHOLDER limit" if has_placeholder_limits(joint) else "limit"
        return (f"beyond {kind} [{math.degrees(joint.lower):+.0f}, "
                f"{math.degrees(joint.upper):+.0f}] deg")
    return ""


def channel_lines(channel, joints, positions, error=None, rate=None):
    title = f"[{channel}] {', '.join(channel_groups(channel)) or 'no mapped group'}"
    if rate is not None:
        title += f"   {rate:6.1f} Hz"
    lines = [title]
    if error is not None:
        lines.append(f"  open failed: {error.reason}")
        lines.append(f"  -> {error.hint}")
    lines.append(f"  {'ID':>3}  {'joint':<20}  {'group':<9}  {'model':<5}  {'position':>28}  note")
    lines.append("  " + "-" * 96)
    for joint in joints:
        if error is not None:
            position, note = None, "channel not open"
        else:
            position = positions.get(joint.motor_id)
            note = position_note(joint, position)
        lines.append(f"  {joint.motor_id:>3}  {joint.hardware_name:<20}  {joint.group:<9}  "
                     f"{joint.motor_model:<5}  {format_position(position):>28}  {note}".rstrip())
    return lines


def read_once(channels, interface, host_id, timeout, log=None):
    missing = 0
    for channel, joints in channels.items():
        positions = {}
        error = None
        try:
            bus = open_bus(channel, interface)
        except BusOpenError as exc:
            error = exc
        else:
            try:
                for joint in joints:
                    positions[joint.motor_id] = read_mech_position(bus, host_id, joint.motor_id, timeout)
                    if log is not None:
                        log.write(channel, joint, positions[joint.motor_id])
            finally:
                bus.shutdown()
        missing += sum(1 for joint in joints if positions.get(joint.motor_id) is None)
        print("\n".join(channel_lines(channel, joints, positions, error)))
        print()
    return missing


def poll_worker(channel, joints, interface, host_id, timeout, shared, stop, log):
    try:
        bus = open_bus(channel, interface)
    except BusOpenError as exc:
        with shared["lock"]:
            shared["errors"][channel] = exc
        return

    t0, count = time.monotonic(), 0
    try:
        while not stop.is_set():
            for joint in joints:
                position = read_mech_position(bus, host_id, joint.motor_id, timeout=timeout)
                if log is not None:
                    log.write(channel, joint, position)
                with shared["lock"]:
                    shared["positions"][joint.motor_id] = position
            count += 1
            now = time.monotonic()
            if now - t0 >= 0.5:
                with shared["lock"]:
                    shared["rates"][channel] = count / (now - t0)
                t0, count = now, 0
    except can.CanError as exc:
        with shared["lock"]:
            shared["notes"].append(f"[{channel}] CAN error: {exc}")
    finally:
        bus.shutdown()


def watch(channels, interface, host_id, timeout, hz, duration, header, log=None, out=sys.stdout):
    shared = {"lock": threading.Lock(), "positions": {}, "rates": {}, "errors": {}, "notes": []}
    stop = threading.Event()
    threads = [
        threading.Thread(target=poll_worker,
                         args=(channel, joints, interface, host_id, timeout, shared, stop, log),
                         daemon=True)
        for channel, joints in channels.items()
    ]
    for thread in threads:
        thread.start()

    start = time.monotonic()
    try:
        while True:
            with shared["lock"]:
                positions = dict(shared["positions"])
                rates = dict(shared["rates"])
                errors = dict(shared["errors"])
                notes = list(shared["notes"])
            lines = [CLEAR_SCREEN + f"joint monitor   {time.strftime('%H:%M:%S')}   (Ctrl-C to quit)"]
            lines.extend(header)
            lines.append("")
            for channel, joints in channels.items():
                lines.extend(channel_lines(channel, joints, positions, errors.get(channel),
                                           rates.get(channel, 0.0)))
                lines.append("")
            lines.extend(notes)
            out.write("\n".join(lines) + "\n")
            out.flush()
            if duration and time.monotonic() - start >= duration:
                return
            time.sleep(1.0 / hz)
    finally:
        stop.set()
        for thread in threads:
            thread.join(timeout=2.0)


def select_channels(args, variant):
    joints = resolve_motors(args.ids, variant)
    channels = motors_by_channel(joints)
    if args.can is not None:
        if args.can not in channels:
            raise SelectionError(
                f"no selected motor is on {args.can} (selected motors use {', '.join(channels)})"
            )
        channels = {args.can: channels[args.can]}
    return channels


def positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise argparse.ArgumentTypeError("must be a positive number")
    return number


def non_negative(value):
    number = float(value)
    if not math.isfinite(number) or number < 0.0:
        raise argparse.ArgumentTypeError("must be zero or a positive number")
    return number


def build_parser():
    parser = argparse.ArgumentParser(
        description="Print the mechanical joint angle of RoboNex motors (read-only; never enables a motor).")
    parser.add_argument("--ids", nargs="+", default=None, metavar="SEL",
                        help=f"Motors to read: {SELECTOR_HELP} (default: every motor of the attached robot)")
    parser.add_argument("--can", default=None, metavar="CH",
                        help="Only read the selected motors on this CAN channel, e.g. can1")
    parser.add_argument("--watch", action="store_true", help="Keep refreshing instead of printing once")
    parser.add_argument("--hz", type=positive, default=10.0,
                        help="Screen refresh rate in --watch mode (default 10); motors are polled as fast as the bus allows")
    parser.add_argument("--duration", type=non_negative, default=0.0,
                        help="Stop --watch after this many seconds (default 0 = until Ctrl-C)")
    parser.add_argument("--csv", default=None, metavar="PATH",
                        help="Write every reading to a new CSV file, one timestamped row each: " + ", ".join(CSV_FIELDS))
    parser.set_defaults(interface=DEFAULT_INTERFACE, host_id=HOST_ID, timeout=None)
    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    timeout = args.timeout
    if timeout is None:
        timeout = WATCH_TIMEOUT if args.watch else ONESHOT_TIMEOUT
    try:
        variant = attached_variant()
        channels = select_channels(args, variant)
    except SelectionError as exc:
        parser.error(str(exc))

    header = [variant_summary(variant, variant_motors(variant)), bus_map_summary()]
    log = CsvLog(args.csv) if args.csv else None
    try:
        if args.watch:
            watch(channels, args.interface, args.host_id, timeout, args.hz, args.duration, header, log)
            return 0
        print("\n".join(header) + "\n")
        missing = read_once(channels, args.interface, args.host_id, timeout, log)
        total = sum(len(joints) for joints in channels.values())
        print(f"{total - missing}/{total} motors answered.")
        return 1 if missing else 0
    except KeyboardInterrupt:
        return 0
    finally:
        if log is not None:
            log.close()
            print(f"CSV: {log.rows} rows -> {args.csv}")


if __name__ == "__main__":
    sys.exit(main())
