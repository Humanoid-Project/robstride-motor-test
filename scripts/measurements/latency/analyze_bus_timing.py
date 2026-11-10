#!/usr/bin/env python3
import argparse
import csv
import glob
import math
import statistics
import sys

DEFAULT_BITRATE = 1_000_000
TAIL_BITS = 1 + 2 + 7 + 3
CRC15_POLY = 0x4599
NUMERIC = ("t_cycle_mono", "t_send_mono", "t_send_wall", "tx_ts", "tx_hw_ts", "t_rx_mono", "rx_ts", "rx_hw_ts")


def number(text):
    if text is None or text == "":
        return None
    return float(text)


def bits_of(value, width):
    return [(value >> (width - 1 - i)) & 1 for i in range(width)]


def crc15(bits):
    crc = 0
    for bit in bits:
        feedback = bit ^ ((crc >> 14) & 1)
        crc = (crc << 1) & 0x7FFF
        if feedback:
            crc ^= CRC15_POLY
    return crc


def stuff_count(bits):
    count = 0
    previous = None
    run = 0
    for bit in bits:
        if bit == previous:
            run += 1
        else:
            previous = bit
            run = 1
        if run == 5:
            count += 1
            previous = 1 - bit
            run = 1
    return count


def frame_bits(arbitration_id, data):
    data = bytes(data)
    head = ([0] + bits_of((arbitration_id >> 18) & 0x7FF, 11) + [1, 1] + bits_of(arbitration_id & 0x3FFFF, 18)
            + [0, 0, 0] + bits_of(len(data), 4))
    for byte in data:
        head += bits_of(byte, 8)
    body = head + bits_of(crc15(head), 15)
    return len(body) + stuff_count(body) + TAIL_BITS


def load(path):
    meta = {}
    rows = []
    header = None
    with open(path, newline="") as fh:
        for record in csv.reader(fh):
            if not record:
                continue
            if record[0].startswith("#"):
                meta[record[0][1:].strip()] = record[1:]
                continue
            if header is None:
                header = record
                continue
            row = dict(zip(header, record))
            for name in NUMERIC:
                row[name] = number(row.get(name))
            row["cycle"] = int(row["cycle"])
            row["slot"] = int(row["slot"])
            row["motor_id"] = int(row["motor_id"])
            row["tx_bits"] = frame_bits(int(row["tx_id"], 16), bytes.fromhex(row["tx_data"]))
            row["rx_bits"] = frame_bits(int(row["rx_id"], 16), bytes.fromhex(row["rx_data"])) if row["rx_id"] else None
            rows.append(row)
    return meta, rows


def pick_clock(rows, wanted):
    complete = [r for r in rows if r["status"] == "ok"]
    has_hw = bool(complete) and all(r["tx_hw_ts"] is not None and r["rx_hw_ts"] is not None for r in complete)
    if wanted == "hw" and not has_hw:
        raise SystemExit("these files have no complete adapter timestamps; use --clock sw")
    if wanted == "sw" or not has_hw:
        return "sw", "tx_ts", "rx_ts"
    return "hw", "tx_hw_ts", "rx_hw_ts"


def percentile(values, q):
    ordered = sorted(values)
    if not ordered:
        return math.nan
    position = (len(ordered) - 1) * q
    low = int(math.floor(position))
    high = min(low + 1, len(ordered) - 1)
    return ordered[low] + (ordered[high] - ordered[low]) * (position - low)


def distribution(values):
    if not values:
        return {"n": 0, "median": math.nan, "p95": math.nan, "max": math.nan}
    return {"n": len(values), "median": statistics.median(values), "p95": percentile(values, 0.95),
            "max": max(values)}


def group_cycles(rows):
    cycles = {}
    for row in rows:
        cycles.setdefault((row["file"], row["cycle"]), []).append(row)
    for members in cycles.values():
        members.sort(key=lambda r: r["slot"])
    return cycles


def summarize(paths, clock="auto", bitrate=DEFAULT_BITRATE):
    if isinstance(paths, str):
        paths = [paths]
    metas = []
    rows = []
    for index, path in enumerate(paths):
        meta, loaded = load(path)
        metas.append(meta)
        for row in loaded:
            row["file"] = index
        rows.extend(loaded)
    if not rows:
        raise SystemExit("no exchanges in these files")
    name, tx_key, rx_key = pick_clock(rows, clock)
    bit_time = 1.0 / bitrate

    motors = {}
    for row in rows:
        entry = motors.setdefault((row["pattern"], row["motor_id"]),
                                  {"trip": [], "turnaround": [], "ok": 0, "no_reply": 0, "no_echo": 0, "n": 0})
        entry["n"] += 1
        if row["status"] != "ok":
            entry[row["status"]] += 1
            continue
        entry["ok"] += 1
        trip = row[rx_key] - row[tx_key]
        entry["trip"].append(trip)
        entry["turnaround"].append(trip - (row["tx_bits"] + row["rx_bits"]) * bit_time)
    per_motor = {}
    for key, entry in motors.items():
        per_motor[key] = {"round_trip": distribution(entry["trip"]), "turnaround": distribution(entry["turnaround"]),
                          "n": entry["n"], "ok": entry["ok"], "no_reply": entry["no_reply"],
                          "no_echo": entry["no_echo"]}

    cycles = group_cycles(rows)
    slots = {}
    cycle_times = {}
    host_times = {}
    busy = {}
    for members in cycles.values():
        pattern = members[0]["pattern"]
        first = members[0]
        complete = all(r["status"] == "ok" for r in members)
        origin = first[tx_key]
        if origin is not None and pattern == "burst":
            for row in members:
                entry = slots.setdefault(row["slot"], {"tx": [], "rx": [], "motors": set()})
                entry["motors"].add(row["motor_id"])
                if row[tx_key] is not None:
                    entry["tx"].append(row[tx_key] - origin)
                if row["status"] == "ok":
                    entry["rx"].append(row[rx_key] - origin)
        if complete:
            span = max(r[rx_key] for r in members) - first[tx_key]
            cycle_times.setdefault(pattern, []).append(span)
            host_times.setdefault(pattern, []).append(max(r["t_rx_mono"] for r in members) - first["t_send_mono"])
            wire = sum(r["tx_bits"] + r["rx_bits"] for r in members) * bit_time
            busy.setdefault(pattern, []).append(wire / span if span > 0 else math.nan)
    per_slot = {slot: {"motors": sorted(entry["motors"]), "tx_offset": distribution(entry["tx"]),
                       "rx_offset": distribution(entry["rx"])} for slot, entry in sorted(slots.items())}
    per_pattern = {pattern: {"cycle": distribution(values), "host_cycle": distribution(host_times[pattern]),
                             "busy": distribution(busy[pattern])} for pattern, values in cycle_times.items()}

    utilisation = []
    for index, meta in enumerate(metas):
        mine = [r for r in rows if r["file"] == index]
        bits = sum(r["tx_bits"] for r in mine) + sum(r["rx_bits"] for r in mine if r["rx_bits"] is not None)
        start = min(r["t_send_mono"] for r in mine)
        end = max(max(r["t_send_mono"] for r in mine),
                  max((r["t_rx_mono"] for r in mine if r["t_rx_mono"] is not None), default=start))
        span = end - start
        utilisation.append({"file": paths[index], "frames": len(mine) + sum(1 for r in mine if r["rx_bits"]),
                            "bits": bits, "span": span,
                            "fraction": bits * bit_time / span if span > 0 else math.nan,
                            "counts": {k: meta[k][0] for k in ("no_reply", "no_echo", "stale_reply", "dup_reply",
                                                                "stray_echo", "other", "error_frames", "late",
                                                                "overrun") if k in meta}})

    comparison = None
    if "burst" in per_pattern and "sequential" in per_pattern:
        burst = per_pattern["burst"]["cycle"]["median"]
        sequential = per_pattern["sequential"]["cycle"]["median"]
        trips = {}
        for (pattern, motor_id), entry in per_motor.items():
            trips.setdefault(motor_id, {})[pattern] = entry["round_trip"]["median"]
        comparison = {"burst": burst, "sequential": sequential, "saved": sequential - burst,
                      "round_trip": {m: (v.get("burst", math.nan), v.get("sequential", math.nan))
                                     for m, v in sorted(trips.items())}}
    return {"clock": name, "bitrate": bitrate, "files": list(paths), "mode": [m.get("mode", [""])[0] for m in metas],
            "channel": [m.get("channel", [""])[0] for m in metas], "per_motor": per_motor, "per_slot": per_slot,
            "per_pattern": per_pattern, "utilisation": utilisation, "comparison": comparison}


def ms(value):
    return "    -" if value is None or (isinstance(value, float) and math.isnan(value)) else f"{value * 1e3:6.3f}"


def report(summary):
    print(f"files: {len(summary['files'])}  mode: {', '.join(sorted(set(summary['mode'])))}  "
          f"channel: {', '.join(sorted(set(summary['channel'])))}  clock: {summary['clock']}  "
          f"bitrate: {summary['bitrate']}")
    print()
    print("Round trip per motor (ms; TX echo -> own reply)")
    print("pattern     id     n    ok  no_reply  no_echo  median     p95     max     rt-wire")
    for (pattern, motor_id), entry in sorted(summary["per_motor"].items()):
        trip = entry["round_trip"]
        print(f"{pattern:<10} {motor_id:>3} {entry['n']:>5} {entry['ok']:>5} {entry['no_reply']:>9} "
              f"{entry['no_echo']:>8}  {ms(trip['median'])}  {ms(trip['p95'])}  {ms(trip['max'])}  "
              f"{ms(entry['turnaround']['median'])}")
    if summary["per_slot"]:
        print()
        print("Burst slots (ms from the cycle's first TX echo)")
        print("slot  ids        tx median    p95   rx median    p95     max")
        for slot, entry in summary["per_slot"].items():
            ids = ",".join(str(m) for m in entry["motors"])
            tx = entry["tx_offset"]
            rx = entry["rx_offset"]
            print(f"{slot:>4}  {ids:<9}   {ms(tx['median'])} {ms(tx['p95'])}    {ms(rx['median'])} {ms(rx['p95'])} "
                  f"{ms(rx['max'])}")
    print()
    print("Cycle (first TX echo -> last reply; complete cycles only)")
    print("pattern         n  median     p95     max  host median  wire busy")
    for pattern, entry in sorted(summary["per_pattern"].items()):
        cycle = entry["cycle"]
        print(f"{pattern:<10} {cycle['n']:>6}  {ms(cycle['median'])}  {ms(cycle['p95'])}  {ms(cycle['max'])}  "
              f"     {ms(entry['host_cycle']['median'])}    {entry['busy']['median'] * 100:5.1f} %")
    print()
    print("Bus load (frame bits with stuffing, CRC and interframe space)")
    for entry in summary["utilisation"]:
        print(f"{entry['file']}: {entry['frames']} frames, {entry['bits']} bits over {entry['span']:.2f} s = "
              f"{entry['fraction'] * 100:.2f} % of the line")
        counts = {k: v for k, v in entry["counts"].items() if v not in ("0", "")}
        if counts:
            print(f"  bookkeeping: {counts}")
    comparison = summary["comparison"]
    if comparison:
        print()
        print(f"Burst vs sequential: cycle median {ms(comparison['burst']).strip()} vs "
              f"{ms(comparison['sequential']).strip()} ms (burst saves {ms(comparison['saved']).strip()} ms)")
        for motor_id, (burst, sequential) in comparison["round_trip"].items():
            print(f"  ID {motor_id:>2}: round trip {ms(burst).strip()} (burst) vs {ms(sequential).strip()} "
                  f"(sequential) ms")


def main(argv=None):
    p = argparse.ArgumentParser(description="Summarize bus_timing.py captures.")
    p.add_argument("paths", nargs="+", help="CSV files or glob patterns")
    p.add_argument("--clock", choices=["auto", "sw", "hw"], default="auto")
    p.add_argument("--bitrate", type=int, default=DEFAULT_BITRATE)
    args = p.parse_args(argv)
    paths = []
    for pattern in args.paths:
        matches = sorted(glob.glob(pattern))
        paths.extend(matches or [pattern])
    report(summarize(paths, args.clock, args.bitrate))
    return 0


if __name__ == "__main__":
    sys.exit(main())
