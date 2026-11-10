#!/usr/bin/env python3
import argparse
import csv
import glob
import math
import statistics
import sys

NOISE_SIGMAS = 5.0
NOISE_PEAK_MARGIN = 1.2
MIN_STEP_FRACTION = 0.1
MAX_THRESHOLD_FRACTION = 0.5
MIN_BASELINE_SAMPLES = 20
PLATEAU_FROM = 0.005
PLATEAU_TO = 0.02
SETTLE_BAND = 0.05
STEADY_FRACTION = 0.25
FIT_DT = 1e-4
FIT_GRID = 60
FIT_J_MIN = 0.001
FIT_J_MAX = 0.3
FIT_REFINE = 30
J_CANDIDATES = {"rs02": (0.0042, 0.0173), "rs03": (0.02, 0.064)}
NUMERIC = ("gap_s", "t_send_mono", "t_send_wall", "tx_ts", "tx_hw_ts", "t_rx_mono", "rx_ts", "rx_hw_ts",
           "cmd_pos", "cmd_torque", "pos_rad", "vel_rad_s", "torque_nm", "temp_c")


def number(text):
    if text is None or text == "":
        return None
    return float(text)


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
            row["trial"] = int(row["trial"])
            row["repeat"] = int(row["repeat"])
            row["step_nm"] = float(row["step_nm"])
            row["edge"] = int(row["edge"])
            rows.append(row)
    return meta, rows


def pick_clock(rows, wanted):
    complete = [r for r in rows if r["status"] == "ok"]
    has_hw = bool(complete) and all(r["tx_hw_ts"] is not None and r["rx_hw_ts"] is not None for r in complete)
    if wanted == "hw" and not has_hw:
        raise SystemExit("this file has no complete adapter timestamps; use --clock sw")
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


def round_trip(rows, tx_key, rx_key):
    good = [r for r in rows if r["status"] == "ok" and r[tx_key] is not None and r[rx_key] is not None]
    kernel = [r[rx_key] - r[tx_key] for r in good]
    host = [r["t_rx_mono"] - r["t_send_mono"] for r in good]
    period = [b[tx_key] - a[tx_key] for a, b in zip(good, good[1:])
              if b["trial"] == a["trial"] and not b["edge"] and not a["edge"]]
    return {"kernel": distribution(kernel), "host": distribution(host), "period": distribution(period)}


def baseline(values):
    mean = statistics.fmean(values)
    sigma = statistics.pstdev(values)
    peak = max(abs(v - mean) for v in values)
    return mean, sigma, peak


def first_crossing(times, values, level):
    for k, value in enumerate(values):
        if value >= level:
            if k == 0 or values[k - 1] >= level or value == values[k - 1]:
                return k, times[k]
            fraction = (level - values[k - 1]) / (value - values[k - 1])
            return k, times[k - 1] + fraction * (times[k] - times[k - 1])
    return None, math.nan


def motion_onset(times, values, base_values, direction):
    mean, sigma, peak = baseline(base_values)
    level = max(NOISE_SIGMAS * sigma, NOISE_PEAK_MARGIN * peak, 1e-9)
    for t, value in zip(times, values):
        if (value - mean) * direction > level:
            return t
    return math.nan


def prepare_edge(before, after, amplitude, tx_key, rx_key, window):
    result = {"amplitude": amplitude, "used": False, "why": ""}
    if not after or not after[0]["edge"]:
        result["why"] = "no edge frame"
        return result
    t0 = after[0][tx_key]
    if after[0]["status"] != "ok" or t0 is None:
        result["why"] = "edge frame has no TX timestamp"
        return result
    base_rows = [r for r in before if r["status"] == "ok"]
    if len(base_rows) < MIN_BASELINE_SAMPLES:
        result["why"] = "baseline too short"
        return result
    mean, sigma, peak = baseline([r["torque_nm"] for r in base_rows])
    direction = 1.0 if amplitude > 0 else -1.0
    span = []
    for row in after:
        if row["status"] != "ok" or row[rx_key] is None:
            result["why"] = "frame without a matched reply inside the window"
            return result
        if row[rx_key] - t0 > window:
            break
        span.append(row)
    result.update(noise_sd=sigma, noise_peak=peak, direction=direction, base_rows=base_rows, span=span,
                  times=[r[rx_key] - t0 for r in span],
                  response=[(r["torque_nm"] - mean) * direction for r in span],
                  gap=after[0]["gap_s"] or 0.0, ready=True)
    return result


def detect_edge(result, threshold):
    size = abs(result["amplitude"])
    times = result["times"]
    response = result["response"]
    index, _ = first_crossing(times, response, threshold)
    if index is None:
        result["why"] = "no response inside the window"
        return result
    span = result["span"]
    base_rows = result["base_rows"]
    result["used"] = True
    result["reply_index"] = index
    result["delay_hi"] = times[index]
    result["delay_lo"] = times[index - 1] if index > 0 else 0.0
    result["peak_fraction"] = max(response) / size
    late = [y for t, y in zip(times, response) if PLATEAU_FROM <= t <= PLATEAU_TO]
    result["plateau_fraction"] = statistics.median(late) / size if late else math.nan
    result["vel_onset"] = motion_onset(times, [r["vel_rad_s"] for r in span],
                                       [r["vel_rad_s"] for r in base_rows], result["direction"])
    result["pos_onset"] = motion_onset(times, [r["pos_rad"] for r in span],
                                       [r["pos_rad"] for r in base_rows], result["direction"])
    return result


def pooled_curve(results, bin_width):
    bins = {}
    for result in results:
        size = abs(result["amplitude"])
        for t, y in zip(result["times"], result["response"]):
            bins.setdefault(int(math.floor(t / bin_width)), []).append(y / size)
    centers = [(k + 0.5) * bin_width for k in sorted(bins)]
    means = [statistics.fmean(bins[k]) for k in sorted(bins)]
    return centers, means


def median_or_nan(values):
    values = [v for v in values if not math.isnan(v)]
    return statistics.median(values) if values else math.nan


def step_scale(meta):
    kind = (meta.get("step_kind") or ["torque"])[0] or "torque"
    if kind == "torque":
        return kind, 1.0
    if kind == "position":
        kp = float((meta.get("kp") or ["nan"])[0])
        if not (math.isfinite(kp) and kp > 0):
            raise SystemExit(f"position steps need a positive kp in the header ({kp})")
        return kind, kp * math.pi / 180.0
    if kind != "velocity":
        raise SystemExit(f"unknown step_kind {kind!r}")
    kd = float((meta.get("kd") or ["nan"])[0])
    if not (math.isfinite(kd) and kd > 0):
        raise SystemExit(f"velocity steps need a positive kd in the header ({kd})")
    return kind, kd


def position_trial(before, after, tx_key, rx_key):
    result = {"used": False}
    if not after or not after[0]["edge"] or after[0]["status"] != "ok" or after[0][tx_key] is None:
        return result
    base = [r["pos_rad"] for r in before if r["status"] == "ok" and r["pos_rad"] is not None]
    if len(base) < MIN_BASELINE_SAMPLES:
        return result
    t0 = after[0][tx_key]
    start = statistics.fmean(base)
    target = after[0]["cmd_pos"]
    if target is None or abs(target - start) < 1e-6:
        return result
    distance = target - start
    span = [r for r in after if r["status"] == "ok" and r[rx_key] is not None and r["pos_rad"] is not None]
    if len(span) < MIN_BASELINE_SAMPLES:
        return result
    times = [r[rx_key] - t0 for r in span]
    direction = 1.0 if distance > 0 else -1.0
    moved = [(r["pos_rad"] - start) * direction for r in span]
    size = abs(distance)
    y = [m / size for m in moved]
    _, t10 = first_crossing(times, y, 0.1)
    _, t50 = first_crossing(times, y, 0.5)
    _, t90 = first_crossing(times, y, 0.9)
    peak_index = max(range(len(y)), key=lambda k: y[k])
    tail = y[int(len(y) * (1.0 - STEADY_FRACTION)):]
    settle = math.nan
    if abs(y[-1] - 1.0) <= SETTLE_BAND:
        settle = 0.0
        for t, value in zip(times, y):
            if abs(value - 1.0) > SETTLE_BAND:
                settle = math.nan
            elif math.isnan(settle):
                settle = t
    result.update(used=True, size=size, direction=direction, times=times, moved=moved, t10=t10, t50=t50, t90=t90,
                  peak_time=times[peak_index], overshoot=(y[peak_index] - 1.0) * 100.0, settle=settle,
                  steady=statistics.fmean(tail) - 1.0, duration=times[-1])
    return result


def simulate(times, size, inertia, kp, kd, delay, coulomb=0.0, stiction=0.0, dt=FIT_DT):
    out = []
    x = 0.0
    v = 0.0
    t = 0.0
    for when in times:
        while t + dt <= when + 1e-12:
            if t >= delay:
                drive = kp * (size - x) - kd * v
            else:
                drive = -kd * v
            if v == 0.0 and abs(drive) <= max(stiction, coulomb):
                acc = 0.0
            else:
                moving = v if v != 0.0 else drive
                acc = (drive - math.copysign(coulomb, moving)) / inertia
            v_new = v + acc * dt
            if v != 0.0 and v_new * v < 0.0 and abs(kp * (size - x)) <= max(stiction, coulomb):
                v_new = 0.0
            v = v_new
            x += v * dt
            t += dt
        out.append(x)
    return out


def fit_rms(trials, inertia, kp, kd, delay, coulomb, stiction, fit_window):
    size = statistics.fmean(t["size"] for t in trials)
    count = int(math.ceil(fit_window / FIT_DT)) + 2
    grid = [k * FIT_DT for k in range(count)]
    model = simulate(grid, size, inertia, kp, kd, delay, coulomb, stiction)
    total = 0.0
    used = 0
    for trial in trials:
        scale = size / trial["size"]
        for t, measured in zip(trial["times"], trial["moved"]):
            if not 0.0 <= t <= fit_window:
                continue
            k = min(int(t / FIT_DT), count - 2)
            fraction = (t - grid[k]) / FIT_DT
            predicted = model[k] + (model[k + 1] - model[k]) * fraction
            total += (measured * scale - predicted) ** 2
            used += 1
    return math.sqrt(total / used) if used else math.nan


def fit_inertia(trials, kp, kd, delay, coulomb, stiction, fit_window, candidates=(), free=True):
    def rms(j):
        return fit_rms(trials, j, kp, kd, delay, coulomb, stiction, fit_window)

    named = [(j, rms(j)) for j in candidates]
    best = None
    if free:
        ratio = (FIT_J_MAX / FIT_J_MIN) ** (1.0 / (FIT_GRID - 1))
        grid = [FIT_J_MIN * ratio ** k for k in range(FIT_GRID)]
        scores = [rms(j) for j in grid]
        k = min(range(len(grid)), key=lambda i: scores[i])
        lo = math.log(grid[max(k - 1, 0)])
        hi = math.log(grid[min(k + 1, len(grid) - 1)])
        golden = (math.sqrt(5.0) - 1.0) / 2.0
        a = hi - golden * (hi - lo)
        b = lo + golden * (hi - lo)
        fa = rms(math.exp(a))
        fb = rms(math.exp(b))
        for _ in range(FIT_REFINE):
            if fa < fb:
                hi, b, fb = b, a, fa
                a = hi - golden * (hi - lo)
                fa = rms(math.exp(a))
            else:
                lo, a, fa = a, b, fb
                b = lo + golden * (hi - lo)
                fb = rms(math.exp(b))
        j = math.exp((lo + hi) / 2.0)
        best = (j, rms(j))
        if scores[k] < best[1]:
            best = (grid[k], scores[k])
    return {"best": best, "named": named}


def ideal_response(inertia, kp, kd):
    wn = math.sqrt(kp / inertia)
    zeta = kd / (2.0 * math.sqrt(kp * inertia))
    if zeta >= 1.0:
        return {"zeta": zeta, "peak_time": math.nan, "overshoot": 0.0}
    wd = wn * math.sqrt(1.0 - zeta * zeta)
    return {"zeta": zeta, "peak_time": math.pi / wd,
            "overshoot": 100.0 * math.exp(-zeta * math.pi / math.sqrt(1.0 - zeta * zeta))}


def position_groups(groups, kp, kd, model, delays, coulomb, stiction, fit_window, candidates, free):
    out = {}
    if candidates is None:
        candidates = J_CANDIDATES.get(model, ())
    for key, trials in groups.items():
        used = [t for t in trials if t["used"]]
        entry = {"n": len(used), "n_total": len(trials)}
        if used:
            for name in ("t10", "t50", "t90", "peak_time", "overshoot", "settle", "steady"):
                entry[name] = median_or_nan([t[name] for t in used])
            entry["size"] = statistics.fmean(t["size"] for t in used)
            entry["rise"] = entry["t90"] - entry["t10"]
            delay = delays.get(key, math.nan)
            if math.isnan(delay):
                delay = 0.0
            entry["delay"] = delay
            window = min(fit_window, min(t["duration"] for t in used))
            entry["fit_window"] = window
            entry["fit"] = fit_inertia(used, kp, kd, delay, coulomb, stiction, window, candidates, free)
        out[key] = entry
    pooled = {}
    for size in sorted({abs(key[0]) for key in groups}):
        keys = [key for key in groups if abs(key[0]) == size]
        used = [t for key in keys for t in groups[key] if t["used"]]
        entry = {"n": len(used), "n_total": sum(len(groups[key]) for key in keys)}
        if used:
            known = [out[key]["delay"] for key in keys if out[key]["n"]]
            entry["delay"] = statistics.median(known)
            entry["fit_window"] = min(fit_window, min(t["duration"] for t in used))
            entry["fit"] = fit_inertia(used, kp, kd, entry["delay"], coulomb, stiction, entry["fit_window"],
                                       candidates, free)
        pooled[size] = entry
    return out, pooled


def summarize(path, clock="auto", window=0.05, bin_width=0.00015, coulomb=0.0, stiction=0.0, fit_window=0.15,
              candidates=None, free=True):
    meta, rows = load(path)
    kind, scale = step_scale(meta)
    clock_name, tx_key, rx_key = pick_clock(rows, clock)
    trip = round_trip(rows, tx_key, rx_key)
    trials = {}
    for row in rows:
        trials.setdefault(row["trial"], {"pre": [], "on": [], "off": []})[row["phase"]].append(row)
    groups = {}
    motion = {}
    for trial in sorted(trials):
        parts = trials[trial]
        if not parts["on"]:
            continue
        step = parts["on"][0]["step_nm"]
        settled_pre = parts["pre"][len(parts["pre"]) // 5:]
        settled_on = parts["on"][len(parts["on"]) // 2:]
        edges = (("on", settled_pre, parts["on"], step * scale), ("off", settled_on, parts["off"], -step * scale))
        for name, before, after, amplitude in edges:
            groups.setdefault((step, name), []).append(
                prepare_edge(before, after, amplitude, tx_key, rx_key, window))
            if kind == "position":
                motion.setdefault((step, name), []).append(position_trial(before, after, tx_key, rx_key))
    edges = {}
    rt = trip["kernel"]["median"]
    for key, results in groups.items():
        ready = [r for r in results if r.get("ready")]
        size = abs(key[0] * scale)
        entry = {"n": 0, "n_total": len(results)}
        if ready:
            sigma = statistics.median(r["noise_sd"] for r in ready)
            peak = statistics.median(r["noise_peak"] for r in ready)
            threshold = max(NOISE_SIGMAS * sigma, NOISE_PEAK_MARGIN * peak, MIN_STEP_FRACTION * size)
            entry.update(threshold=threshold, noise_sd=sigma, noise_peak=peak)
            for r in ready:
                if threshold > MAX_THRESHOLD_FRACTION * size:
                    r["why"] = "step too small for the torque noise"
                else:
                    detect_edge(r, threshold)
        used = [r for r in results if r["used"]]
        entry["n"] = len(used)
        entry["skipped"] = sorted({r["why"] for r in results if not r["used"]})
        if used:
            centers, means = pooled_curve(used, bin_width)
            _, t10 = first_crossing(centers, means, 0.1)
            _, t50 = first_crossing(centers, means, 0.5)
            _, t90 = first_crossing(centers, means, 0.9)
            entry.update(
                reply_index=(min(r["reply_index"] for r in used), max(r["reply_index"] for r in used)),
                delay_lo=median_or_nan([r["delay_lo"] for r in used]),
                delay_hi=median_or_nan([r["delay_hi"] for r in used]),
                delay_hi_min=min(r["delay_hi"] for r in used),
                delay_hi_max=max(r["delay_hi"] for r in used),
                pooled_lo=max(r["delay_lo"] for r in used),
                pooled_hi=min(r["delay_hi"] for r in used),
                t10=t10, t50=t50, t90=t90, rise=t90 - t10,
                peak_fraction=median_or_nan([r["peak_fraction"] for r in used]),
                plateau_fraction=median_or_nan([r["plateau_fraction"] for r in used]),
                vel_onset=median_or_nan([r["vel_onset"] for r in used]),
                pos_onset=median_or_nan([r["pos_onset"] for r in used]),
                curve=(centers, means),
            )
            entry["after_reply_lo"] = entry["pooled_lo"] - rt
            entry["after_reply_hi"] = entry["pooled_hi"] - rt
        entry["trials"] = results
        edges[key] = entry
    summary = {"path": path, "meta": meta, "step_kind": kind, "step_scale": scale, "clock": clock_name,
               "round_trip": trip, "edges": edges, "cycles": len(rows)}
    if kind == "position":
        kp = float(meta["kp"][0])
        kd = float(meta["kd"][0])
        model = (meta.get("model") or [""])[0]
        delays = {key: (e["pooled_lo"] + e["pooled_hi"]) / 2.0 if e["n"] else math.nan
                  for key, e in edges.items()}
        summary["position"], summary["position_pooled"] = position_groups(
            motion, kp, kd, model, delays, coulomb, stiction, fit_window, candidates, free)
        summary["model_args"] = {"coulomb": coulomb, "stiction": stiction, "kp": kp, "kd": kd}
    return summary


def ms(value, digits=2):
    if value is None or math.isnan(value):
        return "-"
    return f"{value * 1e3:.{digits}f}"


def report(summary):
    meta = summary["meta"]

    def first(name):
        return (meta.get(name) or [""])[0]

    print(f"== {summary['path']}")
    print(f"motor ID {first('motor_id')} {first('model').upper()} on {first('channel')}, "
          f"kp {first('kp')}, kd {first('kd')}; {summary['cycles']} cycles; "
          f"clock: {'adapter (hw)' if summary['clock'] == 'hw' else 'kernel (sw)'}")
    if first("aborted"):
        print(f"capture stopped early: {first('aborted')}")
    odd = {name: first(name) for name in ("no_reply", "no_echo", "stale_reply", "stray_echo", "other",
                                          "error_frames") if first(name) not in ("", "0")}
    if odd:
        print(f"frame bookkeeping: {odd}")
    trip = summary["round_trip"]
    print("round trip, command TX -> its own reply RX [ms]:")
    for label, name in (("kernel/adapter stamps", "kernel"), ("host monotonic", "host"),
                        ("cycle period", "period")):
        d = trip[name]
        print(f"  {label:22s} n {d['n']:6d}  median {ms(d['median'], 3)}  p95 {ms(d['p95'], 3)}  "
              f"max {ms(d['max'], 3)}")
    velocity = summary.get("step_kind") == "velocity"
    position = summary.get("step_kind") == "position"
    scale = summary.get("step_scale", 1.0)
    if position:
        print(f"position-target step (torque kp*step, kp {first('kp')}), TX of the step frame -> reply RX [ms]; "
              "onset = first reply past the threshold:")
        print("  step deg (kp*step N*m)  edge   n  thr N*m reply#  onset(median)  onset(pooled)  minus RT      "
              "t10    t50    t90  rise  peak  plateau  vel-onset pos-onset")
    elif velocity:
        print(f"velocity-target step (torque kd*v, kd {scale:g}), TX of the step frame -> reply RX [ms]; "
              "onset = first reply past the threshold:")
        print("  step rad/s (kd*v N*m) edge   n  thr N*m reply#  onset(median)  onset(pooled)  minus RT      "
              "t10    t50    t90  rise  peak  plateau  vel-onset pos-onset")
    else:
        print("torque step, TX of the step frame -> reply RX [ms]; onset = first reply past the threshold:")
        print("  step N*m edge   n  thr N*m reply#  onset(median)  onset(pooled)  minus RT      "
              "t10    t50    t90  rise  peak  plateau  vel-onset pos-onset")
    for (step, edge) in sorted(summary["edges"], key=lambda k: (abs(k[0]), -k[0], k[1] != "on")):
        e = summary["edges"][(step, edge)]
        if position:
            label = f"{step:+8.2f} ({step * scale:+6.2f})       "
        elif velocity:
            label = f"{step:+8.2f} ({step * scale:+6.2f})    "
        else:
            label = f"{step:+8.2f}"
        if not e["n"]:
            print(f"  {label} {edge:4s} {e['n']:3d}/{e['n_total']}  not measured: {'; '.join(e['skipped'])}")
            continue
        flag = "!" if e["pooled_lo"] > e["pooled_hi"] else " "
        print(f"  {label} {edge:4s} {e['n']:3d}  {e['threshold']:6.3f}  "
              f"{e['reply_index'][0]}-{e['reply_index'][1]}    "
              f"{ms(e['delay_lo'])}..{ms(e['delay_hi'])}     "
              f"{ms(e['pooled_lo'])}..{ms(e['pooled_hi'])}{flag}    "
              f"{ms(e['after_reply_lo'])}..{ms(e['after_reply_hi'])}  "
              f"{ms(e['t10']):>5s}  {ms(e['t50']):>5s}  {ms(e['t90']):>5s} {ms(e['rise']):>5s}  "
              f"{e['peak_fraction']:4.2f}  {e['plateau_fraction']:7.2f}  "
              f"{ms(e['vel_onset']):>9s} {ms(e['pos_onset']):>9s}")
        if e["skipped"]:
            print(f"           {e['n_total'] - e['n']} trial(s) skipped: {'; '.join(e['skipped'])}")
    if position:
        report_position(summary)
    print()


def report_position(summary):
    model = summary["model_args"]
    kp = model["kp"]
    kd = model["kd"]
    print()
    print("position response, TX of the step frame -> reply RX [ms]; fractions of the commanded step, medians over "
          "repeats:")
    print("  step deg  edge   n      t10    t50    t90   rise  peak-t  overshoot%  settle5%  steady-err%")
    keys = sorted(summary["position"], key=lambda k: (abs(k[0]), -k[0], k[1] != "on"))
    for key in keys:
        e = summary["position"][key]
        if not e["n"]:
            print(f"  {key[0]:+8.2f}  {key[1]:4s} {e['n']:3d}/{e['n_total']}  not measured")
            continue
        print(f"  {key[0]:+8.2f}  {key[1]:4s} {e['n']:3d}   {ms(e['t10']):>6s} {ms(e['t50']):>6s} "
              f"{ms(e['t90']):>6s} {ms(e['rise']):>6s}  {ms(e['peak_time']):>6s}  {e['overshoot']:9.1f}  "
              f"{ms(e['settle']):>8s}  {e['steady'] * 100.0:10.1f}")
    print()
    print(f"ideal PD + inertia + Coulomb fit (kp {kp:g}, kd {kd:g}, Coulomb {model['coulomb']:g} N*m, stiction "
          f"{model['stiction']:g} N*m; delay = pooled torque onset of the group); rms in mdeg:")
    def fit_text(fit):
        parts = []
        if fit["best"] is not None:
            parts.append(f"best J {fit['best'][0]:.4f} rms {math.degrees(fit['best'][1]) * 1e3:.1f}")
        for j, value in fit["named"]:
            parts.append(f"J {j:g} rms {math.degrees(value) * 1e3:.1f}")
        return ";  ".join(parts)

    for key in keys:
        e = summary["position"][key]
        if not e["n"]:
            continue
        print(f"  {key[0]:+8.2f}  {key[1]:4s} delay {ms(e['delay'])} ms, window {ms(e['fit_window'], 0)} ms:  "
              + fit_text(e["fit"]))
    for size, e in sorted(summary["position_pooled"].items()):
        if not e["n"]:
            continue
        print(f"  |{size:.2f}| all edges, n {e['n']}, delay {ms(e['delay'])} ms, window "
              f"{ms(e['fit_window'], 0)} ms:  " + fit_text(e["fit"]))
    named = [j for j, _ in next((e["fit"]["named"] for e in summary["position"].values() if e["n"]), [])]
    if named:
        print("ideal linear PD (no friction, no delay) for reference:")
        for j in named:
            ideal = ideal_response(j, kp, kd)
            print(f"  J {j:g}: zeta {ideal['zeta']:.2f}, peak time {ms(ideal['peak_time'], 1)} ms, "
                  f"overshoot {ideal['overshoot']:.1f} %")


def main(argv=None):
    p = argparse.ArgumentParser(description="Analyze latency.py captures.")
    p.add_argument("files", nargs="+")
    p.add_argument("--clock", choices=["auto", "sw", "hw"], default="auto")
    p.add_argument("--window-ms", type=float, default=50.0)
    p.add_argument("--bin-ms", type=float, default=0.15)
    p.add_argument("--coulomb", type=float, default=0.0, help="Position mode fit: Coulomb friction in N*m")
    p.add_argument("--stiction", type=float, default=0.0, help="Position mode fit: breakaway torque in N*m")
    p.add_argument("--fit-ms", type=float, default=150.0, help="Position mode fit: window after the step frame")
    p.add_argument("--j-candidates", type=float, nargs="+", default=None,
                   help="Position mode fit: inertias to score (default RS02 0.0042 0.0173, RS03 0.02 0.064)")
    p.add_argument("--no-free-j", action="store_true", help="Position mode fit: score only the candidates")
    args = p.parse_args(argv)
    for name in ("coulomb", "stiction", "fit_ms"):
        value = getattr(args, name)
        if not (math.isfinite(value) and value >= 0.0):
            p.error(f"--{name.replace('_', '-')} must be finite and >= 0 ({value})")
    if args.j_candidates and not all(math.isfinite(j) and j > 0 for j in args.j_candidates):
        p.error("--j-candidates must be finite and positive")
    paths = []
    for pattern in args.files:
        matched = sorted(glob.glob(pattern))
        paths.extend(matched if matched else [pattern])
    for path in paths:
        report(summarize(path, clock=args.clock, window=args.window_ms * 1e-3, bin_width=args.bin_ms * 1e-3,
                         coulomb=args.coulomb, stiction=args.stiction, fit_window=args.fit_ms * 1e-3,
                         candidates=args.j_candidates, free=not args.no_free_j))
    return 0


if __name__ == "__main__":
    sys.exit(main())
