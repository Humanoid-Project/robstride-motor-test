#!/usr/bin/env python3
"""Summarise thermal_hold.py logs: temperature rise, one-node exponential fits of the hold and the cool-down, and the
continuous torque scaled to a 70 C motor limit as tau * sqrt((70 - T_amb) / (T_inf - T_amb))."""

import argparse
import csv
import glob
import math

import numpy as np

LIMIT_C = 70.0
WINDOW_S = 600.0
TAU_GRID = np.geomspace(30.0, 30000.0, 400)


def load(path):
    meta = {}
    with open(path) as fh:
        first = fh.readline().lstrip("# ").strip().split(",")
        meta = dict(zip(first[0::2], first[1::2]))
        rows = list(csv.DictReader(fh))
    return meta, rows


def column(rows, name):
    return np.array([float(r[name]) if r[name] != "" else np.nan for r in rows])


def fit_exponential(t, temp):
    if len(t) < 5 or t[-1] - t[0] < 60.0:
        return None
    t = t - t[0]
    best = None
    for tau in TAU_GRID:
        decay = np.exp(-t / tau)
        a = np.column_stack([np.ones_like(t), decay])
        coef, *_ = np.linalg.lstsq(a, temp, rcond=None)
        rms = float(np.sqrt(np.mean((a @ coef - temp) ** 2)))
        if best is None or rms < best[3]:
            best = (float(coef[0]), float(coef[0] + coef[1]), float(tau), rms)
    return best


def report(path):
    meta, rows = load(path)
    ambient = float(meta.get("ambient_c", "nan"))
    hold = [r for r in rows if r["phase"] == "hold"]
    cool = [r for r in rows if r["phase"] == "cooldown"]
    print(f"{path}")
    print(f"  ID {meta.get('motor_id')} {meta.get('model', '?').upper()}, kp {meta.get('kp')}, kd {meta.get('kd')}, "
          f"tau_ff {meta.get('tau_ff_nm')} N*m, ambient {ambient} C, start {meta.get('start')}")
    if not hold:
        print("  no hold rows")
        return
    t = column(hold, "t_s")
    temp = column(hold, "temp_c")
    torque = column(hold, "torque_nm")
    iq = column(hold, "iq_a")
    last = t >= t[-1] - WINDOW_S
    print(f"  hold {t[-1] / 60:.1f} min: T {temp[0]:.1f} -> {temp[-1]:.1f} C (max {np.nanmax(temp):.1f}), "
          f"dT over ambient {temp[-1] - ambient:+.1f} C")
    rise = temp[-1] - temp[np.argmax(last)]
    iq_text = f"{np.nanmean(iq[last]):+.3f} A" if np.any(np.isfinite(iq[last])) else "unread"
    print(f"  last {min(WINDOW_S, t[-1]) / 60:.0f} min: rise {rise:+.1f} C, "
          f"torque {np.nanmean(torque[last]):+.3f} N*m, iq {iq_text}")
    fit = fit_exponential(t, temp)
    if fit:
        t_inf, t0, tau, rms = fit
        print(f"  hold fit: T_inf {t_inf:.1f} C, tau {tau:.0f} s, rms {rms:.2f} C")
        rise_inf = t_inf - ambient
        tau_load = abs(np.nanmean(torque[last]))
        if rise_inf > 0.5 and ambient < LIMIT_C:
            cont = tau_load * math.sqrt((LIMIT_C - ambient) / rise_inf)
            print(f"  continuous torque at {LIMIT_C:g} C motor temperature: {cont:.2f} N*m "
                  f"(from {tau_load:.2f} N*m, dT_inf {rise_inf:.1f} C)")
    if cool:
        tc = column(cool, "t_s")
        tempc = column(cool, "temp_c")
        print(f"  cool-down {(tc[-1] - tc[0]) / 60:.1f} min: T {tempc[0]:.1f} -> {tempc[-1]:.1f} C")
        fit = fit_exponential(tc, tempc)
        if fit:
            print(f"  cool-down fit: T_inf {fit[0]:.1f} C, tau {fit[2]:.0f} s, rms {fit[3]:.2f} C")


def main():
    p = argparse.ArgumentParser(description="Summarise thermal_hold.py CSV logs.")
    p.add_argument("paths", nargs="+", help="CSV files or glob patterns")
    args = p.parse_args()
    paths = sorted({f for pattern in args.paths for f in (glob.glob(pattern) or [pattern])})
    for path in paths:
        report(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
