#!/usr/bin/env python3
"""Fit the effective position gain from a kp.py capture.

At rest the holding torque is `kp * (target - pos)` plus whatever constant offset the joint
carries, so a least-squares line through (error, torque) gives the gain as its slope.

The intercept is reported but is NOT a loaded/free check: by the same control law it sits near
zero either way. Every capture from the 2026-09-18 feet-on-ground bench landed between -0.018 and
-0.060 N.m, so the old `|intercept| > 0.5` warning could never have fired in the protocol it was
written to police.
"""

import csv
import math
import sys

import numpy as np


def load(path):
    header = {}
    rows = []
    with open(path) as fh:
        for row in csv.reader(fh):
            if not row:
                continue
            if row[0].lstrip().startswith("#"):
                cells = [c.strip().lstrip("#").strip() for c in row]
                cells = [c for c in cells if c]
                for i in range(0, len(cells) - 1, 2):
                    header[cells[i]] = cells[i + 1]
                continue
            if row[0] == "repeat":
                continue
            rows.append([float(v) for v in row])
    return header, np.array(rows)


def main():
    if len(sys.argv) < 2:
        print("usage: analyze_kp.py <capture.csv>")
        return 1
    header, a = load(sys.argv[1])
    if a.size == 0:
        print("no samples")
        return 1
    rep, off, target, pos, vel, torque, temp = (a[:, i] for i in range(7))
    kp_cmd = float(header.get("commanded_kp", "nan"))
    err = target - pos

    print(f"motor ID {header.get('motor_id')}  model {header.get('model')}  "
          f"commanded kp {kp_cmd}  kd {header.get('commanded_kd')}")
    print(f"{len(a)} samples, {len(np.unique(off))} offsets, {len(np.unique(rep))} repeats")
    # |vel| is dominated by feedback noise (std 0.067-0.093 rad/s measured 2026-09-18), so it is
    # never near zero even on a joint that is perfectly still. Net drift is the signal here.
    print(f"velocity: mean {vel.mean():+.4f} rad/s (drift), |vel| mean {np.abs(vel).mean():.4f} "
          f"(mostly feedback noise)")
    print(f"temperature {temp.min():.0f} -> {temp.max():.0f} C\n")

    print(f"{'offset':>8}{'err deg':>10}{'torque':>10}{'n':>5}")
    for o in sorted(np.unique(off)):
        s = off == o
        print(f"{o:>+8.1f}{math.degrees(err[s].mean()):>10.2f}{torque[s].mean():>10.3f}{s.sum():>5}")

    A = np.column_stack([err, np.ones_like(err)])
    (slope, intercept), *_ = np.linalg.lstsq(A, torque, rcond=None)
    resid = torque - A @ (np.array([slope, intercept]))
    ss = 1.0 - resid.var() / torque.var() if torque.var() > 0 else float("nan")

    print(f"\n--- fit: torque = kp * error + offset ---")
    print(f"  effective kp : {slope:8.2f} N.m/rad")
    print(f"  commanded kp : {kp_cmd:8.2f}")
    if math.isfinite(kp_cmd) and kp_cmd:
        print(f"  ratio        : {slope / kp_cmd:8.2f}   <-- 1.00 means the joint honours the command")
    print(f"  offset       : {intercept:+8.3f} N.m   (near zero either loaded or free; not a check)")
    print(f"  R^2          : {ss:8.3f}")
    print(f"  residual RMS : {resid.std():8.3f} N.m")
    if abs(vel.mean()) > 0.02:
        print("  WARNING: the joint drifted during sampling. Increase --settle-time.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
