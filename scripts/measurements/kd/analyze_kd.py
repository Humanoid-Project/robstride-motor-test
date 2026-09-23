#!/usr/bin/env python3
"""Fit the effective velocity gain from a kd.py capture.

MIT mode gives `tau = kp*(pos_t - pos) + kd*(vel_t - vel)`, so regressing
`tau = kp*e + kd*(vel_t - vel) + b` over the sweep returns both gains. kp is reported as a
cross-check against the kp/ bench result; kd is the number this test is for.
"""

import csv
import math
import sys

import numpy as np


def load(path):
    header, rows, columns = {}, [], None
    for row in csv.reader(open(path)):
        if not row:
            continue
        if row[0].lstrip().startswith("#"):
            cells = [c.strip().lstrip("#").strip() for c in row]
            cells = [c for c in cells if c]
            for i in range(0, len(cells) - 1, 2):
                header[cells[i]] = cells[i + 1]
            continue
        if row[0] == "repeat":
            columns = [c.strip() for c in row]
            continue
        rows.append([float(v) for v in row])
    return header, columns, np.array(rows)


def main():
    if len(sys.argv) < 2:
        print("usage: analyze_kd.py <capture.csv> [measured_kp]")
        return 1
    h, columns, a = load(sys.argv[1])
    if a.size == 0:
        print("no samples")
        return 1
    names = ("repeat", "vel_target", "pos_target", "pos_rad", "vel_rad_s", "torque_nm", "temp_c")
    missing = [name for name in names if columns is None or name not in columns]
    if missing:
        print(f"missing columns: {', '.join(missing)}")
        return 1
    rep, vt, pt, pos, vel, tq, temp = (a[:, columns.index(name)] for name in names)
    kp_cmd = float(h.get("commanded_kp", "nan"))
    kd_cmd = float(h.get("commanded_kd", "nan"))
    e = pt - pos
    dv = vt - vel

    print(f"motor ID {h.get('motor_id')}  model {h.get('model')}  "
          f"commanded kp {kp_cmd}  kd {kd_cmd}")
    print(f"{len(a)} samples, {len(np.unique(vt))} velocity targets, {len(np.unique(rep))} repeats")
    print(f"|actual vel| mean {np.abs(vel).mean():.4f} rad/s  "
          f"(the joint should stay near still; large means it ran)")
    print(f"temperature {temp.min():.0f} -> {temp.max():.0f} C\n")

    print(f"{'vel_t':>8}{'pos err deg':>13}{'actual vel':>12}{'torque':>10}{'n':>6}")
    for v in sorted(np.unique(vt)):
        s = vt == v
        print(f"{v:>+8.2f}{math.degrees(e[s].mean()):>13.2f}{vel[s].mean():>12.4f}"
              f"{tq[s].mean():>10.3f}{s.sum():>6}")

    # e and dv are collinear by construction: once the joint settles, kp*e + kd*vel_t balances a
    # roughly constant load, so e is proportional to -vel_t. Regressing on both therefore cannot
    # separate the gains -- it returns physically impossible values such as a negative kp. kp is
    # already known from the kp/ bench, so it is held fixed and only kd is solved for.
    corr = float(np.corrcoef(e, dv)[0, 1])

    print(f"\n--- kd from tau = kp*err + kd*(vel_t - vel), kp held fixed ---")
    # Report both kp choices: kd scales with whatever kp is assumed, so a noisy bench kp
    # propagates straight into kd. On ID 7 a bench kp of 111.3 (against 98.7-103.7 elsewhere)
    # turned kd 2.07 into 2.26; the spread between these two rows is the real uncertainty.
    choices = [("commanded", kp_cmd)]
    if len(sys.argv) > 2:
        choices.append(("measured (argv[2])", float(sys.argv[2])))
    kd_fit = None
    for label, kp_used in choices:
        kd_per = (tq - kp_used * e) / dv
        if kd_fit is None:
            kd_fit, kd_sd = kd_per.mean(), kd_per.std()
        print(f"  kp {kp_used:7.2f} ({label:<18}) -> kd {kd_per.mean():6.3f} +- {kd_per.std():.3f}"
              f"   ratio {kd_per.mean()/kd_cmd:5.2f}")
    kp_used = choices[-1][1]
    print(f"\n  per velocity target:")
    for v in sorted(np.unique(vt)):
        s = vt == v
        k = (tq[s] - kp_used * e[s]) / dv[s]
        print(f"    vel_t {v:+5.2f}:  kd {k.mean():6.3f} +- {k.std():.3f}")
    noise = vel.std()
    print(f"\n  velocity-feedback noise: std {noise:.4f} rad/s = "
          f"{100*noise/np.abs(vt).mean():.1f}% of the mean velocity target")
    print(f"    this noise sits directly on the kd regressor and sets the spread above")
    print(f"  corr(err, vel_t - vel) = {corr:+.4f}")
    if abs(corr) > 0.9:
        print("    the two regressors are collinear, which is expected; this is why kp is fixed")
        print("    rather than fitted. Pass the kp/ bench value as a second argument to use it.")

    # A small |vel_t| puts a near-zero denominator under (tau - kp*e), so one weak target can
    # dominate the spread. ID 5 read sd 2.745 against 0.24-0.52 elsewhere for exactly this.
    snr = np.abs(vt).min() * kd_cmd / max(noise * kd_cmd, 1e-9)
    if np.abs(vt).min() < 4.0 * noise:
        print(f"\n  WARNING: the smallest velocity target ({np.abs(vt).min():.2f}) is within "
              f"{np.abs(vt).min()/noise:.1f}x the velocity noise ({noise:.3f}).")
        print("    That target contributes a near-zero denominator and inflates the spread.")
        print("    Per-target rows above show which ones to trust; re-run without the small ones.")
    if np.ptp(dv) < 0.3:
        print("\n  WARNING: the velocity-target span is small; kd is poorly conditioned.")
    if np.abs(vel).mean() > 0.2:
        print("  WARNING: the joint was actually moving, so inertia contaminates the fit.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
