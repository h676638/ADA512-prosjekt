#!/usr/bin/env python3
"""
ADA512 LAB 2 - Task 3: method 2 (on-off), limit cycle in phase F3.

    python task3_method2.py run_HHMMSS_m2.csv

Reads the run file written by lab2_logger.py (or a PuTTY log of one run) and
prints everything Task 3 asks for, next to the Task 1 predictions:
  a) c_pp and T_osc per cycle and averaged over the full cycles in F3
  b) slopes s_g, s_c (model and measured) and L = c_pp/(s_g + s_c)
  c) T_osc predicted from the model against T_osc measured
  d) K_u, T_u and the PI settings (Ziegler-Nichols), K_I, and K_R for method 6
Saves task3_limit_cycle.png: c(t) and u(t) in F3 with the construction of L.
"""
import math
import sys

import matplotlib.pyplot as plt

R_F3 = 7.0                                   # set point in F3 [K]
T_S = 10.0                                   # controller period [s]
PRED = {"c_pp": 1.24, "T_osc": 94.3, "n": 38.2, "L": 23.0,      # Task 1
        "K_u": 1.02, "T_u": 94.3, "K_R": 0.46, "T_i": 78.6}


def read_run(path):
    rows, settings, summary = [], {}, {}
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if line.startswith("# START"):
                rows, summary = [], {}
                settings = dict(p.split("=", 1) for p in line.split(",")[1:] if "=" in p)
            elif line.startswith("# F3"):
                summary = dict(p.split("=", 1) for p in line.split(",")[1:] if "=" in p)
            elif line[:1].isdigit():
                p = line.split(",")
                if len(p) == 11 and p[1] == "F3" and p[6] != "invalid":
                    rows.append((float(p[0]), float(p[6]), float(p[8])))
    return rows, settings, summary


def smooth(x, k=3):
    """running median over k samples, against the 0.01 K noise at the extremes"""
    h = k // 2
    return [sorted(x[max(0, i - h):i + h + 1])[len(x[max(0, i - h):i + h + 1]) // 2]
            for i in range(len(x))]


def lin_slope(t, c):
    n = len(t)
    if n < 3:
        return math.nan
    mt, mc = sum(t) / n, sum(c) / n
    den = sum((a - mt) ** 2 for a in t)
    return sum((a - mt) * (b - mc) for a, b in zip(t, c)) / den if den else math.nan


def main(path):
    rows, st, summ = read_run(path)
    if not rows:
        sys.exit("no F3 data lines in " + path)
    dT, T_P = float(st.get("DELTA_T", 16.6)), float(st.get("T_P", 307))
    t = [r[0] for r in rows]
    c = smooth([r[1] for r in rows])
    u = [r[2] for r in rows]

    # switching instants (first sample with the new u)
    on = [t[i] for i in range(1, len(u)) if u[i] > 0.5 and u[i - 1] < 0.5]
    off = [t[i] for i in range(1, len(u)) if u[i] < 0.5 and u[i - 1] > 0.5]
    idx = {tt: i for i, tt in enumerate(t)}

    cycles = []                     # one cycle: switch-on to the next switch-on
    for k in range(len(on) - 1):
        t_off = [x for x in off if on[k] < x < on[k + 1]]
        if not t_off:
            continue
        i_on, i_off, i_next = idx[on[k]], idx[t_off[0]], idx[on[k + 1]]
        i_min = min(range(i_on, i_off + 1), key=lambda i: c[i])      # trough after switch-on
        i_max = max(range(i_off, i_next + 1), key=lambda i: c[i])    # peak after switch-off
        # measured slopes: trough -> peak (heating), peak -> next trough (cooling)
        s_up = lin_slope(t[i_min:i_max + 1], c[i_min:i_max + 1])
        nxt = [x for x in off if x > on[k + 1]]
        i_end = idx[nxt[0]] if nxt else len(t) - 1
        i_min2 = min(range(idx[on[k + 1]], i_end + 1), key=lambda i: c[i])
        s_dn = -lin_slope(t[i_max:i_min2 + 1], c[i_max:i_min2 + 1])
        cycles.append(dict(t_on=on[k], t_off=t_off[0], period=on[k + 1] - on[k],
                           c_max=c[i_max], c_min=c[i_min], c_pp=c[i_max] - c[i_min],
                           L_on=t[i_min] - on[k], L_off=t[i_max] - t_off[0],
                           s_up=s_up, s_dn=s_dn, t_max=t[i_max], t_min=t[i_min]))

    print(f"file {path}:  DELTA_T={dT}  T_P={T_P}  T_amb={st.get('T_amb')}")
    print(f"F3: {t[-1] - t[0] + 1:.0f} s, {len(on)} switch-ons, {len(cycles)} full cycles\n")
    if len(cycles) < 3:
        print("!! fewer than three full cycles - Task 3a needs at least three\n")
    print(" cycle  t_on  period   c_max   c_min   c_pp   L_on  L_off   s_up    s_down")
    for i, cy in enumerate(cycles, 1):
        print(f"  {i:3d} {cy['t_on']:6.0f} {cy['period']:6.0f}  {cy['c_max']:6.3f}  {cy['c_min']:6.3f}"
              f"  {cy['c_pp']:5.3f}  {cy['L_on']:5.0f}  {cy['L_off']:5.0f}  {cy['s_up']:.4f}  {cy['s_dn']:.4f}")
    if not cycles:
        return
    n = len(cycles)
    c_pp = sum(cy["c_pp"] for cy in cycles) / n
    T_osc = sum(cy["period"] for cy in cycles) / n
    s_g, s_c = (dT - R_F3) / T_P, R_F3 / T_P
    s_up = sum(cy["s_up"] for cy in cycles) / n
    s_dn = sum(cy["s_dn"] for cy in cycles) / n
    L = c_pp / (s_g + s_c)
    L_dir = sum(cy["L_on"] + cy["L_off"] for cy in cycles) / (2 * n)
    T_osc_model = c_pp * (1 / s_g + 1 / s_c)
    a = c_pp / 2
    K_u = 4 * 0.5 / (math.pi * a)
    K_R, T_i = 0.45 * K_u, T_osc / 1.2
    n_h = 3600 / T_osc

    def row(name, pred, meas, unit, fmt=".3g"):
        d = f"{100 * (meas - pred) / pred:+.0f} %" if pred else ""
        print(f"  {name:<34} {pred:>8{fmt}} {meas:>9{fmt}}  {unit:<6} {d}")

    print("\n                                     predicted  measured")
    row("c_pp (mean of full cycles)", PRED["c_pp"], c_pp, "K")
    row("T_osc (mean of full cycles)", PRED["T_osc"], T_osc, "s")
    row("n = 3600/T_osc", PRED["n"], n_h, "1/h")
    row("L = c_pp/(s_g + s_c)", PRED["L"], L, "s")
    row("K_u = 4d/(pi a)", PRED["K_u"], K_u, "1/K")
    row("T_u = T_osc", PRED["T_u"], T_osc, "s")
    row("K_R = 0.45 K_u", PRED["K_R"], K_R, "1/K")
    row("T_i = T_u/1.2", PRED["T_i"], T_i, "s")

    print(f"\nslopes at r = 7 K:   model s_g = {s_g:.4f}  s_c = {s_c:.4f} K/s"
          f"   measured s_up = {s_up:.4f}  s_down = {s_dn:.4f} K/s")
    print(f"T_osc from the model with measured c_pp: {T_osc_model:.0f} s  against {T_osc:.0f} s measured"
          f"  ({100 * (T_osc - T_osc_model) / T_osc_model:+.0f} %)")
    print(f"L read directly from the actual switching instant to the extremum: {L_dir:.0f} s"
          f" (plant delay only);\n  L from c_pp also contains the decision lag of the controller,"
          f" 0..{T_S:.0f} s: {L - L_dir:+.0f} s here")
    if summ:
        print(f"program F3 summary: c_pp(max-min)={summ.get('c_pp')}  n_h={summ.get('n_h')}"
              f"  IAE={summ.get('IAE')}  E={summ.get('E')}")
    print(f"\nsettings for the next runs:")
    print(f"  method 6:  KR {0.25 * K_u:.3f}   and   KR {0.5 * K_u:.3f}")
    print(f"  methods 7, 9, 10:  KR {K_R:.3f}   TI {T_i:.0f}      method 8:  KI {K_R / T_i:.5f}")

    # ---- figure 3: limit cycle with the construction of L ----
    fig, (ax, axu) = plt.subplots(2, 1, sharex=True, figsize=(10, 6),
                                  gridspec_kw={"height_ratios": [3, 1]})
    t0 = t[0]
    tt = [x - t0 for x in t]
    ax.plot(tt, [r[1] for r in rows], color="#1f77b4", lw=1.4, label="c (NTC3, 120 mm)")
    ax.axhline(R_F3, color="k", ls="--", lw=1, label="r = 7 K")
    axu.step(tt, u, where="post", color="#d62728", lw=1.2)
    cy = cycles[min(1, n - 1)]                      # mark the second full cycle
    for t_sw, t_ext, c_ext, txt in ((cy["t_off"], cy["t_max"], cy["c_max"], "off"),
                                    (cy["t_on"], cy["t_min"], cy["c_min"], "on")):
        ax.axvline(t_sw - t0, color="#999999", ls=":", lw=1)
        ax.annotate("", xy=(t_ext - t0, c_ext), xytext=(t_sw - t0, c_ext),
                    arrowprops=dict(arrowstyle="<->", color="#2ca02c", lw=1.5))
        ax.text((t_sw + t_ext) / 2 - t0, c_ext + (0.04 if txt == "off" else -0.09),
                f"L = {t_ext - t_sw:.0f} s", ha="center", fontsize=9, color="#2ca02c")
    ax.annotate("", xy=(cy["t_max"] - t0 + 3, cy["c_max"]), xytext=(cy["t_max"] - t0 + 3, cy["c_min"]),
                arrowprops=dict(arrowstyle="<->", color="#9467bd"))
    ax.text(cy["t_max"] - t0 + 5, (cy["c_max"] + cy["c_min"]) / 2,
            f"c_pp = {cy['c_pp']:.2f} K", fontsize=8, color="#9467bd", va="center")
    k_end = min(3, n) - 1                           # show three full cycles, wide enough to read
    ax.set_xlim(cycles[0]["t_on"] - t0 - 10, cycles[k_end]["t_on"] + cycles[k_end]["period"] - t0 + 15)
    for t_sw, lab in ((cy["t_off"], "u -> 0"), (cy["t_on"], "u -> 1")):
        axu.axvline(t_sw - t0, color="#999999", ls=":", lw=1)
        axu.text(t_sw - t0 + 1, 0.5, lab, ha="left", va="center", fontsize=8, color="#555555")
    ax.set_title(f"Method 2, phase F3: mean of {n} cycles c_pp = {c_pp:.2f} K, T_osc = {T_osc:.0f} s, "
                 f"L = c_pp/(s_g+s_c) = {L:.1f} s", fontsize=10)
    ax.set_ylabel("c, rise above ambient [K]")
    axu.set_ylabel("u [-]")
    axu.set_xlabel("time from the start of F3 [s]")
    ax.legend(fontsize=8, loc="upper right")
    for a_ in (ax, axu):
        a_.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig("task3_limit_cycle.png", dpi=150)
    print("\nsaved task3_limit_cycle.png")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    main(sys.argv[1])
