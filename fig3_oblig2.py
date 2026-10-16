"""Figure 3: c(t) in F1 for all methods. Usage: python3 fig3.py <dir with run csvs> <out.png>"""
import sys, io, os, glob
import pandas as pd, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

RUNS = [  # (label, filename, colour, linestyle)
    ("1 open loop",         "run_173419_m1.csv",  "#7f7f7f", "--"),
    ("2 on-off",            "run_113740_m2.csv",  "#d62728", "-"),
    ("3 on-off + hyst.",    "run_104549_m3.csv",  "#ff7f0e", "-"),
    ("6 P (0.25·K_u)",      "run_121819_m6.csv",  "#8c564b", "--"),
    ("7 P + reset",         "run_123232_m7.csv",  "#2ca02c", "-"),
    ("8 integral",          "run_134728_m8.csv",  "#9467bd", "-."),
    ("9 PI",                "run_111746_m9.csv",  "#1f77b4", "-"),
    ("10 PI + anti-windup", "run_120127_m10.csv", "#17becf", "-"),
]

def load(path):
    lines = [l for l in open(path) if l[0].isdigit() or l.startswith("t_s")]
    df = pd.read_csv(io.StringIO("".join(lines)))
    f1 = df[df.phase == "F1"].copy()
    f1["tp"] = f1.t_s - f1.t_s.iloc[0]
    return f1

src, out = sys.argv[1], sys.argv[2]
fig, ax = plt.subplots(figsize=(8.0, 4.6), dpi=200)
ax.axhspan(9.5, 10.5, color="#000000", alpha=0.08, lw=0, label="±0.5 K band")
ax.axhline(10, color="black", lw=0.8)
for label, fn, col, ls in RUNS:
    hits = glob.glob(os.path.join(src, "**", fn), recursive=True)
    if not hits:
        print("missing", fn); continue
    f1 = load(hits[0])
    ax.plot(f1.tp, f1.c, color=col, ls=ls, lw=1.3, label=label)
ax.set_xlim(0, 400); ax.set_ylim(4.5, 13)
ax.set_xlabel("time from start of F1 [s]"); ax.set_ylabel("c = T3 − T_amb [K]")
ax.grid(alpha=0.3, lw=0.5)
ax.legend(ncol=3, fontsize=7.5, loc="lower right", frameon=True)
fig.tight_layout(); fig.savefig(out)
print("saved", out)
