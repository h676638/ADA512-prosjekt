#!/usr/bin/env python3
"""
ADA512 LAB 2 - logger with live graph (use this instead of PuTTY/CoolTerm).

Install once:
    pip install pyserial matplotlib

LIVE LOGGING
    python lab2_logger.py                 finds the Arduino by itself
    python lab2_logger.py --port COM5     or name the port (Mac: /dev/cu.usbmodem...)

    Type the lab commands in this window and press Enter:
        A   T 22.41   M 2   KR 0.4   TI 90   KI 0.0045   H 0.5   ?   R   S
    Type  quit  to close the program.

    What gets saved (in the folder you run it from):
        session_<date>_<time>.txt   everything the Arduino sent, plus your commands
        run_<time>_m<method>.csv    one file per run, from "# START" to "# END"
        run_<time>_m<method>.png    the graph of that run, saved at "# END"
        summaries.csv               every "# F1/F2/F3" summary line as a table row

PLOTTING SAVED RUNS (for the report)
    python lab2_logger.py --plot run_1012_m2.csv
        one run: c and r, u and I_term, and T1-T3, with the phases marked
    python lab2_logger.py --plot run_*_m*.csv
        several runs: c in phase F1 on one axis (report figure 1)
    python lab2_logger.py --plot run_*_m9.csv run_*_m10.csv --phases F1,F2,F3
        methods 9 and 10 with I_term underneath (report figure 2)
"""

import argparse
import csv
import glob
import math
import os
import sys
import threading
import time
from datetime import datetime

import matplotlib.pyplot as plt

COLS = ["t_s", "phase", "r", "T1", "T2", "T3", "c", "e", "u", "heater", "I_term"]
SUMMARY_COLS = ["run_file", "phase", "method", "r", "t_phase", "e_ss", "IAE", "ISE",
                "TV", "n_on", "n_h", "t5", "c_p", "c_pp", "E"]
PHASE_COLOURS = {"AP": "#eeeeee", "F0": "#f5f5f5", "F1": "#e3f0fb",
                 "F2": "#fdeee0", "F3": "#e6f5e6"}


# ---------------------------------------------------------------- parsing
def parse_data(line):
    """One data line -> dict, or None if the line is not a data line."""
    parts = line.split(",")
    if len(parts) != len(COLS):
        return None
    try:
        float(parts[0])
    except ValueError:
        return None                       # e.g. the header line "t_s,phase,..."
    row = {}
    for key, val in zip(COLS, parts):
        if key == "phase":
            row[key] = val.strip()
        else:
            try:
                row[key] = float(val)
            except ValueError:
                row[key] = math.nan       # "invalid" sensor
    return row


def parse_keyvals(line):
    """'# F1,method=2,r=10.00,...' -> ('F1', {'method': '2', 'r': '10.00', ...})"""
    parts = line.lstrip("#").strip().split(",")
    head, kv = parts[0].strip(), {}
    for p in parts[1:]:
        if "=" in p:
            k, v = p.split("=", 1)
            kv[k.strip()] = v.strip()
    return head, kv


def read_log(path):
    """Read a saved run file (or session log) -> (rows, settings from # START)."""
    rows, settings = [], {}
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if line.startswith("# START"):
                rows, (_, settings) = [], parse_keyvals(line)   # keep only the last run
            elif not line.startswith("#"):
                row = parse_data(line)
                if row:
                    rows.append(row)
    return rows, settings


# ---------------------------------------------------------------- drawing
def make_figure():
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(11, 8),
                             gridspec_kw={"height_ratios": [3, 1.6, 1.6]})
    return fig, axes


def shade_phases(ax, t, phases, label=True):
    """Coloured background (and a label) for each phase."""
    if not t:
        return
    start = 0
    for i in range(1, len(t) + 1):
        if i == len(t) or phases[i] != phases[start]:
            ph = phases[start]
            ax.axvspan(t[start], t[i - 1] + 1, color=PHASE_COLOURS.get(ph, "#ffffff"), zorder=0)
            if label:
                ax.text((t[start] + t[i - 1]) / 2, 1.0, ph, transform=ax.get_xaxis_transform(),
                        ha="center", va="bottom", fontsize=9, color="#555555")
            start = i


def draw_run(fig, axes, rows, title, layout=True):
    ax_c, ax_u, ax_T = axes
    for ax in axes:
        ax.clear()
    if rows:
        t = [r["t_s"] for r in rows]
        ph = [r["phase"] for r in rows]
        for ax in axes:
            shade_phases(ax, t, ph, label=(ax is ax_c))
        ax_c.plot(t, [r["r"] for r in rows], "k--", lw=1.2, label="r (set point)")
        ax_c.plot(t, [r["c"] for r in rows], color="#1f77b4", lw=1.6, label="c (NTC3 rise)")
        ax_u.step(t, [r["u"] for r in rows], where="post", color="#d62728", lw=1.2, label="u")
        if any(abs(r["I_term"]) > 1e-9 for r in rows if not math.isnan(r["I_term"])):
            ax_u.plot(t, [r["I_term"] for r in rows], color="#9467bd", lw=1.2,
                      ls="--", label="I_term")
            ax_u.axhline(1.0, color="#999999", lw=0.8, ls=":")
        for key, col in (("T1", "#ff7f0e"), ("T2", "#2ca02c"), ("T3", "#1f77b4")):
            ax_T.plot(t, [r[key] for r in rows], color=col, lw=1.2, label=key)
        last = rows[-1]
        title += f"   |   t={last['t_s']:.0f} s  {last['phase']}  c={last['c']:.2f} K  " \
                 f"r={last['r']:.1f} K  u={last['u']:.2f}"
    ax_c.set_ylabel("rise above ambient [K]")
    ax_u.set_ylabel("u [-]")
    ax_T.set_ylabel("temperature [°C]")
    ax_T.set_xlabel("time since R [s]")
    for ax in axes:
        ax.grid(True, alpha=0.3)
        if ax.get_legend_handles_labels()[0]:
            ax.legend(loc="upper left", fontsize=8)
    fig.suptitle(title, fontsize=10)
    if layout:
        fig.tight_layout()


def run_label(settings, path):
    m = settings.get("method", "?")
    label = f"method {m}"
    if m in ("6", "7", "9", "10"):
        label += f", KR={float(settings.get('KR', 0)):.3g}"
    if m in ("9", "10"):
        label += f", TI={float(settings.get('TI', 0)):.3g}"
    if m == "8":
        label += f", KI={float(settings.get('KI', 0)):.3g}"
    if m == "3":
        label += f", h={float(settings.get('H', 0)):.2g}"
    return label


def plot_files(paths, phases):
    files = []
    for p in paths:                        # Windows does not expand * by itself
        files += sorted(glob.glob(p)) or [p]
    if not files:
        sys.exit("no files")

    if len(files) == 1:                    # one run: full picture
        rows, settings = read_log(files[0])
        fig, axes = make_figure()
        draw_run(fig, axes, rows, f"{os.path.basename(files[0])} - {run_label(settings, files[0])}")
        out = os.path.splitext(files[0])[0] + ".png"
    else:                                  # several runs: overlay of the chosen phases
        want = [p.strip() for p in phases.split(",")]
        runs = []
        for f in files:
            rows, settings = read_log(f)
            sel = [r for r in rows if r["phase"] in want]
            if sel:
                runs.append((f, settings, sel))
        has_i = any(abs(r["I_term"]) > 1e-9 for _, _, sel in runs for r in sel
                    if not math.isnan(r["I_term"]))
        n = 2 if has_i else 1
        fig, axes = plt.subplots(n, 1, sharex=True, figsize=(11, 7 if has_i else 5.5),
                                 gridspec_kw={"height_ratios": [3, 1.5][:n]})
        axes = axes if n > 1 else [axes]
        for f, settings, sel in runs:
            t0 = sel[0]["t_s"]
            t = [r["t_s"] - t0 for r in sel]
            line, = axes[0].plot(t, [r["c"] for r in sel], lw=1.5, label=run_label(settings, f))
            if has_i:
                axes[1].plot(t, [r["I_term"] for r in sel], lw=1.2, color=line.get_color())
        f0, s0, sel0 = runs[0]
        t00 = sel0[0]["t_s"]
        axes[0].plot([r["t_s"] - t00 for r in sel0], [r["r"] for r in sel0], "k--", lw=1.2,
                     label="r (set point)")
        axes[0].set_ylabel("c, rise above ambient [K]")
        axes[0].legend(fontsize=8)
        axes[0].set_title(f"Phase {', '.join(want)} - c(t) of {len(runs)} runs")
        if has_i:
            axes[1].axhline(1.0, color="#999999", lw=0.8, ls=":")
            axes[1].set_ylabel("I_term = KR·I/TI [-]")
        axes[-1].set_xlabel(f"time from the start of {want[0]} [s]")
        for ax in axes:
            ax.grid(True, alpha=0.3)
        fig.tight_layout()
        out = f"compare_{'_'.join(want)}.png"
    fig.savefig(out, dpi=150)
    print(f"saved {out}")
    plt.show()


# ---------------------------------------------------------------- live logging
class Logger:
    def __init__(self, ser):
        self.ser = ser
        self.running = True
        self.lock = threading.Lock()
        self.rows = []
        self.title = "waiting for R ..."
        self.dirty = True
        self.png_pending = None
        self.in_run = False
        self.run_file = None
        self.run_base = None
        self.last_status = -1e9
        self.last_msg = ""
        self.bytes_in = 0
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session = open(f"session_{stamp}.txt", "w", encoding="utf-8")
        print(f"logging everything to session_{stamp}.txt")

    def write_session(self, text):
        with self.lock:
            self.session.write(text + "\n")
            self.session.flush()

    def handle(self, line):
        self.write_session(line)

        if line.startswith("# START"):
            _, settings = parse_keyvals(line)
            self.run_base = f"run_{datetime.now().strftime('%H%M%S')}_m{settings.get('method', 'x')}"
            self.run_file = open(self.run_base + ".csv", "w", encoding="utf-8")
            with self.lock:
                self.rows = []
                self.title = f"{self.run_base} - {run_label(settings, '')}"
                self.dirty = True
            self.in_run = True
            print(f"\n=== new run -> {self.run_base}.csv ===")

        if self.run_file:
            self.run_file.write(line + "\n")
            self.run_file.flush()

        if line.startswith("#"):
            print(line)
            with self.lock:
                self.last_msg = line
                self.dirty = True
            head, kv = parse_keyvals(line)
            if head in ("F1", "F2", "F3") and "method" in kv:
                self.save_summary(head, kv)
            if line.startswith("# END") or line.startswith("# STOP"):
                if self.run_file:
                    self.run_file.close()
                    self.run_file = None
                    with self.lock:
                        self.png_pending = self.run_base + ".png"
                        self.dirty = True
                    print(f"=== run finished, saved {self.run_base}.csv ===")
                self.in_run = False
            return

        row = parse_data(line)
        if row:
            with self.lock:
                self.rows.append(row)
                self.dirty = True
            if row["t_s"] - self.last_status >= 10:      # short status every 10 s
                self.last_status = row["t_s"]
                print(f"  t={row['t_s']:6.0f} s  {row['phase']:>2}  r={row['r']:5.2f}  "
                      f"c={row['c']:6.3f}  u={row['u']:.3f}  I_term={row['I_term']:.3f}")
        elif not line.startswith("t_s,"):
            print(line)

    def save_summary(self, head, kv):
        new = not os.path.exists("summaries.csv")
        with open("summaries.csv", "a", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            if new:
                w.writerow(SUMMARY_COLS)
            vals = {"run_file": (self.run_base or "") + ".csv", "phase": head, **kv}
            w.writerow([vals.get(c, "") for c in SUMMARY_COLS])

    def reader(self):
        import serial
        buf = b""
        while self.running:
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
            except serial.SerialException as exc:
                print(f"\n!! lost the serial port: {exc}")
                self.running = False
                break
            self.bytes_in += len(chunk)
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("utf-8", errors="replace").strip()
                if line:
                    try:
                        self.handle(line)
                    except Exception:            # never let one bad line stop the logging
                        import traceback
                        traceback.print_exc()

    def keyboard(self):
        while self.running:
            try:
                cmd = input()
            except (EOFError, KeyboardInterrupt):
                break
            if not self.send(cmd):
                break

    def send(self, cmd):
        """Send one command to the Arduino. Returns False on 'quit'."""
        cmd = cmd.strip()
        if cmd.lower() in ("quit", "exit"):
            self.running = False
            return False
        if cmd:
            self.ser.write((cmd + "\n").encode())
            self.write_session("> " + cmd)
            print(f"> {cmd}")
        return True


def find_port():
    from serial.tools import list_ports
    ports = list(list_ports.comports())
    arduino = [p for p in ports if p.vid == 0x2341 or "arduino" in (p.description or "").lower()]
    pick = arduino or ports
    if len(pick) == 1:
        return pick[0].device
    if not pick:
        sys.exit("No serial port found. Is the Arduino plugged in? (and is the IDE Serial Monitor closed?)")
    print("Several ports found:")
    for i, p in enumerate(pick):
        print(f"  {i}: {p.device}  {p.description}")
    return pick[int(input("Which number? "))].device


def live(port, baud):
    import serial
    port = port or find_port()
    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout = port, baud, 0.5
    ser.dtr = True                      # like PuTTY / Serial Monitor: the board only
    ser.rts = True                      # sends data when DTR is set
    ser.open()
    print(f"connected to {port} at {baud} baud")
    print("Type commands (A, T 22.41, M 2, KR 0.4, TI 90, H 0.5, ?, R, S) in the 'Command' box\n"
          "at the bottom of the graph window and press Enter (typing here in the terminal works too).\n"
          "'quit' closes. NB: closing this program stops the LOGGING, not the run on the Arduino - send S first.\n")

    log = Logger(ser)
    threading.Thread(target=log.reader, daemon=True).start()
    threading.Thread(target=log.keyboard, daemon=True).start()

    def handshake():
        """Ask the board for its settings, so you see at once that data comes back."""
        time.sleep(2.0)
        log.send("?")
        time.sleep(3.0)
        if log.bytes_in == 0:
            print("\n!! NO REPLY from the Arduino. Check: right port? PuTTY / Serial Monitor closed?\n"
                  "   Try unplugging USB, plug back in, and start this program again.\n")
            with log.lock:
                log.last_msg = "NO REPLY from the Arduino - see the terminal"
                log.dirty = True
    threading.Thread(target=handshake, daemon=True).start()

    # the graph window must not grab the keyboard focus on every redraw,
    # and single keys like 'q' or 's' must not close/save the window
    plt.rcParams["figure.raise_window"] = False
    for key in ("keymap.quit", "keymap.save", "keymap.fullscreen", "keymap.grid",
                "keymap.yscale", "keymap.xscale", "keymap.home", "keymap.back",
                "keymap.forward", "keymap.pan", "keymap.zoom"):
        plt.rcParams[key] = []

    from matplotlib.widgets import TextBox
    fig, axes = make_figure()
    fig.canvas.manager.set_window_title("ADA512 LAB 2 - live")
    fig.subplots_adjust(left=0.08, right=0.98, top=0.92, bottom=0.15, hspace=0.22)
    box_ax = fig.add_axes([0.14, 0.02, 0.30, 0.045])
    box = TextBox(box_ax, "Command: ")
    status = fig.text(0.46, 0.042, "click the box, type a command, press Enter",
                      fontsize=8, va="center", color="#444444")

    def on_submit(text):
        if not text.strip():
            return                        # also catches the set_val("") below
        if not log.send(text):
            plt.close(fig)
            return
        box.set_val("")

    box.on_submit(on_submit)
    plt.show(block=False)

    while log.running and plt.fignum_exists(fig.number):
        if log.dirty:
            with log.lock:
                rows, title, msg = list(log.rows), log.title, log.last_msg
                log.dirty = False
            draw_run(fig, axes, rows, title, layout=False)
            if msg:
                status.set_text("last: " + (msg if len(msg) < 95 else msg[:92] + "..."))
            if log.png_pending:
                box_ax.set_visible(False)  # the saved picture without the command box
                status.set_visible(False)
                fig.savefig(log.png_pending, dpi=150)
                box_ax.set_visible(True)
                status.set_visible(True)
                print(f"saved {log.png_pending}")
                log.png_pending = None
            fig.canvas.draw_idle()
        fig.canvas.start_event_loop(0.5)   # keeps the window alive without stealing focus

    if log.in_run:
        print("!! the run on the Arduino is still going - logging stopped. Send S if you want it off.")
    log.running = False
    if log.run_file:
        log.run_file.close()
    log.session.close()
    ser.close()
    print("closed.")
    sys.stdout.flush()
    os._exit(0)                          # do not wait for the keyboard thread


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="ADA512 LAB 2 logger and plotter")
    ap.add_argument("--port", help="serial port, e.g. COM5 or /dev/cu.usbmodem1101")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--plot", nargs="+", metavar="FILE", help="plot saved run file(s) instead of logging")
    ap.add_argument("--phases", default="F1", help="phases to overlay with several files, e.g. F1 or F1,F2,F3")
    a = ap.parse_args()
    if a.plot:
        plot_files(a.plot, a.phases)
    else:
        live(a.port, a.baud)
