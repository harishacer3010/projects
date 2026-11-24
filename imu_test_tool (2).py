#!/usr/bin/env python3
"""
IMU Test Tool for the STM32 + MPU6050 JSON-over-UART firmware.

Expected line format (one JSON object per line, as sent by main.c):
  {"t":<ms>,"roll":<deg>,"pitch":<deg>,"ax":<mg>,"ay":<mg>,"az":<mg>,
   "gx":<mdps>,"gy":<mdps>,"gz":<mdps>}

Features
  - Live plots: accel (g, |a|), gyro (deg/s), roll/pitch (deg)
  - Link health: rate, packets, parse errors, time gaps, max dt, kB/s
  - Static test: mean / sigma / peak-to-peak, gyro drift, |a| check, pass/fail
  - 6-position accelerometer check: offset and scale per axis
  - CSV export of the buffered samples

Install:  pip install pyserial numpy matplotlib
Run:      python imu_test_tool.py
"""
import csv
import datetime
import json
import threading
import time
import tkinter as tk
from collections import deque
from tkinter import filedialog, messagebox, ttk

import matplotlib
matplotlib.use("TkAgg")
import numpy as np
import serial
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from serial.tools import list_ports

FIELDS = ("t", "roll", "pitch", "ax", "ay", "az", "gx", "gy", "gz", "am")
IDX = {k: i for i, k in enumerate(FIELDS)}
MAX_SAMPLES = 60000
BAUDS = ("9600", "38400", "115200", "230400", "460800", "921600")

# pass/fail limits for the static test
LIM_ACC_SIGMA_G = 0.02
LIM_GYRO_SIGMA_DPS = 0.3
LIM_GYRO_MEAN_DPS = 0.5
LIM_ANGLE_SIGMA_DEG = 0.3
LIM_AM_RANGE = (0.95, 1.05)
LIM_MIN_RATE_HZ = 50.0

POSITIONS = [("Z up (flat)", "az"), ("Z down (flipped)", "az"),
             ("X up", "ax"), ("X down", "ax"),
             ("Y up", "ay"), ("Y down", "ay")]


class Link:
    """Serial reader thread + sample buffer. All shared state is guarded by self.lock."""

    def __init__(self):
        self.lock = threading.Lock()
        self.samples = deque(maxlen=MAX_SAMPLES)
        self.rx_times = deque()
        self.paused = False
        self.cap = None          # {"end": monotonic, "rows": [], "start": monotonic}
        self.ser = None
        self.thread = None
        self.running = False
        self.error = ""
        self.reset_stats()

    def reset_stats(self):
        with self.lock:
            self.pk = self.err = self.gaps = 0
            self.max_gap = 0
            self.last_t = None
            self.bytes = 0
            self.last_bad = ""
            self.rx_times.clear()

    def clear(self):
        with self.lock:
            self.samples.clear()
        self.reset_stats()

    def connect(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.running = True
        self.error = ""
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def disconnect(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def _run(self):
        buf = b""
        try:
            while self.running:
                chunk = self.ser.read(512)
                if not chunk:
                    continue
                self.bytes += len(chunk)
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    line = line.strip()
                    if line:
                        self._parse(line)
                if len(buf) > 4000:
                    buf = b""
        except Exception as e:  # port unplugged etc.
            self.error = f"serial error: {e}"
        self.running = False

    def _parse(self, line):
        try:
            o = json.loads(line)
            ax, ay, az = o["ax"] / 1000.0, o["ay"] / 1000.0, o["az"] / 1000.0
            gx, gy, gz = o["gx"] / 1000.0, o["gy"] / 1000.0, o["gz"] / 1000.0
            t = float(o["t"])
            roll = float(o.get("roll", 0.0))
            pitch = float(o.get("pitch", 0.0))
        except (ValueError, KeyError, TypeError):
            with self.lock:
                self.err += 1
                self.last_bad = line[:200].decode(errors="replace")
            return
        row = (t, roll, pitch, ax, ay, az, gx, gy, gz, (ax * ax + ay * ay + az * az) ** 0.5)
        now = time.monotonic()
        with self.lock:
            if self.last_t is not None:
                d = t - self.last_t
                if d > 50:
                    self.gaps += 1
                self.max_gap = max(self.max_gap, d)
            self.last_t = t
            self.pk += 1
            self.rx_times.append(now)
            if not self.paused:
                self.samples.append(row)
            if self.cap is not None:
                self.cap["rows"].append(row)

    # ---- capture helpers ----
    def begin_capture(self, seconds):
        with self.lock:
            now = time.monotonic()
            self.cap = {"start": now, "end": now + seconds, "dur": seconds, "rows": []}

    def capture_progress(self):
        with self.lock:
            if self.cap is None:
                return None
            return min(1.0, (time.monotonic() - self.cap["start"]) / self.cap["dur"])

    def take_capture_if_done(self):
        with self.lock:
            if self.cap is not None and time.monotonic() >= self.cap["end"]:
                rows, dur = self.cap["rows"], self.cap["dur"]
                self.cap = None
                return np.array(rows, dtype=float).reshape(-1, len(FIELDS)), dur
        return None

    def snapshot(self, n):
        with self.lock:
            data = list(self.samples)[-n:]
        return np.array(data, dtype=float).reshape(-1, len(FIELDS))

    def health(self):
        now = time.monotonic()
        with self.lock:
            while self.rx_times and self.rx_times[0] < now - 1.0:
                self.rx_times.popleft()
            return dict(rate=len(self.rx_times), pk=self.pk, err=self.err,
                        gaps=self.gaps, max_gap=self.max_gap, bytes=self.bytes,
                        bad=self.last_bad)


class DroneView:
    """Wireframe quadcopter driven by roll/pitch (sensor convention: roll about X, pitch about Y,
    R = Ry(pitch) @ Rx(roll), same as the firmware's atan2 formulas). Fixed ground grid for reference."""

    def __init__(self, parent):
        self.frame = ttk.LabelFrame(parent, text="Attitude model", padding=4)
        self.cv = tk.Canvas(self.frame, width=340, height=340, bg="#101418", highlightthickness=0)
        self.cv.pack(fill="both", expand=True)
        ctl = ttk.Frame(self.frame)
        ctl.pack(fill="x", pady=(4, 0))
        ttk.Label(ctl, text="Front axis").pack(side="left")
        self.front = tk.StringVar(value="+Y")
        cb = ttk.Combobox(ctl, textvariable=self.front, values=("+Y", "+X"), width=4, state="readonly")
        cb.pack(side="left", padx=4)
        self.inv_roll = tk.BooleanVar(value=False)
        self.inv_pitch = tk.BooleanVar(value=False)
        ttk.Checkbutton(ctl, text="Invert roll", variable=self.inv_roll).pack(side="left", padx=4)
        ttk.Checkbutton(ctl, text="Invert pitch", variable=self.inv_pitch).pack(side="left")
        self.az, self.el = np.radians(-30.0), np.radians(28.0)
        self.last = (0.0, 0.0)
        self._build_model()
        self.cv.bind("<Configure>", lambda e: self.draw(*self.last))
        cb.bind("<<ComboboxSelected>>", lambda e: self.draw(*self.last))

    def _build_model(self):
        # body frame, front = +Y, Z up. items: (points Nx3, outline, width, fill)
        m = 0.8
        th = np.linspace(0, 2 * np.pi, 28, endpoint=False)
        items = []
        for sx in (-1, 1):
            for sy in (-1, 1):
                items.append((np.array([[0, 0, 0], [sx * m, sy * m, 0]]), "#9aa5b1", 3, ""))
        for sx in (-1, 1):
            for sy in (-1, 1):
                col = "#ff6b6b" if sy > 0 else "#d8dee9"
                ring = np.stack([sx * m + 0.55 * np.cos(th), sy * m + 0.55 * np.sin(th),
                                 np.full_like(th, 0.12)], axis=1)
                items.append((ring, col, 2, ""))
                hub = np.stack([sx * m + 0.08 * np.cos(th), sy * m + 0.08 * np.sin(th),
                                np.full_like(th, 0.12)], axis=1)
                items.append((hub, col, 1, col))
        items.append((np.array([[-.3, -.5, 0], [.3, -.5, 0], [.3, .5, 0], [-.3, .5, 0]]), "#4aa3ff", 2, "#1d3a57"))
        items.append((np.array([[-.15, .5, .02], [.15, .5, .02], [0, .9, .02]]), "#ff6b6b", 2, "#ff6b6b"))
        self.items = items
        self.axes = [(np.array([[0, 0, 0], [1.1, 0, 0]]), "#ff6b6b"),
                     (np.array([[0, 0, 0], [0, 1.1, 0]]), "#4ade80"),
                     (np.array([[0, 0, 0], [0, 0, 1.1]]), "#60a5fa")]

    def _xy(self, pts, cx, cy, sc):
        ca, sa = np.cos(self.az), np.sin(self.az)
        x = pts[:, 0] * ca - pts[:, 1] * sa
        y = pts[:, 0] * sa + pts[:, 1] * ca
        sy = y * np.sin(self.el) + pts[:, 2] * np.cos(self.el)
        out = np.empty(2 * len(pts))
        out[0::2] = cx + x * sc
        out[1::2] = cy - sy * sc
        return out.tolist()

    def draw(self, roll, pitch):
        self.last = (roll, pitch)
        r = np.radians(-roll if self.inv_roll.get() else roll)
        p = np.radians(-pitch if self.inv_pitch.get() else pitch)
        cr, sr, cp, sp = np.cos(r), np.sin(r), np.cos(p), np.sin(p)
        R = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        if self.front.get() == "+X":      # rotate the model so its nose points along sensor +X
            R = R @ np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])
        W = self.cv.winfo_width() if self.cv.winfo_width() > 10 else 340
        H = self.cv.winfo_height() if self.cv.winfo_height() > 10 else 340
        cx, cy, sc = W / 2, H / 2 + 0.05 * H, min(W, H) / 5.2
        c = self.cv
        c.delete("all")
        for i in range(-2, 3):            # fixed horizon grid at z = -1.4
            a = np.array([[i, -2, -1.4], [i, 2, -1.4]])
            b = np.array([[-2, i, -1.4], [2, i, -1.4]])
            c.create_line(self._xy(a, cx, cy, sc), fill="#27303a")
            c.create_line(self._xy(b, cx, cy, sc), fill="#27303a")
        for pts, col, wd, fill in self.items:
            q = pts @ R.T
            xy = self._xy(q, cx, cy, sc)
            if len(pts) == 2:
                c.create_line(xy, fill=col, width=wd, capstyle="round")
            else:
                c.create_polygon(xy, outline=col, fill=fill, width=wd)
        for pts, col in self.axes:
            c.create_line(self._xy(pts @ R.T, cx, cy, sc), fill=col, width=2, arrow="last")
        c.create_text(8, 8, anchor="nw", fill="#d8dee9", font=("Consolas", 10, "bold"),
                      text=f"roll  {roll:+7.2f}\npitch {pitch:+7.2f}")
        c.create_text(W - 8, 8, anchor="ne", fill="#8a94a3", font=("Consolas", 8),
                      text="X red  Y green  Z blue\nred props = front")


class App:
    def __init__(self, root):
        self.root = root
        root.title("IMU Test Tool")
        root.geometry("1420x840")
        self.link = Link()
        self.cap_cb = None
        self.six_i = 0
        self.six_r = {}
        self.last_bytes = 0
        self.last_bt = time.monotonic()
        self.kbps = 0.0
        self.conn_t = 0.0
        self._build_ui()
        self._refresh_ports()
        self.root.protocol("WM_DELETE_WINDOW", self._close)
        self.root.after(50, self._tick)

    # ---------------- UI ----------------
    def _build_ui(self):
        top = ttk.Frame(self.root, padding=6)
        top.pack(fill="x")
        self.port_var = tk.StringVar()
        self.port_cb = ttk.Combobox(top, textvariable=self.port_var, width=28, state="readonly")
        self.port_cb.pack(side="left")
        ttk.Button(top, text="Refresh", command=self._refresh_ports).pack(side="left", padx=2)
        self.baud_var = tk.StringVar(value="115200")
        ttk.Combobox(top, textvariable=self.baud_var, values=BAUDS, width=8).pack(side="left", padx=4)
        self.btn_conn = ttk.Button(top, text="Connect", command=self._toggle_conn)
        self.btn_conn.pack(side="left", padx=4)
        self.status = tk.StringVar(value="disconnected")
        ttk.Label(top, textvariable=self.status).pack(side="left", padx=10)
        ttk.Button(top, text="Export CSV", command=self._export).pack(side="right")
        ttk.Button(top, text="Clear", command=self._clear).pack(side="right", padx=4)
        self.btn_pause = ttk.Button(top, text="Pause", command=self._pause)
        self.btn_pause.pack(side="right")
        self.win_var = tk.StringVar(value="600")
        ttk.Combobox(top, textvariable=self.win_var, values=("300", "600", "1500", "4000"),
                     width=6, state="readonly").pack(side="right", padx=4)
        ttk.Label(top, text="Window:").pack(side="right")

        body = ttk.Frame(self.root)
        body.pack(fill="both", expand=True)

        # plots
        self.fig = Figure(figsize=(6, 7), dpi=100, tight_layout=True)
        specs = [("Accel (g)", ("ax", "ay", "az", "am"), ("r", "g", "b", "y"), 2.0),
                 ("Gyro bias-corrected (deg/s)", ("gx", "gy", "gz"), ("r", "g", "b"), 10.0),
                 ("Attitude (deg)", ("roll", "pitch"), ("r", "b"), 10.0)]
        self.plots = []
        for i, (title, keys, cols, minr) in enumerate(specs):
            ax = self.fig.add_subplot(3, 1, i + 1)
            ax.set_title(title, fontsize=9, loc="left")
            ax.grid(True, alpha=0.3)
            lines = [ax.plot([], [], c, lw=1, label=k)[0] for k, c in zip(keys, cols)]
            ax.legend(loc="upper right", fontsize=7, ncol=len(keys))
            self.plots.append((ax, keys, lines, minr))
        self.canvas = FigureCanvasTkAgg(self.fig, master=body)
        self.canvas.get_tk_widget().pack(side="left", fill="both", expand=True)
        self.drone = DroneView(body)
        self.drone.frame.pack(side="left", fill="y", padx=4)

        # right panel
        right = ttk.Frame(body, padding=6, width=420)
        right.pack(side="right", fill="y")

        lf = ttk.LabelFrame(right, text="Live", padding=4)
        lf.pack(fill="x")
        self.live = {}
        for i, k in enumerate(("ax", "ay", "az", "gx", "gy", "gz", "roll", "pitch", "am")):
            v = tk.StringVar(value="-")
            self.live[k] = v
            ttk.Label(lf, text="|a| g" if k == "am" else k, width=6).grid(row=i // 3 * 2, column=i % 3)
            ttk.Label(lf, textvariable=v, font=("Consolas", 11, "bold"), width=9).grid(row=i // 3 * 2 + 1, column=i % 3)

        hf = ttk.LabelFrame(right, text="Link health", padding=4)
        hf.pack(fill="x", pady=6)
        self.health = {}
        names = [("rate", "rate Hz"), ("pk", "packets"), ("err", "parse err"),
                 ("gaps", "gaps>50ms"), ("max_gap", "max dt ms"), ("kbps", "kB/s")]
        for i, (k, lab) in enumerate(names):
            v = tk.StringVar(value="-")
            self.health[k] = v
            ttk.Label(hf, text=lab, width=10).grid(row=i // 3 * 2, column=i % 3)
            ttk.Label(hf, textvariable=v, font=("Consolas", 11, "bold"), width=9).grid(row=i // 3 * 2 + 1, column=i % 3)

        self.hint = tk.StringVar()
        ttk.Label(right, textvariable=self.hint, wraplength=400, foreground="#c0392b",
                  justify="left").pack(fill="x")

        sf = ttk.LabelFrame(right, text="Static test (keep IMU still)", padding=4)
        sf.pack(fill="x")
        row = ttk.Frame(sf)
        row.pack(fill="x")
        self.sdur = tk.StringVar(value="10")
        ttk.Combobox(row, textvariable=self.sdur, values=("5", "10", "30", "60"), width=5,
                     state="readonly").pack(side="left")
        ttk.Label(row, text="s").pack(side="left", padx=(2, 8))
        ttk.Button(row, text="Run static test", command=self._run_static).pack(side="left")
        self.pbar = ttk.Progressbar(sf, maximum=1.0)
        self.pbar.pack(fill="x", pady=4)
        self.msg = tk.StringVar()
        ttk.Label(sf, textvariable=self.msg).pack(anchor="w")

        self.out = tk.Text(right, height=26, width=52, font=("Consolas", 9), bg="#101418", fg="#d8dee9")
        self.out.tag_config("ok", foreground="#3ecf8e")
        self.out.tag_config("bad", foreground="#ef5350")
        self.out.tag_config("hd", foreground="#4aa3ff")
        self.out.pack(fill="both", expand=True, pady=6)

        xf = ttk.LabelFrame(right, text="6-position accel check", padding=4)
        xf.pack(fill="x")
        ttk.Button(xf, text="Capture position", command=self._six_capture).pack(side="left")
        ttk.Button(xf, text="Reset", command=self._six_reset).pack(side="left", padx=4)
        self._six_prompt()

    # ---------------- actions ----------------
    def _refresh_ports(self):
        ports = [p.device for p in list_ports.comports()]
        self.port_cb["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def _toggle_conn(self):
        if self.link.running:
            self.link.disconnect()
            self.btn_conn.config(text="Connect")
            self.status.set("disconnected")
            return
        port = self.port_var.get()
        if not port:
            messagebox.showwarning("IMU Test Tool", "No serial port selected.")
            return
        try:
            self.link.reset_stats()
            self.link.connect(port, int(self.baud_var.get()))
        except Exception as e:
            messagebox.showerror("IMU Test Tool", f"Could not open {port}:\n{e}")
            return
        self.conn_t = time.monotonic()
        self.btn_conn.config(text="Disconnect")
        self.status.set(f"connected {port} @ {self.baud_var.get()}")

    def _pause(self):
        self.link.paused = not self.link.paused
        self.btn_pause.config(text="Resume" if self.link.paused else "Pause")

    def _clear(self):
        self.link.clear()

    def _export(self):
        data = self.link.snapshot(MAX_SAMPLES)
        if not len(data):
            messagebox.showinfo("IMU Test Tool", "No samples to export.")
            return
        name = "imu_log_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv"
        path = filedialog.asksaveasfilename(defaultextension=".csv", initialfile=name,
                                            filetypes=[("CSV", "*.csv")])
        if not path:
            return
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(FIELDS[:-1])
            for r in data:
                w.writerow(r[:-1])
        self.msg.set(f"Saved {len(data)} samples")

    def _start_capture(self, seconds, cb):
        if self.link.cap is not None:
            return
        if not len(self.link.snapshot(1)):
            self.msg.set("No data - connect first.")
            return
        self.cap_cb = cb
        self.link.begin_capture(seconds)

    # ---- static test ----
    def _run_static(self):
        self.out.delete("1.0", "end")
        self.msg.set("Capturing... keep the IMU completely still")
        self._start_capture(float(self.sdur.get()), self._finish_static)

    def _finish_static(self, a, dur):
        self.msg.set(f"Done: {len(a)} samples")
        if len(a) < 10:
            self.out.insert("end", "Not enough samples captured.\n", "bad")
            return
        w = self.out.insert
        w("end", f"{'ch':<6}{'mean':>10}{'sigma':>9}{'p-p':>9}{'drift deg':>11}  result\n", "hd")
        all_ok = True
        tests = [("ax", LIM_ACC_SIGMA_G, False), ("ay", LIM_ACC_SIGMA_G, False), ("az", LIM_ACC_SIGMA_G, False),
                 ("gx", LIM_GYRO_SIGMA_DPS, True), ("gy", LIM_GYRO_SIGMA_DPS, True), ("gz", LIM_GYRO_SIGMA_DPS, True),
                 ("roll", LIM_ANGLE_SIGMA_DEG, False), ("pitch", LIM_ANGLE_SIGMA_DEG, False)]
        for k, lim, is_gyro in tests:
            v = a[:, IDX[k]]
            m, sd, pp = v.mean(), v.std(), np.ptp(v)
            ok = sd < lim and (not is_gyro or abs(m) < LIM_GYRO_MEAN_DPS)
            all_ok &= ok
            drift = f"{m * dur:+11.2f}" if is_gyro else f"{'-':>11}"
            w("end", f"{k:<6}{m:+10.3f}{sd:9.3f}{pp:9.3f}{drift}  ")
            w("end", ("PASS" if ok else "FAIL") + "\n", "ok" if ok else "bad")
        am = a[:, IDX["am"]]
        ok = LIM_AM_RANGE[0] < am.mean() < LIM_AM_RANGE[1]
        all_ok &= ok
        w("end", f"{'|a|':<6}{am.mean():10.3f}{am.std():9.3f}{np.ptp(am):9.3f}{'-':>11}  ")
        w("end", ("PASS" if ok else "FAIL") + "\n", "ok" if ok else "bad")
        rate = len(a) / dur
        ok = rate >= LIM_MIN_RATE_HZ
        all_ok &= ok
        w("end", f"{'rate':<6}{rate:8.1f}Hz{'':>29}")
        w("end", ("PASS" if ok else "FAIL") + "\n", "ok" if ok else "bad")
        t = a[:, IDX["t"]]
        gaps = int(np.sum(np.diff(t) > 50))
        w("end", f"time gaps >50 ms: {gaps}, max dt: {np.max(np.diff(t)):.0f} ms\n")
        w("end", f"\nOverall: {'PASS' if all_ok else 'FAIL'}\n", "ok" if all_ok else "bad")
        w("end", (f"Limits: gyro |mean|<{LIM_GYRO_MEAN_DPS} dps, gyro sigma<{LIM_GYRO_SIGMA_DPS} dps,\n"
                  f"accel sigma<{LIM_ACC_SIGMA_G * 1000:.0f} mg, angle sigma<{LIM_ANGLE_SIGMA_DEG} deg,\n"
                  f"|a| {LIM_AM_RANGE[0]}-{LIM_AM_RANGE[1]} g, rate>={LIM_MIN_RATE_HZ:.0f} Hz\n"))

    # ---- 6-position ----
    def _six_prompt(self):
        if self.six_i < 6:
            self.msg.set(f"6-pos: place IMU {POSITIONS[self.six_i][0]}, hold still, press Capture")

    def _six_reset(self):
        self.six_i, self.six_r = 0, {}
        self.out.delete("1.0", "end")
        self._six_prompt()

    def _six_capture(self):
        if self.six_i >= 6:
            return
        self.msg.set(f"Capturing {POSITIONS[self.six_i][0]} (2 s)...")
        self._start_capture(2.0, self._six_done)

    def _six_done(self, a, dur):
        if len(a) < 10:
            self.msg.set("Not enough samples - try again")
            return
        k = POSITIONS[self.six_i][1]
        self.six_r[self.six_i] = float(a[:, IDX[k]].mean())
        self.six_i += 1
        self.out.delete("1.0", "end")
        self.out.insert("end", f"{'position':<20}{'measured g':>11}\n", "hd")
        for i, (name, _) in enumerate(POSITIONS):
            if i in self.six_r:
                self.out.insert("end", f"{name:<20}{self.six_r[i]:+11.3f}\n")
        if self.six_i < 6:
            self._six_prompt()
            return
        self.msg.set("6-position check complete")
        self.out.insert("end", f"\n{'axis':<6}{'offset g':>10}{'scale':>9}  result\n", "hd")
        for name, p in (("ax", 2), ("ay", 4), ("az", 0)):
            pos, neg = self.six_r[p], self.six_r[p + 1]
            off, sc = (pos + neg) / 2, (pos - neg) / 2
            ok = abs(off) < 0.05 and abs(sc - 1) < 0.03
            self.out.insert("end", f"{name:<6}{off:+10.3f}{sc:9.3f}  ")
            self.out.insert("end", ("PASS" if ok else "FAIL") + "\n", "ok" if ok else "bad")
        self.out.insert("end", "\nLimits: |offset|<0.05 g, scale 1.00 +/- 0.03\n")

    # ---------------- periodic update ----------------
    def _tick(self):
        try:
            self._update()
        finally:
            self.root.after(50, self._tick)

    def _update(self):
        if self.link.error:
            self.status.set(self.link.error)
            self.btn_conn.config(text="Connect")
            self.link.error = ""
        elif not self.link.running and self.btn_conn["text"] == "Disconnect":
            self.btn_conn.config(text="Connect")
            self.status.set("disconnected")

        h = self.link.health()
        now = time.monotonic()
        if self.link.running and h["pk"] == 0 and now - self.conn_t > 2.0:
            if h["bytes"] == 0:
                self.hint.set("No bytes received: check COM port, baud rate, STM32 TX -> adapter RX, common GND.")
            else:
                self.hint.set("Bytes arriving but no valid JSON line. Last bad line:\n" + h["bad"])
        else:
            self.hint.set("")
        if now - self.last_bt >= 1.0:
            self.kbps = (h["bytes"] - self.last_bytes) / 1000.0 / (now - self.last_bt)
            self.last_bytes, self.last_bt = h["bytes"], now
        for k in ("rate", "pk", "err", "gaps", "max_gap"):
            self.health[k].set(f"{h[k]:.0f}")
        self.health["kbps"].set(f"{self.kbps:.1f}")

        prog = self.link.capture_progress()
        self.pbar["value"] = prog if prog is not None else self.pbar["value"]
        done = self.link.take_capture_if_done()
        if done is not None and self.cap_cb:
            self.pbar["value"] = 1.0
            cb, self.cap_cb = self.cap_cb, None
            cb(*done)

        n = int(self.win_var.get())
        d = self.link.snapshot(n)
        if not len(d):
            return
        last = d[-1]
        self.drone.draw(float(last[IDX["roll"]]), float(last[IDX["pitch"]]))
        for k, v in self.live.items():
            v.set(f"{last[IDX[k]]:+.3f}" if k in ("am", "ax", "ay", "az") else f"{last[IDX[k]]:+.2f}")
        x = np.arange(len(d))
        for ax, keys, lines, minr in self.plots:
            lo, hi = np.inf, -np.inf
            for k, ln in zip(keys, lines):
                y = d[:, IDX[k]]
                ln.set_data(x, y)
                lo, hi = min(lo, y.min()), max(hi, y.max())
            c, r = (lo + hi) / 2, max((hi - lo) / 2 * 1.1, minr)
            ax.set_xlim(0, max(len(d) - 1, 1))
            ax.set_ylim(c - r, c + r)
        self.canvas.draw_idle()

    def _close(self):
        self.link.disconnect()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    App(root)
    root.mainloop()
