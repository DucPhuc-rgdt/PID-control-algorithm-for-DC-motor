"""
PID Motor Control Dashboard - Sine Test

UART protocol:
  - RX: "RPM:<val>,SP:<val>,KP:<val>,KI:<val>,KD:<val>\n"
  - TX: "SP:<value>\n"
"""

import collections
import csv
import math
import os
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import matplotlib
import serial
import serial.tools.list_ports

matplotlib.use("TkAgg")
import matplotlib.animation as animation
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


MAX_POINTS = 2000
BAUD_RATE = 115200
UPDATE_MS = 50
SINE_SEND_MS = 50
SINE_ANALYSIS_PERIODS = 3.0
MIN_ANALYSIS_SAMPLES = 30


class SineMetrics:
    def __init__(self):
        self.reset()

    def reset(self):
        self.phase_lag_deg = None
        self.amplitude_ratio_pct = None
        self.setpoint_amp = None
        self.response_amp = None

    def update(self, times, setpoints, rpms, freq_hz):
        if freq_hz is None or freq_hz <= 0:
            self.reset()
            return self._result()

        if len(times) < MIN_ANALYSIS_SAMPLES:
            return self._result()

        period = 1.0 / freq_hz
        window_sec = max(period * SINE_ANALYSIS_PERIODS, 2.0)

        start_idx = len(times) - 1
        while start_idx > 0 and (times[-1] - times[start_idx - 1]) <= window_sec:
            start_idx -= 1

        ts = list(times)[start_idx:]
        ss = list(setpoints)[start_idx:]
        ys = list(rpms)[start_idx:]
        if len(ts) < MIN_ANALYSIS_SAMPLES:
            return self._result()

        sp_amp, sp_phase = self._fit_sine(ts, ss, freq_hz)
        rpm_amp, rpm_phase = self._fit_sine(ts, ys, freq_hz)
        if sp_amp is None or rpm_amp is None or sp_amp < 1e-6:
            return self._result()

        phase_lag = math.degrees(sp_phase - rpm_phase)
        phase_lag = ((phase_lag + 180.0) % 360.0) - 180.0

        self.phase_lag_deg = phase_lag
        self.amplitude_ratio_pct = rpm_amp / sp_amp * 100.0
        self.setpoint_amp = sp_amp
        self.response_amp = rpm_amp
        return self._result()

    def _fit_sine(self, ts, values, freq_hz):
        n = len(ts)
        if n == 0:
            return None, None

        omega = 2.0 * math.pi * freq_hz
        avg = sum(values) / n
        sin_part = 0.0
        cos_part = 0.0

        for t, val in zip(ts, values):
            centered = val - avg
            angle = omega * t
            sin_part += centered * math.sin(angle)
            cos_part += centered * math.cos(angle)

        sin_part *= 2.0 / n
        cos_part *= 2.0 / n
        amplitude = math.hypot(sin_part, cos_part)
        phase = math.atan2(cos_part, sin_part)
        return amplitude, phase

    def _result(self):
        return {
            "phase_lag_deg": self.phase_lag_deg,
            "amplitude_ratio_pct": self.amplitude_ratio_pct,
            "setpoint_amp": self.setpoint_amp,
            "response_amp": self.response_amp,
        }


class PIDDashboard:
    def __init__(self, root):
        self.root = root
        self.root.title("PID Motor Control Dashboard - Sine Test")
        self.root.configure(bg="#1a1a2e")
        self.root.resizable(True, True)

        self.ser = None
        self.running = False
        self.t0 = time.time()

        self.times = collections.deque(maxlen=MAX_POINTS)
        self.rpms = collections.deque(maxlen=MAX_POINTS)
        self.setpoints = collections.deque(maxlen=MAX_POINTS)
        self.all_data = []

        self.sine_metrics = SineMetrics()
        self._commanded_sp = 0.0
        self._last_sent_sp = None

        self.sine_running = False
        self._sine_job = None
        self._sine_start_wall = None

        self._build_ui()
        self._refresh_ports()

    def _build_ui(self):
        BG = "#1a1a2e"
        CARD = "#16213e"
        ACC = "#0f3460"
        HL = "#e94560"
        FG = "#eaeaea"
        MUTED = "#888"

        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TLabel", background=CARD, foreground=FG, font=("Consolas", 12))
        style.configure("Header.TLabel", background=BG, foreground=HL, font=("Consolas", 13, "bold"))
        style.configure("TEntry", fieldbackground=ACC, foreground=FG, font=("Consolas", 13), insertcolor=FG)
        style.configure("TButton", background=HL, foreground="#fff", font=("Consolas", 11, "bold"), relief="flat")
        style.map("TButton", background=[("active", "#c73652")])
        style.configure("Green.TButton", background="#00b894", foreground="#fff", font=("Consolas", 10, "bold"))
        style.map("Green.TButton", background=[("active", "#00a381")])
        style.configure("Blue.TButton", background="#0984e3", foreground="#fff", font=("Consolas", 10, "bold"))
        style.map("Blue.TButton", background=[("active", "#0773c5")])
        style.configure("Warn.TButton", background="#d63031", foreground="#fff", font=("Consolas", 10, "bold"))
        style.map("Warn.TButton", background=[("active", "#b71c1c")])
        style.configure("TCombobox", fieldbackground=ACC, foreground=FG, font=("Consolas", 10))
        style.configure("TFrame", background=BG)
        style.configure("Card.TFrame", background=CARD)

        ttk.Label(
            self.root,
            text="PID Motor Control Dashboard - Sine Test",
            style="Header.TLabel",
            padding=(20, 14, 20, 8),
        ).pack(fill="x")

        main = ttk.Frame(self.root, style="TFrame")
        main.pack(fill="both", expand=True, padx=16, pady=(0, 16))
        main.columnconfigure(0, weight=3)
        main.columnconfigure(1, weight=1)
        main.rowconfigure(0, weight=1)

        left = ttk.Frame(main, style="TFrame")
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        left.rowconfigure(0, weight=1)
        left.columnconfigure(0, weight=1)

        self.fig, self.ax = plt.subplots(figsize=(9, 6))
        self.fig.patch.set_facecolor(CARD)
        self.ax.set_facecolor(ACC)
        self.ax.tick_params(colors="#aaa")
        self.ax.spines[:].set_color("#334")
        self.ax.set_xlabel("Thoi gian (s)", color="#aaa", fontsize=12)
        self.ax.set_ylabel("Toc do (RPM)", color="#aaa", fontsize=12)
        self.ax.set_title("RPM thuc te vs Setpoint sin", color="#eaeaea", fontsize=18)
        self.line_rpm, = self.ax.plot([], [], color=HL, lw=1.8, label="RPM thuc")
        self.line_sp, = self.ax.plot([], [], color="#00cec9", lw=1.4, ls="--", label="Setpoint")
        self.ax.legend(facecolor=BG, edgecolor="#334", labelcolor="#eaeaea", fontsize=8)
        self.fig.tight_layout(pad=1.4)

        canvas = FigureCanvasTkAgg(self.fig, master=left)
        canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")
        self.canvas = canvas

        right = ttk.Frame(main, style="Card.TFrame", padding=16)
        right.grid(row=0, column=1, sticky="nsew")

        def section(parent, title):
            tk.Label(parent, text=title, bg=CARD, fg=MUTED, font=("Consolas", 9)).pack(anchor="w", pady=(12, 2))
            tk.Frame(parent, bg=ACC, height=1).pack(fill="x", pady=(0, 6))

        section(right, "KET NOI UART")
        port_frame = tk.Frame(right, bg=CARD)
        port_frame.pack(fill="x")
        self.port_var = tk.StringVar()
        self.port_cb = ttk.Combobox(port_frame, textvariable=self.port_var, width=10, state="readonly")
        self.port_cb.pack(side="left", fill="x", expand=True)
        ttk.Button(port_frame, text="Refresh", command=self._refresh_ports, width=8).pack(side="left", padx=(4, 0))

        self.conn_btn = ttk.Button(right, text="Ket noi", style="Green.TButton", command=self._toggle_connect)
        self.conn_btn.pack(fill="x", pady=(6, 0))
        self.conn_status = tk.Label(right, text="Chua ket noi", bg=CARD, fg="#e74c3c", font=("Consolas", 9))
        self.conn_status.pack(anchor="w", pady=(4, 0))

        section(right, "GIA TRI THUC TE")
        live_frame = tk.Frame(right, bg=CARD)
        live_frame.pack(fill="x")
        for col, (label, color) in enumerate([("RPM thuc", HL), ("Setpoint", "#00cec9")]):
            tk.Label(live_frame, text=label, bg=CARD, fg=MUTED, font=("Consolas", 12)).grid(row=0, column=col, padx=4, pady=(0, 2))
        self.lbl_rpm = tk.Label(live_frame, text="---", bg=CARD, fg=HL, font=("Consolas", 22, "bold"))
        self.lbl_rpm.grid(row=1, column=0, padx=4)
        self.lbl_sp_live = tk.Label(live_frame, text="---", bg=CARD, fg="#00cec9", font=("Consolas", 22, "bold"))
        self.lbl_sp_live.grid(row=1, column=1, padx=4)

        section(right, "SETPOINT SIN")
        offset_frame = tk.Frame(right, bg=CARD)
        offset_frame.pack(fill="x")
        tk.Label(offset_frame, text="Offset", bg=CARD, fg=MUTED, font=("Consolas", 11)).pack(side="left")
        self.sp_var = tk.StringVar(value="100")
        self.sp_entry = ttk.Entry(offset_frame, textvariable=self.sp_var, width=10)
        self.sp_entry.pack(side="left", fill="x", expand=True, padx=(8, 4))
        tk.Label(offset_frame, text="RPM", bg=CARD, fg=MUTED, font=("Consolas", 11)).pack(side="left")

        self.sp_slider = tk.Scale(
            right,
            from_=-333,
            to=333,
            orient="horizontal",
            resolution=1,
            bg=CARD,
            fg=FG,
            troughcolor=ACC,
            activebackground=HL,
            highlightthickness=0,
            bd=0,
            command=self._slider_changed,
        )
        self.sp_slider.set(100)
        self.sp_slider.pack(fill="x", pady=(4, 4))

        sine_param_frame = tk.Frame(right, bg=CARD)
        sine_param_frame.pack(fill="x")
        tk.Label(sine_param_frame, text="A", bg=CARD, fg=MUTED, font=("Consolas", 11), width=6, anchor="w").grid(row=0, column=0, pady=2)
        tk.Label(sine_param_frame, text="F (Hz)", bg=CARD, fg=MUTED, font=("Consolas", 11), width=6, anchor="w").grid(row=1, column=0, pady=2)
        self.amp_var = tk.StringVar(value="50")
        self.freq_var = tk.StringVar(value="0.20")
        ttk.Entry(sine_param_frame, textvariable=self.amp_var, width=12).grid(row=0, column=1, sticky="ew", padx=(4, 0))
        ttk.Entry(sine_param_frame, textvariable=self.freq_var, width=12).grid(row=1, column=1, sticky="ew", padx=(4, 0))
        sine_param_frame.columnconfigure(1, weight=1)

        btn_row = tk.Frame(right, bg=CARD)
        btn_row.pack(fill="x", pady=(6, 0))
        ttk.Button(btn_row, text="Gui DC", command=self._send_setpoint).pack(side="left", fill="x", expand=True)
        ttk.Button(btn_row, text="Start Sin", style="Blue.TButton", command=self._start_sine).pack(side="left", fill="x", expand=True, padx=4)
        ttk.Button(btn_row, text="Dung Sin", style="Warn.TButton", command=self._stop_sine).pack(side="left", fill="x", expand=True)

        self.sine_status = tk.Label(right, text="Mode: DC", bg=CARD, fg="#74b9ff", font=("Consolas", 9))
        self.sine_status.pack(anchor="w", pady=(4, 0))

        section(right, "PID PARAMETERS")
        pid_frame = tk.Frame(right, bg=CARD)
        pid_frame.pack(fill="x")
        self.pid_labels = {}
        for i, param in enumerate(["KP", "KI", "KD"]):
            tk.Label(pid_frame, text=param, bg=CARD, fg=MUTED, font=("Consolas", 12, "bold"), width=4).grid(row=i, column=0, pady=4, sticky="w")
            lbl = tk.Label(pid_frame, text="---", bg=ACC, fg="#fdcb6e", font=("Consolas", 13), width=14, anchor="w", padx=6, pady=2, relief="flat")
            lbl.grid(row=i, column=1, padx=4, pady=4, sticky="ew")
            self.pid_labels[param] = lbl

        section(right, "GHI DU LIEU")
        ttk.Button(right, text="Xuat CSV", style="Blue.TButton", command=self._export_csv).pack(fill="x", pady=(0, 4))

        self._build_perf_panel(right, CARD, ACC, FG, MUTED)

        section(right, "LOG UART")
        self.log_text = tk.Text(right, height=5, bg="#0a0a1a", fg="#55efc4", font=("Consolas", 10), state="disabled", relief="flat", bd=0)
        self.log_text.pack(fill="x")
        ttk.Button(right, text="Xoa log", command=self._clear_log).pack(anchor="e", pady=(4, 0))

    def _build_perf_panel(self, parent, card, accent, fg, muted):
        frame = tk.Frame(parent, bg=card, pady=6, padx=0)
        frame.pack(fill="x")
        tk.Label(frame, text="SINE RESPONSE METRICS", bg=card, fg=muted, font=("Consolas", 9)).pack(anchor="w")
        tk.Frame(frame, bg=accent, height=1).pack(fill="x", pady=(2, 6))

        metrics_row = tk.Frame(frame, bg=card)
        metrics_row.pack(fill="x")

        metric_defs = [
            ("phase_lag_deg", "Phase Lag", "#00cec9", "deg", "Lech pha giua RPM va setpoint sin"),
            ("amplitude_ratio_pct", "Amp Ratio", "#55efc4", "%", "Bien do RPM dat duoc bao nhieu % so voi setpoint"),
        ]

        self.metric_labels = {}
        self.metric_saved = {}
        for col, (key, label, color, unit, tip) in enumerate(metric_defs):
            cell = tk.Frame(metrics_row, bg=accent, padx=6, pady=6, relief="flat")
            cell.grid(row=0, column=col, padx=3, pady=2, sticky="ew")
            metrics_row.columnconfigure(col, weight=1)
            tk.Label(cell, text=label, bg=accent, fg=muted, font=("Consolas", 10)).pack(anchor="w")
            val_lbl = tk.Label(cell, text="---", bg=accent, fg=color, font=("Consolas", 13, "bold"))
            val_lbl.pack(anchor="w")
            tk.Label(cell, text=unit, bg=accent, fg=muted, font=("Consolas", 10)).pack(anchor="w")
            self._make_tooltip(cell, tip)
            self.metric_labels[key] = val_lbl
            self.metric_saved[key] = None

        btn_frame = tk.Frame(frame, bg=card)
        btn_frame.pack(anchor="e", pady=(4, 0))
        tk.Button(
            btn_frame,
            text="Reset Metrics",
            bg=accent,
            fg=fg,
            font=("Consolas", 9),
            bd=0,
            padx=8,
            pady=4,
            activebackground="#1a1a2e",
            activeforeground=fg,
            command=self._reset_metrics,
        ).pack()

    def _make_tooltip(self, widget, text):
        def on_enter(event):
            widget._tip = tk.Toplevel(widget)
            widget._tip.wm_overrideredirect(True)
            widget._tip.wm_geometry(f"+{event.x_root + 12}+{event.y_root + 8}")
            tk.Label(widget._tip, text=text, bg="#2d3436", fg="#dfe6e9", font=("Consolas", 9), padx=6, pady=3).pack()

        def on_leave(_event):
            if hasattr(widget, "_tip"):
                widget._tip.destroy()

        widget.bind("<Enter>", on_enter)
        widget.bind("<Leave>", on_leave)

    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_cb["values"] = ports
        if ports:
            self.port_cb.current(0)

    def _toggle_connect(self):
        if self.ser and self.ser.is_open:
            self._stop_sine()
            self.running = False
            time.sleep(0.2)
            self.ser.close()
            self.conn_btn.config(text="Ket noi", style="Green.TButton")
            self.conn_status.config(text="Chua ket noi", fg="#e74c3c")
        else:
            port = self.port_var.get()
            if not port:
                messagebox.showerror("Loi", "Chon cong COM truoc!")
                return
            try:
                self.ser = serial.Serial(port, BAUD_RATE, timeout=0.1)
                self.ser.reset_input_buffer()
                self.t0 = time.time()
                self.times.clear()
                self.rpms.clear()
                self.setpoints.clear()
                self.all_data.clear()
                self._reset_metrics()
                self.running = True
                threading.Thread(target=self._read_loop, daemon=True).start()
                self.conn_btn.config(text="Ngat ket noi")
                self.conn_status.config(text=f"Da ket noi: {port}", fg="#00b894")
                self._start_animation()
            except Exception as exc:
                messagebox.showerror("Loi ket noi", str(exc))

    def _read_loop(self):
        buf = ""
        while self.running and self.ser and self.ser.is_open:
            try:
                raw = self.ser.read(64).decode("utf-8", errors="ignore")
                buf += raw
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    self._parse_line(line.strip())
            except Exception:
                break

    def _parse_line(self, line):
        if not line:
            return

        try:
            vals_log = {}
            for part in line.split(","):
                if ":" in part:
                    k, v = part.split(":", 1)
                    vals_log[k.strip()] = v.strip()
            log_parts = []
            if "RPM" in vals_log:
                log_parts.append(f"RPM:{vals_log['RPM']}")
            if "SP" in vals_log:
                log_parts.append(f"SP:{vals_log['SP']}")
            self._log(",".join(log_parts) if log_parts else line)
        except Exception:
            self._log(line)

        if line.startswith("OK") or line.startswith("ERR"):
            return

        try:
            vals = {}
            for part in line.split(","):
                if ":" in part:
                    k, v = part.split(":", 1)
                    vals[k.strip()] = v.strip()

            rpm_val = float(vals["RPM"]) if "RPM" in vals else None
            sp_val = float(vals["SP"]) if "SP" in vals else self._commanded_sp
            kp_val = float(vals.get("KP", 0)) / 100.0 if "KP" in vals else None
            ki_val = float(vals.get("KI", 0)) / 100.0 if "KI" in vals else None
            kd_val = float(vals.get("KD", 0)) / 1000.0 if "KD" in vals else None

            for param, val in [("KP", kp_val), ("KI", ki_val), ("KD", kd_val)]:
                if val is not None:
                    self.root.after(0, lambda p=param, v=val: self.pid_labels[p].config(text=f"{v:.4f}"))

            if rpm_val is None:
                return

            if not self.times:
                self.t0 = time.time()
            now = time.time() - self.t0

            self.times.append(now)
            self.rpms.append(rpm_val)
            self.setpoints.append(sp_val if sp_val is not None else self._commanded_sp)

            sine_metrics = self.sine_metrics.update(self.times, self.setpoints, self.rpms, self._current_freq_hz())

            self.all_data.append(
                {
                    "time": round(now, 3),
                    "rpm": rpm_val,
                    "sp": sp_val,
                    "kp": kp_val,
                    "ki": ki_val,
                    "kd": kd_val,
                    "phase_lag_deg": sine_metrics["phase_lag_deg"],
                    "amplitude_ratio_pct": sine_metrics["amplitude_ratio_pct"],
                    "setpoint_amp": sine_metrics["setpoint_amp"],
                    "response_amp": sine_metrics["response_amp"],
                }
            )

            self.root.after(0, lambda r=rpm_val, s=sp_val, m=sine_metrics: self._update_live(r, s, m))
        except Exception:
            pass

    def _update_live(self, rpm, sp, metrics):
        self.lbl_rpm.config(text=f"{int(round(rpm))}")
        if sp is not None:
            self.lbl_sp_live.config(text=f"{int(round(sp))}")

        fmt = {
            "phase_lag_deg": (metrics.get("phase_lag_deg"), "{:.2f}"),
            "amplitude_ratio_pct": (metrics.get("amplitude_ratio_pct"), "{:.2f}"),
        }
        for key, (val, fmtstr) in fmt.items():
            text = fmtstr.format(val) if val is not None else "---"
            self.metric_labels[key].config(text=text)
            if val is not None:
                self.metric_saved[key] = val

    def _start_animation(self):
        self.ani = animation.FuncAnimation(self.fig, self._animate, interval=UPDATE_MS, blit=False, cache_frame_data=False)
        self.canvas.draw()

    def _animate(self, _frame):
        xs = list(self.times)
        ys = list(self.rpms)
        ss = list(self.setpoints)

        self.line_rpm.set_data(xs, ys)
        self.line_sp.set_data(xs, ss)

        if xs:
            x_end = xs[-1]
            self.ax.set_xlim((x_end - 30, x_end) if x_end > 30 else (0, 30))
        else:
            self.ax.set_xlim(0, 30)

        if ys or ss:
            all_vals = ys + ss
            y_min = min(all_vals)
            y_max = max(all_vals)
            pad = max((y_max - y_min) * 0.15, 20)
            self.ax.set_ylim(y_min - pad, y_max + pad)
        else:
            self.ax.set_ylim(-50, 400)

        return self.line_rpm, self.line_sp

    def _send(self, msg):
        if self.ser and self.ser.is_open:
            self.ser.write((msg + "\r\n").encode())
            self._log(f"[TX] {msg}")
        else:
            messagebox.showwarning("Chua ket noi", "Hay ket noi cong COM truoc!")

    def _send_setpoint_value(self, value):
        value = self._clamp_setpoint(value)
        self._commanded_sp = float(value)
        if self._last_sent_sp != value:
            self._send(f"SP:{value}")
            self._last_sent_sp = value

    def _send_setpoint(self):
        try:
            self._stop_sine()
            value = int(round(self._get_offset()))
            self._send_setpoint_value(value)
            self.sine_status.config(text="Mode: DC", fg="#74b9ff")
        except ValueError as exc:
            messagebox.showerror("Loi", str(exc))

    def _slider_changed(self, val):
        self.sp_var.set(str(int(float(val))))

    def _start_sine(self):
        try:
            offset = self._get_offset()
            amplitude = self._get_amplitude()
            freq_hz = self._get_frequency()
            if amplitude <= 0:
                raise ValueError("Bien do A phai > 0")
            if freq_hz <= 0:
                raise ValueError("F phai > 0")

            self._stop_sine()
            self.sine_running = True
            self._sine_start_wall = time.time()
            self.sine_metrics.reset()
            self._last_sent_sp = None
            self.sine_status.config(
                text=f"Mode: Sin | Offset={offset:.1f} RPM | A={amplitude:.1f} | F={freq_hz:.3f} Hz",
                fg="#00cec9",
            )
            self._run_sine_step()
        except ValueError as exc:
            messagebox.showerror("Loi tham so sin", str(exc))

    def _run_sine_step(self):
        if not self.sine_running:
            return

        offset = self._get_offset()
        amplitude = self._get_amplitude()
        freq_hz = self._get_frequency()
        elapsed = time.time() - self._sine_start_wall
        setpoint = offset + amplitude * math.sin(2.0 * math.pi * freq_hz * elapsed)
        self._send_setpoint_value(int(round(setpoint)))
        self._sine_job = self.root.after(SINE_SEND_MS, self._run_sine_step)

    def _stop_sine(self):
        self.sine_running = False
        if self._sine_job is not None:
            self.root.after_cancel(self._sine_job)
            self._sine_job = None
        self.sine_status.config(text="Mode: DC", fg="#74b9ff")

    def _reset_metrics(self):
        self.sine_metrics.reset()
        for key in self.metric_labels:
            self.metric_labels[key].config(text="---")
            self.metric_saved[key] = None

    def _export_csv(self):
        if not self.all_data:
            messagebox.showinfo("Khong co du lieu", "Chua co du lieu de xuat!")
            return

        summary = {
            "phase_lag_deg": self.metric_saved.get("phase_lag_deg"),
            "amplitude_ratio_pct": self.metric_saved.get("amplitude_ratio_pct"),
        }

        default_name = time.strftime("pid_sine_log_%Y%m%d_%H%M%S.csv")
        path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
            initialfile=default_name,
            title="Luu du lieu CSV",
        )
        if not path:
            return

        try:
            with open(path, "w", newline="", encoding="utf-8") as file:
                writer = csv.writer(file)
                writer.writerow(["# PID Sine Test - Data Export"])
                writer.writerow(["# Export time:", time.strftime("%Y-%m-%d %H:%M:%S")])
                writer.writerow([])
                writer.writerow(["=== PERFORMANCE SUMMARY ==="])
                writer.writerow(["Metric", "Value", "Unit", "Description"])
                writer.writerow(["Phase Lag", self._fmt_csv(summary["phase_lag_deg"], "{:.2f}"), "deg", "Phase difference between response and sine setpoint"])
                writer.writerow(["Amplitude Ratio", self._fmt_csv(summary["amplitude_ratio_pct"], "{:.2f}"), "%", "Response amplitude / setpoint amplitude"])
                writer.writerow([])
                writer.writerow(["=== RAW DATA ==="])
                writer.writerow(
                    [
                        "time(s)",
                        "rpm",
                        "setpoint",
                        "kp",
                        "ki",
                        "kd",
                        "phase_lag(deg)",
                        "amplitude_ratio(%)",
                        "setpoint_amp",
                        "response_amp",
                    ]
                )
                for row in self.all_data:
                    writer.writerow(
                        [
                            row["time"],
                            row["rpm"],
                            row["sp"] if row["sp"] is not None else "",
                            row["kp"] if row["kp"] is not None else "",
                            row["ki"] if row["ki"] is not None else "",
                            row["kd"] if row["kd"] is not None else "",
                            self._fmt_csv(row["phase_lag_deg"], "{:.2f}"),
                            self._fmt_csv(row["amplitude_ratio_pct"], "{:.2f}"),
                            self._fmt_csv(row["setpoint_amp"], "{:.3f}"),
                            self._fmt_csv(row["response_amp"], "{:.3f}"),
                        ]
                    )

            messagebox.showinfo("Xuat thanh cong", f"Da luu {len(self.all_data)} mau vao {os.path.basename(path)}")
            self._log(f"[CSV] Da xuat -> {path}")
        except Exception as exc:
            messagebox.showerror("Loi xuat file", str(exc))

    def _fmt_csv(self, value, fmt):
        return fmt.format(value) if value is not None else "N/A"

    def _log(self, msg):
        ts = time.strftime("%H:%M:%S")
        self.log_text.config(state="normal")
        self.log_text.insert("end", f"[{ts}] {msg}\n")
        self.log_text.see("end")
        if int(self.log_text.index("end-1c").split(".")[0]) > 200:
            self.log_text.delete("1.0", "50.0")
        self.log_text.config(state="disabled")

    def _clear_log(self):
        self.log_text.config(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.config(state="disabled")

    def _get_offset(self):
        return float(self.sp_var.get())

    def _get_amplitude(self):
        value = float(self.amp_var.get())
        if value > 333:
            raise ValueError("Bien do A nen <= 333 RPM")
        return value

    def _get_frequency(self):
        return float(self.freq_var.get())

    def _current_freq_hz(self):
        try:
            return self._get_frequency() if self.sine_running else None
        except ValueError:
            return None

    def _clamp_setpoint(self, value):
        return max(-333, min(333, int(round(value))))


if __name__ == "__main__":
    root = tk.Tk()
    root.geometry("1500x820")
    app = PIDDashboard(root)
    root.mainloop()
