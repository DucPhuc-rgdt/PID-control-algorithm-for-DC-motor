"""
PID Motor Control Dashboard
============================
Giao tiếp UART 2 chiều với STM32:
  - Nhận: "RPM:<val>,SP:<val>,KP:<val>,KI:<val>,KD:<val>\n"
  - Gửi:  "SP:<value>\n"   (chỉ setpoint, Kp/Ki/Kd cố định trên STM32)

Cài đặt:
    pip install pyserial matplotlib

Chạy:
    python pid_dashboard.py
"""

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import serial
import serial.tools.list_ports
import threading
import collections
import time
import csv
import os
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# ─── Cấu hình ────────────────────────────────────────────────
MAX_POINTS  = 2000   # đủ chứa ~30s ở tốc độ sample 50ms/frame
BAUD_RATE   = 115200
UPDATE_MS   = 50

# Ngưỡng settling time: RPM trong vòng ±SETTLE_BAND% so với setpoint
SETTLE_BAND_PCT = 5.0   # ±5%
SETTLE_WINDOW    = 20   
SETTLE_ALLOW_OUT = 2    
# ─────────────────────────────────────────────────────────────


class PerfMetrics:
    def __init__(self):
        self.reset()

    def reset(self):
        self.sp          = None
        self.t_start     = None
        self.rpm_start   = 0.0
        self.peak_rpm    = None

        self.t_10        = None
        self.t_90        = None
        self.rise_time   = None

        self.t_settle    = None
        self.settled     = False

        self.ss_buf      = collections.deque(maxlen=10)
        # FIX 3: đổi settle_buf từ deque() sang deque(maxlen=WINDOW)
        # để không cần .pop(0) — deque không có method pop(0), chỉ có popleft()
        self.settle_buf  = collections.deque(maxlen=SETTLE_WINDOW)

        self.overshoot   = None
        self.steady_err  = None

    def update(self, t, rpm, sp):
        if sp is None:
            return self._result()

        if self.sp is None or abs(sp - self.sp) > 1:
            self.reset()
            self.sp        = sp
            self.t_start   = t
            self.rpm_start = rpm
            self.peak_rpm  = rpm
            return self._result()

        delta = sp - self.rpm_start

        if abs(delta) < 5:
            self.ss_buf.append(rpm)
            avg = sum(self.ss_buf) / len(self.ss_buf)
            self.steady_err = abs(sp - avg) / abs(sp) * 100.0 if sp != 0 else 0.0
            return self._result()

        elapsed = t - self.t_start

        # Rise time
        pct = (rpm - self.rpm_start) / delta
        if self.t_10 is None and pct >= 0.10:
            self.t_10 = elapsed
        if self.t_90 is None and pct >= 0.90:
            self.t_90 = elapsed
            if self.t_10 is not None:
                self.rise_time = self.t_90 - self.t_10

        # Overshoot được theo dõi liên tục theo peak hiện tại của response,
        # không khóa lại sau khi đã "settled".
        if delta > 0:
            if rpm > self.peak_rpm:
                self.peak_rpm = rpm
            self.overshoot = max(0.0, (self.peak_rpm - sp) / abs(delta) * 100.0)
        elif delta < 0:
            if rpm < self.peak_rpm:
                self.peak_rpm = rpm
            self.overshoot = max(0.0, (sp - self.peak_rpm) / abs(delta) * 100.0)

        # Settling time
        band      = max(abs(sp) * SETTLE_BAND_PCT / 100.0, 5.0)
        WINDOW    = SETTLE_WINDOW
        ALLOW_OUT = SETTLE_ALLOW_OUT

        if not self.settled:
            # FIX 3: deque(maxlen=30) tự động bỏ phần tử cũ — không cần pop(0)
            self.settle_buf.append(1 if abs(rpm - sp) <= band else 0)

            if len(self.settle_buf) == WINDOW:
                if sum(self.settle_buf) >= (WINDOW - ALLOW_OUT):
                    self.t_settle    = elapsed
                    self.settled     = True

        if self.settled:
            self.ss_buf.append(rpm)

        if self.settled and len(self.ss_buf) >= 10 and self.steady_err is None:
            avg = sum(self.ss_buf) / len(self.ss_buf)
            self.steady_err = abs(sp - avg) / abs(sp) * 100.0 if sp != 0 else 0.0

        return self._result()

    def _result(self):
        return {
            "rise_time":   self.rise_time,
            "overshoot":   self.overshoot,
            "settle_time": self.t_settle,
            "steady_err":  self.steady_err,
        }



class PIDDashboard:
    def __init__(self, root):
        self.root = root
        self.root.title("PID Motor Control Dashboard")
        self.root.configure(bg="#1a1a2e")
        self.root.resizable(True, True)

        self.ser     = None
        self.running = False

        self.times     = collections.deque(maxlen=MAX_POINTS)
        self.rpms      = collections.deque(maxlen=MAX_POINTS)
        self.setpoints = collections.deque(maxlen=MAX_POINTS)
        # FIX 1: lưu t0 tuyệt đối một lần duy nhất khi connect,
        # KHÔNG dùng xs[0] để tính offset trong _animate (gây mắc ở điểm đầu)
        self.t0        = time.time()

        # Lưu toàn bộ dữ liệu để export CSV
        self.all_data  = []

        self.perf          = PerfMetrics()
        self._last_sp      = None
        self._sp_lock_until = 0.0   # khoá reset metrics khi vừa gửi SP mới

        self._build_ui()
        self._refresh_ports()

    # ─── BUILD UI ────────────────────────────────────────────
    def _build_ui(self):
        BG   = "#1a1a2e"
        CARD = "#16213e"
        ACC  = "#0f3460"
        HL   = "#e94560"
        FG   = "#eaeaea"
        MUTED= "#888"

        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TLabel",         background=CARD, foreground=FG,    font=("Consolas", 12))
        style.configure("Header.TLabel",  background=BG,   foreground=HL,    font=("Consolas", 13, "bold"))
        style.configure("Val.TLabel",     background=CARD, foreground=HL,    font=("Consolas", 22, "bold"))
        style.configure("TEntry",         fieldbackground=ACC, foreground=FG, font=("Consolas", 13), insertcolor=FG)
        style.configure("TButton",        background=HL,   foreground="#fff", font=("Consolas", 12, "bold"), relief="flat")
        style.map("TButton",              background=[("active", "#c73652")])
        style.configure("Green.TButton",  background="#00b894", foreground="#fff", font=("Consolas", 10, "bold"))
        style.map("Green.TButton",        background=[("active", "#00a381")])
        style.configure("Blue.TButton",   background="#0984e3", foreground="#fff", font=("Consolas", 10, "bold"))
        style.map("Blue.TButton",         background=[("active", "#0773c5")])
        style.configure("TCombobox",      fieldbackground=ACC, foreground=FG,  font=("Consolas", 10))
        style.configure("TFrame",         background=BG)
        style.configure("Card.TFrame",    background=CARD)

        # ── Title
        ttk.Label(self.root, text="⚙  PID Motor Control Dashboard",
                  style="Header.TLabel", padding=(20, 14, 20, 8)).pack(fill="x")

        # ── Main layout
        main = ttk.Frame(self.root, style="TFrame")
        main.pack(fill="both", expand=True, padx=16, pady=(0, 16))
        main.columnconfigure(0, weight=3)
        main.columnconfigure(1, weight=1)
        main.rowconfigure(0, weight=1)

        # ── Left: chart
        left = ttk.Frame(main, style="TFrame")
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        left.rowconfigure(0, weight=1)
        left.columnconfigure(0, weight=1)

        self.fig, self.ax = plt.subplots(figsize=(9, 6))
        self.fig.patch.set_facecolor("#16213e")
        self.ax.set_facecolor("#0f3460")
        self.ax.tick_params(colors="#aaa")
        self.ax.spines[:].set_color("#334")
        self.ax.set_xlabel("Thời gian (s)", color="#aaa", fontsize=12)
        self.ax.set_ylabel("Tốc độ (RPM)",  color="#aaa", fontsize=12)
        self.ax.set_title("Tốc độ thực tế", color="#eaeaea", fontsize=20)
        self.line_rpm, = self.ax.plot([], [], color="#e94560", lw=1.8, label="RPM thực")
        self.line_sp,  = self.ax.plot([], [], color="#00cec9", lw=1.4, ls="--", label="Setpoint")
        self.ax.legend(facecolor="#1a1a2e", edgecolor="#334", labelcolor="#eaeaea", fontsize=8)
        self.fig.tight_layout(pad=1.4)

        canvas = FigureCanvasTkAgg(self.fig, master=left)
        canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")
        self.canvas = canvas

        # ── Right: control panel
        right = ttk.Frame(main, style="Card.TFrame", padding=16)
        right.grid(row=0, column=1, sticky="nsew")

        def section(parent, title):
            tk.Label(parent, text=title, bg=CARD, fg=MUTED,
                     font=("Consolas", 9)).pack(anchor="w", pady=(12, 2))
            tk.Frame(parent, bg=ACC, height=1).pack(fill="x", pady=(0, 6))

        # ── Connection
        section(right, "KẾT NỐI UART")
        port_frame = tk.Frame(right, bg=CARD)
        port_frame.pack(fill="x")

        self.port_var = tk.StringVar()
        self.port_cb  = ttk.Combobox(port_frame, textvariable=self.port_var,
                                      width=10, state="readonly")
        self.port_cb.pack(side="left", fill="x", expand=True)
        ttk.Button(port_frame, text="↻",
                   command=self._refresh_ports, width=3).pack(side="left", padx=(4, 0))

        self.conn_btn = ttk.Button(right, text="Kết nối",
                                    style="Green.TButton",
                                    command=self._toggle_connect)
        self.conn_btn.pack(fill="x", pady=(6, 0))

        self.conn_status = tk.Label(right, text="● Chưa kết nối",
                                     bg=CARD, fg="#e74c3c",
                                     font=("Consolas", 9))
        self.conn_status.pack(anchor="w", pady=(4, 0))

        # ── Live values
        section(right, "GIÁ TRỊ THỰC TẾ")
        live_frame = tk.Frame(right, bg=CARD)
        live_frame.pack(fill="x")
        for col, (label, color) in enumerate([("RPM thực", HL), ("Setpoint", "#00cec9")]):
            tk.Label(live_frame, text=label, bg=CARD, fg=MUTED,
                     font=("Consolas", 12)).grid(row=0, column=col, padx=4, pady=(0, 2))
        self.lbl_rpm = tk.Label(live_frame, text="---", bg=CARD, fg=HL,
                                 font=("Consolas", 22, "bold"))
        self.lbl_rpm.grid(row=1, column=0, padx=4)
        self.lbl_sp_live = tk.Label(live_frame, text="---", bg=CARD, fg="#00cec9",
                                     font=("Consolas", 22, "bold"))
        self.lbl_sp_live.grid(row=1, column=1, padx=4)

        # ── Setpoint control
        section(right, "SETPOINT")
        sp_frame = tk.Frame(right, bg=CARD)
        sp_frame.pack(fill="x")
        self.sp_var = tk.StringVar(value="100")
        self.sp_entry = ttk.Entry(sp_frame, textvariable=self.sp_var, width=10)
        self.sp_entry.pack(side="left", fill="x", expand=True)
        self.sp_entry.bind("<Return>", lambda _: self._send_setpoint())
        tk.Label(sp_frame, text="RPM", bg=CARD, fg=MUTED,
                 font=("Consolas", 12)).pack(side="left", padx=4)
        ttk.Button(right, text="Gửi Setpoint ↗",
                   command=self._send_setpoint).pack(fill="x", pady=(6, 0))

        self.sp_slider = tk.Scale(right, from_=0, to=333, orient="horizontal",
                                   resolution=10, bg=CARD, fg=FG,
                                   troughcolor=ACC, activebackground=HL,
                                   highlightthickness=0, bd=0,
                                   command=self._slider_changed)
        self.sp_slider.set(100)
        self.sp_slider.pack(fill="x", pady=(4, 0))

        # ── PID read-only display (nhận từ STM32)
        section(right, "PID PARAMETERS")

        pid_frame = tk.Frame(right, bg=CARD)
        pid_frame.pack(fill="x")

        self.pid_labels = {}
        for i, param in enumerate(["KP", "KI", "KD"]):
            tk.Label(pid_frame, text=param, bg=CARD, fg=MUTED,
                     font=("Consolas", 12, "bold"), width=4).grid(
                         row=i, column=0, pady=4, sticky="w")
            lbl = tk.Label(pid_frame, text="---", bg=ACC, fg="#fdcb6e",
                           font=("Consolas", 13), width=14, anchor="w",
                           padx=6, pady=2, relief="flat")
            lbl.grid(row=i, column=1, padx=4, pady=4, sticky="ew")
            self.pid_labels[param] = lbl

        # ── Export CSV
        section(right, "GHI DỮ LIỆU")
        ttk.Button(right, text="💾  Xuất CSV",
                   style="Blue.TButton",
                   command=self._export_csv).pack(fill="x", pady=(0, 4))

        # ── Performance Metrics
        self._build_perf_panel(right, CARD, ACC, FG, MUTED)

        # ── Log
        section(right, "LOG UART")
        self.log_text = tk.Text(right, height=5, bg="#0a0a1a", fg="#55efc4",
                                 font=("Consolas", 10), state="disabled",
                                 relief="flat", bd=0)
        self.log_text.pack(fill="x")
        ttk.Button(right, text="Xóa log",
                   command=self._clear_log).pack(anchor="e", pady=(4, 0))

    # ─── PERFORMANCE METRICS PANEL ───────────────────────────
    def _build_perf_panel(self, parent, CARD, ACC, FG, MUTED):
        frame = tk.Frame(parent, bg=CARD, pady=6, padx=0)
        frame.pack(fill="x")

        tk.Label(frame, text="PERFORMANCE METRICS",
                 bg=CARD, fg=MUTED,
                 font=("Consolas", 9)).pack(anchor="w")
        tk.Frame(frame, bg=ACC, height=1).pack(fill="x", pady=(2, 6))

        metrics_row = tk.Frame(frame, bg=CARD)
        metrics_row.pack(fill="x")

        metric_defs = [
            ("rise_time",   "Rise Time",     "#fdcb6e", "s",   "Thời gian từ 10% → 90% setpoint"),
            ("overshoot",   "Overshoot",     "#e17055", "%",   "Phần trăm vượt quá setpoint"),
            ("settle_time", "Settling Time", "#74b9ff", "s",   f"Thời gian vào dải ±{SETTLE_BAND_PCT}%"),
            ("steady_err",  "Steady Error",  "#a29bfe", "%", "Sai số xác lập trung bình"),
        ]

        self.metric_labels = {}
        self.metric_saved  = {}

        for col, (key, label, color, unit, tip) in enumerate(metric_defs):
            cell = tk.Frame(metrics_row, bg=ACC, padx=6, pady=6, relief="flat")
            cell.grid(row=0, column=col, padx=3, pady=2, sticky="ew")
            metrics_row.columnconfigure(col, weight=1)

            tk.Label(cell, text=label, bg=ACC, fg=MUTED,
                     font=("Consolas", 12)).pack(anchor="w")
            val_lbl = tk.Label(cell, text="---", bg=ACC, fg=color,
                               font=("Consolas", 14, "bold"))
            val_lbl.pack(anchor="w")
            tk.Label(cell, text=unit, bg=ACC, fg=MUTED,
                     font=("Consolas", 12)).pack(anchor="w")

            def make_tooltip(widget, text):
                def on_enter(e):
                    widget._tip = tk.Toplevel(widget)
                    widget._tip.wm_overrideredirect(True)
                    widget._tip.wm_geometry(f"+{e.x_root+12}+{e.y_root+8}")
                    tk.Label(widget._tip, text=text,
                             bg="#2d3436", fg="#dfe6e9",
                             font=("Consolas", 9), padx=6, pady=3).pack()
                def on_leave(e):
                    if hasattr(widget, "_tip"):
                        widget._tip.destroy()
                widget.bind("<Enter>", on_enter)
                widget.bind("<Leave>", on_leave)

            make_tooltip(cell, tip)

            self.metric_labels[key] = val_lbl
            self.metric_saved[key]  = None

        btn_frame = tk.Frame(frame, bg=CARD)
        btn_frame.pack(anchor="e", pady=(4, 0))
        tk.Button(btn_frame, text="↺ Reset Metrics",
                  bg=ACC, fg=FG,
                  font=("Consolas", 9), bd=0, padx=8, pady=4,
                  activebackground="#1a1a2e", activeforeground=FG,
                  command=self._reset_metrics).pack()

    # ─── PORTS ───────────────────────────────────────────────
    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_cb["values"] = ports
        if ports:
            self.port_cb.current(0)

    # ─── CONNECT ─────────────────────────────────────────────
    def _toggle_connect(self):
        if self.ser and self.ser.is_open:
            self.running = False
            time.sleep(0.2)
            self.ser.close()
            self.conn_btn.config(text="Kết nối", style="Green.TButton")
            self.conn_status.config(text="● Chưa kết nối", fg="#e74c3c")
        else:
            port = self.port_var.get()
            if not port:
                messagebox.showerror("Lỗi", "Chọn cổng COM trước!")
                return
            try:
                self.ser     = serial.Serial(port, BAUD_RATE, timeout=0.1)
                self.ser.reset_input_buffer()   # xóa buffer UART cũ tránh đọc dữ liệu thừa
                self.t0      = time.time()      # t0 reset SAU flush → t=0 khớp dữ liệu thực
                self.times.clear()
                self.rpms.clear()
                self.setpoints.clear()
                self.running = True
                self.all_data.clear()
                threading.Thread(target=self._read_loop, daemon=True).start()
                self.conn_btn.config(text="Ngắt kết nối")
                self.conn_status.config(text=f"● Đã kết nối: {port}", fg="#00b894")
                self._start_animation()
            except Exception as e:
                messagebox.showerror("Lỗi kết nối", str(e))

    # ─── READ LOOP ───────────────────────────────────────────
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
        """
        Nhận dạng:
          "RPM:<val>,SP:<val>,KP:<val>,KI:<val>,KD:<val>"
          "OK SP:<val>"   ← ACK từ STM32 sau khi gửi setpoint
        """
        if not line:
            return

        # Log: chỉ hiển thị RPM và SP
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
            if log_parts:
                self._log(",".join(log_parts))
            else:
                self._log(line)
        except Exception:
            self._log(line)

        # Bỏ qua dòng ACK / ERR
        if line.startswith("OK") or line.startswith("ERR"):
            return

        try:
            vals = {}
            for part in line.split(","):
                if ":" in part:
                    k, v = part.split(":", 1)
                    vals[k.strip()] = v.strip()

            rpm_val = float(vals["RPM"]) if "RPM" in vals else None
            sp_val  = float(vals["SP"])  if "SP"  in vals else None
            kp_val  = float(vals.get("KP", 0)) / 100.0  if "KP" in vals else None
            ki_val  = float(vals.get("KI", 0)) / 100.0  if "KI" in vals else None
            kd_val  = float(vals.get("KD", 0)) / 1000.0 if "KD" in vals else None

            # Cập nhật nhãn Kp/Ki/Kd
            for param, val in [("KP", kp_val), ("KI", ki_val), ("KD", kd_val)]:
                if val is not None:
                    self.root.after(0, lambda p=param, v=val:
                                    self.pid_labels[p].config(text=f"{v:.4f}"))

            if rpm_val is not None:
                # Reset t0 tại dòng RPM đầu tiên → t=0 khớp dữ liệu thực
                if not self.times:
                    self.t0 = time.time()
                now = time.time() - self.t0
                # Nếu vừa gửi SP mới, ép sp_val = perf.sp để tránh
                # STM32 echo SP cũ làm trigger reset metrics sai
                if time.time() < self._sp_lock_until and self.perf.sp is not None:
                    sp_val = self.perf.sp
                self.times.append(now)
                self.rpms.append(rpm_val)
                self.setpoints.append(
                    sp_val if sp_val is not None
                    else (self.setpoints[-1] if self.setpoints else 0)
                )

                # Tính performance metrics
                metrics = self.perf.update(now, rpm_val, sp_val)

                self.all_data.append({
                    "time":    round(now, 3),
                    "rpm":     rpm_val,
                    "sp":      sp_val,
                    "kp":      kp_val,
                    "ki":      ki_val,
                    "kd":      kd_val,
                    "rise_time":    metrics["rise_time"],
                    "overshoot":    metrics["overshoot"],
                    "settle_time":  metrics["settle_time"],
                    "steady_err":   metrics["steady_err"],
                })

                self.root.after(0, lambda r=rpm_val, s=sp_val, m=metrics:
                                self._update_live(r, s, m))
        except Exception:
            pass

    def _update_live(self, rpm, sp, metrics):
        self.lbl_rpm.config(text=f"{int(rpm)}")
        if sp is not None:
            self.lbl_sp_live.config(text=f"{int(sp)}")

        fmt = {
            "rise_time":   (metrics["rise_time"],   "{:.3f}"),
            "overshoot":   (metrics["overshoot"],   "{:.2f}"),
            "settle_time": (metrics["settle_time"], "{:.3f}"),
            "steady_err":  (metrics["steady_err"],  "{:.2f}"),
        }
        for key, (val, fmtstr) in fmt.items():
            text = fmtstr.format(val) if val is not None else "---"
            self.metric_labels[key].config(text=text)
            if val is not None:
                self.metric_saved[key] = val

    # ─── ANIMATION ───────────────────────────────────────────
    def _start_animation(self):
        self.ani = animation.FuncAnimation(
            self.fig, self._animate, interval=UPDATE_MS,
            blit=False, cache_frame_data=False)
        self.canvas.draw()

    def _animate(self, _):
        xs = list(self.times)
        ys = list(self.rpms)
        ss = list(self.setpoints)

        # FIX 1: xs đã tính từ t0 reset khi connect → KHÔNG trừ xs[0] nữa
        # (trừ xs[0] khiến đồ thị luôn bắt đầu lại từ 0 mỗi frame → mắc ở ~20s)

        # times đã được lưu là (time.time() - self.t0) → luôn bắt đầu từ ~0
        # Không dùng xs[0] làm offset vì khi buffer deque đầy, xs[0] sẽ nhảy lên ~20s

        self.line_rpm.set_data(xs, ys)
        self.line_sp.set_data(xs, ss)

        if xs:
            x_end = xs[-1]
            if x_end > 30:
                self.ax.set_xlim(x_end - 30, x_end)
            else:
                self.ax.set_xlim(0, 30)
        else:
            self.ax.set_xlim(0, 30)

        # FIX 2: auto-scale Y theo dữ liệu thực thay vì cố định -333..333
        # Tính min/max từ cả rpm lẫn setpoint, thêm padding 10%
        if ys or ss:
            all_vals = ys + ss
            y_min = min(all_vals)
            y_max = max(all_vals)
            pad   = max((y_max - y_min) * 0.15, 20)  # padding ít nhất 20 RPM
            self.ax.set_ylim(y_min - pad, y_max + pad)
        else:
            self.ax.set_ylim(-50, 400)

        return self.line_rpm, self.line_sp

    # ─── SEND ─────────────────────────────────────────────────
    def _send(self, msg):
        if self.ser and self.ser.is_open:
            self.ser.write((msg + "\r\n").encode())
            self._log(f"[TX] {msg}")
        else:
            messagebox.showwarning("Chưa kết nối", "Hãy kết nối cổng COM trước!")

    def _send_setpoint(self):
        try:
            val = int(float(self.sp_var.get()))
            val = max(-333, min(333, val))
            self._reset_perf_with_current_rpm(val)
            self._send(f"SP:{val}")
        except ValueError:
            messagebox.showerror("Lỗi", "Setpoint phải là số!")

    def _slider_changed(self, val):
        self.sp_var.set(str(val))
        self._reset_perf_with_current_rpm(int(val))
        self._send(f"SP:{val}")

    def _reset_perf_with_current_rpm(self, new_sp):
        """Reset metrics ngay khi gửi SP mới, dùng RPM hiện tại làm rpm_start."""
        current_rpm = self.rpms[-1] if self.rpms else 0.0
        current_t   = self.times[-1] if self.times else 0.0
        self.perf.reset()
        self.perf.sp        = float(new_sp)
        self.perf.t_start   = current_t
        self.perf.rpm_start = current_rpm
        self.perf.peak_rpm  = current_rpm
        # Khoá 0.5s: bỏ qua SP cũ từ UART trong lúc STM32 chưa cập nhật kịp
        self._sp_lock_until = time.time() + 0.5

    # ─── RESET METRICS ───────────────────────────────────────
    def _reset_metrics(self):
        self.perf.reset()
        for key in self.metric_labels:
            self.metric_labels[key].config(text="---")
            self.metric_saved[key] = None

    # ─── EXPORT CSV ──────────────────────────────────────────
    def _export_csv(self):
        if not self.all_data:
            messagebox.showinfo("Không có dữ liệu", "Chưa có dữ liệu để xuất!")
            return

        summary = {
            "rise_time":    self.metric_saved.get("rise_time"),
            "overshoot":    self.metric_saved.get("overshoot"),
            "settle_time":  self.metric_saved.get("settle_time"),
            "steady_err":   self.metric_saved.get("steady_err"),
        }

        default_name = time.strftime("pid_log_%Y%m%d_%H%M%S.csv")
        path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
            initialfile=default_name,
            title="Lưu dữ liệu CSV"
        )
        if not path:
            return

        try:
            with open(path, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)

                writer.writerow(["# PID Motor Control — Data Export"])
                writer.writerow(["# Thời gian xuất:", time.strftime("%Y-%m-%d %H:%M:%S")])
                writer.writerow([])

                writer.writerow(["=== PERFORMANCE SUMMARY ==="])
                writer.writerow(["Metric", "Value", "Unit", "Mô tả"])
                writer.writerow([
                    "Rise Time",
                    f"{summary['rise_time']:.3f}" if summary["rise_time"] is not None else "N/A",
                    "s",
                    "Thời gian từ 10% đến 90% setpoint"
                ])
                writer.writerow([
                    "Overshoot",
                    f"{summary['overshoot']:.2f}" if summary["overshoot"] is not None else "N/A",
                    "%",
                    "Phần trăm vượt quá setpoint"
                ])
                writer.writerow([
                    "Settling Time",
                    f"{summary['settle_time']:.3f}" if summary["settle_time"] is not None else "N/A",
                    "s",
                    f"Thời gian vào dải ±{SETTLE_BAND_PCT}% setpoint"
                ])
                writer.writerow([
                    "Steady-State Error",
                    f"{summary['steady_err']:.2f}" if summary["steady_err"] is not None else "N/A",
                    "RPM",
                    "Sai số xác lập trung bình (10 mẫu cuối)"
                ])
                writer.writerow([])

                writer.writerow(["=== RAW DATA ==="])
                writer.writerow([
                    "time(s)", "rpm", "setpoint",
                    "kp", "ki", "kd",
                    "rise_time(s)", "overshoot(%)",
                    "settling_time(s)", "steady_error(RPM)"
                ])
                for row in self.all_data:
                    writer.writerow([
                        row["time"],
                        row["rpm"],
                        row["sp"] if row["sp"] is not None else "",
                        row["kp"] if row["kp"] is not None else "",
                        row["ki"] if row["ki"] is not None else "",
                        row["kd"] if row["kd"] is not None else "",
                        f"{row['rise_time']:.3f}"   if row["rise_time"]   is not None else "",
                        f"{row['overshoot']:.2f}"   if row["overshoot"]   is not None else "",
                        f"{row['settle_time']:.3f}" if row["settle_time"] is not None else "",
                        f"{row['steady_err']:.2f}"  if row["steady_err"]  is not None else "",
                    ])

            messagebox.showinfo("Xuất thành công",
                                f"Đã lưu {len(self.all_data)} mẫu\nvào {os.path.basename(path)}")
            self._log(f"[CSV] Đã xuất → {path}")

        except Exception as e:
            messagebox.showerror("Lỗi xuất file", str(e))

    # ─── LOG ──────────────────────────────────────────────────
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


if __name__ == "__main__":
    root = tk.Tk()
    root.geometry("1400x800")
    app = PIDDashboard(root)
    root.mainloop()
