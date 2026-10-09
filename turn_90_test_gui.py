from __future__ import annotations

import csv
import math
import queue
import threading
import time
import tkinter as tk
from datetime import datetime
from pathlib import Path
from tkinter import messagebox, scrolledtext, ttk

from straight_line_test_gui import SerialWorker, available_ports, host_timestamp


SPEED_LEVELS = {
    "slow": ("慢速", 0.30),
    "medium": ("中速", 0.55),
    "fast": ("快速", 0.85),
}
TARGET_ANGLE_DEG = 90.0
TURN_KP_RADPS_PER_DEG = 0.035
TURN_MIN_AZ_RADPS = 0.15
ANGLE_TOLERANCE_DEG = 0.8
SETTLE_GYRO_DPS = 2.0
STOP_SETTLE_MS = 750
ATT_MAX_AGE_S = 0.30
WRONG_DIRECTION_DEG = 4.0
MAX_RELATIVE_ANGLE_DEG = 115.0
# Host az and ATT yaw use the same numeric sign after the lower-controller
# BMI088 correction.  Physical left/right is mapped from the chassis test.
SERIAL_AZ_TO_YAW_SIGN = 1.0
# The physical left/right labels on this chassis are opposite to the ATT yaw
# sign observed during the turn test.  Keep ATT feedback closed-loop and map
# the UI labels explicitly instead of changing the serial-control polarity.
PHYSICAL_LEFT_ATT_SIGN = -1.0
PHYSICAL_RIGHT_ATT_SIGN = 1.0

CSV_FIELDS = (
    "host_time",
    "host_elapsed_s",
    "run_id",
    "turn_active",
    "settling",
    "direction",
    "speed_level",
    "max_az_radps",
    "start_yaw_deg",
    "target_yaw_deg",
    "current_yaw_deg",
    "relative_yaw_deg",
    "progress_deg",
    "remaining_deg",
    "gyro_z_dps",
    "bias_z_dps",
    "command_az_radps",
    "frame_type",
    "raw_frame",
)


class Turn90TestApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.session_start_s = time.monotonic()
        self.events: queue.Queue[tuple[str, object]] = queue.Queue()
        self.stop_event = threading.Event()
        self.worker: SerialWorker | None = None
        self.connected = False
        self.closing = False
        self.port_labels: dict[str, str] = {}

        self.run_id = 0
        self.turn_active = False
        self.settling = False
        self.finish_after_id: str | None = None
        self.turn_start_s: float | None = None
        self.turn_timeout_s = 0.0
        self.direction_name = ""
        self.direction_sign = 0.0
        self.active_speed_key = "slow"
        self.start_yaw_deg: float | None = None
        self.target_yaw_deg: float | None = None
        self.current_yaw_deg: float | None = None
        self.current_gyro_z_dps = 0.0
        self.command_az_radps = 0.0
        self.last_att: list[int] | None = None
        self.last_att_received_s: float | None = None
        self.last_ctl: list[int] | None = None
        self.last_mtr: list[int] | None = None

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        log_dir = Path(__file__).resolve().parent / "turn_test_logs"
        log_dir.mkdir(parents=True, exist_ok=True)
        base_name = f"turn_90_test_{timestamp}"
        self.raw_path = log_dir / f"{base_name}_raw.txt"
        self.csv_path = log_dir / f"{base_name}_telemetry.csv"
        self.raw_file = self.raw_path.open("w", encoding="utf-8", newline="")
        self.csv_file = self.csv_path.open("w", encoding="utf-8", newline="")
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=CSV_FIELDS)
        self.csv_writer.writeheader()
        self.csv_file.flush()
        self.raw_file.write(
            "# Closed-loop 90-degree turn test; vx=0; "
            "slow=0.30, medium=0.55, fast=0.85 rad/s\n"
        )
        self.raw_file.write("host_time\tdirection\tframe\n")
        self.raw_file.flush()

        self.port_var = tk.StringVar()
        self.speed_var = tk.StringVar(value="slow")
        self.connection_var = tk.StringVar(value="未连接")
        self.command_var = tk.StringVar(value="CMD: vx=0.000  az=0.000 rad/s")
        self.attitude_var = tk.StringVar(value="ATT: --")
        self.turn_var = tk.StringVar(value="转向: --")
        self.motor_var = tk.StringVar(value="MTR: --")
        self.log_var = tk.StringVar(
            value=f"Raw: {self.raw_path} | CSV: {self.csv_path}"
        )

        self.build_ui()
        self.refresh_ports()
        self.root.protocol("WM_DELETE_WINDOW", self.close_window)
        self.root.bind("<Escape>", lambda _event: self.request_stop("Escape", False))
        self.root.after(50, self.poll_events)
        self.root.after(50, self.safety_tick)

    def build_ui(self) -> None:
        self.root.title("左右90度转向测试")
        self.root.geometry("980x680")
        self.root.minsize(780, 540)

        main = ttk.Frame(self.root, padding=12)
        main.grid(row=0, column=0, sticky="nsew")
        self.root.rowconfigure(0, weight=1)
        self.root.columnconfigure(0, weight=1)
        main.columnconfigure(1, weight=1)
        main.rowconfigure(6, weight=1)

        ttk.Label(main, text="串口").grid(row=0, column=0, sticky="w")
        self.port_combo = ttk.Combobox(
            main, textvariable=self.port_var, state="readonly", width=55
        )
        self.port_combo.grid(row=0, column=1, sticky="ew", padx=8)
        self.refresh_button = ttk.Button(main, text="刷新", command=self.refresh_ports)
        self.refresh_button.grid(row=0, column=2, padx=(0, 8))
        self.connect_button = ttk.Button(main, text="连接", command=self.toggle_connection)
        self.connect_button.grid(row=0, column=3)

        controls = ttk.LabelFrame(main, text="90度原地转向", padding=10)
        controls.grid(row=1, column=0, columnspan=4, sticky="ew", pady=(12, 8))
        ttk.Label(controls, text="速度").grid(row=0, column=0, sticky="w")
        for column, (key, (label, az_radps)) in enumerate(SPEED_LEVELS.items(), start=1):
            ttk.Radiobutton(
                controls,
                text=f"{label} {az_radps:.2f} rad/s",
                variable=self.speed_var,
                value=key,
            ).grid(row=0, column=column, padx=(8, 4), sticky="w")

        self.left_button = ttk.Button(
            controls,
            text="左转 90°",
            command=lambda: self.start_turn("left", PHYSICAL_LEFT_ATT_SIGN),
        )
        self.right_button = ttk.Button(
            controls,
            text="右转 90°",
            command=lambda: self.start_turn("right", PHYSICAL_RIGHT_ATT_SIGN),
        )
        self.stop_button = ttk.Button(
            controls,
            text="立即停止",
            command=lambda: self.request_stop("Stop button", False),
            state="disabled",
        )
        self.left_button.grid(row=1, column=1, pady=(12, 0), padx=4, sticky="ew")
        self.right_button.grid(row=1, column=2, pady=(12, 0), padx=4, sticky="ew")
        self.stop_button.grid(row=1, column=3, pady=(12, 0), padx=4, sticky="ew")
        for column in range(1, 4):
            controls.columnconfigure(column, weight=1)

        status = ttk.LabelFrame(main, text="状态", padding=10)
        status.grid(row=2, column=0, columnspan=4, sticky="ew", pady=(0, 8))
        status.columnconfigure(0, weight=1)
        ttk.Label(status, textvariable=self.connection_var).grid(row=0, column=0, sticky="w")
        ttk.Label(status, textvariable=self.command_var, font=("Consolas", 10)).grid(
            row=1, column=0, sticky="w", pady=(4, 0)
        )
        ttk.Label(status, textvariable=self.attitude_var, font=("Consolas", 10)).grid(
            row=2, column=0, sticky="w", pady=(4, 0)
        )
        ttk.Label(status, textvariable=self.turn_var, font=("Consolas", 10)).grid(
            row=3, column=0, sticky="w", pady=(4, 0)
        )
        ttk.Label(status, textvariable=self.motor_var, font=("Consolas", 10)).grid(
            row=4, column=0, sticky="w", pady=(4, 0)
        )

        ttk.Label(main, text="日志文件").grid(row=3, column=0, sticky="w")
        ttk.Entry(main, textvariable=self.log_var, state="readonly").grid(
            row=3, column=1, columnspan=3, sticky="ew", padx=(8, 0)
        )
        ttk.Label(main, text="串口数据流（Esc立即停车）").grid(
            row=5, column=0, columnspan=4, sticky="w", pady=(10, 0)
        )
        self.stream_text = scrolledtext.ScrolledText(
            main, height=18, wrap=tk.NONE, font=("Consolas", 9), state="disabled"
        )
        self.stream_text.grid(row=6, column=0, columnspan=4, sticky="nsew", pady=(4, 0))
        self.update_controls()

    def selected_port(self) -> str:
        return self.port_labels.get(self.port_var.get(), "")

    def refresh_ports(self) -> None:
        previous_device = self.selected_port()
        self.port_labels.clear()
        selected_label = ""
        for device, label in available_ports():
            self.port_labels[label] = device
            if device == previous_device:
                selected_label = label
        labels = list(self.port_labels)
        self.port_combo["values"] = labels
        self.port_var.set(selected_label or (labels[0] if labels else ""))
        if self.worker is None:
            self.connect_button.configure(state="normal" if labels else "disabled")
        self.connection_var.set(
            f"发现 {len(labels)} 个串口" if labels else "未发现串口"
        )

    def update_controls(self) -> None:
        busy = self.turn_active or self.settling
        can_start = self.connected and not busy
        self.left_button.configure(state="normal" if can_start else "disabled")
        self.right_button.configure(state="normal" if can_start else "disabled")
        self.stop_button.configure(state="normal" if busy else "disabled")
        self.port_combo.configure(state="disabled" if self.worker else "readonly")
        self.refresh_button.configure(state="disabled" if self.worker else "normal")
        self.connect_button.configure(
            text="断开" if self.worker else "连接",
            state="normal" if self.worker or self.port_labels else "disabled",
        )

    def toggle_connection(self) -> None:
        if self.worker is not None:
            self.disconnect()
            return
        port = self.selected_port()
        if not port:
            messagebox.showerror("转向测试", "请先选择串口。")
            return
        self.last_att = None
        self.last_ctl = None
        self.last_mtr = None
        self.stop_event = threading.Event()
        self.worker = SerialWorker(port, self.events, self.stop_event)
        self.connection_var.set(f"正在打开 {port}...")
        self.write_raw("EVENT", f"CONNECT_REQUEST,{port}")
        self.update_controls()
        self.worker.start()

    def disconnect(self) -> None:
        if self.worker is None:
            return
        self.request_stop("Disconnect", False)
        self.connection_var.set("正在断开并持续发送零速命令...")
        self.stop_event.set()

    def telemetry_ready(self) -> bool:
        now = time.monotonic()
        if (
            self.last_att is None
            or self.last_att_received_s is None
            or now - self.last_att_received_s > ATT_MAX_AGE_S
            or (self.last_att[2] & 0x0D) != 0x0D
            or (self.last_att[2] & 0x60) != 0
        ):
            messagebox.showerror("转向测试", "BMI088 yaw/bias 尚未有效。")
            return False
        if (
            self.last_ctl is None
            or self.last_ctl[2] != 1
            or self.last_ctl[3] != 0
            or self.last_ctl[4] != 1
        ):
            messagebox.showerror("转向测试", "下位机未处于正常导航模式。")
            return False
        if (
            self.last_mtr is None
            or self.last_mtr[9] != 1
            or self.last_mtr[10] != 1
            or (self.last_mtr[2] & 0x0A) != 0
        ):
            messagebox.showerror("转向测试", "左右电机反馈未同时在线。")
            return False
        if abs(self.last_ctl[10]) > 10 or abs(self.last_ctl[11]) > 10:
            messagebox.showerror("转向测试", "车轮尚未停止。")
            return False
        return True

    def start_turn(self, direction_name: str, direction_sign: float) -> None:
        if self.worker is None or not self.connected or self.turn_active or self.settling:
            return
        if not self.telemetry_ready():
            return
        assert self.last_att is not None
        speed_key = self.speed_var.get()
        if speed_key not in SPEED_LEVELS:
            return
        self.active_speed_key = speed_key
        speed_label, max_az = SPEED_LEVELS[speed_key]
        self.run_id += 1
        self.turn_active = True
        self.direction_sign = direction_sign
        self.direction_name = direction_name
        self.start_yaw_deg = self.last_att[3] / 1000.0
        self.target_yaw_deg = self.start_yaw_deg + direction_sign * TARGET_ANGLE_DEG
        self.current_yaw_deg = self.start_yaw_deg
        self.current_gyro_z_dps = self.last_att[4] / 1000.0 - self.last_att[5] / 1000.0
        self.turn_start_s = time.monotonic()
        nominal_s = math.radians(TARGET_ANGLE_DEG) / max_az
        self.turn_timeout_s = max(8.0, nominal_s * 2.5 + 3.0)
        self.command_az_radps = SERIAL_AZ_TO_YAW_SIGN * direction_sign * max_az
        self.worker.set_command(0.0, self.command_az_radps)
        self.write_raw(
            "EVENT",
            f"TURN_START,{self.run_id},{self.direction_name},{speed_key},"
            f"max_az={max_az:.3f},start={self.start_yaw_deg:.3f},"
            f"target={self.target_yaw_deg:.3f},timeout={self.turn_timeout_s:.3f}",
        )
        self.command_var.set(f"CMD: vx=0.000  az={self.command_az_radps:+.3f} rad/s")
        self.turn_var.set(
            f"转向: {self.direction_name} {speed_label}  start={self.start_yaw_deg:+.2f}°  "
            f"target={self.target_yaw_deg:+.2f}°"
        )
        self.update_controls()

    def control_turn(self, yaw_deg: float, gyro_z_dps: float) -> None:
        if not self.turn_active or self.worker is None:
            return
        assert self.start_yaw_deg is not None and self.target_yaw_deg is not None
        speed_key = self.active_speed_key
        _speed_label, max_az = SPEED_LEVELS[speed_key]
        relative = yaw_deg - self.start_yaw_deg
        progress = self.direction_sign * relative
        error = self.target_yaw_deg - yaw_deg
        elapsed = time.monotonic() - self.turn_start_s if self.turn_start_s else 0.0

        if elapsed > 0.5 and progress < -WRONG_DIRECTION_DEG:
            self.request_stop("Wrong turn direction", False)
            return
        if abs(relative) > MAX_RELATIVE_ANGLE_DEG:
            self.request_stop("Turn angle safety limit", False)
            return
        if abs(error) <= ANGLE_TOLERANCE_DEG:
            self.command_az_radps = 0.0
            self.worker.set_command(0.0, 0.0)
            self.command_var.set("CMD: vx=0.000  az=0.000 rad/s（等待停稳）")
            if abs(gyro_z_dps) <= SETTLE_GYRO_DPS:
                self.request_stop("Target reached", True)
            return

        magnitude = TURN_KP_RADPS_PER_DEG * abs(error)
        magnitude = min(max_az, max(TURN_MIN_AZ_RADPS, magnitude))
        self.command_az_radps = SERIAL_AZ_TO_YAW_SIGN * math.copysign(magnitude, error)
        self.worker.set_command(0.0, self.command_az_radps)
        self.command_var.set(f"CMD: vx=0.000  az={self.command_az_radps:+.3f} rad/s")
        self.turn_var.set(
            f"转向: progress={progress:+.2f}°  relative={relative:+.2f}°  "
            f"remaining={error:+.2f}°  gyro={gyro_z_dps:+.2f}°/s"
        )

    def request_stop(self, reason: str, success: bool) -> None:
        if self.worker is not None:
            self.worker.set_command(0.0, 0.0)
        self.command_az_radps = 0.0
        self.command_var.set("CMD: vx=0.000  az=0.000 rad/s")
        if not self.turn_active and not self.settling:
            return
        self.turn_active = False
        self.settling = True
        relative = (
            self.current_yaw_deg - self.start_yaw_deg
            if self.current_yaw_deg is not None and self.start_yaw_deg is not None
            else 0.0
        )
        self.write_raw(
            "EVENT",
            f"TURN_STOP_REQUEST,{self.run_id},{reason},success={int(success)},"
            f"relative={relative:.3f},gyro={self.current_gyro_z_dps:.3f}",
        )
        self.turn_var.set(f"正在停车: {reason}  relative={relative:+.2f}°")
        self.update_controls()
        if self.finish_after_id is not None:
            self.root.after_cancel(self.finish_after_id)
        self.finish_after_id = self.root.after(
            STOP_SETTLE_MS, lambda: self.finish_turn(reason, success)
        )

    def finish_turn(self, reason: str, success: bool) -> None:
        self.finish_after_id = None
        if not self.settling:
            return
        self.settling = False
        relative = (
            self.current_yaw_deg - self.start_yaw_deg
            if self.current_yaw_deg is not None and self.start_yaw_deg is not None
            else 0.0
        )
        error = self.direction_sign * TARGET_ANGLE_DEG - relative
        self.write_raw(
            "EVENT",
            f"TURN_STOP,{self.run_id},{reason},success={int(success)},"
            f"relative={relative:.3f},final_error={error:.3f}",
        )
        self.turn_var.set(
            f"完成: {self.direction_name}  relative={relative:+.2f}°  "
            f"error={error:+.2f}°  ({reason})"
        )
        self.update_controls()

    def safety_tick(self) -> None:
        if self.turn_active:
            now = time.monotonic()
            if self.last_att_received_s is None or now - self.last_att_received_s > ATT_MAX_AGE_S:
                self.request_stop("ATT timeout", False)
            elif self.turn_start_s is not None and now - self.turn_start_s > self.turn_timeout_s:
                self.request_stop("Turn timeout", False)
        if self.root.winfo_exists():
            self.root.after(50, self.safety_tick)

    def handle_att(self, values: list[int], received_s: float) -> None:
        self.last_att = values
        self.last_att_received_s = received_s
        yaw_deg = values[3] / 1000.0
        gyro_z_dps = values[4] / 1000.0
        bias_z_dps = values[5] / 1000.0
        corrected_gyro = gyro_z_dps - bias_z_dps
        self.current_yaw_deg = yaw_deg
        self.current_gyro_z_dps = corrected_gyro
        self.attitude_var.set(
            f"ATT: status=0x{values[2]:02X} yaw={yaw_deg:+.3f}°  "
            f"gyro={gyro_z_dps:+.3f}°/s bias={bias_z_dps:+.3f}°/s"
        )
        if self.turn_active:
            if (values[2] & 0x0D) != 0x0D or (values[2] & 0x60) != 0:
                self.request_stop("Invalid BMI088 state", False)
            else:
                self.control_turn(yaw_deg, corrected_gyro)

    def handle_ctl(self, values: list[int]) -> None:
        self.last_ctl = values
        if self.turn_active and (values[2] != 1 or values[3] != 0 or values[4] != 1):
            self.request_stop("Invalid CTL state", False)

    def handle_mtr(self, values: list[int]) -> None:
        self.last_mtr = values
        self.motor_var.set(
            f"MTR: status=0x{values[2]:02X} online={values[9]}/{values[10]}  "
            f"speed-loop-I={values[5]}/{values[6]} current={values[3]}/{values[4]}"
        )
        if self.turn_active:
            if values[9] != 1 or values[10] != 1:
                self.request_stop("Motor feedback offline", False)
            elif values[2] & 0x0A:
                self.request_stop("Lower drive fault", False)

    def log_frame(self, timestamp: str, received_s: float, frame_type: str, line: str) -> None:
        relative = (
            self.current_yaw_deg - self.start_yaw_deg
            if self.current_yaw_deg is not None and self.start_yaw_deg is not None
            else ""
        )
        progress = (
            self.direction_sign * float(relative) if relative != "" else ""
        )
        remaining = (
            self.target_yaw_deg - self.current_yaw_deg
            if self.target_yaw_deg is not None and self.current_yaw_deg is not None
            else ""
        )
        speed_key = self.active_speed_key
        max_az = SPEED_LEVELS.get(speed_key, ("", 0.0))[1]
        self.csv_writer.writerow(
            {
                "host_time": timestamp,
                "host_elapsed_s": f"{received_s - self.session_start_s:.6f}",
                "run_id": self.run_id if self.run_id else "",
                "turn_active": int(self.turn_active),
                "settling": int(self.settling),
                "direction": self.direction_name,
                "speed_level": speed_key,
                "max_az_radps": f"{max_az:.6f}",
                "start_yaw_deg": "" if self.start_yaw_deg is None else f"{self.start_yaw_deg:.6f}",
                "target_yaw_deg": "" if self.target_yaw_deg is None else f"{self.target_yaw_deg:.6f}",
                "current_yaw_deg": "" if self.current_yaw_deg is None else f"{self.current_yaw_deg:.6f}",
                "relative_yaw_deg": "" if relative == "" else f"{float(relative):.6f}",
                "progress_deg": "" if progress == "" else f"{float(progress):.6f}",
                "remaining_deg": "" if remaining == "" else f"{float(remaining):.6f}",
                "gyro_z_dps": f"{self.current_gyro_z_dps:.6f}",
                "bias_z_dps": "" if self.last_att is None else f"{self.last_att[5] / 1000.0:.6f}",
                "command_az_radps": f"{self.command_az_radps:.6f}",
                "frame_type": frame_type,
                "raw_frame": line,
            }
        )
        self.csv_file.flush()

    def handle_line(self, timestamp: str, received_s: float, line: str) -> None:
        self.write_raw("RX", line, timestamp)
        fields = [field.strip() for field in line.split(",")]
        frame_type = fields[0] if fields else ""
        try:
            if frame_type == "ATT" and len(fields) == 8:
                self.handle_att([int(field, 10) for field in fields[1:]], received_s)
            elif frame_type == "CTL" and len(fields) == 15:
                self.handle_ctl([int(field, 10) for field in fields[1:]])
            elif frame_type == "MTR" and len(fields) == 25:
                self.handle_mtr([int(field, 10) for field in fields[1:]])
        except ValueError:
            self.append_stream(f"PARSE ERROR: {line}")
        self.log_frame(timestamp, received_s, frame_type, line)
        if frame_type in ("ATT", "CTL", "MTR", "ENC", "HLD", "SYN"):
            self.append_stream(f"{timestamp}  {line}")

    def write_raw(self, direction: str, frame: str, timestamp: str | None = None) -> None:
        self.raw_file.write(f"{timestamp or host_timestamp()}\t{direction}\t{frame}\n")
        self.raw_file.flush()

    def append_stream(self, line: str) -> None:
        self.stream_text.configure(state="normal")
        self.stream_text.insert(tk.END, line + "\n")
        if int(self.stream_text.index("end-1c").split(".")[0]) > 700:
            self.stream_text.delete("1.0", "151.0")
        self.stream_text.see(tk.END)
        self.stream_text.configure(state="disabled")

    def poll_events(self) -> None:
        try:
            while True:
                kind, value = self.events.get_nowait()
                if kind == "connected":
                    self.connected = True
                    self.connection_var.set(f"已连接 {value}，持续发送零速")
                    self.write_raw("EVENT", f"CONNECTED,{value}")
                    self.update_controls()
                elif kind == "tx":
                    timestamp, command_text = value  # type: ignore[misc]
                    if self.turn_active or self.settling or str(command_text) != "0.000000,0.000000":
                        self.write_raw("TX", str(command_text), str(timestamp))
                elif kind == "line":
                    timestamp, received_s, line = value  # type: ignore[misc]
                    self.handle_line(str(timestamp), float(received_s), str(line))
                elif kind == "notice":
                    self.write_raw("NOTICE", str(value))
                    self.append_stream(str(value))
                elif kind == "disconnected":
                    error = str(value)
                    if self.turn_active or self.settling:
                        self.request_stop("Serial disconnected", False)
                    self.connected = False
                    self.worker = None
                    self.connection_var.set(f"已断开: {error}" if error else "已断开")
                    self.write_raw("EVENT", f"DISCONNECTED,{error}")
                    self.update_controls()
                    if self.closing:
                        self.finish_close()
                        return
        except queue.Empty:
            pass
        if self.root.winfo_exists():
            self.root.after(50, self.poll_events)

    def close_window(self) -> None:
        if self.worker is not None:
            self.closing = True
            self.disconnect()
            return
        self.finish_close()

    def finish_close(self) -> None:
        if self.finish_after_id is not None:
            try:
                self.root.after_cancel(self.finish_after_id)
            except tk.TclError:
                pass
            self.finish_after_id = None
        if not self.raw_file.closed:
            self.write_raw("EVENT", "SESSION_END")
            self.raw_file.close()
        if not self.csv_file.closed:
            self.csv_file.close()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    Turn90TestApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
