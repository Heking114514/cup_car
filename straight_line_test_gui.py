from __future__ import annotations

import csv
import math
import queue
import re
import threading
import time
import tkinter as tk
from datetime import datetime
from pathlib import Path
from tkinter import messagebox, scrolledtext, ttk

import serial
from serial.tools import list_ports
from turn_response_test import RATES as TURN_RATES, SUMMARY_FIELDS as TURN_SUMMARY_FIELDS
from turn_response_test import Sweep as TurnSweep, Telemetry as TurnTelemetry

MODE_STRAIGHT = "直行：90秒阶梯测试"
MODE_TURN = "转弯：四档90°往返"


BAUD_RATE = 115200
COMMAND_PERIOD_S = 0.1
SERIAL_TIMEOUT_S = 0.02
STOP_COMMAND = b"0.000000,0.000000\r\n"
# 2026-10-11 最新标定：左右轮各 3 次、每次转 10 圈。
LEFT_ENCODER_COUNTS_PER_WHEEL_REV = 10232.7667
RIGHT_ENCODER_COUNTS_PER_WHEEL_REV = 10235.2333
WHEEL_CIRCUMFERENCE_M = math.pi * 0.0604
ENCODER_MAX_INTERVAL_MS = 500
STOP_SETTLE_MS = 750
MAX_LINEAR_SPEED_MPS = 0.40
MAX_ANGULAR_SPEED_RADPS = 1.50
DEFAULT_LEFT_TURN_RATE_RADPS = 0.60
PROFILE_SPEEDS_MPS = (0.03, 0.06, 0.09, 0.12, 0.16)
PROFILE_STAGE_DURATIONS_S = (5.0, 10.0, 15.0, 30.0, 30.0)
if len(PROFILE_SPEEDS_MPS) != len(PROFILE_STAGE_DURATIONS_S):
    raise RuntimeError("speed profile and duration profile lengths differ")
PROFILE_TOTAL_S = sum(PROFILE_STAGE_DURATIONS_S)
LEFT_TURN_ANGLE_DEG = 90.0
TURN_TIMEOUT_S = 12.0
IMU_YAW_VALID = 0x04
IMU_BIAS_VALID = 0x08
IMU_HEADING_READY = IMU_YAW_VALID | IMU_BIAS_VALID

CSV_FIELDS = (
    "host_time",
    "host_elapsed_s",
    "run_id",
    "run_active",
    "test_mode",
    "turn_angle_source",
    "turn_control_yaw_deg",
    "turn_relative_yaw_deg",
    "requested_vx_mps",
    "requested_az_radps",
    "route_phase",
    "route_phase_elapsed_s",
    "frame_type",
    "parse_ok",
    "parse_error",
    "mcu_time_ms",
    "sequence",
    "left_total_count",
    "right_total_count",
    "left_unwrapped_count",
    "right_unwrapped_count",
    "run_left_delta_count",
    "run_right_delta_count",
    "run_phase_error_count",
    "run_left_wheel_rev",
    "run_right_wheel_rev",
    "run_phase_error_wheel_rev",
    "run_left_distance_m",
    "run_right_distance_m",
    "run_phase_error_m",
    "mode",
    "emergency",
    "command_valid",
    "command_age_ms",
    "command_vx_mmps",
    "command_az_mradps",
    "target_left_mmps",
    "target_right_mmps",
    "measured_left_mmps",
    "measured_right_mmps",
    "left_speed_error_mmps",
    "right_speed_error_mmps",
    "wheel_speed_difference_mmps",
    "wheel_speed_difference_ratio",
    "ctl_host_interval_ms",
    "left_output",
    "right_output",
    "drive_status",
    "left_current",
    "right_current",
    "left_integral_output",
    "right_integral_output",
    "left_temperature_C",
    "right_temperature_C",
    "left_feedback_online",
    "right_feedback_online",
    "left_feedback_age_ms",
    "right_feedback_age_ms",
    "left_feedback_count",
    "right_feedback_count",
    "can_tx_success_count",
    "can_tx_failure_count",
    "can_last_tx_failure_age_ms",
    "can_state",
    "can_hal_error_code",
    "can_protocol_last_error_code",
    "can_bus_off",
    "can_error_passive",
    "can_tx_fifo_free_level",
    "raw_imu_status",
    "raw_gyro_x_count",
    "raw_gyro_y_count",
    "raw_gyro_z_count",
    "raw_temperature_cC",
    "imu_status",
    "imu_yaw_mdeg",
    "imu_yaw_deg",
    "imu_gyro_z_mdps",
    "imu_gyro_z_dps",
    "imu_bias_z_mdps",
    "imu_bias_z_dps",
    "imu_startup_samples",
    "heading_active",
    "heading_ref_mdeg",
    "heading_ref_deg",
    "heading_error_mdeg",
    "heading_error_deg",
    "heading_correction_milli_ref",
    "heading_correction_ref",
    "heading_i_term_milli_ref",
    "heading_i_term_ref",
    "heading_fused_error_mdeg",
    "heading_fused_error_deg",
    "sync_active",
    "sync_phase_error_mdeg",
    "sync_phase_error_deg",
    "sync_correction_milli_ref",
    "sync_correction_ref",
    "rfid_status",
    "rfid_protocol_status",
    "rfid_tag_type",
    "rfid_sak",
    "rfid_number",
    "rfid_uid_hex",
    "raw_frame",
)


def host_timestamp() -> str:
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


def signed_int32_delta(end: int, start: int) -> int:
    value = (end - start) & 0xFFFFFFFF
    return value if value <= 0x7FFFFFFF else value - 0x100000000


def wrap_deg_delta(current_deg: float, start_deg: float) -> float:
    return (current_deg - start_deg + 180.0) % 360.0 - 180.0


def read_tuning_macros(project_dir: Path) -> dict[str, str]:
    sources = {
        "application/robot_def.h": (
            "CHASSIS_WHEEL_SYNC_KP",
            "CHASSIS_WHEEL_SYNC_KI",
            "CHASSIS_WHEEL_SYNC_MAX_RATIO",
            "CHASSIS_WHEEL_SYNC_PHASE_POSITION_KP",
            "CHASSIS_USE_BMI088_YAW_HOLD",
            "CHASSIS_YAW_HOLD_KP",
            "CHASSIS_YAW_HOLD_KI",
            "CHASSIS_YAW_HOLD_KD",
            "CHASSIS_YAW_HOLD_DIR",
            "CHASSIS_YAW_HOLD_MAX_RATIO",
            "CHASSIS_REFERENCE_SCALE",
            "CHASSIS_WIDTH",
            "WHEEL_WIDTH",
            "TRACK_WIDTH",
            "RADIUS_WHEEL",
            "MOTOR_ENCODER_COUNTS_PER_REV",
            "LEFT_ENCODER_COUNTS_PER_WHEEL_REV",
            "RIGHT_ENCODER_COUNTS_PER_WHEEL_REV",
            "LEFT_REDUCTION_RATIO_WHEEL",
            "RIGHT_REDUCTION_RATIO_WHEEL",
        ),
        "application/chassis/chassis.c": (
            "CHASSIS_STRAIGHT_LEFT_TRIM",
            "CHASSIS_STRAIGHT_RIGHT_TRIM",
        ),
    }
    result: dict[str, str] = {}
    for relative_path, names in sources.items():
        path = project_dir / relative_path
        text = path.read_text(encoding="utf-8") if path.is_file() else ""
        for name in names:
            match = re.search(rf"^\s*#define\s+{name}\s+([^\r\n]+)", text, re.MULTILINE)
            if match:
                result[name] = match.group(1).split("//", 1)[0].strip()
            else:
                result[name] = "not_found"
    return result


def available_ports() -> list[tuple[str, str]]:
    ports = list(list_ports.comports())
    ports.sort(
        key=lambda port: (
            0 if (port.vid, port.pid) == (0x0483, 0x5740) else 1,
            port.device,
        )
    )
    result: list[tuple[str, str]] = []
    for port in ports:
        identity = ""
        if port.vid is not None and port.pid is not None:
            identity = f" VID:PID={port.vid:04X}:{port.pid:04X}"
        result.append(
            (port.device, f"{port.device} | {port.description or 'Serial port'}{identity}")
        )
    return result


class SerialWorker(threading.Thread):
    def __init__(
        self,
        port: str,
        events: queue.Queue[tuple[str, object]],
        stop_event: threading.Event,
    ) -> None:
        super().__init__(daemon=True)
        self.port = port
        self.events = events
        self.stop_event = stop_event
        self.command_lock = threading.Lock()
        self.command = STOP_COMMAND
        self.command_text = "0.000000,0.000000"
        self.command_generation = 0

    def emit(self, kind: str, value: object) -> None:
        self.events.put((kind, value))

    def set_command(self, vx_mps: float, az_radps: float) -> None:
        command_text = f"{vx_mps:.6f},{az_radps:.6f}"
        with self.command_lock:
            self.command_text = command_text
            self.command = (command_text + "\r\n").encode("ascii")
            self.command_generation += 1

    def command_snapshot(self) -> tuple[bytes, str, int]:
        with self.command_lock:
            return self.command, self.command_text, self.command_generation

    def send_stop(self, device: serial.Serial, repeat: int) -> None:
        for index in range(repeat):
            device.write(STOP_COMMAND)
            self.emit("tx", (host_timestamp(), "0.000000,0.000000"))
            if index + 1 < repeat:
                time.sleep(0.03)

    def run(self) -> None:
        device: serial.Serial | None = None
        error = ""
        buffer = bytearray()
        try:
            device = serial.Serial(
                port=self.port,
                baudrate=BAUD_RATE,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=SERIAL_TIMEOUT_S,
                write_timeout=0.25,
            )
            device.reset_input_buffer()
            device.reset_output_buffer()
            self.send_stop(device, repeat=3)
            self.emit("connected", self.port)

            sent_generation = -1
            next_command_s = time.monotonic()
            while not self.stop_event.is_set():
                now_s = time.monotonic()
                command, command_text, generation = self.command_snapshot()
                if generation != sent_generation or now_s >= next_command_s:
                    device.write(command)
                    self.emit("tx", (host_timestamp(), command_text))
                    sent_generation = generation
                    next_command_s = now_s + COMMAND_PERIOD_S

                chunk = device.read(device.in_waiting or 1)
                if not chunk:
                    continue
                buffer.extend(chunk)
                if len(buffer) > 8192:
                    buffer.clear()
                    self.emit("notice", "RX buffer overflow; partial line discarded")
                    continue

                while b"\n" in buffer:
                    raw_line, _, remainder = buffer.partition(b"\n")
                    buffer = bytearray(remainder)
                    line = raw_line.rstrip(b"\r").decode("ascii", errors="replace")
                    if line:
                        self.emit("line", (host_timestamp(), time.monotonic(), line))
        except (serial.SerialException, OSError) as exc:
            error = str(exc)
        finally:
            if device is not None and device.is_open:
                try:
                    self.send_stop(device, repeat=5)
                except (serial.SerialException, OSError):
                    pass
                try:
                    device.close()
                except (serial.SerialException, OSError) as exc:
                    if not error:
                        error = f"close failed: {exc}"
            self.emit("disconnected", error)


class StraightLineTestApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.session_start_s = time.monotonic()
        self.events: queue.Queue[tuple[str, object]] = queue.Queue()
        self.stop_event = threading.Event()
        self.worker: SerialWorker | None = None
        self.connected = False
        self.imu_ready = False
        self.imu_invalid_samples = 0
        self.closing = False
        self.port_labels: dict[str, str] = {}

        self.run_active = False
        self.stop_pending = False
        self.stop_after_id: str | None = None
        self.run_id = 0
        self.run_start_s: float | None = None
        self.requested_vx_mps = 0.0
        self.requested_az_radps = 0.0
        self.run_start_left: int | None = None
        self.run_start_right: int | None = None
        self.route_phase = "idle"
        self.route_phase_start_s: float | None = None
        self.route_after_id: str | None = None
        self.progress_after_id: str | None = None
        self.route_vx_mps = 0.0
        self.route_turn_rate_radps = 0.0
        self.profile_index = -1
        self.turn_start_yaw_deg: float | None = None
        self.turn_target_delta_deg = 0.0
        self.run_mode = MODE_STRAIGHT
        self.turn_sweep: TurnSweep | None = None
        self.turn_telemetry = TurnTelemetry()
        self.turn_record_count = 0
        self.turn_state: tuple[int, str, str] | None = None

        self.last_encoder_raw: tuple[int, int] | None = None
        self.left_unwrapped: int | None = None
        self.right_unwrapped: int | None = None
        self.last_mcu_time_ms: int | None = None
        self.last_ctl: list[int] | None = None
        self.last_att: list[int] | None = None
        self.last_hld: list[int] | None = None
        self.last_syn: list[int] | None = None
        self.last_mtr: list[int] | None = None
        self.rx_line_count = 0
        self.parse_error_count = 0
        self.last_rx_monotonic_s: float | None = None
        self.last_ctl_host_s: float | None = None
        self.last_stream_telemetry_s = -math.inf
        self.run_control_samples: list[tuple[int, int, int, int]] = []

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        log_dir = Path(__file__).resolve().parent / "straight_test_logs"
        log_dir.mkdir(parents=True, exist_ok=True)
        base_name = f"straight_test_{timestamp}"
        self.raw_path = log_dir / f"{base_name}_raw.txt"
        self.text_path = log_dir / f"{base_name}_telemetry.txt"
        self.csv_path = log_dir / f"{base_name}_telemetry.csv"
        self.turn_summary_path = log_dir / f"{base_name}_turn_summary.csv"
        self.raw_file = self.raw_path.open("w", encoding="utf-8", newline="")
        self.text_file = self.text_path.open("w", encoding="utf-8", newline="")
        self.csv_file = self.csv_path.open("w", encoding="utf-8", newline="")
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=CSV_FIELDS)
        self.csv_writer.writeheader()
        self.csv_file.flush()
        self.raw_file.write(
            f"# straight speed tuning; encoder_counts_per_wheel_rev="
            f"{LEFT_ENCODER_COUNTS_PER_WHEEL_REV:.4f}/{RIGHT_ENCODER_COUNTS_PER_WHEEL_REV:.4f}; "
            f"wheel_diameter_m={WHEEL_CIRCUMFERENCE_M / math.pi:.4f}; "
            f"baud={BAUD_RATE}; command_period_s={COMMAND_PERIOD_S:.3f}; "
            f"mcu_encoder_period_ms=20; mcu_control_period_ms=100\n"
        )
        tuning = read_tuning_macros(Path(__file__).resolve().parent)
        self.raw_file.write(
            "# firmware_tuning=" + ",".join(f"{key}:{value}" for key, value in tuning.items()) + "\n"
        )
        self.raw_file.write("host_time\tdirection\tframe\n")
        self.raw_file.flush()
        self.text_file.write("host_time\trun_id\trun_active\troute_phase\tframe\n")
        self.text_file.flush()

        self.port_var = tk.StringVar()
        self.mode_var = tk.StringVar(value=MODE_STRAIGHT)
        self.vx_var = tk.StringVar(value="0.03 / 0.06 / 0.09 / 0.12 / 0.16")
        self.az_var = tk.StringVar(value="0.00")
        self.progress_var = tk.DoubleVar(value=0.0)
        self.progress_text_var = tk.StringVar(
            value=f"Idle; 0.0 / {PROFILE_TOTAL_S:.1f} s"
        )
        self.connection_var = tk.StringVar(value="Disconnected")
        self.command_var = tk.StringVar(value="Command: 0.000, 0.000")
        self.encoder_var = tk.StringVar(value="ENC: --")
        self.phase_var = tk.StringVar(value="Run delta: --")
        self.control_var = tk.StringVar(value="CTL: --")
        self.imu_var = tk.StringVar(value="ATT: --")
        self.hold_var = tk.StringVar(value="HLD: --")
        self.sync_var = tk.StringVar(value="SYN: --")
        self.motor_var = tk.StringVar(value="MTR: --")
        self.log_var = tk.StringVar(
            value=f"Raw: {self.raw_path} | TXT: {self.text_path} | CSV: {self.csv_path}"
        )

        self.build_ui()
        self.update_mode_controls()
        self.refresh_ports()
        self.root.protocol("WM_DELETE_WINDOW", self.close_window)
        self.root.bind("<Escape>", lambda _event: self.stop_run("Escape key"))
        self.root.after(10, self.poll_events)

    def build_ui(self) -> None:
        self.root.title("底盘测试：直行 / 转弯响应")
        self.root.geometry("1000x690")
        self.root.minsize(820, 590)

        main = ttk.Frame(self.root, padding=12)
        main.grid(row=0, column=0, sticky="nsew")
        self.root.rowconfigure(0, weight=1)
        self.root.columnconfigure(0, weight=1)
        main.columnconfigure(1, weight=1)
        main.rowconfigure(8, weight=1)

        ttk.Label(main, text="Serial port").grid(row=0, column=0, sticky="w")
        self.port_combo = ttk.Combobox(
            main, textvariable=self.port_var, state="readonly", width=55
        )
        self.port_combo.grid(row=0, column=1, sticky="ew", padx=8)
        self.refresh_button = ttk.Button(main, text="Refresh", command=self.refresh_ports)
        self.refresh_button.grid(row=0, column=2, padx=(0, 8))
        self.connect_button = ttk.Button(main, text="Connect", command=self.toggle_connection)
        self.connect_button.grid(row=0, column=3)

        controls = ttk.LabelFrame(main, text="测试模式与指令", padding=10)
        controls.grid(row=1, column=0, columnspan=4, sticky="ew", pady=(12, 8))
        ttk.Label(controls, text="测试模式").grid(row=0, column=0, sticky="w")
        self.mode_combo = ttk.Combobox(
            controls, textvariable=self.mode_var,
            values=(MODE_STRAIGHT, MODE_TURN), state="readonly", width=26,
        )
        self.mode_combo.grid(row=0, column=1, columnspan=3, sticky="w", padx=6, pady=(0, 8))
        self.mode_combo.bind("<<ComboboxSelected>>", lambda _event: self.update_mode_controls())
        self.vx_label = ttk.Label(controls, text="速度档位 (m/s)")
        self.vx_label.grid(row=1, column=0, sticky="w")
        self.vx_entry = ttk.Entry(controls, textvariable=self.vx_var, width=28, state="readonly")
        self.vx_entry.grid(row=1, column=1, padx=(6, 20))
        self.az_label = ttk.Label(controls, text="角速度 (rad/s)")
        self.az_label.grid(row=1, column=2, sticky="w")
        self.az_entry = ttk.Entry(controls, textvariable=self.az_var, width=24, state="readonly")
        self.az_entry.grid(row=1, column=3, padx=(6, 24))
        self.start_button = ttk.Button(
            controls, text="开始测试", command=self.start_run, state="disabled"
        )
        self.start_button.grid(row=0, column=4, padx=(0, 8))
        self.stop_button = ttk.Button(
            controls, text="停止", command=lambda: self.stop_run("Stop button"), state="disabled"
        )
        self.stop_button.grid(row=0, column=5)
        controls.columnconfigure(6, weight=1)

        progress = ttk.Frame(main)
        progress.grid(row=3, column=0, columnspan=4, sticky="ew", pady=(0, 8))
        progress.columnconfigure(1, weight=1)
        ttk.Label(progress, text="测试进度").grid(row=0, column=0, sticky="w")
        self.progress_bar = ttk.Progressbar(
            progress,
            orient="horizontal",
            mode="determinate",
            maximum=PROFILE_TOTAL_S,
            variable=self.progress_var,
        )
        self.progress_bar.grid(row=0, column=1, sticky="ew", padx=(8, 12))
        ttk.Label(progress, textvariable=self.progress_text_var).grid(
            row=0, column=2, sticky="e"
        )

        state = ttk.Frame(main)
        state.grid(row=2, column=0, columnspan=4, sticky="ew", pady=(2, 4))
        state.columnconfigure(0, weight=1)
        state.columnconfigure(1, weight=1)
        ttk.Label(state, textvariable=self.connection_var).grid(row=0, column=0, sticky="w")
        ttk.Label(state, textvariable=self.command_var).grid(row=0, column=1, sticky="w")

        telemetry = ttk.LabelFrame(main, text="Live telemetry", padding=10)
        telemetry.grid(row=4, column=0, columnspan=4, sticky="ew", pady=(0, 8))
        telemetry.columnconfigure(0, weight=1)
        ttk.Label(telemetry, textvariable=self.encoder_var, font=("Consolas", 10)).grid(
            row=0, column=0, sticky="w"
        )
        ttk.Label(telemetry, textvariable=self.phase_var, font=("Consolas", 10)).grid(
            row=1, column=0, sticky="w", pady=(4, 0)
        )
        ttk.Label(telemetry, textvariable=self.control_var, font=("Consolas", 10)).grid(
            row=2, column=0, sticky="w", pady=(4, 0)
        )
        ttk.Label(telemetry, textvariable=self.imu_var, font=("Consolas", 10)).grid(
            row=3, column=0, sticky="w", pady=(4, 0)
        )
        ttk.Label(telemetry, textvariable=self.hold_var, font=("Consolas", 10)).grid(
            row=4, column=0, sticky="w", pady=(4, 0)
        )
        ttk.Label(telemetry, textvariable=self.sync_var, font=("Consolas", 10)).grid(
            row=5, column=0, sticky="w", pady=(4, 0)
        )
        ttk.Label(telemetry, textvariable=self.motor_var, font=("Consolas", 10)).grid(
            row=6, column=0, sticky="w", pady=(4, 0)
        )

        ttk.Label(main, text="Log files").grid(row=5, column=0, sticky="w")
        ttk.Entry(main, textvariable=self.log_var, state="readonly").grid(
            row=5, column=1, columnspan=3, sticky="ew", padx=(8, 0)
        )
        ttk.Label(main, text="Stream (Esc stops the run)").grid(
            row=7, column=0, columnspan=4, sticky="w", pady=(10, 0)
        )
        self.stream_text = scrolledtext.ScrolledText(
            main, height=20, wrap=tk.NONE, font=("Consolas", 9), state="disabled"
        )
        self.stream_text.grid(row=8, column=0, columnspan=4, sticky="nsew", pady=(4, 0))

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
            f"Found {len(labels)} serial port(s)" if labels else "No serial ports found"
        )

    def set_connection_controls(self, connecting: bool) -> None:
        self.port_combo.configure(state="disabled" if connecting else "readonly")
        self.refresh_button.configure(state="disabled" if connecting else "normal")
        self.connect_button.configure(text="Disconnect" if connecting else "Connect")
        self.connect_button.configure(
            state="normal" if connecting or self.port_labels else "disabled"
        )
        self.start_button.configure(
            state=(
                "normal"
                if self.connected and self.imu_ready and not self.run_active
                else "disabled"
            )
        )

    def toggle_connection(self) -> None:
        if self.worker is not None:
            self.disconnect()
            return
        port = self.selected_port()
        if not port:
            messagebox.showerror("Straight-line test", "Select a serial port first.")
            return
        self.reset_encoder_stream()
        self.imu_ready = False
        self.imu_invalid_samples = 0
        self.last_ctl = None
        self.last_att = None
        self.last_hld = None
        self.last_syn = None
        self.last_mtr = None
        self.turn_telemetry = TurnTelemetry()
        self.stop_event = threading.Event()
        self.worker = SerialWorker(port, self.events, self.stop_event)
        self.set_connection_controls(connecting=True)
        self.connection_var.set(f"Opening {port}...")
        self.write_raw(host_timestamp(), "EVENT", f"CONNECT_REQUEST,{port}")
        self.worker.start()

    def disconnect(self) -> None:
        if self.worker is None:
            return
        self.stop_run("Disconnect", settle=False)
        self.connection_var.set("Stopping link; sending zero command...")
        self.connect_button.configure(state="disabled")
        self.stop_event.set()

    def update_mode_controls(self) -> None:
        if self.run_active:
            return
        turning = self.mode_var.get() == MODE_TURN
        self.vx_var.set("0.00（原地转向）" if turning else "0.03 / 0.06 / 0.09 / 0.12 / 0.16")
        self.az_var.set("0.1 / 0.3 / 0.6 / 0.9" if turning else "0.00")
        self.start_button.configure(text="开始四档转弯" if turning else "开始90秒直行")
        self.progress_bar.configure(maximum=8 if turning else PROFILE_TOTAL_S)
        self.progress_var.set(0.0)
        self.progress_text_var.set("待开始；左转90°后返回，共8段" if turning else f"待开始；共{PROFILE_TOTAL_S:.0f}秒")

    def turn_route_step(self, now_s: float) -> None:
        sweep = self.turn_sweep
        if sweep is None:
            self.stop_run("转弯状态丢失")
            return
        telemetry = self.turn_telemetry
        try:
            command = sweep.tick(now_s, telemetry.control_yaw, telemetry.gyro, telemetry.ready(now_s))
            if (sweep.phase == "turn" and now_s - sweep.since > 1.5
                    and now_s - telemetry.ctl_seen < 0.3
                    and telemetry.command_valid
                    and abs(telemetry.command_az - command) < 0.002
                    and telemetry.targets == (0, 0)):
                raise RuntimeError("固件已收到转向指令但轮目标为零，请烧录低速转向阈值修复固件")
        except RuntimeError as exc:
            self.append_stream(f"转弯测试停止：{exc}")
            self.stop_run(str(exc))
            return
        while self.turn_record_count < len(sweep.records):
            record = sweep.records[self.turn_record_count]
            self.turn_record_count += 1
            needs_header = not self.turn_summary_path.exists()
            with self.turn_summary_path.open("a", encoding="utf-8", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=("run_id",) + TURN_SUMMARY_FIELDS)
                if needs_header:
                    writer.writeheader()
                writer.writerow({"run_id": self.run_id, **record})
            leg = "左转" if record["leg"] == "left" else "返回"
            summary = (f"{record['rate_radps']:.1f} rad/s {leg}："
                       f"耗时{record['duration_s']:.2f}s，停车误差{record['endpoint_error_deg']:+.2f}°")
            self.append_stream(summary)
            self.write_raw(host_timestamp(), "SUMMARY", "TURN_RESULT," + repr(record))
        if sweep.phase == "done":
            self.stop_run("Turn profile complete")
            return
        self.profile_index = sweep.index
        self.route_turn_rate_radps = TURN_RATES[sweep.index]
        state = (sweep.index, sweep.leg, sweep.phase)
        if state != self.turn_state:
            self.turn_state = state
            self.set_route_phase(
                f"turn_{sweep.index + 1}_{sweep.leg}_{sweep.phase}", 0.0, command,
            )
        else:
            self.worker.set_command(0.0, command)
            self.requested_vx_mps, self.requested_az_radps = 0.0, command
        self.schedule_route_step(20)

    def validate_command(self) -> tuple[float, float] | None:
        try:
            vx_mps = float(self.vx_var.get())
            az_radps = float(self.az_var.get())
        except ValueError:
            messagebox.showerror("Straight-line test", "vx and az must be numbers.")
            return None
        if not math.isfinite(vx_mps) or not math.isfinite(az_radps):
            messagebox.showerror("Straight-line test", "vx and az must be finite numbers.")
            return None
        if abs(vx_mps) < 1e-6:
            messagebox.showerror("Straight-line test", "Set a non-zero vx for a test run.")
            return None
        if abs(vx_mps) > MAX_LINEAR_SPEED_MPS:
            messagebox.showerror(
                "Straight-line test", f"|vx| must not exceed {MAX_LINEAR_SPEED_MPS:.2f} m/s."
            )
            return None
        if abs(az_radps) > MAX_ANGULAR_SPEED_RADPS:
            messagebox.showerror(
                "Straight-line test",
                f"|az| must not exceed {MAX_ANGULAR_SPEED_RADPS:.2f} rad/s.",
            )
            return None
        return vx_mps, az_radps

    def set_route_phase(self, phase: str, vx_mps: float, az_radps: float) -> None:
        if self.worker is None:
            return
        self.route_phase = phase
        self.route_phase_start_s = time.monotonic()
        self.requested_vx_mps = vx_mps
        self.requested_az_radps = az_radps
        self.worker.set_command(vx_mps, az_radps)
        stages = len(TURN_RATES) if self.run_mode == MODE_TURN else len(PROFILE_SPEEDS_MPS)
        self.command_var.set(
            f"Run {self.run_id}: stage {self.profile_index + 1}/{stages} "
            f"{phase} vx={vx_mps:.3f}, az={az_radps:.3f}"
        )
        self.write_raw(
            host_timestamp(),
            "EVENT",
            f"ROUTE_PHASE,{self.run_id},{phase},{vx_mps:.6f},{az_radps:.6f}",
        )

    def schedule_route_step(self, delay_ms: int) -> None:
        if self.route_after_id is not None:
            try:
                self.root.after_cancel(self.route_after_id)
            except tk.TclError:
                pass
        self.route_after_id = self.root.after(delay_ms, self.route_step)

    def cancel_route_timer(self) -> None:
        if self.route_after_id is None:
            return
        try:
            self.root.after_cancel(self.route_after_id)
        except tk.TclError:
            pass
        self.route_after_id = None

    def cancel_progress_timer(self) -> None:
        if self.progress_after_id is None:
            return
        try:
            self.root.after_cancel(self.progress_after_id)
        except tk.TclError:
            pass
        self.progress_after_id = None

    def update_progress(self) -> None:
        self.progress_after_id = None
        if not self.run_active or self.run_start_s is None:
            return

        elapsed_s = max(0.0, time.monotonic() - self.run_start_s)
        if self.run_mode == MODE_TURN and self.turn_sweep is not None:
            sweep = self.turn_sweep
            completed = len(sweep.records)
            fraction = 0.0
            if sweep.phase == "turn":
                span = abs(sweep.target - sweep.start_yaw)
                fraction = min(1.0, abs(self.turn_telemetry.control_yaw - sweep.start_yaw) / max(span, 1e-6))
            elif sweep.phase == "settle":
                fraction = 1.0
            self.progress_var.set(min(8, completed + fraction))
            leg = "左转" if sweep.leg == "left" else "返回"
            phase = {"ready": "等待稳定", "turn": "转动", "settle": "停车稳定", "done": "完成"}[sweep.phase]
            delta = self.turn_telemetry.control_yaw - sweep.initial if sweep.initial is not None else 0.0
            target = (sweep.target - sweep.initial) if sweep.phase in ("turn", "settle") else 0.0
            self.progress_text_var.set(
                f"{TURN_RATES[min(sweep.index, 3)]:.1f} rad/s {leg} {phase}；"
                f"陀螺积分角度{delta:+.1f}° / 目标{target:+.0f}°；{completed}/8段；{elapsed_s:.1f}s"
            )
            self.progress_after_id = self.root.after(100, self.update_progress)
            return
        total_elapsed_s = min(elapsed_s, PROFILE_TOTAL_S)
        self.progress_var.set(total_elapsed_s)
        if self.stop_pending:
            self.progress_text_var.set(
                f"Stopping; {total_elapsed_s:.1f} / {PROFILE_TOTAL_S:.1f} s"
            )
            return

        stage_elapsed_s = (
            max(0.0, time.monotonic() - self.route_phase_start_s)
            if self.route_phase_start_s is not None else 0.0
        )
        stage_no = self.profile_index + 1
        self.progress_text_var.set(
            f"第 {stage_no}/{len(PROFILE_SPEEDS_MPS)} 档 "
            f"{stage_elapsed_s:.1f}/{PROFILE_STAGE_DURATIONS_S[self.profile_index]:.1f} s; "
            f"总计 {total_elapsed_s:.1f}/{PROFILE_TOTAL_S:.1f} s"
        )
        self.progress_after_id = self.root.after(100, self.update_progress)

    def route_step(self) -> None:
        self.route_after_id = None
        if not self.run_active or self.stop_pending or self.worker is None:
            return

        now_s = time.monotonic()
        if self.run_mode == MODE_TURN:
            self.turn_route_step(now_s)
            return
        phase_elapsed_s = (
            now_s - self.route_phase_start_s
            if self.route_phase_start_s is not None
            else 0.0
        )

        if self.route_phase.startswith("straight_"):
            if self.profile_index + 1 < len(PROFILE_SPEEDS_MPS):
                self.profile_index += 1
                speed = PROFILE_SPEEDS_MPS[self.profile_index]
                self.set_route_phase(f"straight_{self.profile_index + 1}", speed, 0.0)
                self.schedule_route_step(
                    int(PROFILE_STAGE_DURATIONS_S[self.profile_index] * 1000.0)
                )
            else:
                self.stop_run("Speed profile complete")
            return

        self.stop_run("Unknown route phase")

    def start_run(self) -> None:
        if self.worker is None or not self.connected or self.run_active:
            return
        if not self.imu_ready:
            messagebox.showwarning(
                "Straight-line test",
                "BMI088 航向尚未完成静止 bias 校准，请保持车辆静止并等待 ATT 状态包含 0x0C。",
            )
            return
        self.run_mode = self.mode_var.get()
        az_radps = 0.0
        vx_mps = 0.0 if self.run_mode == MODE_TURN else PROFILE_SPEEDS_MPS[0]
        self.run_id += 1
        self.run_active = True
        self.stop_pending = False
        self.run_start_s = time.monotonic()
        self.route_vx_mps = vx_mps
        self.route_turn_rate_radps = az_radps
        self.profile_index = 0
        self.turn_start_yaw_deg = None
        self.turn_target_delta_deg = LEFT_TURN_ANGLE_DEG
        self.run_start_left = self.left_unwrapped
        self.run_start_right = self.right_unwrapped
        self.progress_var.set(0.0)
        self.progress_text_var.set(
            f"第 1/{len(PROFILE_SPEEDS_MPS)} 档 "
            f"0.0/{PROFILE_STAGE_DURATIONS_S[0]:.1f} s; "
            f"总计 0.0/{PROFILE_TOTAL_S:.1f} s"
        )
        self.vx_entry.configure(state="disabled")
        self.az_entry.configure(state="disabled")
        self.start_button.configure(state="disabled")
        self.stop_button.configure(state="normal")
        self.run_control_samples.clear()
        self.mode_combo.configure(state="disabled")
        self.progress_bar.configure(maximum=8 if self.run_mode == MODE_TURN else PROFILE_TOTAL_S)
        if self.run_mode == MODE_TURN:
            self.turn_sweep = TurnSweep(self.run_start_s)
            self.turn_record_count = 0
            self.turn_state = None
            self.log_var.set(f"CSV: {self.csv_path} | 转弯汇总: {self.turn_summary_path}")
            self.write_raw(host_timestamp(), "EVENT", f"RUN_START,{self.run_id},mode=turn,rates=0.1/0.3/0.6/0.9,angle_deg=90,return_to_start=1")
            self.set_route_phase("turn_1_left_ready", 0.0, 0.0)
            self.update_progress()
            self.schedule_route_step(20)
            return
        self.turn_sweep = None
        self.write_raw(
            host_timestamp(), "EVENT",
            f"RUN_START,{self.run_id},profile=0.03/0.06/0.09/0.12/0.16,"
            f"stage_s=5/10/15/30/30,total_s={PROFILE_TOTAL_S:.3f},"
            f"az={az_radps:.6f}"
        )
        self.set_route_phase("straight_1", vx_mps, az_radps)
        self.update_progress()
        self.schedule_route_step(int(PROFILE_STAGE_DURATIONS_S[0] * 1000.0))

    def stop_run(self, reason: str, settle: bool = True) -> None:
        self.cancel_route_timer()
        worker = self.worker
        if worker is not None:
            worker.set_command(0.0, 0.0)
        if not self.run_active:
            return
        if self.stop_pending:
            if not settle:
                self.finish_stop(reason)
            return

        self.stop_pending = True
        self.route_phase = "stopping"
        self.route_phase_start_s = time.monotonic()
        self.requested_vx_mps = 0.0
        self.requested_az_radps = 0.0
        self.start_button.configure(state="disabled")
        self.stop_button.configure(state="disabled")
        self.command_var.set(f"Stopping ({reason}); transmitting 0,0")
        self.write_raw(
            host_timestamp(), "EVENT", f"RUN_STOP_REQUEST,{self.run_id},{reason}"
        )
        if settle and self.connected and worker is not None:
            self.stop_after_id = self.root.after(
                STOP_SETTLE_MS, lambda: self.finish_stop(reason)
            )
        else:
            self.finish_stop(reason)

    def finish_stop(self, reason: str) -> None:
        if not self.run_active:
            return
        self.cancel_progress_timer()
        if self.stop_after_id is not None:
            try:
                self.root.after_cancel(self.stop_after_id)
            except tk.TclError:
                pass
            self.stop_after_id = None
        duration_s = time.monotonic() - self.run_start_s if self.run_start_s is not None else 0.0
        phase = self.current_run_deltas()
        phase_text = ""
        if phase is not None:
            left_delta, right_delta, phase_error = phase
            phase_text = f",{left_delta},{right_delta},{phase_error}"
        self.write_raw(
            host_timestamp(),
            "EVENT",
            f"RUN_STOP,{self.run_id},{reason},{duration_s:.3f}{phase_text}",
        )
        if self.run_control_samples:
            left_errors = [sample[0] for sample in self.run_control_samples]
            right_errors = [sample[1] for sample in self.run_control_samples]
            speed_diffs = [sample[2] for sample in self.run_control_samples]
            self.write_raw(
                host_timestamp(), "SUMMARY",
                "CTL_SAMPLES={},left_error_mean_mmps={:.2f},left_error_max_abs_mmps={},"
                "right_error_mean_mmps={:.2f},right_error_max_abs_mmps={},"
                "wheel_diff_mean_mmps={:.2f},wheel_diff_max_abs_mmps={},"
                "wheel_diff_rms_mmps={:.2f},command_age_max_ms={}".format(
                    len(left_errors), sum(left_errors) / len(left_errors),
                    max(abs(v) for v in left_errors),
                    sum(right_errors) / len(right_errors),
                    max(abs(v) for v in right_errors),
                    sum(speed_diffs) / len(speed_diffs),
                    max(abs(v) for v in speed_diffs),
                    math.sqrt(sum(v * v for v in speed_diffs) / len(speed_diffs)),
                    max(sample[3] for sample in self.run_control_samples),
                ),
            )
        self.run_active = False
        self.stop_pending = False
        self.run_start_s = None
        self.route_phase = "idle"
        self.route_phase_start_s = None
        self.profile_index = -1
        self.turn_start_yaw_deg = None
        self.vx_entry.configure(state="readonly")
        self.az_entry.configure(state="readonly")
        self.mode_combo.configure(state="readonly")
        self.start_button.configure(
            state="normal" if self.connected and self.imu_ready else "disabled"
        )
        self.stop_button.configure(state="disabled")
        self.command_var.set(f"Stopped ({reason}); transmitting 0,0")
        if self.run_mode == MODE_TURN:
            completed = len(self.turn_sweep.records) if self.turn_sweep else 0
            self.progress_var.set(completed)
            status = "已完成" if reason == "Turn profile complete" else "已停止"
            self.progress_text_var.set(f"{status}；{completed}/8段；{duration_s:.1f}s")
            return
        if reason == "Speed profile complete":
            self.progress_var.set(PROFILE_TOTAL_S)
            self.progress_text_var.set(
                f"Completed; {PROFILE_TOTAL_S:.1f} / {PROFILE_TOTAL_S:.1f} s"
            )
        else:
            elapsed_s = min(duration_s, PROFILE_TOTAL_S)
            self.progress_var.set(elapsed_s)
            self.progress_text_var.set(
                f"Stopped; {elapsed_s:.1f} / "
                f"{PROFILE_TOTAL_S:.1f} s"
            )

    def reset_encoder_stream(self) -> None:
        self.last_encoder_raw = None
        self.left_unwrapped = None
        self.right_unwrapped = None
        self.last_mcu_time_ms = None
        self.run_start_left = None
        self.run_start_right = None
        self.encoder_var.set("ENC: --")
        self.phase_var.set("Run delta: --")

    def current_run_deltas(self) -> tuple[int, int, int] | None:
        if (
            self.left_unwrapped is None
            or self.right_unwrapped is None
            or self.run_start_left is None
            or self.run_start_right is None
        ):
            return None
        left_delta = self.left_unwrapped - self.run_start_left
        right_delta = self.right_unwrapped - self.run_start_right
        return left_delta, right_delta, right_delta - left_delta

    def update_encoder(self, values: list[int]) -> dict[str, object]:
        mcu_time_ms, sequence, left_count, right_count = values
        if self.last_mcu_time_ms is not None:
            interval_ms = (mcu_time_ms - self.last_mcu_time_ms) & 0xFFFFFFFF
            if interval_ms == 0 or interval_ms > ENCODER_MAX_INTERVAL_MS:
                reason = f"MCU encoder time discontinuity ({interval_ms} ms)"
                self.write_raw(host_timestamp(), "NOTICE", reason)
                self.stop_run(reason, settle=False)
                self.reset_encoder_stream()
        if self.last_encoder_raw is None:
            self.left_unwrapped = left_count
            self.right_unwrapped = right_count
        else:
            previous_left, previous_right = self.last_encoder_raw
            assert self.left_unwrapped is not None and self.right_unwrapped is not None
            self.left_unwrapped += signed_int32_delta(left_count, previous_left)
            self.right_unwrapped += signed_int32_delta(right_count, previous_right)
        self.last_encoder_raw = (left_count, right_count)
        self.last_mcu_time_ms = mcu_time_ms

        if self.run_active and (self.run_start_left is None or self.run_start_right is None):
            self.run_start_left = self.left_unwrapped
            self.run_start_right = self.right_unwrapped

        row: dict[str, object] = {
            "mcu_time_ms": mcu_time_ms,
            "sequence": sequence,
            "left_total_count": left_count,
            "right_total_count": right_count,
            "left_unwrapped_count": self.left_unwrapped,
            "right_unwrapped_count": self.right_unwrapped,
        }
        self.encoder_var.set(f"ENC: L={left_count}  R={right_count}  seq={sequence}")
        deltas = self.current_run_deltas()
        if self.run_active and deltas is not None:
            left_delta, right_delta, phase_error = deltas
            left_rev = left_delta / LEFT_ENCODER_COUNTS_PER_WHEEL_REV
            right_rev = right_delta / RIGHT_ENCODER_COUNTS_PER_WHEEL_REV
            phase_rev = right_rev - left_rev
            left_distance_m = left_rev * WHEEL_CIRCUMFERENCE_M
            right_distance_m = right_rev * WHEEL_CIRCUMFERENCE_M
            phase_distance_m = right_distance_m - left_distance_m
            row.update(
                {
                    "run_left_delta_count": left_delta,
                    "run_right_delta_count": right_delta,
                    "run_phase_error_count": phase_error,
                    "run_left_wheel_rev": f"{left_rev:.9f}",
                    "run_right_wheel_rev": f"{right_rev:.9f}",
                    "run_phase_error_wheel_rev": f"{phase_rev:.9f}",
                    "run_left_distance_m": f"{left_distance_m:.9f}",
                    "run_right_distance_m": f"{right_distance_m:.9f}",
                    "run_phase_error_m": f"{phase_distance_m:.9f}",
                }
            )
            self.phase_var.set(
                f"Run: L={left_distance_m:+.4f}m  R={right_distance_m:+.4f}m  "
                f"R-L={phase_distance_m * 1000.0:+.2f}mm ({phase_rev:+.6f} rev)"
            )
        return row

    def update_control(self, values: list[int]) -> dict[str, object]:
        self.last_ctl = values
        now_s = time.monotonic()
        ctl_interval_ms = (
            (now_s - self.last_ctl_host_s) * 1000.0
            if self.last_ctl_host_s is not None else 0.0
        )
        self.last_ctl_host_s = now_s
        left_error = values[8] - values[10]
        right_error = values[9] - values[11]
        speed_difference = values[10] - values[11]
        reference_scale = max(abs(values[8]), abs(values[9]), 1)
        difference_ratio = speed_difference / reference_scale
        row = {
            "mcu_time_ms": values[0],
            "sequence": values[1],
            "mode": values[2],
            "emergency": values[3],
            "command_valid": values[4],
            "command_age_ms": values[5],
            "command_vx_mmps": values[6],
            "command_az_mradps": values[7],
            "target_left_mmps": values[8],
            "target_right_mmps": values[9],
            "measured_left_mmps": values[10],
            "measured_right_mmps": values[11],
            "left_speed_error_mmps": left_error,
            "right_speed_error_mmps": right_error,
            "wheel_speed_difference_mmps": speed_difference,
            "wheel_speed_difference_ratio": f"{difference_ratio:.6f}",
            "ctl_host_interval_ms": f"{ctl_interval_ms:.3f}",
            "left_output": values[12],
            "right_output": values[13],
        }
        if self.run_active and not self.stop_pending:
            self.run_control_samples.append(
                (left_error, right_error, speed_difference, values[5])
            )
        self.control_var.set(
            "CTL: "
            f"mode={values[2]} valid={values[4]} age={values[5]} ms  "
            f"target={values[8]}/{values[9]} mm/s  "
            f"measured={values[10]}/{values[11]} mm/s  "
            f"err={left_error}/{right_error} diff={speed_difference} mm/s  "
            f"output={values[12]}/{values[13]}"
        )
        if self.run_active and not self.stop_pending and (
            values[2] != 1 or values[3] != 0 or values[4] != 1
        ):
            self.stop_run("Invalid CTL control state")
        return row

    @staticmethod
    def update_raw_imu(values: list[int]) -> dict[str, object]:
        return {
            "mcu_time_ms": values[0],
            "sequence": values[1],
            "raw_imu_status": values[2],
            "raw_gyro_x_count": values[3],
            "raw_gyro_y_count": values[4],
            "raw_gyro_z_count": values[5],
            "raw_temperature_cC": values[6],
        }

    def update_motor(self, values: list[int]) -> dict[str, object]:
        self.last_mtr = values
        row = {
            "mcu_time_ms": values[0],
            "sequence": values[1],
            "drive_status": values[2],
            "left_current": values[3],
            "right_current": values[4],
            "left_integral_output": values[5],
            "right_integral_output": values[6],
            "left_temperature_C": values[7],
            "right_temperature_C": values[8],
        }
        if len(values) != 24:
            raise ValueError(f"MTR expected 24 values, got {len(values)}")
        self.motor_var.set(
            "MTR: "
            f"status=0x{values[2]:02X} current={values[3]}/{values[4]}  "
            f"temp={values[7]}/{values[8]}C online={values[9]}/{values[10]} "
            f"age={values[11]}/{values[12]}ms CAN txfail={values[16]} "
            f"busoff={values[21]} fifo={values[23]}"
        )
        row.update({
            "drive_status": values[2],
            "left_current": values[3],
            "right_current": values[4],
            "left_integral_output": values[5],
            "right_integral_output": values[6],
            "left_temperature_C": values[7],
            "right_temperature_C": values[8],
            "left_feedback_online": values[9],
            "right_feedback_online": values[10],
            "left_feedback_age_ms": values[11],
            "right_feedback_age_ms": values[12],
            "left_feedback_count": values[13],
            "right_feedback_count": values[14],
            "can_tx_success_count": values[15],
            "can_tx_failure_count": values[16],
            "can_last_tx_failure_age_ms": values[17],
            "can_state": values[18],
            "can_hal_error_code": values[19],
            "can_protocol_last_error_code": values[20],
            "can_bus_off": values[21],
            "can_error_passive": values[22],
            "can_tx_fifo_free_level": values[23],
        })
        if self.run_active and not self.stop_pending and (values[2] & 0x0A):
            reason = (
                "Lower heading-divergence fault"
                if values[2] & 0x08
                else "Lower drive-start fault"
            )
            self.stop_run(reason)
        return row

    @staticmethod
    def update_rfid(fields: list[str]) -> dict[str, object]:
        values = [int(field, 10) for field in fields[1:8]]
        return {
            "mcu_time_ms": values[0],
            "sequence": values[1],
            "rfid_status": values[2],
            "rfid_protocol_status": values[3],
            "rfid_tag_type": values[4],
            "rfid_sak": values[5],
            "rfid_number": values[6],
            "rfid_uid_hex": fields[8],
        }

    def update_attitude(self, values: list[int]) -> dict[str, object]:
        self.last_att = values
        imu_status = values[2]
        ready = (imu_status & IMU_HEADING_READY) == IMU_HEADING_READY
        if ready:
            if not self.imu_ready:
                self.write_raw(
                    host_timestamp(), "NOTICE",
                    f"BMI088_READY,status=0x{imu_status:02X},startup_samples={values[6]}",
                )
            self.imu_ready = True
            self.imu_invalid_samples = 0
        else:
            self.imu_invalid_samples += 1
            if self.run_active and not self.stop_pending and self.imu_invalid_samples >= 3:
                self.stop_run(
                    f"BMI088 heading invalid status=0x{imu_status:02X} "
                    f"startup_samples={values[6]}"
                )
        if self.connected and not self.run_active:
            self.set_connection_controls(connecting=True)
        yaw_deg = values[3] / 1000.0
        gyro_z_dps = values[4] / 1000.0
        bias_z_dps = values[5] / 1000.0
        row = {
            "mcu_time_ms": values[0],
            "sequence": values[1],
            "imu_status": imu_status,
            "imu_yaw_mdeg": values[3],
            "imu_yaw_deg": f"{yaw_deg:.6f}",
            "imu_gyro_z_mdps": values[4],
            "imu_gyro_z_dps": f"{gyro_z_dps:.6f}",
            "imu_bias_z_mdps": values[5],
            "imu_bias_z_dps": f"{bias_z_dps:.6f}",
            "imu_startup_samples": values[6],
        }
        self.imu_var.set(
            "ATT: "
            f"status=0x{values[2]:02X} yaw={yaw_deg:+.3f} deg  "
            f"gz={gyro_z_dps:+.3f} dps bias={bias_z_dps:+.4f} dps  "
            f"cal={values[6]}"
        )
        # ATT remains diagnostic data; HLD separately records the active
        # BMI088 heading-hold state and correction during this test.
        return row

    def update_heading_hold(self, values: list[int]) -> dict[str, object]:
        self.last_hld = values
        ref_deg = values[3] / 1000.0
        error_deg = values[4] / 1000.0
        correction_ref = values[5] / 1000.0
        i_term_ref = values[6] / 1000.0
        fused_error_deg = values[7] / 1000.0
        row = {
            "mcu_time_ms": values[0],
            "sequence": values[1],
            "heading_active": values[2],
            "heading_ref_mdeg": values[3],
            "heading_ref_deg": f"{ref_deg:.6f}",
            "heading_error_mdeg": values[4],
            "heading_error_deg": f"{error_deg:.6f}",
            "heading_correction_milli_ref": values[5],
            "heading_correction_ref": f"{correction_ref:.6f}",
            "heading_i_term_milli_ref": values[6],
            "heading_i_term_ref": f"{i_term_ref:.6f}",
            "heading_fused_error_mdeg": values[7],
            "heading_fused_error_deg": f"{fused_error_deg:.6f}",
        }
        self.hold_var.set(
            "HLD: "
            f"active={values[2]} ref={ref_deg:+.3f} deg  "
            f"err={error_deg:+.3f} deg corr={correction_ref:+.3f} "
            f"I={i_term_ref:+.3f} fused={fused_error_deg:+.3f}"
        )
        return row

    def update_straight_sync(self, values: list[int]) -> dict[str, object]:
        self.last_syn = values
        phase_error_deg = values[3] / 1000.0
        correction_ref = values[4] / 1000.0
        row = {
            "mcu_time_ms": values[0],
            "sequence": values[1],
            "sync_active": values[2],
            "sync_phase_error_mdeg": values[3],
            "sync_phase_error_deg": f"{phase_error_deg:.6f}",
            "sync_correction_milli_ref": values[4],
            "sync_correction_ref": f"{correction_ref:.6f}",
        }
        self.sync_var.set(
            "SYN: "
            f"active={values[2]} phase={phase_error_deg:+.3f} wheel-deg  "
            f"corr={correction_ref:+.3f}"
        )
        return row

    def log_telemetry(
        self, timestamp: str, received_s: float, line: str, record: bool
    ) -> None:
        fields = [field.strip() for field in line.split(",")]
        if not fields or fields[0] not in (
            "ENC", "IMU", "CTL", "ATT", "HLD", "SYN", "MTR", "RFID"
        ):
            return
        frame_type = fields[0]
        phase_elapsed_s = (
            received_s - self.route_phase_start_s
            if self.route_phase_start_s is not None else 0.0
        )
        row: dict[str, object] = {
            "host_time": timestamp,
            "host_elapsed_s": f"{received_s - self.session_start_s:.6f}",
            "run_id": self.run_id if self.run_id else "",
            "run_active": int(self.run_active),
            "test_mode": "turn" if self.run_mode == MODE_TURN else "straight",
            "turn_angle_source": "gyro_integrated_mcu" if self.run_mode == MODE_TURN else "",
            "turn_control_yaw_deg": f"{self.turn_telemetry.control_yaw:.6f}" if self.run_mode == MODE_TURN else "",
            "turn_relative_yaw_deg": f"{self.turn_telemetry.control_yaw - self.turn_sweep.initial:.6f}" if self.run_mode == MODE_TURN and self.turn_sweep is not None and self.turn_sweep.initial is not None else "",
            "route_phase": self.route_phase,
            "route_phase_elapsed_s": f"{phase_elapsed_s:.6f}",
            "requested_vx_mps": f"{self.requested_vx_mps:.6f}",
            "requested_az_radps": f"{self.requested_az_radps:.6f}",
            "frame_type": frame_type,
            "parse_ok": 0,
            "raw_frame": line,
        }
        try:
            expected_fields_by_type = {
                "ENC": 5,
                "IMU": 8,
                "CTL": 15,
                "ATT": 8,
                "HLD": 9,
                "SYN": 6,
                "MTR": 25,
                "RFID": 9,
            }
            expected_fields = expected_fields_by_type[frame_type]
            if len(fields) != expected_fields:
                raise ValueError(f"expected {expected_fields} fields, got {len(fields)}")
            if frame_type == "RFID":
                row.update(self.update_rfid(fields))
                values = []
            else:
                values = [int(field, 10) for field in fields[1:]]

            if frame_type == "ENC":
                if not (
                    0 <= values[0] <= 0xFFFFFFFF
                    and 0 <= values[1] <= 0xFFFFFFFF
                    and -0x80000000 <= values[2] <= 0x7FFFFFFF
                    and -0x80000000 <= values[3] <= 0x7FFFFFFF
                ):
                    raise ValueError("ENC value out of range")
                row.update(self.update_encoder(values))
            elif frame_type == "IMU":
                row.update(self.update_raw_imu(values))
            elif frame_type == "CTL":
                row.update(self.update_control(values))
            elif frame_type == "ATT":
                row.update(self.update_attitude(values))
            elif frame_type == "HLD":
                row.update(self.update_heading_hold(values))
            elif frame_type == "SYN":
                row.update(self.update_straight_sync(values))
            elif frame_type == "MTR":
                row.update(self.update_motor(values))
            row["parse_ok"] = 1
        except ValueError as exc:
            self.parse_error_count += 1
            row["parse_error"] = str(exc)
            self.append_stream(f"PARSE ERROR: {exc}: {line}")

        self.text_file.write(
            f"{timestamp}\t{self.run_id if self.run_id else ''}\t"
            f"{row['run_active']}\t{row['route_phase']}\t"
            f"{line}\n"
        )
        self.csv_writer.writerow(row)

    def write_raw(self, timestamp: str, direction: str, frame: str) -> None:
        self.raw_file.write(f"{timestamp}\t{direction}\t{frame}\n")

    def append_stream(self, line: str) -> None:
        self.stream_text.configure(state="normal")
        self.stream_text.insert(tk.END, line + "\n")
        if int(self.stream_text.index("end-1c").split(".")[0]) > 800:
            self.stream_text.delete("1.0", "201.0")
        self.stream_text.see(tk.END)
        self.stream_text.configure(state="disabled")

    def handle_line(self, timestamp: str, received_s: float, line: str) -> None:
        self.rx_line_count += 1
        if self.last_rx_monotonic_s is not None:
            gap_ms = (received_s - self.last_rx_monotonic_s) * 1000.0
            if gap_ms > 50.0:
                self.write_raw(timestamp, "NOTICE", f"RX_GAP,{gap_ms:.3f}ms")
        self.last_rx_monotonic_s = received_s
        self.turn_telemetry.feed(line, received_s)
        self.write_raw(timestamp, "RX", line)
        self.log_telemetry(timestamp, received_s, line, True)
        telemetry_prefixes = (
            "ENC,", "IMU,", "CTL,", "ATT,", "HLD,", "SYN,", "MTR,", "RFID,"
        )
        if line.startswith(telemetry_prefixes):
            if received_s - self.last_stream_telemetry_s >= 0.1:
                self.last_stream_telemetry_s = received_s
                self.append_stream(f"{timestamp}  {line}")
        else:
            self.append_stream(line)

    def poll_events(self) -> None:
        try:
            while True:
                kind, value = self.events.get_nowait()
                if kind == "connected":
                    self.connected = True
                    self.imu_ready = False
                    self.imu_invalid_samples = 0
                    self.connection_var.set(
                        f"已连接 {value}；发送零速，等待 BMI088 bias 有效"
                    )
                    self.write_raw(host_timestamp(), "EVENT", f"CONNECTED,{value}")
                    self.set_connection_controls(connecting=True)
                elif kind == "tx":
                    timestamp, command_text = value  # type: ignore[misc]
                    self.write_raw(str(timestamp), "TX", str(command_text))
                elif kind == "line":
                    timestamp, received_s, line = value  # type: ignore[misc]
                    self.handle_line(str(timestamp), float(received_s), str(line))
                elif kind == "notice":
                    self.write_raw(host_timestamp(), "NOTICE", str(value))
                    self.append_stream(str(value))
                elif kind == "disconnected":
                    error = str(value)
                    self.stop_run("Serial disconnected", settle=False)
                    self.connected = False
                    self.imu_ready = False
                    self.imu_invalid_samples = 0
                    self.worker = None
                    self.reset_encoder_stream()
                    self.last_ctl = None
                    self.last_att = None
                    self.last_hld = None
                    self.last_syn = None
                    self.last_mtr = None
                    self.sync_var.set("SYN: --")
                    self.motor_var.set("MTR: --")
                    self.set_connection_controls(connecting=False)
                    self.connection_var.set(f"Disconnected: {error}" if error else "Disconnected")
                    self.write_raw(host_timestamp(), "EVENT", f"DISCONNECTED,{error}")
                    if self.closing:
                        self.finish_close()
                        return
        except queue.Empty:
            pass
        if not self.raw_file.closed:
            self.raw_file.flush()
        if not self.text_file.closed:
            self.text_file.flush()
        if not self.csv_file.closed:
            self.csv_file.flush()
        if self.root.winfo_exists():
            self.root.after(10, self.poll_events)

    def close_window(self) -> None:
        if self.worker is not None:
            self.closing = True
            self.disconnect()
            return
        self.finish_close()

    def finish_close(self) -> None:
        if not self.raw_file.closed:
            self.write_raw(
                host_timestamp(), "SUMMARY",
                f"rx_lines={self.rx_line_count},parse_errors={self.parse_error_count}"
            )
            self.write_raw(host_timestamp(), "EVENT", "SESSION_END")
            self.raw_file.close()
        if not self.text_file.closed:
            self.text_file.close()
        if not self.csv_file.closed:
            self.csv_file.close()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    StraightLineTestApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
