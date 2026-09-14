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

import serial
from serial.tools import list_ports


BAUD_RATE = 115200
COMMAND_PERIOD_S = 0.1
SERIAL_TIMEOUT_S = 0.02
STOP_COMMAND = b"0.000000,0.000000\r\n"
ENCODER_COUNTS_PER_WHEEL_REV = 8192 * 36
ENCODER_MAX_INTERVAL_MS = 500
STOP_SETTLE_MS = 750
MAX_LINEAR_SPEED_MPS = 0.40

CSV_FIELDS = (
    "host_time",
    "host_elapsed_s",
    "run_id",
    "run_active",
    "requested_vx_mps",
    "requested_az_radps",
    "frame_type",
    "parse_ok",
    "mcu_time_ms",
    "sequence",
    "left_total_count",
    "right_total_count",
    "left_unwrapped_count",
    "right_unwrapped_count",
    "run_left_delta_count",
    "run_right_delta_count",
    "run_phase_error_count",
    "run_phase_error_wheel_rev",
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
    "left_output",
    "right_output",
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
    "raw_frame",
)


def host_timestamp() -> str:
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


def signed_int32_delta(end: int, start: int) -> int:
    value = (end - start) & 0xFFFFFFFF
    return value if value <= 0x7FFFFFFF else value - 0x100000000


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

        self.last_encoder_raw: tuple[int, int] | None = None
        self.left_unwrapped: int | None = None
        self.right_unwrapped: int | None = None
        self.last_mcu_time_ms: int | None = None
        self.last_ctl: list[int] | None = None
        self.last_att: list[int] | None = None
        self.last_hld: list[int] | None = None

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        log_dir = Path(__file__).resolve().parent / "straight_test_logs"
        log_dir.mkdir(parents=True, exist_ok=True)
        base_name = f"straight_test_{timestamp}"
        self.raw_path = log_dir / f"{base_name}_raw.txt"
        self.text_path = log_dir / f"{base_name}_telemetry.txt"
        self.csv_path = log_dir / f"{base_name}_telemetry.csv"
        self.raw_file = self.raw_path.open("w", encoding="utf-8", newline="")
        self.text_file = self.text_path.open("w", encoding="utf-8", newline="")
        self.csv_file = self.csv_path.open("w", encoding="utf-8", newline="")
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=CSV_FIELDS)
        self.csv_writer.writeheader()
        self.csv_file.flush()
        self.raw_file.write("host_time\tdirection\tframe\n")
        self.raw_file.flush()
        self.text_file.write("host_time\trun_id\trun_active\tframe\n")
        self.text_file.flush()

        self.port_var = tk.StringVar()
        self.vx_var = tk.StringVar(value="0.10")
        self.az_var = tk.StringVar(value="0.0")
        self.connection_var = tk.StringVar(value="Disconnected")
        self.command_var = tk.StringVar(value="Command: 0.000, 0.000")
        self.encoder_var = tk.StringVar(value="ENC: --")
        self.phase_var = tk.StringVar(value="Run delta: --")
        self.control_var = tk.StringVar(value="CTL: --")
        self.imu_var = tk.StringVar(value="ATT: --")
        self.hold_var = tk.StringVar(value="HLD: --")
        self.log_var = tk.StringVar(
            value=f"Raw: {self.raw_path} | TXT: {self.text_path} | CSV: {self.csv_path}"
        )

        self.build_ui()
        self.refresh_ports()
        self.root.protocol("WM_DELETE_WINDOW", self.close_window)
        self.root.bind("<Escape>", lambda _event: self.stop_run("Escape key"))
        self.root.after(50, self.poll_events)

    def build_ui(self) -> None:
        self.root.title("Straight-line speed-loop test")
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

        controls = ttk.LabelFrame(main, text="Velocity command", padding=10)
        controls.grid(row=1, column=0, columnspan=4, sticky="ew", pady=(12, 8))
        ttk.Label(controls, text="vx (m/s)").grid(row=0, column=0, sticky="w")
        self.vx_entry = ttk.Entry(controls, textvariable=self.vx_var, width=10)
        self.vx_entry.grid(row=0, column=1, padx=(6, 20))
        ttk.Label(controls, text="az (rad/s)").grid(row=0, column=2, sticky="w")
        self.az_entry = ttk.Entry(controls, textvariable=self.az_var, width=10)
        self.az_entry.grid(row=0, column=3, padx=(6, 24))
        self.az_entry.configure(state="disabled")
        self.start_button = ttk.Button(
            controls, text="Start", command=self.start_run, state="disabled"
        )
        self.start_button.grid(row=0, column=4, padx=(0, 8))
        self.stop_button = ttk.Button(
            controls, text="Stop", command=lambda: self.stop_run("Stop button"), state="disabled"
        )
        self.stop_button.grid(row=0, column=5)
        controls.columnconfigure(6, weight=1)

        state = ttk.Frame(main)
        state.grid(row=2, column=0, columnspan=4, sticky="ew", pady=(2, 8))
        state.columnconfigure(0, weight=1)
        state.columnconfigure(1, weight=1)
        ttk.Label(state, textvariable=self.connection_var).grid(row=0, column=0, sticky="w")
        ttk.Label(state, textvariable=self.command_var).grid(row=0, column=1, sticky="w")

        telemetry = ttk.LabelFrame(main, text="Live telemetry", padding=10)
        telemetry.grid(row=3, column=0, columnspan=4, sticky="ew", pady=(0, 8))
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

        ttk.Label(main, text="Log files").grid(row=4, column=0, sticky="w")
        ttk.Entry(main, textvariable=self.log_var, state="readonly").grid(
            row=4, column=1, columnspan=3, sticky="ew", padx=(8, 0)
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
            state="normal" if self.connected and not self.run_active else "disabled"
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
        self.last_ctl = None
        self.last_att = None
        self.last_hld = None
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
        if abs(az_radps) > 1e-6:
            messagebox.showerror(
                "Straight-line test",
                "az must be 0 for the straight-line speed-loop test.",
            )
            return None
        return vx_mps, az_radps

    def start_run(self) -> None:
        if self.worker is None or not self.connected or self.run_active:
            return
        command = self.validate_command()
        if command is None:
            return
        vx_mps, az_radps = command
        self.run_id += 1
        self.run_active = True
        self.stop_pending = False
        self.run_start_s = time.monotonic()
        self.requested_vx_mps = vx_mps
        self.requested_az_radps = az_radps
        self.run_start_left = self.left_unwrapped
        self.run_start_right = self.right_unwrapped
        self.worker.set_command(vx_mps, az_radps)
        self.vx_entry.configure(state="disabled")
        self.start_button.configure(state="disabled")
        self.stop_button.configure(state="normal")
        self.command_var.set(f"Run {self.run_id}: vx={vx_mps:.3f}, az={az_radps:.3f}")
        self.write_raw(
            host_timestamp(), "EVENT", f"RUN_START,{self.run_id},{vx_mps:.6f},{az_radps:.6f}"
        )

    def stop_run(self, reason: str, settle: bool = True) -> None:
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
        self.run_active = False
        self.stop_pending = False
        self.run_start_s = None
        self.vx_entry.configure(state="normal")
        self.start_button.configure(state="normal" if self.connected else "disabled")
        self.stop_button.configure(state="disabled")
        self.command_var.set(f"Stopped ({reason}); transmitting 0,0")

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
            phase_rev = phase_error / ENCODER_COUNTS_PER_WHEEL_REV
            row.update(
                {
                    "run_left_delta_count": left_delta,
                    "run_right_delta_count": right_delta,
                    "run_phase_error_count": phase_error,
                    "run_phase_error_wheel_rev": f"{phase_rev:.9f}",
                }
            )
            self.phase_var.set(
                f"Run delta: L={left_delta}  R={right_delta}  "
                f"R-L={phase_error} count ({phase_rev:+.6f} wheel rev)"
            )
        return row

    def update_control(self, values: list[int]) -> dict[str, object]:
        self.last_ctl = values
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
            "left_output": values[12],
            "right_output": values[13],
        }
        self.control_var.set(
            "CTL: "
            f"mode={values[2]} valid={values[4]} age={values[5]} ms  "
            f"target={values[8]}/{values[9]} mm/s  "
            f"measured={values[10]}/{values[11]} mm/s  "
            f"output={values[12]}/{values[13]}"
        )
        return row

    def update_attitude(self, values: list[int]) -> dict[str, object]:
        self.last_att = values
        yaw_deg = values[3] / 1000.0
        gyro_z_dps = values[4] / 1000.0
        bias_z_dps = values[5] / 1000.0
        row = {
            "mcu_time_ms": values[0],
            "sequence": values[1],
            "imu_status": values[2],
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
        return row

    def update_heading_hold(self, values: list[int]) -> dict[str, object]:
        self.last_hld = values
        ref_deg = values[3] / 1000.0
        error_deg = values[4] / 1000.0
        correction_ref = values[5] / 1000.0
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
        }
        self.hold_var.set(
            "HLD: "
            f"active={values[2]} ref={ref_deg:+.3f} deg  "
            f"err={error_deg:+.3f} deg corr={correction_ref:+.3f}"
        )
        return row

    def log_telemetry(self, timestamp: str, received_s: float, line: str) -> None:
        fields = [field.strip() for field in line.split(",")]
        if not fields or fields[0] not in ("ENC", "CTL", "ATT", "HLD", "PHY", "MTR"):
            return
        frame_type = fields[0]
        row: dict[str, object] = {
            "host_time": timestamp,
            "host_elapsed_s": f"{received_s - self.session_start_s:.6f}",
            "run_id": self.run_id if self.run_id else "",
            "run_active": int(self.run_active),
            "requested_vx_mps": f"{self.requested_vx_mps:.6f}",
            "requested_az_radps": f"{self.requested_az_radps:.6f}",
            "frame_type": frame_type,
            "parse_ok": 0,
            "raw_frame": line,
        }
        try:
            expected_fields_by_type = {
                "ENC": 5,
                "CTL": 15,
                "ATT": 8,
                "HLD": 7,
                "PHY": 10,
                "MTR": 16,
            }
            expected_fields = expected_fields_by_type[frame_type]
            if len(fields) != expected_fields:
                raise ValueError(f"expected {expected_fields} fields, got {len(fields)}")
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
            elif frame_type == "CTL":
                row.update(self.update_control(values))
            elif frame_type == "ATT":
                row.update(self.update_attitude(values))
            elif frame_type == "HLD":
                row.update(self.update_heading_hold(values))
            elif frame_type == "PHY":
                row.update(
                    {
                        "mcu_time_ms": values[0],
                        "sequence": values[1],
                    }
                )
            else:
                row.update(
                    {
                        "mcu_time_ms": values[0],
                        "sequence": values[1],
                    }
                )
            row["run_active"] = int(self.run_active)
            row["requested_vx_mps"] = f"{self.requested_vx_mps:.6f}"
            row["requested_az_radps"] = f"{self.requested_az_radps:.6f}"
            row["parse_ok"] = 1
        except ValueError as exc:
            self.append_stream(f"PARSE ERROR: {exc}: {line}")

        self.text_file.write(
            f"{timestamp}\t{self.run_id if self.run_id else ''}\t{int(self.run_active)}\t{line}\n"
        )
        self.text_file.flush()
        self.csv_writer.writerow(row)
        self.csv_file.flush()

    def write_raw(self, timestamp: str, direction: str, frame: str) -> None:
        self.raw_file.write(f"{timestamp}\t{direction}\t{frame}\n")
        self.raw_file.flush()

    def append_stream(self, line: str) -> None:
        self.stream_text.configure(state="normal")
        self.stream_text.insert(tk.END, line + "\n")
        if int(self.stream_text.index("end-1c").split(".")[0]) > 800:
            self.stream_text.delete("1.0", "201.0")
        self.stream_text.see(tk.END)
        self.stream_text.configure(state="disabled")

    def handle_line(self, timestamp: str, received_s: float, line: str) -> None:
        self.write_raw(timestamp, "RX", line)
        self.log_telemetry(timestamp, received_s, line)
        if line.startswith(("ENC,", "CTL,", "ATT,", "HLD,", "MTR,", "PHY,")):
            self.append_stream(f"{timestamp}  {line}")
        else:
            self.append_stream(line)

    def poll_events(self) -> None:
        try:
            while True:
                kind, value = self.events.get_nowait()
                if kind == "connected":
                    self.connected = True
                    self.connection_var.set(f"Connected to {value}; zero command active")
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
                    self.worker = None
                    self.reset_encoder_stream()
                    self.last_ctl = None
                    self.last_att = None
                    self.last_hld = None
                    self.set_connection_controls(connecting=False)
                    self.connection_var.set(f"Disconnected: {error}" if error else "Disconnected")
                    self.write_raw(host_timestamp(), "EVENT", f"DISCONNECTED,{error}")
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
        if not self.raw_file.closed:
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
