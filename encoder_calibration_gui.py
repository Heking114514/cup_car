
from __future__ import annotations

import csv
import queue
import statistics
import threading
import time
import tkinter as tk
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from tkinter import messagebox, scrolledtext, ttk

import serial
from serial.tools import list_ports


BAUD_RATE = 115200
STOP_COMMAND = b"0.000,0.000\r\n"
COMMAND_PERIOD_S = 0.1
SERIAL_TIMEOUT_S = 0.02
SNAPSHOT_SAMPLES = 7
SNAPSHOT_MAX_AGE_S = 0.5
SNAPSHOT_MAX_WINDOW_S = 0.75
SNAPSHOT_MAX_GAP_S = 0.2
SNAPSHOT_MAX_SPREAD = 3
ENCODER_MAX_INTERVAL_MS = 250
CTL_SAFE_STREAK_REQUIRED = 2
CTL_MAX_GAP_S = 0.25
OTHER_WHEEL_MAX_DELTA = 20


@dataclass(frozen=True)
class EncoderFrame:
    received_s: float
    mcu_time_ms: int
    sequence: int
    left_count: int
    right_count: int
    left_unwrapped: int | None = None
    right_unwrapped: int | None = None


@dataclass(frozen=True)
class EncoderSnapshot:
    pc_time: str
    mcu_time_ms: int
    sequence: int
    left_count: int
    right_count: int
    left_unwrapped: int
    right_unwrapped: int


@dataclass(frozen=True)
class ControllerState:
    received_s: float
    mode: int
    emergency: int
    command_valid: int
    command_vx_mmps: int
    command_az_mradps: int
    target_left_mmps: int
    target_right_mmps: int
    left_output: int
    right_output: int


def signed_int32_delta(end: int, start: int) -> int:
    """Return end-start across a signed 32-bit counter rollover."""
    value = (end - start) & 0xFFFFFFFF
    return value if value <= 0x7FFFFFFF else value - 0x100000000


def signed_int32(value: int) -> int:
    value &= 0xFFFFFFFF
    return value if value <= 0x7FFFFFFF else value - 0x100000000


def parse_encoder(line: str, received_s: float) -> EncoderFrame | None:
    fields = [field.strip() for field in line.split(",")]
    if len(fields) != 5 or fields[0] != "ENC":
        return None
    try:
        values = [int(field, 10) for field in fields[1:]]
    except ValueError:
        return None
    if not (
        0 <= values[0] <= 0xFFFFFFFF
        and 0 <= values[1] <= 0xFFFFFFFF
        and -0x80000000 <= values[2] <= 0x7FFFFFFF
        and -0x80000000 <= values[3] <= 0x7FFFFFFF
    ):
        return None
    return EncoderFrame(received_s, values[0], values[1], values[2], values[3])


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

    def emit(self, kind: str, value: object) -> None:
        self.events.put((kind, value))

    @staticmethod
    def send_stop(device: serial.Serial, repeat: int = 1) -> None:
        for _ in range(repeat):
            device.write(STOP_COMMAND)
            if repeat > 1:
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
            self.emit("tx", "0.000,0.000 x3")
            self.emit("connected", self.port)
            next_command_s = time.monotonic()

            while not self.stop_event.is_set():
                now_s = time.monotonic()
                if now_s >= next_command_s:
                    self.send_stop(device)
                    self.emit("tx", "0.000,0.000")
                    next_command_s = now_s + COMMAND_PERIOD_S

                chunk = device.read(device.in_waiting or 1)
                if not chunk:
                    continue
                buffer.extend(chunk)
                if len(buffer) > 4096:
                    buffer.clear()
                    self.emit("notice", "RX buffer overflow; partial line discarded")
                    continue

                while b"\n" in buffer:
                    raw_line, _, remainder = buffer.partition(b"\n")
                    buffer = bytearray(remainder)
                    line = raw_line.rstrip(b"\r").decode("ascii", errors="replace")
                    if line:
                        self.emit("line", (time.monotonic(), line))
        except (serial.SerialException, OSError) as exc:
            error = str(exc)
        finally:
            if device is not None and device.is_open:
                try:
                    self.send_stop(device, repeat=5)
                    self.emit("tx", "0.000,0.000 x5")
                except (serial.SerialException, OSError):
                    pass
                try:
                    device.close()
                except (serial.SerialException, OSError) as exc:
                    if not error:
                        error = f"close failed: {exc}"
            self.emit("disconnected", error)


class EncoderCalibrationApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.events: queue.Queue[tuple[str, object]] = queue.Queue()
        self.stop_event = threading.Event()
        self.worker: SerialWorker | None = None
        self.connected = False
        self.port_labels: dict[str, str] = {}
        self.frames: deque[EncoderFrame] = deque(maxlen=25)
        self.controller_state: ControllerState | None = None
        self.ctl_safe_streak = 0
        self.sample_start: EncoderSnapshot | None = None
        self.sample_wheel = ""
        self.sample_turns = 0
        self.trial_number = 0
        self.results: dict[str, list[float]] = {"left": [], "right": []}
        self.closing = False

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        log_dir = Path(__file__).resolve().parent / "encoder_calibration_logs"
        log_dir.mkdir(parents=True, exist_ok=True)
        self.raw_path = log_dir / f"encoder_calibration_{timestamp}_raw.txt"
        self.csv_path = log_dir / f"encoder_calibration_{timestamp}_trials.csv"
        self.summary_path = log_dir / f"encoder_calibration_{timestamp}_summary.txt"
        self.raw_file = self.raw_path.open("w", encoding="utf-8", newline="")
        self.csv_file = self.csv_path.open("w", encoding="utf-8", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(
            (
                "trial",
                "start_pc_time",
                "end_pc_time",
                "port",
                "wheel",
                "turns",
                "start_mcu_time_ms",
                "end_mcu_time_ms",
                "start_sequence",
                "end_sequence",
                "start_left_count",
                "end_left_count",
                "left_delta",
                "start_right_count",
                "end_right_count",
                "right_delta",
                "selected_delta",
                "counts_per_wheel_rev",
                "other_wheel_delta",
                "duration_ms",
                "valid",
                "warning",
            )
        )
        self.csv_file.flush()
        self.raw_file.write(
            "# Safe encoder calibration: host transmits only 0.000,0.000\\r\\n\n"
            "# ENC,time_ms,sequence,left_total_count,right_total_count\n"
            "pc_time\tdirection\tframe\n"
        )
        self.raw_file.flush()

        self.port_var = tk.StringVar()
        self.turns_var = tk.StringVar(value="10")
        self.wheel_var = tk.StringVar(value="left")
        self.status_var = tk.StringVar(value="Disconnected")
        self.left_var = tk.StringVar(value="--")
        self.right_var = tk.StringVar(value="--")
        self.telemetry_var = tk.StringVar(value="No encoder telemetry")
        self.capture_var = tk.StringVar(value="No active capture")
        self.summary_var = tk.StringVar(value="Left: -- | Right: --")
        self.ctl_var = tk.StringVar(value="CTL: --")
        self.log_var = tk.StringVar(
            value=f"CSV: {self.csv_path} | Raw: {self.raw_path} | Summary: {self.summary_path}"
        )

        self.build_ui()
        self.refresh_ports()
        self.root.protocol("WM_DELETE_WINDOW", self.close_window)
        self.root.after(50, self.poll_events)

    def build_ui(self) -> None:
        self.root.title("Encoder 10-turn calibration")
        self.root.geometry("930x650")
        self.root.minsize(800, 560)

        main = ttk.Frame(self.root, padding=12)
        main.grid(row=0, column=0, sticky="nsew")
        self.root.rowconfigure(0, weight=1)
        self.root.columnconfigure(0, weight=1)
        main.columnconfigure(1, weight=1)
        main.rowconfigure(8, weight=1)

        ttk.Label(main, text="Serial port").grid(row=0, column=0, sticky="w")
        self.port_combo = ttk.Combobox(
            main, textvariable=self.port_var, state="readonly", width=52
        )
        self.port_combo.grid(row=0, column=1, sticky="ew", padx=8)
        self.refresh_button = ttk.Button(main, text="Refresh", command=self.refresh_ports)
        self.refresh_button.grid(row=0, column=2, padx=(0, 8))
        self.connect_button = ttk.Button(main, text="Connect", command=self.toggle_connection)
        self.connect_button.grid(row=0, column=3)

        status = ttk.Frame(main)
        status.grid(row=1, column=0, columnspan=4, sticky="ew", pady=(12, 4))
        for column in range(4):
            status.columnconfigure(column, weight=1)
        self.make_value(status, 0, "Left total count", self.left_var)
        self.make_value(status, 1, "Right total count", self.right_var)
        self.make_value(status, 2, "Encoder stream", self.telemetry_var)
        self.make_value(status, 3, "Controller", self.ctl_var)

        controls = ttk.LabelFrame(main, text="Manual wheel calibration", padding=10)
        controls.grid(row=2, column=0, columnspan=4, sticky="ew", pady=(8, 6))
        ttk.Label(controls, text="Wheel").grid(row=0, column=0, sticky="w")
        self.left_radio = ttk.Radiobutton(
            controls, text="Left", variable=self.wheel_var, value="left"
        )
        self.right_radio = ttk.Radiobutton(
            controls, text="Right", variable=self.wheel_var, value="right"
        )
        self.left_radio.grid(row=0, column=1, padx=(8, 4))
        self.right_radio.grid(row=0, column=2, padx=4)
        ttk.Label(controls, text="Turns").grid(row=0, column=3, padx=(24, 4))
        self.turns_entry = ttk.Entry(controls, textvariable=self.turns_var, width=6)
        self.turns_entry.grid(row=0, column=4)
        self.start_button = ttk.Button(
            controls, text="Capture start", command=self.capture_start, state="disabled"
        )
        self.start_button.grid(row=0, column=5, padx=(24, 6))
        self.end_button = ttk.Button(
            controls, text="Capture end and save", command=self.capture_end, state="disabled"
        )
        self.end_button.grid(row=0, column=6)
        controls.columnconfigure(7, weight=1)
        ttk.Label(controls, textvariable=self.capture_var).grid(
            row=1, column=0, columnspan=8, sticky="w", pady=(10, 0)
        )

        ttk.Label(main, textvariable=self.status_var).grid(
            row=3, column=0, columnspan=4, sticky="w", pady=(2, 6)
        )
        ttk.Label(main, text="Trial log").grid(row=4, column=0, sticky="w")
        ttk.Entry(main, textvariable=self.log_var, state="readonly").grid(
            row=4, column=1, columnspan=3, sticky="ew", padx=(8, 0)
        )

        columns = ("trial", "wheel", "turns", "left_delta", "right_delta", "cpr")
        self.trials = ttk.Treeview(main, columns=columns, show="headings", height=8)
        headings = {
            "trial": "Trial",
            "wheel": "Wheel",
            "turns": "Turns",
            "left_delta": "Left delta",
            "right_delta": "Right delta",
            "cpr": "Count / wheel rev",
        }
        widths = {"trial": 55, "wheel": 75, "turns": 65, "cpr": 155}
        for column in columns:
            self.trials.heading(column, text=headings[column])
            self.trials.column(column, width=widths.get(column, 140), anchor="center")
        self.trials.grid(row=5, column=0, columnspan=4, sticky="ew", pady=(4, 10))

        ttk.Label(main, textvariable=self.summary_var).grid(
            row=6, column=0, columnspan=4, sticky="w", pady=(0, 8)
        )

        ttk.Label(main, text="Raw stream").grid(row=7, column=0, columnspan=4, sticky="w")
        self.raw_text = scrolledtext.ScrolledText(
            main, height=12, wrap=tk.NONE, font=("Consolas", 9), state="disabled"
        )
        self.raw_text.grid(row=8, column=0, columnspan=4, sticky="nsew", pady=(4, 0))

    @staticmethod
    def make_value(parent: ttk.Frame, column: int, label: str, variable: tk.StringVar) -> None:
        cell = ttk.Frame(parent)
        cell.grid(row=0, column=column, sticky="ew", padx=(0, 12))
        ttk.Label(cell, text=label).grid(row=0, column=0, sticky="w")
        ttk.Label(cell, textvariable=variable, font=("Consolas", 11)).grid(
            row=1, column=0, sticky="w"
        )

    def write_raw(self, direction: str, frame: str) -> None:
        self.raw_file.write(f"{datetime.now().isoformat(timespec='milliseconds')}\t{direction}\t{frame}\n")
        self.raw_file.flush()

    def append_raw(self, line: str) -> None:
        self.raw_text.configure(state="normal")
        self.raw_text.insert(tk.END, line + "\n")
        if int(self.raw_text.index("end-1c").split(".")[0]) > 700:
            self.raw_text.delete("1.0", "201.0")
        self.raw_text.see(tk.END)
        self.raw_text.configure(state="disabled")

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
        if not self.connected:
            self.connect_button.configure(state="normal" if labels else "disabled")
        self.status_var.set(
            f"Found {len(labels)} serial port(s)" if labels else "No serial ports found"
        )

    def selected_port(self) -> str:
        return self.port_labels.get(self.port_var.get(), "")

    def toggle_connection(self) -> None:
        if self.worker is not None:
            self.disconnect()
            return
        port = self.selected_port()
        if not port:
            messagebox.showerror("Encoder calibration", "Select a serial port first.")
            return
        self.frames.clear()
        self.controller_state = None
        self.ctl_safe_streak = 0
        self.sample_start = None
        self.stop_event = threading.Event()
        self.worker = SerialWorker(port, self.events, self.stop_event)
        self.set_connection_controls(connecting=True)
        self.status_var.set(f"Opening {port}...")
        self.worker.start()

    def set_connection_controls(self, connecting: bool) -> None:
        self.port_combo.configure(state="disabled" if connecting else "readonly")
        self.refresh_button.configure(state="disabled" if connecting else "normal")
        self.connect_button.configure(text="Disconnect" if connecting else "Connect")
        self.connect_button.configure(
            state="normal" if connecting or self.port_labels else "disabled"
        )

    def disconnect(self) -> None:
        if self.worker is not None:
            self.status_var.set("Stopping zero-speed link...")
            self.stop_event.set()
            self.connect_button.configure(state="disabled")

    def snapshot(self) -> EncoderSnapshot | None:
        if len(self.frames) < SNAPSHOT_SAMPLES:
            messagebox.showerror(
                "Encoder calibration", "Not enough fresh ENC frames. Hold the wheel still briefly."
            )
            return None
        samples = list(self.frames)[-SNAPSHOT_SAMPLES:]
        if time.monotonic() - samples[-1].received_s > SNAPSHOT_MAX_AGE_S:
            messagebox.showerror("Encoder calibration", "Encoder telemetry is stale.")
            return None

        if samples[-1].received_s - samples[0].received_s > SNAPSHOT_MAX_WINDOW_S:
            messagebox.showerror(
                "Encoder calibration", "ENC sample window contains an old gap. Hold briefly and retry."
            )
            return None
        if any(
            newer.received_s - older.received_s > SNAPSHOT_MAX_GAP_S
            for older, newer in zip(samples, samples[1:])
        ):
            messagebox.showerror(
                "Encoder calibration", "ENC telemetry was interrupted. Hold briefly and retry."
            )
            return None
        if not self.controller_is_safe():
            messagebox.showerror(
                "Encoder calibration",
                "CTL does not confirm navigation mode, valid zero command, and zero motor output.",
            )
            return None

        left_values = [sample.left_unwrapped for sample in samples]
        right_values = [sample.right_unwrapped for sample in samples]
        if any(value is None for value in left_values + right_values):
            messagebox.showerror("Encoder calibration", "Internal encoder unwrap state is invalid.")
            return None
        left_unwrapped = [int(value) for value in left_values if value is not None]
        right_unwrapped = [int(value) for value in right_values if value is not None]
        if (
            max(left_unwrapped) - min(left_unwrapped) > SNAPSHOT_MAX_SPREAD
            or max(right_unwrapped) - min(right_unwrapped) > SNAPSHOT_MAX_SPREAD
        ):
            messagebox.showwarning(
                "Encoder calibration", "Counts are still changing. Stop both wheels, then capture again."
            )
            return None

        newest = samples[-1]
        median_left = int(statistics.median(left_unwrapped))
        median_right = int(statistics.median(right_unwrapped))
        return EncoderSnapshot(
            pc_time=datetime.now().isoformat(timespec="milliseconds"),
            mcu_time_ms=newest.mcu_time_ms,
            sequence=newest.sequence,
            left_count=signed_int32(median_left),
            right_count=signed_int32(median_right),
            left_unwrapped=median_left,
            right_unwrapped=median_right,
        )

    @staticmethod
    def controller_values_safe(state: ControllerState) -> bool:
        return (
            state.mode == 1
            and state.emergency == 0
            and state.command_valid == 1
            and state.command_vx_mmps == 0
            and state.command_az_mradps == 0
            and state.target_left_mmps == 0
            and state.target_right_mmps == 0
            and state.left_output == 0
            and state.right_output == 0
        )

    def controller_is_safe(self) -> bool:
        state = self.controller_state
        if state is None or time.monotonic() - state.received_s > SNAPSHOT_MAX_AGE_S:
            return False
        return (
            self.ctl_safe_streak >= CTL_SAFE_STREAK_REQUIRED
            and self.controller_values_safe(state)
        )

    def update_capture_ready(self) -> None:
        ready = (
            self.connected
            and self.sample_start is None
            and len(self.frames) >= SNAPSHOT_SAMPLES
            and self.controller_is_safe()
        )
        self.start_button.configure(state="normal" if ready else "disabled")

    def abort_capture(self, reason: str) -> None:
        if self.sample_start is None:
            return
        self.write_raw("EVENT", f"CAPTURE_ABORT,{self.sample_wheel},{reason}")
        self.sample_start = None
        self.sample_wheel = ""
        self.sample_turns = 0
        self.capture_var.set(f"Capture aborted: {reason}")
        self.end_button.configure(state="disabled")
        self.left_radio.configure(state="normal")
        self.right_radio.configure(state="normal")
        self.turns_entry.configure(state="normal")
        self.update_capture_ready()

    def capture_start(self) -> None:
        try:
            turns = int(self.turns_var.get(), 10)
        except ValueError:
            messagebox.showerror("Encoder calibration", "Turns must be a whole number.")
            return
        if not 1 <= turns <= 1000:
            messagebox.showerror("Encoder calibration", "Turns must be between 1 and 1000.")
            return
        sample = self.snapshot()
        if sample is None:
            return
        self.sample_start = sample
        self.sample_wheel = self.wheel_var.get()
        self.sample_turns = turns
        self.capture_var.set(
            f"Start captured for {self.sample_wheel}: L={sample.left_count}, "
            f"R={sample.right_count}; rotate exactly {turns} turns, then stop."
        )
        self.start_button.configure(state="disabled")
        self.end_button.configure(state="normal")
        self.left_radio.configure(state="disabled")
        self.right_radio.configure(state="disabled")
        self.turns_entry.configure(state="disabled")
        self.write_raw("EVENT", f"CAPTURE_START,{self.sample_wheel},{turns},{sample.left_count},{sample.right_count}")

    def capture_end(self) -> None:
        start = self.sample_start
        if start is None:
            return
        end = self.snapshot()
        if end is None:
            return

        left_delta = end.left_unwrapped - start.left_unwrapped
        right_delta = end.right_unwrapped - start.right_unwrapped
        selected_delta = left_delta if self.sample_wheel == "left" else right_delta
        other_delta = right_delta if self.sample_wheel == "left" else left_delta
        if selected_delta == 0:
            messagebox.showerror("Encoder calibration", "Selected wheel count did not change.")
            return

        counts_per_rev = abs(selected_delta) / self.sample_turns
        duration_ms = (end.mcu_time_ms - start.mcu_time_ms) & 0xFFFFFFFF
        valid = abs(other_delta) <= OTHER_WHEEL_MAX_DELTA
        warning = "" if valid else f"other wheel moved {other_delta} counts"
        self.trial_number += 1
        self.csv_writer.writerow(
            (
                self.trial_number,
                start.pc_time,
                end.pc_time,
                self.selected_port(),
                self.sample_wheel,
                self.sample_turns,
                start.mcu_time_ms,
                end.mcu_time_ms,
                start.sequence,
                end.sequence,
                start.left_count,
                end.left_count,
                left_delta,
                start.right_count,
                end.right_count,
                right_delta,
                selected_delta,
                f"{counts_per_rev:.6f}",
                other_delta,
                duration_ms,
                int(valid),
                warning,
            )
        )
        self.csv_file.flush()
        self.write_raw(
            "EVENT",
            f"CAPTURE_END,{self.sample_wheel},{self.sample_turns},{left_delta},"
            f"{right_delta},{counts_per_rev:.6f},{int(valid)},{warning}",
        )
        self.trials.insert(
            "",
            tk.END,
            values=(
                self.trial_number,
                self.sample_wheel,
                self.sample_turns,
                left_delta,
                right_delta,
                f"{counts_per_rev:.3f}",
            ),
        )
        if valid:
            self.results[self.sample_wheel].append(counts_per_rev)
        summaries = []
        for wheel in ("left", "right"):
            values = self.results[wheel]
            if not values:
                summaries.append(f"{wheel.title()}: --")
                continue
            spread = statistics.pstdev(values) if len(values) > 1 else 0.0
            summaries.append(
                f"{wheel.title()}: n={len(values)}, mean={statistics.mean(values):.3f}, "
                f"sigma={spread:.3f} count/rev"
            )
        self.summary_var.set(" | ".join(summaries))
        self.write_summary()
        self.capture_var.set(
            f"Trial {self.trial_number} saved: {self.sample_wheel} = "
            f"{counts_per_rev:.3f} count/rev (signed delta {selected_delta})"
            + (f"; INVALID: {warning}." if warning else ".")
        )
        self.sample_start = None
        self.start_button.configure(state="normal")
        self.end_button.configure(state="disabled")
        self.left_radio.configure(state="normal")
        self.right_radio.configure(state="normal")
        self.turns_entry.configure(state="normal")
        self.update_capture_ready()
        if warning:
            messagebox.showwarning(
                "Encoder calibration",
                f"Trial saved as invalid because the {('right' if self.sample_wheel == 'left' else 'left')} "
                f"wheel moved {other_delta} counts.",
            )

    def write_summary(self) -> None:
        lines = [
            "Encoder calibration summary",
            f"Raw log: {self.raw_path}",
            f"Trial CSV: {self.csv_path}",
        ]
        for wheel in ("left", "right"):
            values = self.results[wheel]
            if not values:
                lines.append(f"{wheel}: no valid trials")
                continue
            lines.append(
                f"{wheel}: n={len(values)}, mean={statistics.mean(values):.6f}, "
                f"sigma={statistics.pstdev(values) if len(values) > 1 else 0.0:.6f} "
                f"count/rev, values={','.join(f'{item:.6f}' for item in values)}"
            )
        self.summary_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    def handle_line(self, received_s: float, line: str) -> None:
        self.write_raw("RX", line)
        frame = parse_encoder(line, received_s)
        if frame is not None:
            previous = self.frames[-1] if self.frames else None
            if previous is not None:
                interval_ms = (frame.mcu_time_ms - previous.mcu_time_ms) & 0xFFFFFFFF
                if interval_ms == 0 or interval_ms > ENCODER_MAX_INTERVAL_MS:
                    self.abort_capture(f"MCU encoder time discontinuity ({interval_ms} ms)")
                    self.frames.clear()
                    previous = None

            if previous is None:
                left_unwrapped = frame.left_count
                right_unwrapped = frame.right_count
            else:
                if previous.left_unwrapped is None or previous.right_unwrapped is None:
                    self.abort_capture("internal unwrap state lost")
                    self.frames.clear()
                    left_unwrapped = frame.left_count
                    right_unwrapped = frame.right_count
                else:
                    left_unwrapped = previous.left_unwrapped + signed_int32_delta(
                        frame.left_count, previous.left_count
                    )
                    right_unwrapped = previous.right_unwrapped + signed_int32_delta(
                        frame.right_count, previous.right_count
                    )
            frame = EncoderFrame(
                received_s=frame.received_s,
                mcu_time_ms=frame.mcu_time_ms,
                sequence=frame.sequence,
                left_count=frame.left_count,
                right_count=frame.right_count,
                left_unwrapped=left_unwrapped,
                right_unwrapped=right_unwrapped,
            )
            self.frames.append(frame)
            self.left_var.set(str(frame.left_count))
            self.right_var.set(str(frame.right_count))
            self.telemetry_var.set(f"seq {frame.sequence}")
            if frame.sequence % 10 == 0:
                self.append_raw(line)
            self.update_capture_ready()
            return

        fields = line.split(",")
        if len(fields) == 15 and fields[0] == "CTL":
            try:
                values = [int(field, 10) for field in fields[1:]]
                state = ControllerState(
                    received_s=received_s,
                    mode=values[2],
                    emergency=values[3],
                    command_valid=values[4],
                    command_vx_mmps=values[6],
                    command_az_mradps=values[7],
                    target_left_mmps=values[8],
                    target_right_mmps=values[9],
                    left_output=values[12],
                    right_output=values[13],
                )
                previous_state = self.controller_state
                if self.controller_values_safe(state):
                    if (
                        previous_state is not None
                        and self.controller_values_safe(previous_state)
                        and 0.0 <= state.received_s - previous_state.received_s <= CTL_MAX_GAP_S
                    ):
                        self.ctl_safe_streak += 1
                    else:
                        self.ctl_safe_streak = 1
                else:
                    self.ctl_safe_streak = 0
                    self.abort_capture("controller left the safe zero-output state")
                self.controller_state = state
                self.ctl_var.set(
                    f"mode={values[2]} valid={values[4]} "
                    f"out={values[12]}/{values[13]}"
                )
                self.update_capture_ready()
            except ValueError:
                self.controller_state = None
                self.ctl_safe_streak = 0
                self.abort_capture("invalid CTL frame")
                self.ctl_var.set("CTL parse error")
        elif not line.startswith("CTL,"):
            self.append_raw(line)

    def poll_events(self) -> None:
        try:
            while True:
                kind, value = self.events.get_nowait()
                if kind == "connected":
                    self.connected = True
                    self.status_var.set(f"Connected to {value}; transmitting zero speed only")
                    self.write_raw("EVENT", f"CONNECTED,{value}")
                elif kind == "tx":
                    self.write_raw("TX", str(value))
                elif kind == "line":
                    received_s, line = value  # type: ignore[misc]
                    self.handle_line(float(received_s), str(line))
                elif kind == "notice":
                    self.write_raw("NOTICE", str(value))
                    self.append_raw(str(value))
                elif kind == "disconnected":
                    error = str(value)
                    self.abort_capture(f"serial disconnected{': ' + error if error else ''}")
                    self.connected = False
                    self.worker = None
                    self.frames.clear()
                    self.controller_state = None
                    self.ctl_safe_streak = 0
                    if self.sample_start is None and not self.capture_var.get().startswith("Capture aborted"):
                        self.capture_var.set("No active capture")
                    self.start_button.configure(state="disabled")
                    self.end_button.configure(state="disabled")
                    self.left_radio.configure(state="normal")
                    self.right_radio.configure(state="normal")
                    self.turns_entry.configure(state="normal")
                    self.set_connection_controls(connecting=False)
                    self.status_var.set(f"Disconnected: {error}" if error else "Disconnected")
                    self.write_raw("EVENT", f"DISCONNECTED,{error}")
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
        self.write_summary()
        if not self.raw_file.closed:
            self.raw_file.close()
        if not self.csv_file.closed:
            self.csv_file.close()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    EncoderCalibrationApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
