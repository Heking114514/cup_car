#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None


DEFAULT_BAUD = 115200
SERIAL_TIMEOUT_S = 0.05

FRAME_FIELDS = {
    "ENC": (
        "time_ms",
        "seq",
        "left_total_count",
        "right_total_count",
    ),
    "IMU": (
        "time_ms",
        "seq",
        "status",
        "raw_gx",
        "raw_gy",
        "raw_gz",
        "temperature_cC",
    ),
    "ATT": (
        "time_ms",
        "seq",
        "status",
        "yaw_mdeg",
        "gyro_z_mdps",
        "bias_z_mdps",
        "startup_samples",
    ),
    "HLD": (
        "time_ms",
        "seq",
        "heading_active",
        "heading_ref_mdeg",
        "heading_error_mdeg",
        "heading_correction_milli_ref",
    ),
    "SYN": (
        "time_ms",
        "seq",
        "sync_active",
        "phase_error_mdeg",
        "sync_correction_milli_ref",
    ),
    "CTL": (
        "time_ms",
        "seq",
        "navigation_mode",
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
    ),
    "MTR": (
        "time_ms",
        "seq",
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
    ),
    "RFID": (
        "time_ms",
        "seq",
        "status",
        "protocol_status",
        "tag_type",
        "sak",
        "number",
        "uid_hex",
    ),
}

BMI088_STATUS_BITS = (
    (0x01, "gyro_valid"),
    (0x02, "temp_valid"),
    (0x04, "yaw_valid"),
    (0x08, "bias_valid"),
    (0x10, "stationary"),
    (0x20, "saturated"),
    (0x40, "sample_timeout"),
)


def host_timestamp() -> str:
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


def compact_timestamp() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def parse_number(text: str) -> Any:
    value = text.strip()
    if value == "":
        return value
    try:
        return int(value, 0)
    except ValueError:
        return value


def add_scaled_fields(fields: dict[str, Any]) -> None:
    for name, value in list(fields.items()):
        if not isinstance(value, int):
            continue
        if name.endswith("_mdeg"):
            fields[f"{name[:-5]}_deg"] = value / 1000.0
        elif name.endswith("_mdps"):
            fields[f"{name[:-5]}_dps"] = value / 1000.0
        elif name.endswith("_mmps"):
            fields[f"{name[:-5]}_mps"] = value / 1000.0
        elif name.endswith("_mradps"):
            fields[f"{name[:-7]}_radps"] = value / 1000.0
        elif name == "temperature_cC":
            fields["temperature_C"] = value / 100.0
        elif name.endswith("_milli_ref"):
            fields[f"{name[:-10]}_ref"] = value / 1000.0


def decode_bmi088_status(status: Any) -> str:
    if not isinstance(status, int):
        return str(status)
    flags = [name for bit, name in BMI088_STATUS_BITS if status & bit]
    if not flags:
        flags.append("none")
    unknown = status & ~sum(bit for bit, _name in BMI088_STATUS_BITS)
    if unknown:
        flags.append(f"unknown_0x{unknown:02X}")
    return f"0x{status:02X}:" + "|".join(flags)


def parse_frame(line: str) -> tuple[str, dict[str, Any], str]:
    parts = [part.strip() for part in line.split(",")]
    frame = parts[0].upper() if parts and parts[0] else "UNKNOWN"
    names = FRAME_FIELDS.get(frame)
    if not names:
        return frame, {}, "unparsed"

    values = parts[1:]
    fields: dict[str, Any] = {}
    for index, name in enumerate(names):
        if index < len(values):
            fields[name] = parse_number(values[index])
        else:
            fields[name] = ""

    if len(values) > len(names):
        fields["extra"] = ",".join(values[len(names):])

    add_scaled_fields(fields)
    if frame in ("ATT", "IMU"):
        fields["status_flags"] = decode_bmi088_status(fields.get("status"))
    return frame, fields, format_kv(fields)


def format_value(value: Any) -> str:
    if isinstance(value, float):
        return f"{value:.3f}"
    return str(value)


def format_kv(fields: dict[str, Any]) -> str:
    return " ".join(f"{name}={format_value(value)}" for name, value in fields.items())


def snapshot_fields(state: dict[str, dict[str, Any]]) -> dict[str, Any]:
    ctl = state.get("CTL", {})
    att = state.get("ATT", {})
    hld = state.get("HLD", {})
    sync = state.get("SYN", {})
    enc = state.get("ENC", {})
    imu = state.get("IMU", {})
    motor = state.get("MTR", {})
    rfid = state.get("RFID", {})

    fields: dict[str, Any] = {}
    for source in (ctl, att, hld, sync, enc, imu, motor, rfid):
        if "time_ms" in source:
            fields["latest_mcu_time_ms"] = source["time_ms"]
    if "seq" in ctl:
        fields["ctl_seq"] = ctl["seq"]

    fields.update(
        {
            "cmd_valid": ctl.get("command_valid", "-"),
            "cmd_vx_mps": ctl.get("command_vx_mps", "-"),
            "cmd_az_radps": ctl.get("command_az_radps", "-"),
            "target_l_mps": ctl.get("target_left_mps", "-"),
            "target_r_mps": ctl.get("target_right_mps", "-"),
            "meas_l_mps": ctl.get("measured_left_mps", "-"),
            "meas_r_mps": ctl.get("measured_right_mps", "-"),
            "out_l": ctl.get("left_output", "-"),
            "out_r": ctl.get("right_output", "-"),
            "current_l": motor.get("left_current", "-"),
            "current_r": motor.get("right_current", "-"),
            "integral_l": motor.get("left_integral_output", "-"),
            "integral_r": motor.get("right_integral_output", "-"),
            "drive_status": motor.get("drive_status", "-"),
            "motor_online_l": motor.get("left_feedback_online", "-"),
            "motor_online_r": motor.get("right_feedback_online", "-"),
            "motor_age_l_ms": motor.get("left_feedback_age_ms", "-"),
            "motor_age_r_ms": motor.get("right_feedback_age_ms", "-"),
            "motor_rx_l": motor.get("left_feedback_count", "-"),
            "motor_rx_r": motor.get("right_feedback_count", "-"),
            "can_tx_ok": motor.get("can_tx_success_count", "-"),
            "can_tx_fail": motor.get("can_tx_failure_count", "-"),
            "can_state": motor.get("can_state", "-"),
            "can_error": motor.get("can_hal_error_code", "-"),
            "can_bus_off": motor.get("can_bus_off", "-"),
            "enc_l": enc.get("left_total_count", "-"),
            "enc_r": enc.get("right_total_count", "-"),
            "yaw_deg": att.get("yaw_deg", "-"),
            "gyro_z_dps": att.get("gyro_z_dps", "-"),
            "bias_z_dps": att.get("bias_z_dps", "-"),
            "att_status": att.get("status_flags", "-"),
            "hld_active": hld.get("heading_active", "-"),
            "hld_ref_deg": hld.get("heading_ref_deg", "-"),
            "hld_err_deg": hld.get("heading_error_deg", "-"),
            "hld_corr": hld.get("heading_correction_ref", "-"),
            "sync_active": sync.get("sync_active", "-"),
            "sync_phase_deg": sync.get("phase_error_deg", "-"),
            "sync_corr": sync.get("sync_correction_ref", "-"),
            "imu_raw_gz": imu.get("raw_gz", "-"),
            "rfid_number": rfid.get("number", "-"),
            "rfid_uid": rfid.get("uid_hex", "-"),
        }
    )
    return fields


def port_sort_key(port: Any) -> tuple[int, str]:
    text = " ".join(
        str(item or "")
        for item in (
            getattr(port, "description", ""),
            getattr(port, "manufacturer", ""),
            getattr(port, "product", ""),
            getattr(port, "hwid", ""),
        )
    ).lower()
    score = 100
    if (getattr(port, "vid", None), getattr(port, "pid", None)) == (0x0483, 0x5740):
        score -= 70
    if "stmicroelectronics" in text or "stm32" in text or "stlink" in text:
        score -= 25
    if "cdc" in text or "virtual" in text:
        score -= 10
    device = getattr(port, "device", "")
    if device.startswith("/dev/ttyACM"):
        score -= 8
    if device.startswith("/dev/ttyUSB"):
        score -= 4
    return score, device


def format_port(port: Any) -> str:
    vid = getattr(port, "vid", None)
    pid = getattr(port, "pid", None)
    vid_pid = f" VID:PID={vid:04X}:{pid:04X}" if vid is not None and pid is not None else ""
    desc = getattr(port, "description", "") or "Serial port"
    hwid = getattr(port, "hwid", "") or ""
    return f"{port.device} | {desc}{vid_pid} | {hwid}"


def list_serial_ports() -> list[Any]:
    if list_ports is None:
        return []
    ports = list(list_ports.comports())
    ports.sort(key=port_sort_key)
    return ports


def choose_port(requested: str | None) -> str:
    if requested:
        return requested

    ports = list_serial_ports()
    if not ports:
        raise RuntimeError("No serial ports found. Check USB cable and permissions.")
    return ports[0].device


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Record lower-machine VCP telemetry into a timestamped txt log."
    )
    parser.add_argument("--port", help="Serial device, for example /dev/ttyACM0.")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Serial baud rate.")
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Stop after this many seconds. Default 0 means run until Ctrl-C.",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path(__file__).resolve().parent / "logs",
        help="Directory for log files.",
    )
    parser.add_argument(
        "--snapshot-hz",
        type=float,
        default=20.0,
        help="Write merged SNAP lines at this rate. Use 0 to disable.",
    )
    parser.add_argument("--list", action="store_true", help="List serial ports and exit.")
    parser.add_argument(
        "--assert-dtr",
        action="store_true",
        help="Assert DTR/RTS after opening. Default leaves them deasserted.",
    )
    parser.add_argument(
        "--no-console",
        action="store_true",
        help="Do not mirror received frames to the terminal.",
    )
    return parser


def write_header(log_file: Any, args: argparse.Namespace, port: str, ports: list[Any]) -> None:
    log_file.write("# car_state_log_v1\n")
    log_file.write(f"# start_time={host_timestamp()}\n")
    log_file.write(f"# port={port}\n")
    log_file.write(f"# baud={args.baud}\n")
    log_file.write(f"# snapshot_hz={args.snapshot_hz}\n")
    log_file.write(f"# duration_s={args.duration}\n")
    log_file.write(f"# command={' '.join(sys.argv)}\n")
    log_file.write("# known_frames=ENC,IMU,ATT,HLD,SYN,CTL,MTR,RFID\n")
    log_file.write("# bmi088_status_bits=0x01 gyro_valid,0x02 temp_valid,0x04 yaw_valid,")
    log_file.write("0x08 bias_valid,0x10 stationary,0x20 saturated,0x40 sample_timeout\n")
    log_file.write("# detected_ports:\n")
    for item in ports:
        log_file.write(f"#   {format_port(item)}\n")
    log_file.write("# columns: host_time\telapsed_s\tframe\tparsed\traw\n")
    log_file.flush()


def log_record(
    log_file: Any,
    start_s: float,
    frame: str,
    parsed: str,
    raw: str,
    mirror: bool,
) -> None:
    elapsed_s = time.monotonic() - start_s
    line = f"{host_timestamp()}\t{elapsed_s:.3f}\t{frame}\t{parsed}\t{raw}\n"
    log_file.write(line)
    if mirror:
        print(line, end="")


def open_serial(port: str, baud: int, assert_dtr: bool) -> Any:
    device = serial.Serial(
        port=port,
        baudrate=baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=SERIAL_TIMEOUT_S,
        write_timeout=0.25,
        rtscts=False,
        dsrdtr=False,
    )
    device.setDTR(bool(assert_dtr))
    device.setRTS(bool(assert_dtr))
    device.reset_input_buffer()
    return device


def run_logger(args: argparse.Namespace) -> int:
    if serial is None:
        print("pyserial is missing. Install it with: sudo apt install python3-serial", file=sys.stderr)
        return 2

    ports = list_serial_ports()
    if args.list:
        if not ports:
            print("No serial ports found.")
            return 1
        for port in ports:
            print(format_port(port))
        return 0

    try:
        port = choose_port(args.port)
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        return 1

    args.out_dir.mkdir(parents=True, exist_ok=True)
    log_path = args.out_dir / f"car_state_{compact_timestamp()}.txt"
    state: dict[str, dict[str, Any]] = {}
    buffer = bytearray()
    start_s = time.monotonic()
    next_snapshot_s = start_s
    snapshot_period_s = 1.0 / args.snapshot_hz if args.snapshot_hz > 0.0 else 0.0

    print(f"Opening {port} at {args.baud} baud")
    print(f"Logging to {log_path}")
    print("Press Ctrl-C to stop.")

    try:
        with log_path.open("w", encoding="utf-8", newline="") as log_file:
            write_header(log_file, args, port, ports)
            with open_serial(port, args.baud, args.assert_dtr) as device:
                log_record(log_file, start_s, "EVENT", f"opened_port={port}", "OPEN", not args.no_console)
                while True:
                    if args.duration > 0.0 and time.monotonic() - start_s >= args.duration:
                        break

                    chunk = device.read(device.in_waiting or 1)
                    if not chunk:
                        continue
                    buffer.extend(chunk)

                    if len(buffer) > 8192:
                        log_record(
                            log_file,
                            start_s,
                            "EVENT",
                            "rx_buffer_overflow=1",
                            "RX_BUFFER_OVERFLOW",
                            not args.no_console,
                        )
                        buffer.clear()
                        continue

                    while b"\n" in buffer:
                        raw_line, _separator, remainder = buffer.partition(b"\n")
                        buffer = bytearray(remainder)
                        line = raw_line.rstrip(b"\r").decode("ascii", errors="replace")
                        if not line:
                            continue

                        frame, fields, parsed = parse_frame(line)
                        if fields:
                            state[frame] = fields
                        log_record(log_file, start_s, frame, parsed, line, not args.no_console)

                        now_s = time.monotonic()
                        if snapshot_period_s > 0.0 and now_s >= next_snapshot_s and state:
                            log_record(
                                log_file,
                                start_s,
                                "SNAP",
                                format_kv(snapshot_fields(state)),
                                "-",
                                not args.no_console,
                            )
                            next_snapshot_s = now_s + snapshot_period_s
                    log_file.flush()
    except KeyboardInterrupt:
        print("\nStopped by Ctrl-C.")
    except (serial.SerialException, OSError) as exc:
        print(f"Serial error: {exc}", file=sys.stderr)
        print("If the device is busy, close the ROS/upper-machine serial node first.", file=sys.stderr)
        return 1

    print(f"Saved: {log_path}")
    return 0


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()
    return run_logger(args)


if __name__ == "__main__":
    raise SystemExit(main())
