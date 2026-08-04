from __future__ import annotations

import argparse
import sys
import time
from datetime import datetime
from pathlib import Path

import serial
from serial.tools import list_ports


def available_ports() -> list[tuple[str, str]]:
    return sorted(
        ((port.device, port.description) for port in list_ports.comports()),
        key=lambda item: item[0],
    )


def select_port(requested: str | None) -> str:
    if requested:
        return requested

    ports = available_ports()
    if len(ports) == 1:
        return ports[0][0]
    if not ports:
        raise RuntimeError("No serial port found. Specify one with --port.")

    choices = "\n".join(f"  {device}: {description}" for device, description in ports)
    raise RuntimeError(f"Multiple serial ports found. Use --port:\n{choices}")


def default_output_path() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path(__file__).resolve().parent / "logs" / f"straight_run_{stamp}.txt"


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Record cup-car encoder telemetry to TXT."
    )
    parser.add_argument("--port", help="Serial device, for example COM5 or /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--output", type=Path, help="Output TXT path")
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Recording duration in seconds; 0 records until Ctrl+C",
    )
    parser.add_argument("--list-ports", action="store_true", help="List ports and exit")
    return parser.parse_args()


def print_ports() -> None:
    ports = available_ports()
    if not ports:
        print("No serial ports found.")
        return
    for device, description in ports:
        print(f"{device}\t{description}")


def format_encoder_status(line: str) -> str | None:
    fields = line.split(",")
    if len(fields) != 5 or fields[0] != "ENC":
        return None
    try:
        mcu_ms, sequence, left_total, right_total = [int(value) for value in fields[1:]]
    except ValueError:
        return None

    return (
        f"mcu={mcu_ms} ms seq={sequence} "
        f"left={left_total:+d} right={right_total:+d}"
    )


def record(port: str, baud: int, output_path: Path, duration: float) -> int:
    output_path = output_path.expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    enc_count = 0
    other_count = 0
    last_status_at = 0.0
    latest_encoder_status: str | None = None
    started_wall = datetime.now().astimezone()
    started_monotonic = time.monotonic()

    with serial.Serial(port=port, baudrate=baud, timeout=0.2) as device, output_path.open(
        "w", encoding="utf-8", newline="\n"
    ) as output:
        device.reset_input_buffer()
        output.write("# Cup car straight-run telemetry\n")
        output.write(f"# Started: {started_wall.isoformat(timespec='milliseconds')}\n")
        output.write(f"# Port: {port}, baud: {baud}\n")
        output.write("# ENC fields: mcu_ms,sample_sequence,left_total,right_total\n")
        output.write("pc_elapsed_s\tpc_time\tframe\n")
        output.flush()

        print(f"Recording {port} at {baud} baud")
        print(f"Output: {output_path}")
        print("Drive straight with the remote. Press Ctrl+C to stop.")

        try:
            while duration <= 0.0 or time.monotonic() - started_monotonic < duration:
                raw = device.readline()
                if not raw:
                    continue

                received_at = datetime.now().astimezone()
                elapsed = time.monotonic() - started_monotonic
                line = raw.decode("ascii", errors="replace").strip("\r\n")
                if not line:
                    continue

                output.write(
                    f"{elapsed:.6f}\t{received_at.isoformat(timespec='milliseconds')}\t{line}\n"
                )
                output.flush()

                if line.startswith("ENC,"):
                    enc_count += 1
                    latest_encoder_status = format_encoder_status(line)
                else:
                    other_count += 1

                if elapsed - last_status_at >= 1.0:
                    status = latest_encoder_status or "waiting for ENC telemetry"
                    print(
                        f"{elapsed:7.1f}s  ENC={enc_count} other={other_count}  {status}"
                    )
                    last_status_at = elapsed
        except KeyboardInterrupt:
            print("\nRecording stopped.")

    print(f"Saved {enc_count} ENC frames to {output_path}")
    return 0 if enc_count > 0 else 2


def main() -> int:
    args = parse_arguments()
    if args.list_ports:
        print_ports()
        return 0
    if args.duration < 0.0:
        raise ValueError("--duration cannot be negative")

    try:
        port = select_port(args.port)
        return record(port, args.baud, args.output or default_output_path(), args.duration)
    except (RuntimeError, ValueError, serial.SerialException, OSError) as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
