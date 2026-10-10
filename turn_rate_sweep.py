#!/usr/bin/env python3
"""Sweep serial angular velocity in both directions and log chassis response."""

from __future__ import annotations

import argparse
import csv
import sys
import time
from datetime import datetime
from pathlib import Path

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("缺少 pyserial：sudo apt install python3-serial", file=sys.stderr)
    raise SystemExit(2)


def choose_port(requested: str | None) -> str:
    if requested:
        return requested
    ports = list(list_ports.comports())
    if len(ports) == 1:
        return ports[0].device
    if not ports:
        raise RuntimeError("没有发现串口，请用 --port 指定设备")
    values = ", ".join(port.device for port in ports)
    raise RuntimeError(f"发现多个串口 ({values})，请用 --port 指定")


def send_command(device: serial.Serial, vx: float, az: float) -> None:
    device.write(f"{vx:.6f},{az:.6f}\n".encode("ascii"))


def parse_frame(line: str) -> tuple[str, list[str]]:
    fields = line.strip().split(",")
    return (fields[0] if fields else "", fields)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="测试 vx 前进时左右转向角速度 0.01..0.15 rad/s，并记录 CTL/MTR/ATT"
    )
    parser.add_argument("--port", help="虚拟串口，例如 /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--vx", type=float, default=0.03,
                        help="前进速度 m/s，默认 0.03；设为 0 可测试原地转向")
    parser.add_argument("--hold", type=float, default=0.5,
                        help="每档保持秒数，默认 0.5")
    parser.add_argument("--gap", type=float, default=0.5,
                        help="每档之间停车秒数，默认 0.5")
    parser.add_argument("--start", type=float, default=0.01)
    parser.add_argument("--stop", type=float, default=0.15)
    parser.add_argument("--step", type=float, default=0.01)
    parser.add_argument("--output-dir", default="turn_rate_sweep_logs")
    args = parser.parse_args()

    if args.hold <= 0 or args.gap < 0 or args.step <= 0:
        parser.error("hold/gap/step 参数必须为正，gap 可为 0")
    if args.start <= 0 or args.stop < args.start:
        parser.error("角速度范围无效")
    if abs(args.vx) > 2.0:
        parser.error("|vx| 不能超过 2.0 m/s")

    port = choose_port(args.port)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    raw_path = out_dir / f"turn_rate_sweep_{stamp}_raw.txt"
    csv_path = out_dir / f"turn_rate_sweep_{stamp}.csv"

    fields = [
        "host_time", "elapsed_s", "direction", "stage", "command_vx_mps",
        "command_az_radps", "frame_type", "raw_frame", "mcu_time_ms",
        "sequence", "status", "yaw_mdeg", "gyro_z_mdps", "bias_z_mdps",
        "left_measured_mps", "right_measured_mps", "left_output",
        "right_output",
    ]

    print(f"串口: {port}")
    print(f"日志: {raw_path}")
    print("开始前请确认车轮区域安全；Ctrl-C 会立即发送停车命令。")

    started = time.monotonic()
    stage = "settle"
    direction = "none"
    command_vx = 0.0
    command_az = 0.0

    with raw_path.open("w", encoding="utf-8") as raw_file, \
            csv_path.open("w", encoding="utf-8", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fields)
        writer.writeheader()
        raw_file.write(
            f"# vx={args.vx:.6f} m/s, az={args.start:.2f}..{args.stop:.2f}, "
            f"step={args.step:.2f}, hold={args.hold:.3f}s, gap={args.gap:.3f}s\n"
        )
        raw_file.write("host_time\tdirection\tstage\ttx/rx\tframe\n")

        try:
            with serial.Serial(port, args.baud, timeout=0.02,
                               write_timeout=0.25) as device:
                def transmit(vx: float, az: float, label: str, phase: str) -> None:
                    nonlocal command_vx, command_az, stage
                    command_vx, command_az, stage = vx, az, phase
                    send_command(device, vx, az)
                    raw_file.write(
                        f"{datetime.now().astimezone().isoformat()}\t{label}\t"
                        f"{phase}\tTX\t{vx:.6f},{az:.6f}\n"
                    )
                    raw_file.flush()

                transmit(0.0, 0.0, "none", "settle")
                time.sleep(1.0)

                values = []
                value = args.start
                while value <= args.stop + args.step * 0.25:
                    values.append(round(value, 6))
                    value += args.step

                for label, sign in (("left", 1.0), ("right", -1.0)):
                    for index, magnitude in enumerate(values, start=1):
                        transmit(args.vx, sign * magnitude, label, f"hold_{index:02d}")
                        deadline = time.monotonic() + args.hold
                        while time.monotonic() < deadline:
                            line = device.readline().decode("utf-8", "replace").strip()
                            if not line:
                                continue
                            now = datetime.now().astimezone().isoformat()
                            elapsed = time.monotonic() - started
                            frame_type, parts = parse_frame(line)
                            raw_file.write(
                                f"{now}\t{label}\t{stage}\tRX\t{line}\n"
                            )
                            row = {key: "" for key in fields}
                            row.update({
                                "host_time": now,
                                "elapsed_s": f"{elapsed:.3f}",
                                "direction": label,
                                "stage": stage,
                                "command_vx_mps": f"{command_vx:.6f}",
                                "command_az_radps": f"{command_az:.6f}",
                                "frame_type": frame_type,
                                "raw_frame": line,
                            })
                            if frame_type == "ATT" and len(parts) >= 8:
                                row.update({
                                    "mcu_time_ms": parts[1], "sequence": parts[2],
                                    "status": parts[3], "yaw_mdeg": parts[4],
                                    "gyro_z_mdps": parts[5], "bias_z_mdps": parts[6],
                                })
                            elif frame_type == "CTL" and len(parts) >= 15:
                                row.update({
                                    "mcu_time_ms": parts[1], "sequence": parts[2],
                                    "status": parts[5],
                                    "left_measured_mps": parts[11],
                                    "right_measured_mps": parts[12],
                                    "left_output": parts[13], "right_output": parts[14],
                                })
                            writer.writerow(row)
                            print(f"[{label} {magnitude:.2f}] {line}")
                        transmit(0.0, 0.0, label, "gap")
                        time.sleep(args.gap)

                transmit(0.0, 0.0, "none", "complete")
                time.sleep(0.2)
        except KeyboardInterrupt:
            print("\n收到 Ctrl-C，发送停车命令。")
        finally:
            try:
                if 'device' in locals() and device.is_open:
                    send_command(device, 0.0, 0.0)
                    time.sleep(0.1)
            except (serial.SerialException, OSError):
                pass
            raw_file.flush()
            csv_file.flush()

    print(f"完成。原始日志: {raw_path}")
    print(f"解析日志: {csv_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
