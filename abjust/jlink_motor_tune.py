#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import statistics
import struct
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path


ARM_KEY = 0x4D54554E
MAX_ABS_REF_DPS = 4500
MIN_DURATION_MS = 100
MAX_DURATION_MS = 1500
COMMAND_SYMBOL = "chassis_jlink_tune_command"
CAPTURE_SYMBOL = "chassis_jlink_tune_capture"

HEADER_FORMAT = "<6I8i8I3H4B2x"
SAMPLE_FORMAT = "<3I16h2H5B3x"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
SAMPLE_SIZE = struct.calcsize(SAMPLE_FORMAT)

HEADER_FIELDS = (
    "capture_seq",
    "request_seq",
    "start_time_ms",
    "end_time_ms",
    "requested_duration_ms",
    "control_mode",
    "left_ref_dps",
    "right_ref_dps",
    "left_kp_milli",
    "left_ki_milli",
    "right_kp_milli",
    "right_ki_milli",
    "left_current_kp_milli",
    "right_current_kp_milli",
    "left_feedback_start",
    "right_feedback_start",
    "left_feedback_end",
    "right_feedback_end",
    "can_tx_failure_start",
    "can_tx_failure_end",
    "can_hal_error_code",
    "can_protocol_last_error_code",
    "sample_count",
    "sample_capacity",
    "sample_period_ms",
    "state",
    "stop_reason",
    "can_bus_off",
    "can_error_passive",
)

SAMPLE_FIELDS = (
    "time_ms",
    "left_pid_dt_us",
    "right_pid_dt_us",
    "left_ref_dps",
    "right_ref_dps",
    "left_speed_dps",
    "right_speed_dps",
    "left_error_dps",
    "right_error_dps",
    "left_p_output",
    "right_p_output",
    "left_i_output",
    "right_i_output",
    "left_feedforward",
    "right_feedforward",
    "left_command",
    "right_command",
    "left_current",
    "right_current",
    "left_feedback_age_ms",
    "right_feedback_age_ms",
    "left_online",
    "right_online",
    "drive_status",
    "can_flags",
    "can_last_error_code",
)

STATE_NAMES = {
    0: "idle",
    1: "active",
    2: "complete",
    3: "rejected",
    4: "aborted",
}

STOP_NAMES = {
    0: "none",
    1: "duration",
    2: "bad_key",
    3: "bad_request",
    4: "feedback_lost",
    5: "can_fault",
    6: "overtemperature",
    7: "overspeed",
    8: "host_stop",
}


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(
        description="Run one bounded wheel-speed step through J-Link and save its capture."
    )
    parser.add_argument("--left-ref", type=int, default=0, help="Normalized left motor ref in deg/s.")
    parser.add_argument("--right-ref", type=int, default=0, help="Normalized right motor ref in deg/s.")
    parser.add_argument("--duration-ms", type=int, default=1200)
    parser.add_argument("--left-kp", type=float, default=0.30)
    parser.add_argument("--left-ki", type=float, default=0.08)
    parser.add_argument("--right-kp", type=float, default=0.22)
    parser.add_argument("--right-ki", type=float, default=0.06)
    parser.add_argument(
        "--control-mode",
        choices=("speed", "cascade"),
        default="speed",
        help="Use the direct speed loop or the speed-plus-current cascade.",
    )
    parser.add_argument("--left-current-kp", type=float, default=0.50)
    parser.add_argument("--right-current-kp", type=float, default=0.50)
    parser.add_argument("--device", default="STM32H723VG")
    parser.add_argument("--speed-khz", type=int, default=1000)
    parser.add_argument("--jlink", default="JLinkExe")
    parser.add_argument("--nm", default="arm-none-eabi-nm")
    parser.add_argument("--elf", type=Path, default=root / "build/Basic_Framework_MC02.elf")
    parser.add_argument("--out-dir", type=Path, default=root / "abjust/logs/jlink_motor")
    return parser.parse_args()


def gain_to_milli(value: float, name: str) -> int:
    result = int(round(value * 1000.0))
    if result < 0 or result > (1000 if name.endswith("kp") else 500):
        raise ValueError(f"{name} is outside the firmware safety range")
    return result


def validate_args(args: argparse.Namespace) -> None:
    if args.left_ref == 0 and args.right_ref == 0:
        raise ValueError("at least one wheel reference must be nonzero")
    if abs(args.left_ref) > MAX_ABS_REF_DPS or abs(args.right_ref) > MAX_ABS_REF_DPS:
        raise ValueError(f"wheel references are limited to +/-{MAX_ABS_REF_DPS} deg/s")
    if not MIN_DURATION_MS <= args.duration_ms <= MAX_DURATION_MS:
        raise ValueError(
            f"duration must be {MIN_DURATION_MS}..{MAX_DURATION_MS} ms"
        )
    for name in (
        "left_kp",
        "left_ki",
        "right_kp",
        "right_ki",
        "left_current_kp",
        "right_current_kp",
    ):
        gain_to_milli(getattr(args, name), name)
    if args.control_mode == "cascade" and (
        args.left_current_kp <= 0.0 or args.right_current_kp <= 0.0
    ):
        raise ValueError("cascade mode requires positive current-loop gains")
    if not args.elf.is_file():
        raise ValueError(f"ELF not found: {args.elf}")


def read_symbols(args: argparse.Namespace) -> dict[str, tuple[int, int]]:
    result = subprocess.run(
        [args.nm, "-S", "--defined-only", str(args.elf)],
        check=True,
        capture_output=True,
        text=True,
    )
    symbols: dict[str, tuple[int, int]] = {}
    wanted = {COMMAND_SYMBOL, CAPTURE_SYMBOL}
    for line in result.stdout.splitlines():
        parts = line.split()
        if len(parts) >= 4 and parts[3] in wanted:
            symbols[parts[3]] = (int(parts[0], 16), int(parts[1], 16))
    missing = wanted - symbols.keys()
    if missing:
        raise RuntimeError(f"missing ELF symbols: {', '.join(sorted(missing))}")
    expected_capture_size = HEADER_SIZE + 160 * SAMPLE_SIZE
    if symbols[COMMAND_SYMBOL][1] != 48:
        raise RuntimeError("unexpected J-Link command structure size")
    if symbols[CAPTURE_SYMBOL][1] != expected_capture_size:
        raise RuntimeError("unexpected J-Link capture structure size")
    return symbols


def jlink_script(args: argparse.Namespace, commands: list[str], retries: int = 3) -> str:
    common = [
        "si SWD",
        f"speed {args.speed_khz}",
        f"device {args.device}",
        "connect",
    ]
    payload = "\n".join(common + commands + ["qc", ""]) 
    last_output = ""
    for attempt in range(retries):
        result = subprocess.run(
            [args.jlink],
            input=payload,
            capture_output=True,
            text=True,
            timeout=20,
        )
        last_output = result.stdout + result.stderr
        connected = "Connecting to J-Link ...O.K." in last_output
        target_found = "Cortex-M7 identified." in last_output
        if connected and target_found and "FAILED:" not in last_output:
            return last_output
        if attempt + 1 < retries:
            time.sleep(0.5)
    raise RuntimeError("J-Link connection failed:\n" + last_output[-1200:])


def word(value: int) -> str:
    return f"0x{value & 0xFFFFFFFF:08X}"


def request_commands(
    address: int,
    request_seq: int,
    left_ref: int,
    right_ref: int,
    duration_ms: int,
    gains_milli: tuple[int, int, int, int],
    control_mode: int,
    current_gains_milli: tuple[int, int],
) -> list[str]:
    values = (
        ARM_KEY,
        left_ref,
        right_ref,
        duration_ms,
        *gains_milli,
        control_mode,
        *current_gains_milli,
        request_seq,
    )
    commands = ["h"]
    for index, value in enumerate(values[:-1]):
        commands.append(f"w4 0x{address + index * 4:08X}, {word(value)}")
    commands.append(
        f"w4 0x{address + (len(values) - 1) * 4:08X}, {word(values[-1])}"
    )
    commands.append("g")
    return commands


def send_stop(
    args: argparse.Namespace,
    command_address: int,
    request_seq: int,
    gains_milli: tuple[int, int, int, int],
    control_mode: int,
    current_gains_milli: tuple[int, int],
) -> None:
    try:
        jlink_script(
            args,
            request_commands(
                command_address,
                request_seq,
                0,
                0,
                MIN_DURATION_MS,
                gains_milli,
                control_mode,
                current_gains_milli,
            ),
            retries=1,
        )
    except Exception as exc:
        print(f"warning: explicit stop request failed ({exc})", file=sys.stderr)


def decode_capture(data: bytes) -> tuple[dict[str, int], list[dict[str, int]]]:
    if len(data) < HEADER_SIZE:
        raise RuntimeError("capture is shorter than its header")
    header = dict(zip(HEADER_FIELDS, struct.unpack_from(HEADER_FORMAT, data)))
    count = min(header["sample_count"], header["sample_capacity"])
    expected = HEADER_SIZE + count * SAMPLE_SIZE
    if len(data) < expected:
        raise RuntimeError("capture is truncated")
    samples = []
    for index in range(count):
        values = struct.unpack_from(SAMPLE_FORMAT, data, HEADER_SIZE + index * SAMPLE_SIZE)
        samples.append(dict(zip(SAMPLE_FIELDS, values)))
    return header, samples


def write_csv(path: Path, header: dict[str, int], samples: list[dict[str, int]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        stream.write("# " + ",".join(f"{key}={value}" for key, value in header.items()) + "\n")
        writer = csv.DictWriter(stream, fieldnames=SAMPLE_FIELDS)
        writer.writeheader()
        writer.writerows(samples)


def summarize_wheel(side: str, samples: list[dict[str, int]], duration_ms: int) -> str:
    speed_name = f"{side}_speed_dps"
    error_name = f"{side}_error_dps"
    command_name = f"{side}_command"
    current_name = f"{side}_current"
    steady_start = max(0, duration_ms - 300)
    steady = [sample for sample in samples if sample["time_ms"] >= steady_start]
    if not steady:
        steady = samples
    if not steady:
        return f"{side}: no samples"
    speeds = [sample[speed_name] for sample in steady]
    errors = [abs(sample[error_name]) for sample in steady]
    commands = [sample[command_name] for sample in steady]
    currents = [sample[current_name] for sample in steady]
    return (
        f"{side}: speed={statistics.mean(speeds):.1f} dps, "
        f"sigma={statistics.pstdev(speeds):.1f}, MAE={statistics.mean(errors):.1f}, "
        f"range={min(speeds)}..{max(speeds)}, "
        f"command={statistics.mean(commands):.1f}, current={statistics.mean(currents):.1f}"
    )


def main() -> int:
    args = parse_args()
    try:
        validate_args(args)
        symbols = read_symbols(args)
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    command_address, _ = symbols[COMMAND_SYMBOL]
    capture_address, capture_size = symbols[CAPTURE_SYMBOL]
    gains_milli = (
        gain_to_milli(args.left_kp, "left_kp"),
        gain_to_milli(args.left_ki, "left_ki"),
        gain_to_milli(args.right_kp, "right_kp"),
        gain_to_milli(args.right_ki, "right_ki"),
    )
    control_mode = 1 if args.control_mode == "cascade" else 0
    current_gains_milli = (
        gain_to_milli(args.left_current_kp, "left_current_kp"),
        gain_to_milli(args.right_current_kp, "right_current_kp"),
    )
    request_seq = time.time_ns() & 0x7FFFFFFF or 1
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    args.out_dir.mkdir(parents=True, exist_ok=True)
    stem = (
        f"motor_step_{timestamp}_{args.control_mode}_"
        f"L{args.left_ref}_R{args.right_ref}"
    )
    bin_path = args.out_dir / f"{stem}.bin"
    csv_path = args.out_dir / f"{stem}.csv"
    try:
        print(
            f"request seq={request_seq} L={args.left_ref} R={args.right_ref} "
            f"duration={args.duration_ms} ms mode={args.control_mode} "
            f"speed_gains={gains_milli} current_gains={current_gains_milli}"
        )
        commands = request_commands(
            command_address,
            request_seq,
            args.left_ref,
            args.right_ref,
            args.duration_ms,
            gains_milli,
            control_mode,
            current_gains_milli,
        )
        commands.extend(
            [
                f"sleep {args.duration_ms + 250}",
                "h",
                f"savebin {bin_path}, 0x{capture_address:08X}, 0x{capture_size:X}",
                "g",
            ]
        )
        jlink_script(
            args,
            commands,
        )
        header, samples = decode_capture(bin_path.read_bytes())
        write_csv(csv_path, header, samples)
    except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    state = STATE_NAMES.get(header["state"], str(header["state"]))
    reason = STOP_NAMES.get(header["stop_reason"], str(header["stop_reason"]))
    print(
        f"capture={header['capture_seq']} state={state} reason={reason} "
        f"samples={len(samples)} CAN_fail={header['can_tx_failure_end'] - header['can_tx_failure_start']} "
        f"bus_off={header['can_bus_off']} error_passive={header['can_error_passive']}"
    )
    print(summarize_wheel("left", samples, header["requested_duration_ms"]))
    print(summarize_wheel("right", samples, header["requested_duration_ms"]))
    print(f"CSV: {csv_path}")
    print(f"BIN: {bin_path}")
    return 0 if header["state"] == 2 else 1


if __name__ == "__main__":
    raise SystemExit(main())
