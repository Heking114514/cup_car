from __future__ import annotations

import argparse
import statistics
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import serial


@dataclass
class ImuFrame:
    tick_ms: int
    sequence: int
    state: int
    valid: bool
    stationary: bool
    address: int
    who_am_i: int
    calibration_samples: int
    calibration_required: int
    accel_x_g: float
    accel_y_g: float
    accel_z_g: float
    gyro_x_dps: float
    gyro_y_dps: float
    gyro_z_dps: float
    bias_x_dps: float
    bias_y_dps: float
    bias_z_dps: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    temperature_c: float
    io_errors: int
    recoveries: int
    yaw_rate_dps: float
    moving_bias_active: bool
    moving_bias_updates: int


def parse_imu(line: str) -> ImuFrame | None:
    fields = line.split(",")
    if (len(fields) != 25 and len(fields) < 28) or fields[0] != "IMU":
        return None
    try:
        integers = [int(value) for value in fields[1:28]]
    except ValueError:
        return None
    return ImuFrame(
        tick_ms=integers[0],
        sequence=integers[1],
        state=integers[2],
        valid=bool(integers[3]),
        stationary=bool(integers[4]),
        address=integers[5],
        who_am_i=integers[6],
        calibration_samples=integers[7],
        calibration_required=integers[8],
        accel_x_g=integers[9] / 1000.0,
        accel_y_g=integers[10] / 1000.0,
        accel_z_g=integers[11] / 1000.0,
        gyro_x_dps=integers[12] / 1000.0,
        gyro_y_dps=integers[13] / 1000.0,
        gyro_z_dps=integers[14] / 1000.0,
        bias_x_dps=integers[15] / 1000.0,
        bias_y_dps=integers[16] / 1000.0,
        bias_z_dps=integers[17] / 1000.0,
        roll_deg=integers[18] / 1000.0,
        pitch_deg=integers[19] / 1000.0,
        yaw_deg=integers[20] / 1000.0,
        temperature_c=integers[21] / 1000.0,
        io_errors=integers[22],
        recoveries=integers[23],
        yaw_rate_dps=(
            integers[24] / 1000.0 if len(integers) >= 27 else integers[14] / 1000.0
        ),
        moving_bias_active=(bool(integers[25]) if len(integers) >= 27 else False),
        moving_bias_updates=(integers[26] if len(integers) >= 27 else 0),
    )


def summarize(frames: list[ImuFrame]) -> list[str]:
    if not frames:
        return ["No valid IMU frames received."]

    calibration_indices = [index for index, frame in enumerate(frames) if frame.state == 2]
    analysis_start = calibration_indices[-1] + 1 if calibration_indices else 0
    ready = [
        frame for frame in frames[analysis_start:] if frame.state == 3 and frame.valid
    ]
    stationary = [frame for frame in ready if frame.stationary]
    last = frames[-1]
    lines = [
        f"frames={len(frames)}, ready={len(ready)}, stationary={len(stationary)}",
        f"device=0x{last.address:02X}, who_am_i=0x{last.who_am_i:02X}, "
        f"state={last.state}, io_errors={last.io_errors}, recoveries={last.recoveries}",
        f"calibration={last.calibration_samples}/{last.calibration_required}",
        f"yaw_rate_dps={last.yaw_rate_dps:.6f}, "
        f"moving_bias_active={int(last.moving_bias_active)}, "
        f"moving_bias_updates={last.moving_bias_updates}",
    ]

    if not stationary:
        return lines + ["No stationary READY interval available for drift analysis."]

    first = stationary[0]
    final = stationary[-1]
    duration_s = (final.tick_ms - first.tick_ms) / 1000.0
    yaw_delta = final.yaw_deg - first.yaw_deg
    drift_per_min = yaw_delta * 60.0 / duration_s if duration_s > 0.0 else 0.0
    gz_values = [frame.gyro_z_dps for frame in stationary]
    yaw_values = [frame.yaw_deg for frame in stationary]
    lines.extend(
        [
            f"stationary_duration_s={duration_s:.3f}",
            f"gyro_z_mean_dps={statistics.fmean(gz_values):.6f}, "
            f"gyro_z_std_dps={statistics.pstdev(gz_values):.6f}",
            f"unfrozen_yaw_drift_deg_per_min={statistics.fmean(gz_values) * 60.0:.6f}",
            f"yaw_start_deg={first.yaw_deg:.6f}, yaw_end_deg={final.yaw_deg:.6f}, "
            f"yaw_range_deg={max(yaw_values) - min(yaw_values):.6f}",
            f"yaw_drift_deg_per_min={drift_per_min:.6f}",
            f"bias_dps=({final.bias_x_dps:.6f},{final.bias_y_dps:.6f},"
            f"{final.bias_z_dps:.6f})",
            f"temperature_start_c={first.temperature_c:.3f}, "
            f"temperature_end_c={final.temperature_c:.3f}",
        ]
    )
    return lines


def main() -> int:
    parser = argparse.ArgumentParser(description="Capture and summarize MPU6050 UART3 telemetry")
    parser.add_argument("--port", default="COM6")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--seconds", type=float, default=70.0)
    parser.add_argument("--no-calibrate", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    output = args.output
    if output is None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output = Path(__file__).resolve().parent / "imu_logs" / f"imu_static_{stamp}.txt"
    output.parent.mkdir(parents=True, exist_ok=True)

    frames: list[ImuFrame] = []
    started = time.monotonic()
    next_progress = 5.0
    with serial.Serial(args.port, args.baud, timeout=0.2) as uart, output.open(
        "w", encoding="ascii", newline="\n",
        buffering=1,
    ) as log:
        uart.reset_input_buffer()
        if not args.no_calibrate:
            uart.write(b"IMU,CAL\n")
            uart.flush()
        log.write(f"# started={datetime.now().isoformat()}\n")
        log.write(f"# port={args.port}, baud={args.baud}, seconds={args.seconds}\n")
        log.write("pc_elapsed_s\tframe\n")

        while True:
            elapsed = time.monotonic() - started
            if elapsed >= args.seconds:
                break
            raw = uart.readline()
            if raw:
                line = raw.decode("ascii", errors="replace").strip()
                log.write(f"{elapsed:.6f}\t{line}\n")
                frame = parse_imu(line)
                if frame is not None:
                    frames.append(frame)
            if elapsed >= next_progress:
                state = frames[-1].state if frames else -1
                calibration = frames[-1].calibration_samples if frames else 0
                print(
                    f"{elapsed:5.1f}s frames={len(frames)} state={state} cal={calibration}",
                    flush=True,
                )
                next_progress += 5.0

        summary = summarize(frames)
        log.write("# summary\n")
        for line in summary:
            log.write(f"# {line}\n")

    print(f"log={output}")
    for line in summarize(frames):
        print(line)
    return 0 if frames else 2


if __name__ == "__main__":
    raise SystemExit(main())
