"""Calibrate BMI088 gyro on the host, then read a separate stationary interval."""

from __future__ import annotations

import argparse
import csv
import statistics
import time
from datetime import datetime
from pathlib import Path

import serial


GYRO_DEG_PER_S_PER_COUNT = 250.0 / 32768.0
FIELDS = ("host_elapsed_s", "mcu_time_ms", "sequence", "status", "gx", "gy", "gz",
          "temp_cC", "gz_corrected_deg_s", "yaw_corrected_deg")


def analyze(samples: list[dict[str, float]], encoder_first: tuple[int, int] | None,
            encoder_last: tuple[int, int] | None, calibration_seconds: float) -> list[str]:
    valid = [sample for sample in samples if int(sample["status"]) & 1]
    results = [f"IMU frames: {len(samples)}; valid gyro frames: {len(valid)}"]
    if encoder_first is not None and encoder_last is not None:
        results.append(f"Encoder delta: L={encoder_last[0] - encoder_first[0]}, "
                       f"R={encoder_last[1] - encoder_first[1]} counts")
    if len(valid) < 20:
        results.append("Insufficient valid samples; check BMI088 SPI and IMU status.")
        return results

    start = valid[0]["host_elapsed_s"]
    end = valid[-1]["host_elapsed_s"]
    calibration = [sample for sample in valid
                   if sample["host_elapsed_s"] - start < calibration_seconds]
    holdout = [sample for sample in valid
               if sample["host_elapsed_s"] - start >= calibration_seconds]
    if len(calibration) < 20 or len(holdout) < 20:
        results.append("Need valid gyro frames in both calibration and holdout intervals.")
        return results

    bias = statistics.mean(sample["gz"] for sample in calibration) * GYRO_DEG_PER_S_PER_COUNT
    corrected = [sample["gz_corrected_deg_s"] for sample in holdout
                 if sample["gz_corrected_deg_s"] != ""]
    yaw = [sample["yaw_corrected_deg"] for sample in holdout
           if sample["yaw_corrected_deg"] != ""]
    if len(corrected) < 20:
        results.append("No host-calibrated holdout samples were produced.")
        return results
    gaps = sum(not 0 < current["mcu_time_ms"] - previous["mcu_time_ms"] <= 75
               for previous, current in zip(valid, valid[1:]))
    temperatures = [sample["temp_cC"] / 100.0 for sample in valid
                    if int(sample["status"]) & 2]
    results.extend((
        f"Duration: {end - start:.1f} s; rate: {len(valid) / max(end - start, 1):.1f} Hz; "
        f"IMU timing gaps: {gaps}",
        f"Calibration: {len(calibration)} frames / {calibration_seconds:.1f} s, "
        f"Z bias={bias:+.5f} deg/s",
        f"Holdout: {len(holdout)} frames / {holdout[-1]['host_elapsed_s'] - holdout[0]['host_elapsed_s']:.1f} s, "
        f"corrected Z mean={statistics.mean(corrected):+.5f} deg/s, "
        f"std={statistics.stdev(corrected):.5f} deg/s",
        f"Holdout corrected yaw={yaw[-1]:+.3f} deg; peak magnitude="
        f"{max(abs(value) for value in yaw):.3f} deg",
    ))
    if temperatures:
        results.append(f"Temperature: {min(temperatures):.2f} to {max(temperatures):.2f} C")
    else:
        results.append("Temperature unavailable (temp-valid flag never set).")
    results.append("Correction is host-side only; MCU yaw control depends on the flashed firmware mode.")
    results.append("Stationary drift/noise only; absolute angle accuracy needs a known-angle test.")
    return results


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--seconds", type=float, default=120.0)
    parser.add_argument("--calibration-seconds", type=float, default=60.0)
    args = parser.parse_args()
    if args.calibration_seconds <= 0 or args.seconds <= args.calibration_seconds:
        parser.error("--seconds must be greater than --calibration-seconds > 0")

    output_dir = Path(__file__).resolve().parent / "imu_test_logs"
    output_dir.mkdir(exist_ok=True)
    output = output_dir / f"bmi088_stationary_{datetime.now():%Y%m%d_%H%M%S}.csv"
    samples: list[dict[str, float]] = []
    encoder_first = None
    encoder_last = None
    first_valid_s = None
    calibration_rates: list[float] = []
    bias_deg_s = None
    previous_corrected = None
    yaw_deg = 0.0
    last_report_s = 0.0
    began = time.monotonic()
    with serial.Serial(args.port, 115200, timeout=0.3) as port, output.open(
        "w", encoding="utf-8", newline=""
    ) as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=FIELDS)
        writer.writeheader()
        while time.monotonic() - began < args.seconds:
            line = port.readline().decode("ascii", "replace").strip()
            fields = line.split(",")
            if len(fields) == 5 and fields[0] == "ENC":
                try:
                    counts = (int(fields[3]), int(fields[4]))
                except ValueError:
                    continue
                if encoder_first is None:
                    encoder_first = counts
                encoder_last = counts
            if len(fields) != 8 or fields[0] != "IMU":
                continue
            try:
                numbers = [int(field) for field in fields[1:]]
            except ValueError:
                continue
            record = {"host_elapsed_s": round(time.monotonic() - began, 4),
                      **dict(zip(FIELDS[1:8], numbers)),
                      "gz_corrected_deg_s": "", "yaw_corrected_deg": ""}
            if record["status"] & 1:
                if first_valid_s is None:
                    first_valid_s = record["host_elapsed_s"]
                rate = record["gz"] * GYRO_DEG_PER_S_PER_COUNT
                if record["host_elapsed_s"] - first_valid_s < args.calibration_seconds:
                    calibration_rates.append(rate)
                else:
                    if bias_deg_s is None and len(calibration_rates) >= 20:
                        bias_deg_s = statistics.mean(calibration_rates)
                        print(f"Host calibration finished: Z bias={bias_deg_s:+.5f} deg/s "
                              f"from {len(calibration_rates)} samples", flush=True)
                    if bias_deg_s is not None:
                        corrected_rate = rate - bias_deg_s
                        if previous_corrected is not None:
                            dt = (record["mcu_time_ms"] - previous_corrected[0]) / 1000.0
                            if 0 < dt < 0.5:
                                yaw_deg += (previous_corrected[1] + corrected_rate) * 0.5 * dt
                        previous_corrected = (record["mcu_time_ms"], corrected_rate)
                        record["gz_corrected_deg_s"] = round(corrected_rate, 6)
                        record["yaw_corrected_deg"] = round(yaw_deg, 6)
                        if record["host_elapsed_s"] - last_report_s >= 10.0:
                            print(f"t={record['host_elapsed_s']:.1f}s corrected Z="
                                  f"{corrected_rate:+.3f} deg/s, yaw={yaw_deg:+.3f} deg",
                                  flush=True)
                            last_report_s = record["host_elapsed_s"]
            samples.append(record)
            writer.writerow(record)
        csv_file.flush()

    results = analyze(samples, encoder_first, encoder_last, args.calibration_seconds)
    summary = output.with_suffix(".txt")
    summary.write_text("\n".join(results) + "\n", encoding="utf-8")
    print(f"Saved {output} and {summary}")
    print("\n".join(results))


if __name__ == "__main__":
    main()
