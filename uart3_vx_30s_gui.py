from __future__ import annotations

import csv
import math
import queue
import statistics
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from tkinter import messagebox, scrolledtext, ttk
from typing import Any, TextIO

import serial
from serial.tools import list_ports


BAUD_RATE = 115200
DEFAULT_VX_MPS = 0.6
DEFAULT_AZ_RADPS = 0.0
DEFAULT_DURATION_S = 60.0
DEFAULT_ROUTE_ENABLED = False
DEFAULT_TURN_ANGLE_DEG = 90.0
DEFAULT_TURN_TIMEOUT_S = 6.0
DEFAULT_SECOND_STRAIGHT_S = 6.0
COMMAND_PERIOD_S = 0.1
PREFLIGHT_TIMEOUT_S = 2.0
STOP_REPEAT_COUNT = 5
STOP_REPEAT_PERIOD_S = 0.04
FINAL_DRAIN_S = 0.5
INTERPHASE_STOP_S = 0.6
TURN_CONTROL_KP = 1.5
TURN_MAX_AZ_RADPS = 0.40
TURN_MIN_AZ_RADPS = 0.10
TURN_TOLERANCE_DEG = 1.0
TURN_SETTLED_GYRO_DPS = 3.0
TURN_SETTLED_TIME_S = 0.25

WHEEL_DIAMETER_M = 0.0626
TRACK_WIDTH_M = 0.1408
LEFT_COUNTS_PER_REV = 1060.1667
RIGHT_COUNTS_PER_REV = 1060.9333
HEADING_LIMIT_RADPS = 0.45
HEADING_KP = 3.00
HEADING_KI = 0.60
HEADING_KD = 0.10

STOP_COMMAND = b"0.000,0.000\r\n"

FRAME_SCHEMAS = (
    "ENC,time_ms,sequence,left_total_count,right_total_count",
    "CTL,time_ms,sequence,mode,emergency,command_valid,command_age_ms,"
    "command_vx_mmps,command_az_mradps,target_left_mmps,target_right_mmps,"
    "measured_left_mmps,measured_right_mmps,left_pwm,right_pwm",
    "IMU,time_ms,sequence,state,sample_valid,stationary,address,who_am_i,"
    "calibration_samples,calibration_required,accel_x_mg,accel_y_mg,"
    "accel_z_mg,gyro_x_mdps,gyro_y_mdps,gyro_z_mdps,bias_x_mdps,"
    "bias_y_mdps,bias_z_mdps,roll_mdeg,pitch_mdeg,yaw_mdeg,"
    "temperature_mdeg_c,io_errors,recoveries,yaw_rate_mdps,"
    "moving_bias_active,moving_bias_updates",
    "HDG,time_ms,active,imu_valid,target_mdeg,yaw_mdeg,error_mdeg,"
    "correction_mradps,requested_az_mradps,controlled_az_mradps",
)

CSV_FIELDS = (
    "pc_elapsed_s", "pc_time", "phase", "phase_elapsed_s", "run_elapsed_s",
    "phase_imu_yaw_delta_deg", "phase_encoder_yaw_delta_deg", "mcu_time_ms",
    "hdg_time_ms", "hdg_active", "hdg_imu_valid", "target_yaw_deg",
    "imu_yaw_deg", "imu_yaw_unwrapped_deg", "imu_yaw_relative_deg",
    "heading_error_deg", "heading_error_abs_deg", "heading_correction_radps",
    "heading_output_saturated", "requested_az_radps", "controlled_az_radps",
    "imu_time_ms", "imu_age_ms", "imu_sequence", "imu_state",
    "imu_sample_valid", "imu_stationary", "imu_address_7bit", "imu_who_am_i",
    "imu_calibration_samples", "imu_calibration_required", "accel_x_g",
    "accel_y_g", "accel_z_g", "accel_norm_g", "gyro_x_dps", "gyro_y_dps",
    "gyro_z_dps", "yaw_rate_dps", "gyro_raw_x_dps", "gyro_raw_y_dps", "gyro_raw_z_dps",
    "gyro_bias_x_dps", "gyro_bias_y_dps", "gyro_bias_z_dps", "roll_deg",
    "pitch_deg", "temperature_c", "imu_io_errors", "imu_recoveries",
    "moving_bias_active", "moving_bias_updates",
    "enc_time_ms", "enc_age_ms", "enc_sequence", "left_total_count",
    "right_total_count", "left_run_count", "right_run_count",
    "left_run_distance_m", "right_run_distance_m", "encoder_yaw_rad",
    "encoder_yaw_deg", "encoder_yaw_rate_radps", "encoder_yaw_rate_dps",
    "imu_minus_encoder_yaw_deg", "ctl_time_ms", "ctl_age_ms", "ctl_sequence",
    "mode", "emergency_stop", "command_valid", "command_age_ms",
    "command_vx_mps", "command_az_radps", "target_left_mps",
    "target_right_mps", "measured_left_mps", "measured_right_mps",
    "left_speed_error_mps", "right_speed_error_mps", "measured_linear_speed_mps",
    "wheel_speed_yaw_rate_radps", "left_pwm", "right_pwm",
    "pwm_difference_right_minus_left",
)


@dataclass(frozen=True)
class RunConfig:
    vx_mps: float
    az_radps: float
    duration_s: float
    route_enabled: bool = DEFAULT_ROUTE_ENABLED
    turn_angle_deg: float = DEFAULT_TURN_ANGLE_DEG
    turn_timeout_s: float = DEFAULT_TURN_TIMEOUT_S
    second_straight_s: float = DEFAULT_SECOND_STRAIGHT_S

    @property
    def command(self) -> bytes:
        return f"{self.vx_mps:.3f},{self.az_radps:.3f}\r\n".encode("ascii")

    @property
    def planned_duration_s(self) -> float:
        if not self.route_enabled:
            return self.duration_s
        return (
            self.duration_s
            + self.turn_timeout_s
            + self.second_straight_s
            + 2.0 * INTERPHASE_STOP_S
        )


@dataclass(frozen=True)
class LogPaths:
    raw: Path
    samples: Path
    summary: Path

    def display(self) -> str:
        return f"Raw: {self.raw} | CSV: {self.samples} | Summary: {self.summary}"


def available_ports() -> list[tuple[str, str]]:
    return sorted(
        ((port.device, port.description or "Serial port") for port in list_ports.comports()),
        key=lambda item: item[0],
    )


def filename_number(value: float) -> str:
    return f"{value:g}".replace("-", "m").replace(".", "p")


def make_log_paths(config: RunConfig) -> LogPaths:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    directory = Path(__file__).resolve().parent / "uart3_heading_logs"
    directory.mkdir(parents=True, exist_ok=True)
    stem = (
        f"heading_vx{filename_number(config.vx_mps)}_"
        f"az{filename_number(config.az_radps)}_"
        f"kp{filename_number(HEADING_KP)}_ki{filename_number(HEADING_KI)}_"
        f"kd{filename_number(HEADING_KD)}_"
        f"{filename_number(config.duration_s)}s"
        + (
            f"_turn{filename_number(config.turn_angle_deg)}_"
            f"s2_{filename_number(config.second_straight_s)}s"
            if config.route_enabled else "_straight_only"
        )
        + f"_{timestamp}"
    )
    return LogPaths(
        directory / f"{stem}_raw.txt",
        directory / f"{stem}_samples.csv",
        directory / f"{stem}_summary.txt",
    )


def parse_ctl_state(line: str) -> tuple[int, bool] | None:
    fields = [item.strip() for item in line.split(",")]
    if len(fields) < 6 or fields[0] != "CTL":
        return None
    try:
        return int(fields[3]), bool(int(fields[4]))
    except ValueError:
        return None


def mcu_age(reference: Any, sample: Any) -> int | str:
    if not isinstance(reference, int) or not isinstance(sample, int):
        return ""
    age = (reference - sample) & 0xFFFFFFFF
    return age if age <= 0x7FFFFFFF else ""


def turn_rate_command(error_deg: float) -> float:
    if abs(error_deg) <= TURN_TOLERANCE_DEG:
        return 0.0
    az_radps = TURN_CONTROL_KP * math.radians(error_deg)
    az_radps = max(-TURN_MAX_AZ_RADPS, min(TURN_MAX_AZ_RADPS, az_radps))
    if abs(az_radps) < TURN_MIN_AZ_RADPS:
        az_radps = math.copysign(TURN_MIN_AZ_RADPS, az_radps)
    return az_radps


class TelemetryRecorder:
    """Parse all MCU frames and write one synchronized row for every HDG frame."""

    def __init__(self, path: Path, config: RunConfig) -> None:
        self.config = config
        self.file = path.open("w", encoding="utf-8", newline="")
        self.writer = csv.DictWriter(self.file, fieldnames=CSV_FIELDS)
        self.writer.writeheader()
        self.latest: dict[str, Any] = {}
        self.phase = "preflight"
        self.motion_start_pc_s: float | None = None
        self.phase_start_pc_s: float | None = None
        self.phase_start_imu_yaw_deg: float | None = None
        self.phase_start_encoder_yaw_deg: float | None = None
        self.encoder_start: tuple[int, int] | None = None
        self.imu_start_unwrapped_deg: float | None = None
        self.previous_imu_yaw_deg: float | None = None
        self.imu_unwrapped_deg: float | None = None
        self.previous_encoder: tuple[int, float] | None = None
        self.frame_counts = {name: 0 for name in ("ENC", "CTL", "IMU", "HDG")}
        self.parse_errors = 0
        self.unknown_frames = 0
        self.rows_written = 0
        self.last_live_emit_s = -1.0
        self.motion_samples: list[dict[str, Any]] = []

    def mark_motion_start(self, pc_elapsed_s: float) -> None:
        self.motion_start_pc_s = pc_elapsed_s
        left = self.latest.get("left_total_count")
        right = self.latest.get("right_total_count")
        if isinstance(left, int) and isinstance(right, int):
            self.encoder_start = (left, right)
        if self.imu_unwrapped_deg is not None:
            self.imu_start_unwrapped_deg = self.imu_unwrapped_deg
        self.set_phase("straight_1", pc_elapsed_s)

    def set_phase(self, phase: str, pc_elapsed_s: float) -> None:
        self.phase = phase
        self.phase_start_pc_s = pc_elapsed_s
        self.phase_start_imu_yaw_deg = self.imu_unwrapped_deg
        encoder_yaw = self.latest.get("encoder_yaw_deg")
        self.phase_start_encoder_yaw_deg = (
            encoder_yaw if isinstance(encoder_yaw, float) else None
        )

    def mark_postflight(self) -> None:
        self.phase = "postflight"

    def parse_encoder(self, fields: list[str]) -> None:
        if len(fields) < 5:
            raise ValueError("short ENC frame")
        time_ms, sequence, left_total, right_total = map(int, fields[1:5])
        circumference = math.pi * WHEEL_DIAMETER_M
        left_absolute_m = left_total * circumference / LEFT_COUNTS_PER_REV
        right_absolute_m = right_total * circumference / RIGHT_COUNTS_PER_REV
        absolute_yaw_rad = (right_absolute_m - left_absolute_m) / TRACK_WIDTH_M
        yaw_rate_radps: float | str = ""
        if self.previous_encoder is not None:
            old_time_ms, old_yaw_rad = self.previous_encoder
            elapsed_ms = (time_ms - old_time_ms) & 0xFFFFFFFF
            if 0 < elapsed_ms <= 1000:
                yaw_rate_radps = (absolute_yaw_rad - old_yaw_rad) / (elapsed_ms * 0.001)
        self.previous_encoder = (time_ms, absolute_yaw_rad)

        if self.motion_start_pc_s is not None and self.encoder_start is None:
            self.encoder_start = (left_total, right_total)
        left_run: int | str = ""
        right_run: int | str = ""
        left_distance: float | str = ""
        right_distance: float | str = ""
        encoder_yaw_rad: float | str = ""
        if self.encoder_start is not None:
            left_run = left_total - self.encoder_start[0]
            right_run = right_total - self.encoder_start[1]
            left_distance = left_run * circumference / LEFT_COUNTS_PER_REV
            right_distance = right_run * circumference / RIGHT_COUNTS_PER_REV
            encoder_yaw_rad = (right_distance - left_distance) / TRACK_WIDTH_M

        self.latest.update(
            enc_time_ms=time_ms,
            enc_sequence=sequence,
            left_total_count=left_total,
            right_total_count=right_total,
            left_run_count=left_run,
            right_run_count=right_run,
            left_run_distance_m=left_distance,
            right_run_distance_m=right_distance,
            encoder_yaw_rad=encoder_yaw_rad,
            encoder_yaw_deg=(
                math.degrees(encoder_yaw_rad) if isinstance(encoder_yaw_rad, float) else ""
            ),
            encoder_yaw_rate_radps=yaw_rate_radps,
            encoder_yaw_rate_dps=(
                math.degrees(yaw_rate_radps) if isinstance(yaw_rate_radps, float) else ""
            ),
        )

    def parse_control(self, fields: list[str]) -> None:
        if len(fields) < 15:
            raise ValueError("short CTL frame")
        value = list(map(int, fields[1:15]))
        self.latest.update(
            ctl_time_ms=value[0], ctl_sequence=value[1], mode=value[2],
            emergency_stop=value[3], command_valid=value[4], command_age_ms=value[5],
            command_vx_mps=value[6] / 1000.0, command_az_radps=value[7] / 1000.0,
            target_left_mps=value[8] / 1000.0, target_right_mps=value[9] / 1000.0,
            measured_left_mps=value[10] / 1000.0,
            measured_right_mps=value[11] / 1000.0,
            left_pwm=value[12], right_pwm=value[13],
        )

    def parse_imu(self, fields: list[str]) -> None:
        if len(fields) < 25:
            raise ValueError("short IMU frame")
        value = list(map(int, fields[1:25]))
        yaw_deg = value[20] / 1000.0
        if self.previous_imu_yaw_deg is None:
            self.imu_unwrapped_deg = yaw_deg
        else:
            delta = (yaw_deg - self.previous_imu_yaw_deg + 180.0) % 360.0 - 180.0
            self.imu_unwrapped_deg = (self.imu_unwrapped_deg or 0.0) + delta
        self.previous_imu_yaw_deg = yaw_deg
        if self.motion_start_pc_s is not None and self.imu_start_unwrapped_deg is None:
            self.imu_start_unwrapped_deg = self.imu_unwrapped_deg

        ax, ay, az = (value[index] / 1000.0 for index in (9, 10, 11))
        gx, gy, gz = (value[index] / 1000.0 for index in (12, 13, 14))
        bx, by, bz = (value[index] / 1000.0 for index in (15, 16, 17))
        yaw_rate_dps = gz
        moving_bias_active: int | str = ""
        moving_bias_updates: int | str = ""
        if len(fields) >= 28:
            yaw_rate_dps = int(fields[25]) / 1000.0
            moving_bias_active = int(fields[26])
            moving_bias_updates = int(fields[27])
        relative_yaw: float | str = ""
        if self.imu_unwrapped_deg is not None and self.imu_start_unwrapped_deg is not None:
            relative_yaw = self.imu_unwrapped_deg - self.imu_start_unwrapped_deg
        self.latest.update(
            imu_time_ms=value[0], imu_sequence=value[1], imu_state=value[2],
            imu_sample_valid=value[3], imu_stationary=value[4],
            imu_address_7bit=value[5], imu_who_am_i=value[6],
            imu_calibration_samples=value[7], imu_calibration_required=value[8],
            accel_x_g=ax, accel_y_g=ay, accel_z_g=az,
            accel_norm_g=math.sqrt(ax * ax + ay * ay + az * az),
            gyro_x_dps=gx, gyro_y_dps=gy, gyro_z_dps=gz,
            yaw_rate_dps=yaw_rate_dps,
            gyro_raw_x_dps=gx + bx, gyro_raw_y_dps=gy + by, gyro_raw_z_dps=gz + bz,
            gyro_bias_x_dps=bx, gyro_bias_y_dps=by, gyro_bias_z_dps=bz,
            roll_deg=value[18] / 1000.0, pitch_deg=value[19] / 1000.0,
            imu_yaw_deg=yaw_deg, imu_yaw_unwrapped_deg=self.imu_unwrapped_deg,
            imu_yaw_relative_deg=relative_yaw, temperature_c=value[21] / 1000.0,
            imu_io_errors=value[22], imu_recoveries=value[23],
            moving_bias_active=moving_bias_active,
            moving_bias_updates=moving_bias_updates,
        )

    def parse_heading(self, fields: list[str]) -> None:
        if len(fields) < 10:
            raise ValueError("short HDG frame")
        value = list(map(int, fields[1:10]))
        self.latest.update(
            hdg_time_ms=value[0], hdg_active=value[1], hdg_imu_valid=value[2],
            target_yaw_deg=value[3] / 1000.0, imu_yaw_deg=value[4] / 1000.0,
            heading_error_deg=value[5] / 1000.0,
            heading_correction_radps=value[6] / 1000.0,
            requested_az_radps=value[7] / 1000.0,
            controlled_az_radps=value[8] / 1000.0,
        )

    def build_row(self, pc_elapsed_s: float, pc_time: str) -> dict[str, Any]:
        row = {field: "" for field in CSV_FIELDS}
        row.update(self.latest)
        reference = row.get("hdg_time_ms")
        row.update(
            pc_elapsed_s=round(pc_elapsed_s, 6), pc_time=pc_time, phase=self.phase,
            phase_elapsed_s=(
                round(pc_elapsed_s - self.phase_start_pc_s, 6)
                if self.phase_start_pc_s is not None else ""
            ),
            run_elapsed_s=(
                round(pc_elapsed_s - self.motion_start_pc_s, 6)
                if self.motion_start_pc_s is not None else ""
            ),
            mcu_time_ms=reference if isinstance(reference, int) else "",
            imu_age_ms=mcu_age(reference, row.get("imu_time_ms")),
            enc_age_ms=mcu_age(reference, row.get("enc_time_ms")),
            ctl_age_ms=mcu_age(reference, row.get("ctl_time_ms")),
        )
        unwrapped_yaw = row.get("imu_yaw_unwrapped_deg")
        if isinstance(unwrapped_yaw, float) and self.phase_start_imu_yaw_deg is not None:
            row["phase_imu_yaw_delta_deg"] = (
                unwrapped_yaw - self.phase_start_imu_yaw_deg
            )
        phase_encoder_yaw = row.get("encoder_yaw_deg")
        if (
            isinstance(phase_encoder_yaw, float)
            and self.phase_start_encoder_yaw_deg is not None
        ):
            row["phase_encoder_yaw_delta_deg"] = (
                phase_encoder_yaw - self.phase_start_encoder_yaw_deg
            )
        error = row.get("heading_error_deg")
        correction = row.get("heading_correction_radps")
        if isinstance(error, float):
            row["heading_error_abs_deg"] = abs(error)
        if isinstance(correction, float):
            row["heading_output_saturated"] = int(
                abs(correction) >= HEADING_LIMIT_RADPS - 0.005
            )
        imu_yaw = row.get("imu_yaw_relative_deg")
        encoder_yaw = row.get("encoder_yaw_deg")
        if isinstance(imu_yaw, float) and isinstance(encoder_yaw, float):
            row["imu_minus_encoder_yaw_deg"] = imu_yaw - encoder_yaw
        tl, tr = row.get("target_left_mps"), row.get("target_right_mps")
        ml, mr = row.get("measured_left_mps"), row.get("measured_right_mps")
        if isinstance(tl, float) and isinstance(ml, float):
            row["left_speed_error_mps"] = tl - ml
        if isinstance(tr, float) and isinstance(mr, float):
            row["right_speed_error_mps"] = tr - mr
        if isinstance(ml, float) and isinstance(mr, float):
            row["measured_linear_speed_mps"] = (ml + mr) * 0.5
            row["wheel_speed_yaw_rate_radps"] = (mr - ml) / TRACK_WIDTH_M
        left_pwm, right_pwm = row.get("left_pwm"), row.get("right_pwm")
        if isinstance(left_pwm, int) and isinstance(right_pwm, int):
            row["pwm_difference_right_minus_left"] = right_pwm - left_pwm
        return row

    def handle_frame(
        self, line: str, pc_elapsed_s: float, pc_time: str
    ) -> dict[str, str] | None:
        fields = [item.strip() for item in line.split(",")]
        parser = {
            "ENC": self.parse_encoder, "CTL": self.parse_control,
            "IMU": self.parse_imu, "HDG": self.parse_heading,
        }.get(fields[0] if fields else "")
        if parser is None:
            self.unknown_frames += 1
            return None
        self.frame_counts[fields[0]] += 1
        try:
            parser(fields)
        except (ValueError, IndexError, OverflowError):
            self.parse_errors += 1
            return None

        if fields[0] == "HDG":
            row = self.build_row(pc_elapsed_s, pc_time)
            self.writer.writerow(row)
            self.file.flush()
            self.rows_written += 1
            if self.phase not in ("preflight", "postflight"):
                sample = {
                    name: float(row[name])
                    for name in (
                        "run_elapsed_s", "imu_yaw_relative_deg", "encoder_yaw_deg",
                        "imu_minus_encoder_yaw_deg", "heading_error_deg",
                        "heading_correction_radps", "gyro_z_dps", "yaw_rate_dps",
                        "gyro_bias_z_dps", "moving_bias_active", "moving_bias_updates",
                        "encoder_yaw_rate_dps", "measured_left_mps",
                        "measured_right_mps", "left_speed_error_mps",
                        "right_speed_error_mps", "left_pwm", "right_pwm",
                        "imu_yaw_unwrapped_deg", "phase_imu_yaw_delta_deg",
                        "phase_encoder_yaw_delta_deg",
                    )
                    if isinstance(row.get(name), (int, float))
                }
                sample["phase"] = self.phase
                self.motion_samples.append(sample)

        if pc_elapsed_s - self.last_live_emit_s >= 0.05:
            self.last_live_emit_s = pc_elapsed_s
            return self.live_values()
        return None

    def live_values(self) -> dict[str, str]:
        row = self.build_row(0.0, "")

        def fmt(name: str, digits: int = 3, suffix: str = "") -> str:
            value = row.get(name)
            return f"{value:.{digits}f}{suffix}" if isinstance(value, (int, float)) else "--"

        return {
            "phase": self.phase,
            "yaw": fmt("imu_yaw_relative_deg", 3, " deg rel"),
            "target": fmt("target_yaw_deg", 3, " deg"),
            "error": fmt("heading_error_deg", 3, " deg"),
            "correction": fmt("heading_correction_radps", 3, " rad/s"),
            "gyro_z": f"{fmt('gyro_z_dps')} body / {fmt('yaw_rate_dps')} yaw deg/s",
            "encoder_yaw": fmt("encoder_yaw_deg", 3, " deg"),
            "yaw_difference": fmt("imu_minus_encoder_yaw_deg", 3, " deg"),
            "wheel_measured": f"{fmt('measured_left_mps')} / {fmt('measured_right_mps')} m/s",
            "wheel_target": f"{fmt('target_left_mps')} / {fmt('target_right_mps')} m/s",
            "pwm": f"{fmt('left_pwm', 0)} / {fmt('right_pwm', 0)}",
            "imu": (
                f"state={row.get('imu_state', '--')} valid={row.get('imu_sample_valid', '--')} "
                f"still={row.get('imu_stationary', '--')} "
                f"mb={row.get('moving_bias_active', '--')}/"
                f"{row.get('moving_bias_updates', '--')} io={row.get('imu_io_errors', '--')}"
            ),
        }

    def metric(self, name: str, phase_prefix: str | None = "straight") -> list[float]:
        return [
            sample[name]
            for sample in self.motion_samples
            if name in sample
            and (
                phase_prefix is None
                or str(sample.get("phase", "")).startswith(phase_prefix)
            )
        ]

    def finish(self, summary_path: Path, result: str, success: bool) -> None:
        self.file.flush()
        self.file.close()
        with summary_path.open("w", encoding="utf-8", newline="\n") as output:
            output.write("UART3 heading-control run summary\n")
            output.write(f"Result: {result}\nCompleted normally: {int(success)}\n")
            output.write(
                f"Command: vx={self.config.vx_mps:.6f} m/s, "
                f"az={self.config.az_radps:.6f} rad/s, "
                f"straight_1={self.config.duration_s:.3f} s\n"
            )
            output.write(
                f"Route: enabled={int(self.config.route_enabled)}, "
                f"turn={self.config.turn_angle_deg:.3f} deg, "
                f"turn_timeout={self.config.turn_timeout_s:.3f} s, "
                f"straight_2={self.config.second_straight_s:.3f} s\n"
            )
            output.write(
                f"Heading controller: kp={HEADING_KP:.6f}, ki={HEADING_KI:.6f}, "
                f"kd={HEADING_KD:.6f}, output_limit={HEADING_LIMIT_RADPS:.6f} rad/s\n"
            )
            output.write(
                f"Geometry: wheel_diameter={WHEEL_DIAMETER_M:.6f} m, "
                f"track_width={TRACK_WIDTH_M:.6f} m, left_cpr={LEFT_COUNTS_PER_REV:.4f}, "
                f"right_cpr={RIGHT_COUNTS_PER_REV:.4f}\n"
            )
            output.write(f"Structured rows: {self.rows_written}\n")
            output.write(
                "Frames: " + ", ".join(
                    f"{name}={count}" for name, count in self.frame_counts.items()
                ) + f", parse_errors={self.parse_errors}, unknown={self.unknown_frames}\n"
            )
            output.write(
                "Caution: encoder yaw is wheel odometry, not an absolute reference; "
                "wheel slip and wheel-radius mismatch remain error sources.\n\n"
            )
            metrics = (
                ("IMU relative yaw", "imu_yaw_relative_deg", "deg"),
                ("Encoder relative yaw", "encoder_yaw_deg", "deg"),
                ("IMU minus encoder yaw", "imu_minus_encoder_yaw_deg", "deg"),
                ("Heading error", "heading_error_deg", "deg"),
                ("Heading correction", "heading_correction_radps", "rad/s"),
                ("Gyro Z", "gyro_z_dps", "deg/s"),
                ("Ground yaw rate", "yaw_rate_dps", "deg/s"),
                ("Gyro Z bias", "gyro_bias_z_dps", "deg/s"),
                ("Encoder yaw rate", "encoder_yaw_rate_dps", "deg/s"),
                ("Measured left speed", "measured_left_mps", "m/s"),
                ("Measured right speed", "measured_right_mps", "m/s"),
                ("Left speed error", "left_speed_error_mps", "m/s"),
                ("Right speed error", "right_speed_error_mps", "m/s"),
                ("Left PWM", "left_pwm", "count"),
                ("Right PWM", "right_pwm", "count"),
            )
            for label, field, unit in metrics:
                values = self.metric(field)
                if not values:
                    output.write(f"{label}: unavailable\n")
                    continue
                std = statistics.pstdev(values) if len(values) > 1 else 0.0
                output.write(
                    f"{label}: mean={statistics.fmean(values):.6f} {unit}, "
                    f"std={std:.6f}, min={min(values):.6f}, "
                    f"max={max(values):.6f}, final={values[-1]:.6f}\n"
                )
            corrections = self.metric("heading_correction_radps")
            if corrections:
                saturated = sum(abs(value) >= HEADING_LIMIT_RADPS - 0.005 for value in corrections)
                output.write(
                    f"Heading saturation: {saturated}/{len(corrections)} "
                    f"({100.0 * saturated / len(corrections):.3f}%)\n"
                )
            turn_yaw = self.metric("phase_imu_yaw_delta_deg", "turn")
            turn_encoder = self.metric("phase_encoder_yaw_delta_deg", "turn")
            turn_time = self.metric("run_elapsed_s", "turn")
            if turn_yaw:
                output.write(
                    f"Turn IMU delta: final={turn_yaw[-1]:.6f} deg, "
                    f"min={min(turn_yaw):.6f}, max={max(turn_yaw):.6f}\n"
                )
            if turn_encoder:
                output.write(f"Turn encoder delta: final={turn_encoder[-1]:.6f} deg\n")
            if len(turn_time) >= 2:
                output.write(f"Turn sampled duration: {turn_time[-1] - turn_time[0]:.6f} s\n")


class SerialRunWorker(threading.Thread):
    def __init__(
        self,
        port: str,
        config: RunConfig,
        events: queue.Queue[tuple[str, object]],
        stop_event: threading.Event,
    ) -> None:
        super().__init__(daemon=True)
        self.port = port
        self.config = config
        self.events = events
        self.stop_event = stop_event
        self.session_started = 0.0
        self.sequence_started = 0.0
        self.raw_file: TextIO | None = None
        self.recorder: TelemetryRecorder | None = None

    def emit(self, kind: str, value: object) -> None:
        self.events.put((kind, value))

    def record(self, direction: str, frame: str, show: bool = True) -> tuple[float, str]:
        elapsed = time.monotonic() - self.session_started
        wall_time = datetime.now().astimezone().isoformat(timespec="milliseconds")
        clean = frame.replace("\t", "\\t").replace("\r", "\\r").replace("\n", "\\n")
        if self.raw_file is not None and not self.raw_file.closed:
            self.raw_file.write(f"{elapsed:.6f}\t{wall_time}\t{direction}\t{clean}\n")
            self.raw_file.flush()
        if show:
            self.emit("line", f"{elapsed:8.3f}  {direction:<10}  {clean}")
        return elapsed, wall_time

    def send(self, device: serial.Serial, data: bytes, show: bool = True) -> None:
        device.write(data)
        device.flush()
        self.record("TX", data.decode("ascii").rstrip("\r\n"), show)

    def read_lines(self, device: serial.Serial, buffer: bytearray) -> list[str]:
        buffer.extend(device.read(max(1, min(device.in_waiting, 4096))))
        lines: list[str] = []
        while True:
            newline = buffer.find(b"\n")
            if newline < 0:
                if len(buffer) > 8192:
                    self.record("RX_PARTIAL", bytes(buffer).decode("ascii", errors="replace"))
                    buffer.clear()
                break
            raw = bytes(buffer[:newline]).rstrip(b"\r")
            del buffer[: newline + 1]
            line = raw.decode("ascii", errors="replace")
            if not line:
                continue
            elapsed, wall_time = self.record("RX", line)
            if self.recorder is not None:
                live = self.recorder.handle_frame(line, elapsed, wall_time)
                if live is not None:
                    self.emit("telemetry", live)
            lines.append(line)
        return lines

    @staticmethod
    def check_ctl(lines: list[str]) -> int | None:
        mode: int | None = None
        for line in lines:
            state = parse_ctl_state(line)
            if state is None:
                continue
            mode, emergency = state
            if emergency:
                raise RuntimeError("MCU emergency stop is active")
        return mode

    def preflight(self, device: serial.Serial, buffer: bytearray) -> None:
        self.emit("status", "Checking UART3 telemetry and MCU mode...")
        deadline = time.monotonic() + PREFLIGHT_TIMEOUT_S
        next_stop = 0.0
        while time.monotonic() < deadline:
            if self.stop_event.is_set():
                raise InterruptedError("Stopped before the run started")
            now = time.monotonic()
            if now >= next_stop:
                self.send(device, STOP_COMMAND, show=False)
                next_stop = now + COMMAND_PERIOD_S
            mode = self.check_ctl(self.read_lines(device, buffer))
            if mode == 1:
                self.emit("status", "Navigation mode confirmed. Running...")
                return
            if mode == 0:
                raise RuntimeError(
                    "MCU is in remote mode. Press SELECT on the controller, then start again."
                )
        raise RuntimeError(
            "No valid CTL telemetry from UART3. Check the COM port, baud rate, and firmware."
        )

    def send_stop(self, device: serial.Serial) -> None:
        for _ in range(STOP_REPEAT_COUNT):
            try:
                self.send(device, STOP_COMMAND)
            except (serial.SerialException, OSError):
                return
            time.sleep(STOP_REPEAT_PERIOD_S)

    def drain(self, device: serial.Serial, buffer: bytearray) -> None:
        deadline = time.monotonic() + FINAL_DRAIN_S
        while time.monotonic() < deadline:
            try:
                self.read_lines(device, buffer)
            except (serial.SerialException, OSError):
                return
        if buffer:
            self.record("RX_PARTIAL", bytes(buffer).decode("ascii", errors="replace"))
            buffer.clear()

    def update_progress(self) -> None:
        elapsed = time.monotonic() - self.sequence_started
        self.emit("progress", min(elapsed, self.config.planned_duration_s))

    def set_phase(self, phase: str) -> None:
        elapsed, _ = self.record("EVENT", f"Phase started: {phase}")
        if self.recorder is not None:
            self.recorder.set_phase(phase, elapsed)
        self.emit("status", f"Running phase: {phase}")

    def run_timed_phase(
        self,
        device: serial.Serial,
        buffer: bytearray,
        phase: str,
        command: bytes,
        duration_s: float,
        phase_already_set: bool = False,
    ) -> None:
        if not phase_already_set:
            self.set_phase(phase)
        started = time.monotonic()
        next_command = started
        next_progress = started
        while time.monotonic() - started < duration_s:
            if self.stop_event.is_set():
                raise InterruptedError("Stopped by user")
            now = time.monotonic()
            if now >= next_command:
                self.send(device, command, show=False)
                next_command = now + COMMAND_PERIOD_S
            if self.check_ctl(self.read_lines(device, buffer)) == 0:
                raise RuntimeError("MCU switched to remote mode during the run")
            if now >= next_progress:
                self.update_progress()
                next_progress = now + 0.1

    def turn_to_angle(self, device: serial.Serial, buffer: bytearray) -> float:
        if self.recorder is None or self.recorder.imu_unwrapped_deg is None:
            raise RuntimeError("No valid IMU yaw before the turn")
        turn_start_yaw = self.recorder.imu_unwrapped_deg
        target_yaw = turn_start_yaw + self.config.turn_angle_deg
        self.set_phase("turn")
        self.record(
            "EVENT",
            f"Turn target: start={turn_start_yaw:.3f} deg, "
            f"target={target_yaw:.3f} deg, timeout={self.config.turn_timeout_s:.3f} s",
        )
        started = time.monotonic()
        next_command = started
        next_progress = started
        settled_since: float | None = None

        while time.monotonic() - started < self.config.turn_timeout_s:
            if self.stop_event.is_set():
                raise InterruptedError("Stopped by user")
            lines = self.read_lines(device, buffer)
            if self.check_ctl(lines) == 0:
                raise RuntimeError("MCU switched to remote mode during the turn")

            current_yaw = self.recorder.imu_unwrapped_deg
            gyro_z = self.recorder.latest.get("yaw_rate_dps")
            imu_valid = (
                self.recorder.latest.get("imu_state") == 3
                and self.recorder.latest.get("imu_sample_valid") == 1
            )
            if current_yaw is None or not imu_valid:
                raise RuntimeError("IMU yaw became invalid during the turn")

            now = time.monotonic()
            error_deg = target_yaw - current_yaw
            gyro_settled = isinstance(gyro_z, float) and abs(gyro_z) <= TURN_SETTLED_GYRO_DPS
            if abs(error_deg) <= TURN_TOLERANCE_DEG and gyro_settled:
                if settled_since is None:
                    settled_since = now
                if now - settled_since >= TURN_SETTLED_TIME_S:
                    self.send(device, STOP_COMMAND, show=False)
                    achieved = current_yaw - turn_start_yaw
                    self.record(
                        "EVENT",
                        f"Turn completed: achieved={achieved:.3f} deg, "
                        f"error={self.config.turn_angle_deg - achieved:.3f} deg",
                    )
                    return achieved
            else:
                settled_since = None

            az_radps = turn_rate_command(error_deg)

            if now >= next_command:
                command = f"0.000,{az_radps:.3f}\r\n".encode("ascii")
                self.send(device, command, show=False)
                next_command = now + COMMAND_PERIOD_S
            if now >= next_progress:
                self.update_progress()
                next_progress = now + 0.1

        self.send(device, STOP_COMMAND, show=False)
        achieved = self.recorder.imu_unwrapped_deg - turn_start_yaw
        self.record(
            "EVENT",
            f"Turn timeout: achieved={achieved:.3f} deg, "
            f"error={self.config.turn_angle_deg - achieved:.3f} deg",
        )
        raise RuntimeError(
            f"Turn did not settle within {self.config.turn_timeout_s:g} s "
            f"(achieved {achieved:.2f} deg)"
        )

    def run_motion(self, device: serial.Serial, buffer: bytearray) -> str:
        marker, _ = self.record(
            "EVENT",
            f"Sequence started: vx={self.config.vx_mps:.3f}, "
            f"straight_1={self.config.duration_s:.3f} s",
        )
        if self.recorder is not None:
            self.recorder.mark_motion_start(marker)
        self.sequence_started = time.monotonic()
        self.run_timed_phase(
            device,
            buffer,
            "straight_1",
            self.config.command,
            self.config.duration_s,
            phase_already_set=True,
        )
        if not self.config.route_enabled:
            self.emit("progress", self.config.planned_duration_s)
            return f"Completed {self.config.duration_s:g} second straight run"

        self.run_timed_phase(
            device, buffer, "settle_before_turn", STOP_COMMAND, INTERPHASE_STOP_S
        )
        achieved = self.turn_to_angle(device, buffer)
        self.run_timed_phase(
            device, buffer, "settle_after_turn", STOP_COMMAND, INTERPHASE_STOP_S
        )
        self.run_timed_phase(
            device,
            buffer,
            "straight_2",
            self.config.command,
            self.config.second_straight_s,
        )
        self.emit("progress", self.config.planned_duration_s)
        return f"Completed route; IMU turn={achieved:.2f} deg"

    def run(self) -> None:
        paths = make_log_paths(self.config)
        self.session_started = time.monotonic()
        self.emit("log_path", paths.display())
        result = "Run failed"
        success = False
        device: serial.Serial | None = None
        buffer = bytearray()
        try:
            self.raw_file = paths.raw.open("w", encoding="utf-8", newline="\n")
            self.recorder = TelemetryRecorder(paths.samples, self.config)
            self.raw_file.write("# UART3 heading-control straight-run raw log\n")
            self.raw_file.write(
                f"# Started: {datetime.now().astimezone().isoformat(timespec='milliseconds')}\n"
                f"# Port: {self.port}, baud: {BAUD_RATE}, format: 8N1\n"
                f"# Command: vx={self.config.vx_mps:.3f} m/s, "
                f"az={self.config.az_radps:.3f} rad/s, "
                f"straight_1={self.config.duration_s:.3f} s\n"
                f"# Route: enabled={int(self.config.route_enabled)}, "
                f"turn={self.config.turn_angle_deg:.3f} deg, "
                f"turn_timeout={self.config.turn_timeout_s:.3f} s, "
                f"straight_2={self.config.second_straight_s:.3f} s\n"
                f"# PC turn controller: kp={TURN_CONTROL_KP:.3f}, "
                f"min_az={TURN_MIN_AZ_RADPS:.3f}, max_az={TURN_MAX_AZ_RADPS:.3f}, "
                f"tolerance={TURN_TOLERANCE_DEG:.3f} deg\n"
                f"# Heading controller: kp={HEADING_KP:.3f}, ki={HEADING_KI:.3f}, "
                f"kd={HEADING_KD:.3f}, output_limit={HEADING_LIMIT_RADPS:.3f} rad/s\n"
                f"# Geometry: wheel_diameter={WHEEL_DIAMETER_M:.4f} m, "
                f"track_width={TRACK_WIDTH_M:.4f} m, left_cpr={LEFT_COUNTS_PER_REV:.4f}, "
                f"right_cpr={RIGHT_COUNTS_PER_REV:.4f}\n"
            )
            for schema in FRAME_SCHEMAS:
                self.raw_file.write(f"# {schema}\n")
            self.raw_file.write("pc_elapsed_s\tpc_time\tdirection\tframe\n")
            self.raw_file.flush()
            device = serial.Serial(
                port=self.port, baudrate=BAUD_RATE, bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                timeout=0.02, write_timeout=0.25,
            )
            device.reset_input_buffer()
            device.reset_output_buffer()
            self.record("EVENT", "Serial port opened")
            self.preflight(device, buffer)
            result = self.run_motion(device, buffer)
            success = result.startswith("Completed")
        except InterruptedError as error:
            result = str(error)
        except (RuntimeError, serial.SerialException, OSError) as error:
            result = str(error)
        except Exception as error:
            result = f"Unexpected error: {error}"
        finally:
            if self.recorder is not None:
                self.recorder.mark_postflight()
            if device is not None and device.is_open:
                self.emit("status", "Stopping and collecting final telemetry...")
                try:
                    self.send_stop(device)
                    self.drain(device, buffer)
                finally:
                    device.close()
            if self.raw_file is not None and not self.raw_file.closed:
                self.record("RESULT", result)
                self.raw_file.close()
            if self.recorder is not None:
                try:
                    self.recorder.finish(paths.summary, result, success)
                except OSError as error:
                    result += f"; summary write failed: {error}"
                    success = False
            self.raw_file = None
            self.recorder = None
            self.emit("done", (success, result, paths.display()))


class Uart3RunApp:
    LIVE_ITEMS = (
        ("Phase", "phase"), ("IMU yaw", "yaw"), ("Target", "target"),
        ("Heading error", "error"), ("Correction", "correction"),
        ("Gyro Z / yaw rate", "gyro_z"), ("Encoder yaw", "encoder_yaw"),
        ("IMU - encoder", "yaw_difference"),
        ("Wheel measured L/R", "wheel_measured"),
        ("Wheel target L/R", "wheel_target"), ("PWM L/R", "pwm"),
        ("IMU health", "imu"),
    )

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.events: queue.Queue[tuple[str, object]] = queue.Queue()
        self.stop_event = threading.Event()
        self.worker: SerialRunWorker | None = None
        self.running = False
        self.closing = False
        self.port_labels: dict[str, str] = {}
        default_config = RunConfig(DEFAULT_VX_MPS, DEFAULT_AZ_RADPS, DEFAULT_DURATION_S)
        self.run_duration_s = default_config.planned_duration_s
        self.port_var = tk.StringVar()
        self.vx_var = tk.StringVar(value=f"{DEFAULT_VX_MPS:g}")
        self.az_var = tk.StringVar(value=f"{DEFAULT_AZ_RADPS:g}")
        self.duration_var = tk.StringVar(value=f"{DEFAULT_DURATION_S:g}")
        self.route_enabled_var = tk.BooleanVar(value=DEFAULT_ROUTE_ENABLED)
        self.turn_angle_var = tk.StringVar(value=f"{DEFAULT_TURN_ANGLE_DEG:g}")
        self.turn_timeout_var = tk.StringVar(value=f"{DEFAULT_TURN_TIMEOUT_S:g}")
        self.second_straight_var = tk.StringVar(value=f"{DEFAULT_SECOND_STRAIGHT_S:g}")
        self.status_var = tk.StringVar(value="Ready")
        self.log_path_var = tk.StringVar(value="No run recorded")
        self.progress_var = tk.DoubleVar(value=0.0)
        self.remaining_var = tk.StringVar(value=f"{self.run_duration_s:.1f} s max")
        self.live_vars = {key: tk.StringVar(value="--") for _, key in self.LIVE_ITEMS}
        self.build_ui()
        self.refresh_ports()
        self.root.protocol("WM_DELETE_WINDOW", self.close_window)
        self.root.after(50, self.poll_events)

    def build_ui(self) -> None:
        self.root.title("UART3 Heading-Control Logger")
        self.root.geometry("1040x720")
        self.root.minsize(860, 600)
        main = ttk.Frame(self.root, padding=12)
        main.grid(row=0, column=0, sticky="nsew")
        self.root.rowconfigure(0, weight=1)
        self.root.columnconfigure(0, weight=1)
        main.columnconfigure(1, weight=1)
        main.rowconfigure(8, weight=1)

        ttk.Label(main, text="Serial port").grid(row=0, column=0, sticky="w", padx=(0, 8))
        self.port_combo = ttk.Combobox(main, textvariable=self.port_var, state="readonly", width=44)
        self.port_combo.grid(row=0, column=1, sticky="ew")
        self.refresh_button = ttk.Button(main, text="Refresh", command=self.refresh_ports, width=10)
        self.refresh_button.grid(row=0, column=2, padx=(8, 0))

        settings = ttk.Frame(main)
        settings.grid(row=1, column=0, columnspan=3, sticky="ew", pady=(12, 4))
        entries = (
            ("VX (m/s)", self.vx_var), ("AZ (rad/s)", self.az_var),
            ("Straight 1 (s)", self.duration_var),
        )
        self.setting_entries: list[ttk.Entry] = []
        for index, (label, variable) in enumerate(entries):
            ttk.Label(settings, text=label).grid(row=0, column=index * 2, sticky="w")
            entry = ttk.Entry(settings, textvariable=variable, width=10)
            entry.grid(row=0, column=index * 2 + 1, padx=(6, 18))
            self.setting_entries.append(entry)

        self.route_check = ttk.Checkbutton(
            settings,
            text="Add turn + second straight",
            variable=self.route_enabled_var,
            command=self.update_route_entry_state,
        )
        self.route_check.grid(row=1, column=0, columnspan=2, sticky="w", pady=(8, 0))
        route_entries = (
            ("Turn (+left, deg)", self.turn_angle_var),
            ("Turn timeout (s)", self.turn_timeout_var),
            ("Straight 2 (s)", self.second_straight_var),
        )
        self.route_entries: list[ttk.Entry] = []
        for index, (label, variable) in enumerate(route_entries, start=1):
            ttk.Label(settings, text=label).grid(
                row=1, column=index * 2, sticky="w", pady=(8, 0)
            )
            entry = ttk.Entry(settings, textvariable=variable, width=8)
            entry.grid(row=1, column=index * 2 + 1, padx=(6, 18), pady=(8, 0))
            self.route_entries.append(entry)
        self.update_route_entry_state()

        controls = ttk.Frame(main)
        controls.grid(row=2, column=0, columnspan=3, sticky="ew", pady=(8, 8))
        controls.columnconfigure(2, weight=1)
        self.start_button = ttk.Button(controls, text="Start run", command=self.start_run, width=14)
        self.start_button.grid(row=0, column=0, padx=(0, 8))
        self.stop_button = ttk.Button(
            controls, text="Stop", command=self.stop_run, state="disabled", width=10
        )
        self.stop_button.grid(row=0, column=1, padx=(0, 12))
        self.progress = ttk.Progressbar(
            controls, variable=self.progress_var, maximum=DEFAULT_DURATION_S,
            mode="determinate",
        )
        self.progress.grid(row=0, column=2, sticky="ew")
        ttk.Label(controls, textvariable=self.remaining_var, width=9, anchor="e").grid(
            row=0, column=3, padx=(8, 0)
        )
        ttk.Label(main, textvariable=self.status_var, wraplength=980).grid(
            row=3, column=0, columnspan=3, sticky="w"
        )
        ttk.Label(main, text="Outputs").grid(row=4, column=0, sticky="nw", pady=(6, 6))
        ttk.Entry(main, textvariable=self.log_path_var, state="readonly").grid(
            row=4, column=1, columnspan=2, sticky="ew", pady=(6, 6)
        )

        live = ttk.LabelFrame(main, text="Live synchronized telemetry", padding=8)
        live.grid(row=5, column=0, columnspan=3, sticky="ew", pady=(4, 6))
        for column in range(4):
            live.columnconfigure(column, weight=1)
        for index, (label, key) in enumerate(self.LIVE_ITEMS):
            cell = ttk.Frame(live)
            cell.grid(row=index // 4, column=index % 4, sticky="ew", padx=6, pady=3)
            ttk.Label(cell, text=label).grid(row=0, column=0, sticky="w")
            ttk.Label(cell, textvariable=self.live_vars[key]).grid(row=1, column=0, sticky="w")

        ttk.Separator(main).grid(row=6, column=0, columnspan=3, sticky="ew", pady=(2, 8))
        ttk.Label(main, text="Raw stream").grid(row=7, column=0, columnspan=3, sticky="w")
        self.log_text = scrolledtext.ScrolledText(
            main, height=16, wrap=tk.NONE, font=("Consolas", 9), state="disabled"
        )
        self.log_text.grid(row=8, column=0, columnspan=3, sticky="nsew", pady=(4, 0))

    def refresh_ports(self) -> None:
        previous = self.port_var.get()
        self.port_labels = {
            f"{device} | {description}": device
            for device, description in available_ports()
        }
        labels = list(self.port_labels)
        self.port_combo["values"] = labels
        self.port_var.set(previous if previous in self.port_labels else (labels[0] if labels else ""))
        if not self.running:
            self.start_button.configure(state="normal" if labels else "disabled")
        self.status_var.set(f"Found {len(labels)} serial port(s)" if labels else "No serial ports found")

    def append_line(self, line: str) -> None:
        self.log_text.configure(state="normal")
        self.log_text.insert(tk.END, line + "\n")
        if int(self.log_text.index("end-1c").split(".")[0]) > 2000:
            self.log_text.delete("1.0", "501.0")
        self.log_text.see(tk.END)
        self.log_text.configure(state="disabled")

    def read_config(self) -> RunConfig | None:
        try:
            values = tuple(float(item.get()) for item in (self.vx_var, self.az_var, self.duration_var))
        except ValueError:
            messagebox.showerror("UART3", "VX, AZ, and duration must be numbers.")
            return None
        vx, az, duration = values
        if not all(math.isfinite(value) for value in values):
            messagebox.showerror("UART3", "VX, AZ, and duration must be finite numbers.")
            return None
        if abs(vx) > 1.0 or abs(az) > 10.0 or not 0.5 <= duration <= 600.0:
            messagebox.showerror(
                "UART3", "Allowed ranges: VX +/-1.0 m/s, AZ +/-10 rad/s, duration 0.5-600 s."
            )
            return None
        route_enabled = bool(self.route_enabled_var.get())
        turn_angle = DEFAULT_TURN_ANGLE_DEG
        turn_timeout = DEFAULT_TURN_TIMEOUT_S
        second_straight = DEFAULT_SECOND_STRAIGHT_S
        if route_enabled:
            try:
                route_values = tuple(
                    float(item.get())
                    for item in (
                        self.turn_angle_var,
                        self.turn_timeout_var,
                        self.second_straight_var,
                    )
                )
            except ValueError:
                messagebox.showerror(
                    "UART3", "Turn angle, timeout, and straight 2 duration must be numbers."
                )
                return None
            if not all(math.isfinite(value) for value in route_values):
                messagebox.showerror("UART3", "Route settings must be finite numbers.")
                return None
            turn_angle, turn_timeout, second_straight = route_values
            if not 5.0 <= abs(turn_angle) <= 360.0:
                messagebox.showerror("UART3", "Turn angle must be between -360 and +360 deg, excluding -5 to +5 deg.")
                return None
            if not 1.0 <= turn_timeout <= 60.0:
                messagebox.showerror("UART3", "Turn timeout must be between 1 and 60 s.")
                return None
            if not 0.5 <= second_straight <= 600.0:
                messagebox.showerror("UART3", "Straight 2 duration must be between 0.5 and 600 s.")
                return None
        return RunConfig(
            vx_mps=vx,
            az_radps=az,
            duration_s=duration,
            route_enabled=route_enabled,
            turn_angle_deg=turn_angle,
            turn_timeout_s=turn_timeout,
            second_straight_s=second_straight,
        )

    def update_route_entry_state(self) -> None:
        enabled = not self.running and bool(self.route_enabled_var.get())
        for entry in self.route_entries:
            entry.configure(state="normal" if enabled else "disabled")

    def set_inputs_enabled(self, enabled: bool) -> None:
        self.refresh_button.configure(state="normal" if enabled else "disabled")
        self.port_combo.configure(state="readonly" if enabled else "disabled")
        for entry in self.setting_entries:
            entry.configure(state="normal" if enabled else "disabled")
        self.route_check.configure(state="normal" if enabled else "disabled")
        self.update_route_entry_state()

    def start_run(self) -> None:
        port = self.port_labels.get(self.port_var.get())
        if port is None:
            messagebox.showerror("UART3", "Select a serial port first.")
            return
        config = self.read_config()
        if config is None:
            return
        self.running = True
        self.run_duration_s = config.planned_duration_s
        self.stop_event = threading.Event()
        self.progress_var.set(0.0)
        self.progress.configure(maximum=config.planned_duration_s)
        self.remaining_var.set(f"{config.planned_duration_s:.1f} s max")
        self.status_var.set("Starting...")
        self.log_path_var.set("Creating raw, CSV, and summary logs...")
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", tk.END)
        self.log_text.configure(state="disabled")
        for variable in self.live_vars.values():
            variable.set("--")
        self.start_button.configure(state="disabled")
        self.stop_button.configure(state="normal")
        self.set_inputs_enabled(False)
        self.worker = SerialRunWorker(port, config, self.events, self.stop_event)
        self.worker.start()

    def stop_run(self) -> None:
        if self.running:
            self.stop_event.set()
            self.stop_button.configure(state="disabled")
            self.status_var.set("Stop requested...")

    def handle_done(self, payload: tuple[bool, str, str]) -> None:
        success, result, paths = payload
        self.running = False
        self.worker = None
        self.status_var.set(result)
        self.log_path_var.set(paths)
        self.stop_button.configure(state="disabled")
        self.set_inputs_enabled(True)
        self.start_button.configure(state="normal" if self.port_labels else "disabled")
        if not success:
            self.append_line(f"RESULT  {result}")
        if self.closing:
            self.root.destroy()

    def poll_events(self) -> None:
        try:
            while True:
                kind, value = self.events.get_nowait()
                if kind == "line":
                    self.append_line(str(value))
                elif kind == "status":
                    self.status_var.set(str(value))
                elif kind == "log_path":
                    self.log_path_var.set(str(value))
                elif kind == "progress":
                    elapsed = min(float(value), self.run_duration_s)
                    self.progress_var.set(elapsed)
                    self.remaining_var.set(
                        f"{max(0.0, self.run_duration_s - elapsed):.1f} s max"
                    )
                elif kind == "telemetry" and isinstance(value, dict):
                    for key, text in value.items():
                        if key in self.live_vars:
                            self.live_vars[key].set(str(text))
                elif kind == "done":
                    self.handle_done(value)  # type: ignore[arg-type]
                    if self.closing:
                        return
        except queue.Empty:
            pass
        if self.root.winfo_exists():
            self.root.after(50, self.poll_events)

    def close_window(self) -> None:
        if self.running:
            self.closing = True
            self.stop_run()
            self.status_var.set("Stopping before exit...")
            return
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    Uart3RunApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
