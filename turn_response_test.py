#!/usr/bin/env python3
"""Four in-place 90-degree turns and returns, with raw/CSV telemetry.

Usage: python3 turn_response_test.py [--port /dev/ttyACM0]
Installed convention inferred from preceding runs: host +az = left,
ATT yaw decreases on a left turn. The first movement verifies this sign.
"""
from __future__ import annotations
import argparse
import csv
import json
import math
from datetime import datetime
from pathlib import Path
import time
import serial
from serial.tools import list_ports

RATES = (0.1, 0.3, 0.6, 0.9)
SUMMARY_FIELDS = (
    'rate_radps', 'leg', 'command_radps', 'duration_s', 'target_yaw_deg',
    'start_yaw_deg', 'cutoff_yaw_deg', 'settled_yaw_deg', 'endpoint_error_deg',
    'travel_deg', 'response_10_s', 'response_90_s', 'mean_rate_radps',
    'peak_rate_radps', 'angle_source',
)


class Sweep:
    """State machine shared by serial operation and deterministic simulation."""
    def __init__(self, now: float, command_sign: int = 1, yaw_sign: int = -1):
        self.command_sign, self.yaw_sign = command_sign, yaw_sign
        self.phase, self.since = 'ready', now
        self.index, self.leg, self.command = 0, 'left', 0.0
        self.records: list[dict] = []
        self.samples: list[tuple[float, float]] = []
        self.initial = self.still_since = self.pending = None

    def begin(self, now: float, yaw: float) -> None:
        rate = RATES[self.index]
        self.target = self.initial + self.yaw_sign * 90 if self.leg == 'left' else self.initial
        self.start_yaw = yaw
        self.direction = 1 if self.target > yaw else -1
        self.command = self.command_sign * rate * (1 if self.leg == 'left' else -1)
        self.since, self.phase = now, 'turn'
        self.samples = []
        self.response10 = self.response90 = None

    def tick(self, now: float, yaw: float, gyro: float, ready: bool) -> float:
        if self.phase == 'done':
            return 0.0
        if not ready:
            self.still_since = None
            if self.phase != 'ready':
                raise RuntimeError('有效 IMU/电机遥测丢失，测试中止')
            if now - self.since > 30:
                raise RuntimeError('等待 IMU Bias 与双轮反馈超过 30 秒')
            return 0.0
        if self.phase in ('ready', 'settle'):
            if abs(gyro) < 0.8:
                if self.still_since is None:
                    self.still_since = now
            else:
                self.still_since = None
            if self.still_since is not None and now - self.still_since >= 1.0:
                self.still_since = None
                if self.phase == 'ready':
                    self.initial = yaw
                else:
                    self.pending['settled_yaw_deg'] = yaw
                    self.pending['endpoint_error_deg'] = yaw - self.pending['target_yaw_deg']
                    self.pending['travel_deg'] = yaw - self.pending['start_yaw_deg']
                    self.records.append(self.pending)
                    self.pending = None
                    if self.leg == 'left':
                        self.leg = 'return'
                    else:
                        self.index += 1
                        self.leg = 'left'
                        # Each rate is an independent 90-degree trial.  Record
                        # return error before rebasing, rather than adding the
                        # preceding trial's overshoot to the next left turn.
                        self.initial = yaw
                    if self.index == len(RATES):
                        self.phase = 'done'
                        return 0.0
                self.begin(now, yaw)
            elif self.phase == 'settle' and now - self.since > 8:
                raise RuntimeError('停车后 8 秒内仍未稳定')
            return self.command if self.phase == 'turn' else 0.0
        rate = RATES[self.index]
        elapsed = now - self.since
        progress = self.direction * (yaw - self.start_yaw)
        measured_rate = self.direction * math.radians(gyro)
        self.samples.append((elapsed, measured_rate))
        if self.response10 is None and measured_rate >= rate * 0.1:
            self.response10 = elapsed
        if self.response90 is None and measured_rate >= rate * 0.9:
            self.response90 = elapsed
        if progress < -3.0:
            raise RuntimeError('转向与预期 IMU 符号相反，已停止；核对两个方向参数')
        if abs(yaw - self.initial) > 110:
            raise RuntimeError('相对起始航向超过 110 度，已停止')
        if elapsed > max(8.0, math.pi / (2 * rate) * 2.5 + 3):
            raise RuntimeError('本次转向超时，可能低速未启动或角速度不足')
        if self.direction * (yaw - self.target) >= 0:
            stable_rates = [g for t, g in self.samples if t >= 0.3]
            self.pending = dict(
                rate_radps=rate, leg=self.leg, command_radps=self.command,
                duration_s=elapsed, target_yaw_deg=self.target,
                start_yaw_deg=self.start_yaw, cutoff_yaw_deg=yaw,
                response_10_s=self.response10, response_90_s=self.response90,
                mean_rate_radps=(sum(stable_rates)/len(stable_rates) if stable_rates else None),
                peak_rate_radps=max(g for _, g in self.samples),
                angle_source='gyro_integrated_mcu',
            )
            self.phase, self.since, self.command = 'settle', now, 0.0
        return self.command


class Telemetry:
    def __init__(self):
        self.yaw = 0.0  # Firmware-reported yaw, retained for diagnostics.
        self.control_yaw = 0.0  # Independent angle integrated from gyro samples.
        self.last_corrected_gyro = None
        self.angle_initialized = False
        self.angle_ok = False
        self.last_yaw = None
        self.gyro = 0.0
        self.att_seen = self.motor_seen = -math.inf
        self.sample_stamp = None
        self.imu_ok = self.motor_ok = False
        self.ctl_seen = -math.inf
        self.command_valid = False
        self.command_az = 0.0
        self.targets = (0, 0)

    def feed(self, line: str, now: float) -> None:
        fields = line.split(',')
        try:
            if fields[0] == 'ATT' and len(fields) >= 8:
                stamp, status = int(fields[1]), int(fields[3])
                yaw, gyro = int(fields[4])/1000, int(fields[5])/1000
                bias = int(fields[6]) / 1000
                if stamp == self.sample_stamp:
                    return
                previous_stamp = self.sample_stamp
                self.sample_stamp = stamp
                self.imu_ok = (status & 0x0C) == 0x0C and not (status & 0x60)
                if self.last_yaw is None:
                    self.yaw = yaw
                else:
                    self.yaw += (yaw - self.last_yaw + 180) % 360 - 180
                corrected = gyro - bias
                dt = ((stamp - previous_stamp) & 0xFFFFFFFF) / 1000 if previous_stamp is not None else 0.0
                self.angle_ok = self.imu_ok
                if self.imu_ok:
                    if not self.angle_initialized:
                        self.control_yaw = self.yaw
                        self.angle_initialized = True
                    elif not (0.0 < dt <= 0.35) or self.last_corrected_gyro is None:
                        # No extrapolation across resets or missing samples.
                        self.angle_ok = False
                    else:
                        self.control_yaw += (self.last_corrected_gyro + corrected) * 0.5 * dt
                    self.last_corrected_gyro = corrected
                else:
                    self.last_corrected_gyro = None
                self.last_yaw, self.gyro, self.att_seen = yaw, corrected, now
            elif fields[0] == 'CTL' and len(fields) >= 15:
                self.command_valid = int(fields[5]) == 1
                self.command_az = int(fields[8]) / 1000
                self.targets = (int(fields[9]), int(fields[10]))
                self.ctl_seen = now
            elif fields[0] == 'MTR' and len(fields) >= 25:
                self.motor_ok = (int(fields[10]) == int(fields[11]) == 1 and
                                 max(int(fields[12]), int(fields[13])) <= 200 and
                                 max(int(fields[8]), int(fields[9])) < 75 and
                                 int(fields[22]) == int(fields[23]) == 0 and
                                 int(fields[3]) == 0)
                self.motor_seen = now
        except (ValueError, IndexError):
            return

    def ready(self, now: float) -> bool:
        return (self.imu_ok and self.angle_ok and self.motor_ok and now - self.att_seen < 0.35
                and now - self.motor_seen < 0.4)


def run(port: str, args) -> int:
    directory = Path(__file__).resolve().parent / 'turn_response_logs'
    directory.mkdir(exist_ok=True)
    prefix = directory / datetime.now().strftime('turn_response_%Y%m%d_%H%M%S_%f')
    start = time.monotonic()
    sweep = Sweep(start, args.left_command_sign, args.left_yaw_sign)
    telemetry = Telemetry()
    error, device = '', None
    try:
        with prefix.with_suffix('.txt').open('w', encoding='utf-8') as raw, prefix.with_suffix('.csv').open('w', newline='', encoding='utf-8') as trace:
            writer = csv.writer(trace)
            writer.writerow(['host_elapsed_s', 'phase', 'rate_radps', 'leg', 'command_radps', 'reported_yaw_deg', 'control_yaw_deg', 'gyro_dps', 'raw_frame'])
            raw.write(json.dumps(vars(args), ensure_ascii=False) + '\n')
            device = serial.Serial(port, 115200, timeout=0.01, write_timeout=0.2, exclusive=True)
            device.write(b'0,0\r\n')
            buffer = bytearray()
            sent, next_send, phase, count = None, 0.0, None, 0
            while sweep.phase != 'done':
                now = time.monotonic()
                chunk = device.read(min(device.in_waiting or 1, 8192))
                buffer.extend(chunk)
                if len(buffer) > 65536:
                    raise RuntimeError('串口行缓冲区溢出')
                while b'\n' in buffer:
                    line, _, tail = buffer.partition(b'\n')
                    buffer = bytearray(tail)
                    line = line.decode('ascii', errors='replace').strip()
                    raw.write(f'{now-start:.6f} RX {line}\n')
                    telemetry.feed(line, now)
                    writer.writerow([now-start, sweep.phase, RATES[min(sweep.index, 3)], sweep.leg, sweep.command, telemetry.yaw, telemetry.control_yaw, telemetry.gyro, line])
                command = sweep.tick(now, telemetry.control_yaw, telemetry.gyro, telemetry.ready(now))
                if (sweep.phase == 'turn' and now - sweep.since > 1.5
                        and now - telemetry.ctl_seen < 0.3
                        and telemetry.command_valid
                        and abs(telemetry.command_az - command) < 0.002
                        and telemetry.targets == (0, 0)):
                    raise RuntimeError('固件收到角速度指令但左右轮目标为零；请烧录修复停车阈值的新固件')
                if command != sent or now >= next_send:
                    packet = f'0.000000,{command:.6f}\r\n'.encode('ascii')
                    device.write(packet)
                    raw.write(f'{now-start:.6f} TX {packet.decode().strip()}\n')
                    sent, next_send = command, now + 0.05
                current = (sweep.index, sweep.leg, sweep.phase)
                if current != phase:
                    print(f'{now-start:6.1f}s  {RATES[min(sweep.index, 3)]:.1f} rad/s  {sweep.leg} / {sweep.phase}  yaw={telemetry.control_yaw:.2f}°', flush=True)
                    phase = current
                    raw.flush()
                    trace.flush()
                if len(sweep.records) != count:
                    count = len(sweep.records)
                    record = sweep.records[-1]
                    print(f"  耗时 {record['duration_s']:.2f}s，停车后误差 {record['endpoint_error_deg']:+.2f}°", flush=True)
    except (RuntimeError, OSError, serial.SerialException, KeyboardInterrupt) as exc:
        error = str(exc) or '用户中止'
        print('测试停止：' + error, flush=True)
    finally:
        if device is not None:
            try:
                for _ in range(5):
                    device.write(b'0,0\r\n')
                    time.sleep(0.03)
            except (OSError, serial.SerialException):
                pass
            device.close()
        with prefix.with_name(prefix.name + '_summary.csv').open('w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=SUMMARY_FIELDS)
            writer.writeheader()
            writer.writerows(sweep.records)
        prefix.with_suffix('.json').write_text(json.dumps({'port': port, 'complete': sweep.phase == 'done', 'error': error, 'arguments': vars(args), 'results': sweep.records}, ensure_ascii=False, indent=2), encoding='utf-8')
        print('日志：' + str(prefix), flush=True)
    return 0 if sweep.phase == 'done' else 1


def self_test() -> None:
    sweep = Sweep(0)
    yaw = 17.0
    for step in range(20000):
        now = step * 0.01
        command = sweep.tick(now, yaw, -math.degrees(sweep.command), True)
        yaw -= math.degrees(command) * 0.01
        if sweep.phase == 'done':
            break
    assert sweep.phase == 'done' and len(sweep.records) == 8
    for r in sweep.records:
        assert abs(r['endpoint_error_deg']) < 0.6
        # Absolute targets include both the preceding and current cutoff
        # errors; at 0.9 rad/s, two 10 ms samples span at most 1.04 deg.
        assert abs(abs(r['travel_deg']) - 90) < 1.1
        assert abs(r['mean_rate_radps'] - r['rate_radps']) < 1e-6
    assert abs(yaw - 17) < 0.6
    for case in ('wrong_direction', 'feedback_lost', 'timeout'):
        s = Sweep(0)
        s.tick(0, 0, 0, True)
        s.tick(1.1, 0, 0, True)
        try:
            s.tick(100 if case == 'timeout' else 2, 4 if case == 'wrong_direction' else 0, 0, case != 'feedback_lost')
        except RuntimeError:
            pass
        else:
            raise AssertionError(case)
    telemetry = Telemetry()
    telemetry.feed('ATT,100,1,13,179900,0,0,600', 0)
    telemetry.feed('ATT,120,2,13,-179900,0,0,600', 0.02)
    assert abs(telemetry.yaw - 180.1) < 1e-6
    telemetry.feed('MTR,120,1,0,0,0,0,0,29,29,1,1,2,2,10,10,100,0,0,2,0,0,0,0,4', 0.02)
    assert telemetry.ready(0.02) and not telemetry.ready(0.5)
    frozen = Telemetry()
    for i in range(51):
        # 95 dps reported minus 5 dps bias = 90 dps for exactly 1 second.
        frozen.feed(f'ATT,{100 + i * 20},{i},13,0,95000,5000,600', i * .02)
    assert frozen.yaw == 0.0 and abs(frozen.control_yaw - 90) < 1e-6
    before = frozen.control_yaw
    frozen.feed('ATT,1100,51,13,0,95000,5000,600', 1.01)
    assert frozen.control_yaw == before  # Duplicate MCU sample cannot integrate twice.
    wrapped = Telemetry()
    wrapped.feed('ATT,4294967280,1,13,0,10000,0,600', 0)
    wrapped.feed('ATT,4,2,13,0,10000,0,600', .02)
    assert abs(wrapped.control_yaw - .2) < 1e-6
    trials = Sweep(0)
    trials.tick(0, 0, 0, True)
    trials.tick(1.1, 0, 0, True)
    for index in range(4):
        assert trials.index == index and trials.leg == 'left'
        assert abs(trials.target - trials.start_yaw + 90) < 1e-6
        for sign in (-1, 1):
            target = trials.target
            trials.tick(trials.since + 1, target + sign * .1, 0, True)
            assert trials.phase == 'settle'
            settle_start = trials.since
            trials.tick(settle_start + .1, target + sign * 2, 0, True)
            trials.tick(settle_start + 1.2, target + sign * 2, 0, True)
    assert trials.phase == 'done' and len(trials.records) == 8
    assert all(abs(abs(r['endpoint_error_deg']) - 2) < 1e-6 for r in trials.records)
    print('PASS: 四档独立90度基准、八段往返、异常停车、冻结Yaw积分、零偏及MCU时间回绕')


def main() -> int:
    parser = argparse.ArgumentParser(description='0.1/0.3/0.6/0.9 rad/s 原地左转90度并返回；Ctrl-C停车')
    parser.add_argument('--port', help='STM32 VCP，如 /dev/ttyACM0')
    parser.add_argument('--left-command-sign', type=int, choices=(-1, 1), default=1)
    parser.add_argument('--left-yaw-sign', type=int, choices=(-1, 1), default=-1)
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return 0
    port = args.port
    if not port:
        candidates = [p.device for p in list_ports.comports() if (p.vid, p.pid) == (0x0483, 0x5740)]
        if len(candidates) != 1:
            parser.exit(2, '未发现唯一的 STM32 USB 串口。请连接小车并关闭占用串口的GUI，再用 --port 指定。\n')
        port = candidates[0]
    return run(port, args)


if __name__ == '__main__':
    raise SystemExit(main())
