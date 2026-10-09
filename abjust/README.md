# abjust serial logger

This folder is for remote-control tuning logs. The logger only reads the STM32
virtual serial port by default. It does not send speed commands.

## Commands

List serial ports:

```bash
python3 abjust/record_car_state.py --list
```

Start one log session with auto-selected VCP:

```bash
python3 abjust/record_car_state.py
```

Start one log session with an explicit port:

```bash
python3 abjust/record_car_state.py --port /dev/ttyACM0 --baud 115200
```

Stop logging with `Ctrl-C`. Each run creates a new file under `abjust/logs/`,
for example:

```text
abjust/logs/car_state_20261005_213000.txt
```

## Logged frames

Known firmware frames are decoded and the original raw line is kept too:

- `ENC`: MCU time, sequence, left/right encoder total counts.
- `IMU`: raw BMI088 gyro counts and temperature.
- `ATT`: BMI088 yaw, gyro Z, bias Z, and validity status.
- `HLD`: yaw-hold state, reference, error, and correction.
- `SYN`: normalized left/right encoder phase error and the encoder contribution
  blended into the single complementary heading PID.
- `CTL`: command validity, wheel targets, measured speeds, and motor outputs.
- `MTR`: measured motor currents, speed-loop integrals, temperatures, and drive status.
- `RFID`: latest RFID state.
- `SNAP`: merged host-side snapshot for quick tuning.

For straight-line tuning, the most useful fields are `target_l_mps`,
`target_r_mps`, `meas_l_mps`, `meas_r_mps`, `out_l`, `out_r`, `yaw_deg`,
`hld_err_deg`, and `hld_corr`.

`MTR.drive_status` uses bit `0x01` while wheel starts are mismatched and bit
`0x02` after the 2.5 second start timeout has stopped both wheels. Bit `0x04`
means the bounded anti-stall reference boost is active. Return the controls to
zero before retrying after a timeout. Bit `0x08` means straight-line heading
error exceeded 12 degrees for 0.5 seconds and both wheels were stopped.

The extended `MTR` fields include per-motor feedback online flags, feedback
ages and counters, plus CAN1 transmit success/failure counters, last failure
age, HAL/protocol error codes, bus-off/error-passive flags, and TX FIFO space.
