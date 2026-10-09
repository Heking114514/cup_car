# Wheel encoder calibration - 2026-10-06

## Conditions

- Every raw value below is the encoder increment for exactly 10 wheel revolutions.
- Wheel diameter remains 60.4 mm (radius 30.2 mm).
- Motor encoder resolution is 8192 count per motor revolution.

## Raw measurements

Left wheel, five 10-revolution increments:

```text
3696635
3691354
3683265
3696397
3675710
```

Right wheel, six 10-revolution increments:

```text
3703248
3685361
3693220
3675951
3692265
3682981
```

## Derived constants

```text
left sum                 = 18443361 count
left count/wheel-rev     = 18443361 / (5 * 10)
                         = 368867.22

right sum                = 22133026 count
right count/wheel-rev    = 22133026 / (6 * 10)
                         = 368883.76666666666

left effective reduction = 368867.22 / 8192
                         = 45.02773681640625
right effective reduction= 368883.76666666666 / 8192
                         = 45.02975667317708
```

The population standard deviations of the 10-revolution measurements are
8093.54 count (0.2194%) on the left and 8670.48 count (0.2350%) on the right.
The mean right/left CPR difference is only 0.00449%, but independent values are
kept so odometry does not turn this small scale difference into yaw drift.

With the unchanged 60.4 mm wheel diameter:

```text
left count/m  = 1943941.768462438
right count/m = 1944028.969912494
```

These per-wheel CPR values are authoritative for the STM32 conversion and the
ROS wheel-odometry configuration until a newer complete calibration supersedes
this record.
