# Wheel encoder calibration - latest 2026-10-11

## Conditions

- Each sample is the encoder increment for exactly 10 wheel revolutions.
- Three samples were taken for each wheel.
- Wheel diameter remains 60.4 mm (radius 30.2 mm).
- Motor encoder resolution is 8192 count per motor revolution.
- Wheel outer distance is 142 mm and wheel width is 23 mm, so wheel-center
  track width is 119 mm.

## Raw measurements

Right wheel, three 10-revolution increments:

```text
102578
101736
102743
```

Left wheel, three 10-revolution increments:

```text
101387
103127
102469
```

## Derived constants

```text
left sum                  = 306983 count
left mean per 10 rev      = 306983 / 3 = 102327.6667 count
left count/wheel-rev      = 10232.7667 count
left effective reduction  = 10232.7667 / 8192 = 1.2491170

right sum                 = 307057 count
right mean per 10 rev     = 307057 / 3 = 102352.3333 count
right count/wheel-rev     = 10235.2333 count
right effective reduction = 10235.2333 / 8192 = 1.2494181
```

The left/right CPR difference is about 0.0241%, so the separate values are kept
in the firmware and test GUI for straight-line synchronization.
