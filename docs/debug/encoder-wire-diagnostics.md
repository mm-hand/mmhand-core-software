# Magnetic encoder link diagnostics (CAN path)

The S3 checks the encoder wiring and ships the verdict to the PC over the normal
`S3 -> CAN -> P4 -> USB` link. There is no separate USB diagnostic path.

## Status bar

| Label | Colour | Meaning |
| --- | --- | --- |
| `CAN: OK` | green | P4 is fresh and encoder raw frames keep arriving over CAN. |
| `CAN: ERROR` | red | P4 is fresh but no encoder raw frame arrived for > 1 s. |
| `CAN: UNKNOWN` | grey | P4 is not connected, or the 3 s connect grace period has not elapsed. |

## Per-joint badges

On Joint Calibration, Motor Calibration, Feedback Control and Direct Control
each joint row gets **one extra red badge** only when its encoder is faulty; a
normal joint adds nothing.

| Badge | Meaning |
| --- | --- |
| `CAN ERROR` | CAN is down, so no joint can be judged (shown on every row). |
| `ENCODER OPEN` | The finger chain is not connected or broken. |
| `ENCODER SHORT` | The finger's MISO is shorted or held. |
| `ENCODER ERROR` | CAN works, but this joint returns no valid raw. |

## How the class travels over CAN

The S3 probe reads each finger once with the MISO internal pull-up and once
with the pull-down:

- follows the pull (`0xFFFF`/`0x0000`) -> open (`N`)
- `0x0000`/`0x0000` -> short (`S`)
- `0xFFFF`/`0xFFFF` -> held high (`H`, reported as short)
- otherwise -> driven (`O`)

The class is encoded in the encoder raw word, which is already reserved for
"invalid" values outside the 14-bit angle range:

| Canonical CAN value | P4 -> PC value | Meaning |
| --- | --- | --- |
| `0xFFFF` | `0x7FFF` | invalid / generic encoder error |
| `0x7FFE` | `0x7FFE` | open |
| `0x7FFD` | `0x7FFD` | short |

The P4 preserves these values and forwards them unchanged; the app parses them
per joint. Fingers classified as open/short have all their joints marked
invalid in the S3 snapshot, so floating garbage never looks like an angle.

Because the sentinel meaning is part of the wire protocol, the protocol
version is bumped together with both firmwares and the app.

## Limits

- Internal pull-up/pull-down must take effect on a pin owned by the SPI
  peripheral; verified on the bench (`up=FFFF`, `down=0000` on an unconnected
  finger).
- A sensor sitting exactly at angle 0 also reads `0x0000`/`0x0000` and is
  reported `short`; check the physical wiring before replacing the sensor.
- A broken MOSI/SCLK shares the "no device drives" signature with an unplugged
  finger; confirm with a scope.
